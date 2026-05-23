"""
Reader utilities for Dynawo ``.bin`` curves files.

File layout (all integers little-endian):

    Header:
      magic    : u8[4]   = b"DYNB"
      version  : u32     = 1
      n_vars   : u32     (number of tracked variables, NOT counting time)
      for i in [0, n_vars):
          name_len : u32
          name     : u8[name_len]   (UTF-8)

    Records (appended):
      time   : f64
      values : f64[n_vars]

The module exposes:

* ``load_bin(data)``        — parse a whole file given as ``bytes`` and return
                              a pandas DataFrame. Used by the Streamlit app.
* ``read_header(f)``        — parse just the header of an open file. Cheap
                              even on 5 GB files because the record section
                              is skipped.
* ``count_records(path)``   — number of records, header-only cost.
* a command-line interface:

      python binary_curves.py names       <input.bin> [-o names.txt]
      python binary_curves.py count       <input.bin>
      python binary_curves.py extract     <input.bin> -p PATTERN ... [-o out.csv]
      python binary_curves.py extract-bin <input.bin> -p PATTERN ... [-o out.bin]

The ``extract`` commands stream the record section in fixed-size chunks so they
can handle files that do not fit in RAM.
"""

from __future__ import annotations

import argparse
import csv
import fnmatch
import os
import re
import struct
import sys
from dataclasses import dataclass
from typing import BinaryIO, Iterable, Iterator, List, Optional, Sequence

import numpy as np

# ── Format constants ─────────────────────────────────────────────────────────

MAGIC = b"DYNB"
VERSION = 1
HEADER_FIXED_SIZE = 12           # 4 (magic) + 4 (version) + 4 (n_vars)
DOUBLE_SIZE = 8
DEFAULT_CHUNK_BYTES = 1024 * 1024 * 1024  # 1 GiB working set per chunk


@dataclass(frozen=True)
class Header:
    """Parsed header of a .bin file."""
    version: int
    n_vars: int
    names: List[str]
    data_offset: int          # absolute byte offset where records start

    @property
    def record_size(self) -> int:
        return (1 + self.n_vars) * DOUBLE_SIZE


# ── Header parsing ───────────────────────────────────────────────────────────

def read_header(f: BinaryIO) -> Header:
    """Read and validate the header from an open binary file.

    On return, ``f`` is positioned at the start of the record section.
    """
    fixed = f.read(HEADER_FIXED_SIZE)
    if len(fixed) != HEADER_FIXED_SIZE:
        raise ValueError("File too short to contain a valid header")
    magic = fixed[:4]
    if magic != MAGIC:
        raise ValueError(f"Invalid magic: {magic!r} (expected {MAGIC!r})")
    version = struct.unpack_from("<I", fixed, 4)[0]
    if version != VERSION:
        raise ValueError(f"Unsupported version: {version}")
    n_vars = struct.unpack_from("<I", fixed, 8)[0]

    names: List[str] = []
    for _ in range(n_vars):
        raw_len = f.read(4)
        if len(raw_len) != 4:
            raise ValueError("Truncated header (reading name length)")
        name_len = struct.unpack("<I", raw_len)[0]
        raw_name = f.read(name_len)
        if len(raw_name) != name_len:
            raise ValueError("Truncated header (reading name payload)")
        names.append(raw_name.decode("utf-8"))

    return Header(
        version=version,
        n_vars=n_vars,
        names=names,
        data_offset=f.tell(),
    )


# ── Start-record resolution ──────────────────────────────────────────────────

def _resolve_start_record(
    data: np.ndarray,
    t_min: Optional[float],
    record_min: Optional[int],
) -> int:
    """Return the first record index to emit.

    Both bounds are optional and compose by ``max`` (the more restrictive
    start wins). ``t_min`` is matched against the monotonic non-decreasing
    time column via ``np.searchsorted`` (zero-copy strided view of the mmap,
    ~log N page touches). ``record_min`` is a direct index, clamped to
    ``[0, n_records]`` so callers needn't pre-clamp it.
    """
    n = data.shape[0]
    start = 0
    if t_min is not None and n > 0:
        start = max(start, int(np.searchsorted(data[:, 0], t_min, side="left")))
    if record_min is not None:
        start = max(start, max(0, min(record_min, n)))
    return start


# ── Record count ─────────────────────────────────────────────────────────────

def _read_header_and_count(path: str) -> tuple[Header, int]:
    """Parse the header and compute the number of records in one pass.

    Touches only the names section (proportional to ``n_vars``) plus one
    ``stat`` for the file size — no record bytes are read.
    """
    with open(path, "rb") as f:
        header = read_header(f)
    file_size = os.path.getsize(path)
    data_bytes = file_size - header.data_offset
    if data_bytes < 0 or data_bytes % header.record_size != 0:
        raise ValueError(
            f"Trailing partial record: {data_bytes} byte(s) past header is "
            f"not a multiple of record size {header.record_size}"
        )
    return header, data_bytes // header.record_size


def count_records(path: str) -> int:
    """Return the number of records in a Dynawo .bin file.

    Header-only cost: a read proportional to the names section (microseconds
    for typical models, a few ms for very wide ones) plus one ``stat`` call.
    No record data is read.
    """
    return _read_header_and_count(path)[1]


# ── Whole-file loader (used by the Streamlit app) ────────────────────────────

def load_bin(data: bytes):
    """Parse a .bin file given as bytes and return a pandas DataFrame.

    First column is ``time``; remaining columns are the variable names.
    ``pandas`` is imported lazily so the CLI can run without it installed.
    """
    import io

    import pandas as pd  # local import — optional dependency for CLI users

    with io.BytesIO(data) as f:
        header = read_header(f)
        remaining = len(data) - header.data_offset
        if remaining % header.record_size != 0:
            raise ValueError(
                f"Data section size {remaining} is not a multiple of "
                f"record size {header.record_size}"
            )
        arr = np.frombuffer(
            data, dtype="<f8",
            count=(remaining // DOUBLE_SIZE),
            offset=header.data_offset,
        ).reshape(-1, 1 + header.n_vars)
    return pd.DataFrame(arr, columns=["time"] + header.names)


# ── Streaming helpers ────────────────────────────────────────────────────────

def _records_per_chunk(record_size: int, chunk_bytes: int) -> int:
    """Return at least one record per chunk, capped so each chunk fits the budget."""
    return max(1, chunk_bytes // record_size)


def iter_chunks(
    f: BinaryIO,
    header: Header,
    chunk_bytes: int = DEFAULT_CHUNK_BYTES,
) -> Iterator[np.ndarray]:
    """Yield successive ``(N, 1 + n_vars)`` float64 chunks of records.

    ``f`` must be positioned at the start of the record section (the state
    ``read_header`` leaves it in). The last chunk may be smaller than the rest.
    A trailing partial record triggers ``ValueError``.
    """
    row_elems = 1 + header.n_vars
    per_chunk = _records_per_chunk(header.record_size, chunk_bytes)
    chunk_byte_count = per_chunk * header.record_size
    while True:
        buf = f.read(chunk_byte_count)
        if not buf:
            return
        n_bytes = len(buf)
        if n_bytes % header.record_size != 0:
            raise ValueError(
                f"Trailing partial record: {n_bytes % header.record_size} "
                "extra bytes at end of file"
            )
        n_records = n_bytes // header.record_size
        yield np.frombuffer(buf, dtype="<f8", count=n_records * row_elems).reshape(
            n_records, row_elems
        )


# ── Variable selection ───────────────────────────────────────────────────────

def select_indices(
    names: Sequence[str],
    patterns: Sequence[str] = (),
    *,
    regex: bool = False,
    contains: Sequence[str] = (),
) -> List[int]:
    """Return the indices of ``names`` matching at least one pattern.

    ``patterns`` are fnmatch-style globs (``*``, ``?``, ``[seq]``) by default
    or full regular expressions when ``regex=True``. ``contains`` are plain
    substrings — a name matches when any of them appears anywhere in it.
    The two filters are OR-combined; order of the result follows ``names``,
    not the order of patterns, and duplicates are removed.
    """
    if not patterns and not contains:
        return []
    if regex:
        pattern_matchers = [re.compile(p).search for p in patterns]
    else:
        pattern_matchers = [
            (lambda n, p=p: fnmatch.fnmatchcase(n, p)) for p in patterns
        ]
    contains_list = list(contains)
    return [
        i
        for i, n in enumerate(names)
        if any(m(n) for m in pattern_matchers) or any(s in n for s in contains_list)
    ]


# ── High-level operations ────────────────────────────────────────────────────

def write_variable_names(bin_path: str, txt_path: str) -> int:
    """Write every variable name to ``txt_path``, one per line. Returns the count."""
    with open(bin_path, "rb") as f:
        header = read_header(f)
    with open(txt_path, "w", encoding="utf-8") as out:
        for name in header.names:
            out.write(name)
            out.write("\n")
    return header.n_vars


def extract_to_csv(
    bin_path: str,
    csv_path: str,
    patterns: Sequence[str] = (),
    *,
    regex: bool = False,
    contains: Sequence[str] = (),
    chunk_bytes: int = DEFAULT_CHUNK_BYTES,
    fmt: str = "%.7g",
    t_min: Optional[float] = None,
    record_min: Optional[int] = None,
) -> tuple[int, int]:
    """Extract columns matching ``patterns`` or ``contains`` to ``csv_path``.

    The file is streamed chunk-by-chunk so files larger than RAM are handled
    without issue. Each row starts with ``time`` followed by the matching
    variables in their original order. ``t_min`` (inclusive) drops records
    earlier than the given time; ``record_min`` is a direct index for
    "last N points" use cases. When both are given the more restrictive
    start wins.

    Returns ``(n_matched_columns, n_records_written)``.
    """
    if not patterns and not contains:
        raise ValueError("extract_to_csv: provide at least one pattern or substring")
    with open(bin_path, "rb") as f:
        header = read_header(f)
    matched = select_indices(header.names, patterns, regex=regex, contains=contains)
    if not matched:
        raise SystemExit(
            "No variable names matched the given patterns.\n"
            "Use the 'names' subcommand to list what's available."
        )
    # +1 shift because column 0 in a record row is time.
    selected_cols = np.array([0] + [i + 1 for i in matched], dtype=np.intp)
    out_header = ["time"] + [header.names[i] for i in matched]

    file_size = os.path.getsize(bin_path)
    data_bytes = file_size - header.data_offset
    if data_bytes % header.record_size != 0:
        raise ValueError(
            f"Data section size {data_bytes} is not a multiple of "
            f"record size {header.record_size}"
        )
    n_records = data_bytes // header.record_size

    with open(csv_path, "wb") as out:
        out.write((",".join(out_header) + "\n").encode("utf-8"))
        if n_records == 0:
            return len(matched), 0
        data = np.memmap(
            bin_path, dtype="<f8", mode="r",
            offset=header.data_offset,
            shape=(n_records, 1 + header.n_vars),
        )
        start_rec = _resolve_start_record(data, t_min, record_min)
        per_chunk = max(1, chunk_bytes // header.record_size)
        for chunk_start in range(start_rec, n_records, per_chunk):
            chunk_stop = min(chunk_start + per_chunk, n_records)
            sub = data[chunk_start:chunk_stop, selected_cols]
            np.savetxt(out, sub, fmt=fmt, delimiter=",")
        return len(matched), n_records - start_rec


def extract_to_bin(
    bin_path: str,
    out_path: str,
    patterns: Sequence[str] = (),
    *,
    regex: bool = False,
    contains: Sequence[str] = (),
    chunk_records: int = 1_000_000,
    t_min: Optional[float] = None,
    record_min: Optional[int] = None,
) -> tuple[int, int]:
    """Extract columns matching ``patterns`` or ``contains`` to a new .bin file.

    The output is itself a valid Dynawo .bin file (same DYNB header layout)
    containing ``time`` plus the matched variables in their original order.
    Bypassing text formatting makes this much faster than ``extract_to_csv``
    on large files, and the result can be re-fed into any reader in this
    module — including this same extractor for further sub-selection.

    ``t_min`` (inclusive) drops records earlier than the given time. ``record_min``
    is an absolute record index — useful for "give me the last N points" without
    a time lookup (set ``record_min = n_records - N``). When both are given,
    the more restrictive start wins.

    Returns ``(n_matched_columns, n_records_written)``.
    """
    if not patterns and not contains:
        raise ValueError("extract_to_bin: provide at least one pattern or substring")
    with open(bin_path, "rb") as f:
        header = read_header(f)
    matched = select_indices(header.names, patterns, regex=regex, contains=contains)
    if not matched:
        raise SystemExit(
            "No variable names matched the given patterns.\n"
            "Use the 'names' subcommand to list what's available."
        )
    # +1 shift because column 0 in a record row is time; we always emit time.
    selected_cols = np.array([0] + [i + 1 for i in matched], dtype=np.intp)
    out_names = [header.names[i] for i in matched]

    file_size = os.path.getsize(bin_path)
    data_bytes = file_size - header.data_offset
    if data_bytes % header.record_size != 0:
        raise ValueError(
            f"Data section size {data_bytes} is not a multiple of "
            f"record size {header.record_size}"
        )
    n_records = data_bytes // header.record_size

    with open(out_path, "wb") as out:
        # New DYNB header for the subset.
        out.write(MAGIC)
        out.write(struct.pack("<I", VERSION))
        out.write(struct.pack("<I", len(out_names)))
        for name in out_names:
            encoded = name.encode("utf-8")
            out.write(struct.pack("<I", len(encoded)))
            out.write(encoded)

        if n_records == 0:
            return len(matched), 0

        # mmap the data section so column slicing is a zero-copy view and
        # the OS pages in only what we touch.
        data = np.memmap(
            bin_path, dtype="<f8", mode="r",
            offset=header.data_offset,
            shape=(n_records, 1 + header.n_vars),
        )
        start_rec = _resolve_start_record(data, t_min, record_min)
        n_emitted = n_records - start_rec
        for chunk_start in range(start_rec, n_records, chunk_records):
            chunk_stop = min(chunk_start + chunk_records, n_records)
            # ascontiguousarray forces the fancy-indexed slice into a single
            # C-contiguous block so tofile() emits one bulk write.
            np.ascontiguousarray(data[chunk_start:chunk_stop, selected_cols]).tofile(out)

    return len(matched), n_emitted


# ── Command-line interface ───────────────────────────────────────────────────

def _default_output(input_path: str, new_ext: str) -> str:
    base, _ = os.path.splitext(input_path)
    return base + new_ext


def _resolve_tail_args(
    input_path: str, args: argparse.Namespace
) -> tuple[Optional[float], Optional[int]]:
    """Translate CLI --tail / --tail-time / --t-min into (t_min, record_min).

    --tail N         : last N records → record_min = n_records - N
    --tail-time DT   : last DT seconds → t_min = last_record_time - DT
    --t-min T        : passed through; composes with the tail bound by
                       letting _resolve_start_record take the max.
    """
    t_min = args.t_min
    record_min: Optional[int] = None
    if args.tail is None and args.tail_time is None:
        return t_min, record_min

    # Need the file's record count (and maybe its last time) to translate.
    header, n_records = _read_header_and_count(input_path)

    if args.tail is not None:
        record_min = max(0, n_records - args.tail)
    else:  # args.tail_time is not None
        if n_records > 0:
            data = np.memmap(
                input_path, dtype="<f8", mode="r",
                offset=header.data_offset,
                shape=(n_records, 1 + header.n_vars),
            )
            t_last = float(data[-1, 0])
            tail_t_min = t_last - args.tail_time
            t_min = tail_t_min if t_min is None else max(t_min, tail_t_min)
    return t_min, record_min


def _cmd_names(args: argparse.Namespace) -> int:
    out = args.output or _default_output(args.input, ".names.txt")
    count = write_variable_names(args.input, out)
    print(f"Wrote {count} variable names to {out}", file=sys.stderr)
    return 0


def _cmd_count(args: argparse.Namespace) -> int:
    n = count_records(args.input)
    print(n)
    return 0


def _cmd_extract(args: argparse.Namespace) -> int:
    if not args.pattern and not args.contains:
        raise SystemExit("extract: provide at least one --pattern or --contains")
    out = args.output or _default_output(args.input, ".extract.csv")
    t_min, record_min = _resolve_tail_args(args.input, args)
    n_cols, n_rows = extract_to_csv(
        args.input,
        out,
        args.pattern,
        regex=args.regex,
        contains=args.contains,
        chunk_bytes=args.chunk_bytes,
        fmt=args.fmt,
        t_min=t_min,
        record_min=record_min,
    )
    print(
        f"Wrote {n_cols} variable(s) × {n_rows} record(s) to {out}",
        file=sys.stderr,
    )
    return 0


def _cmd_extract_bin(args: argparse.Namespace) -> int:
    if not args.pattern and not args.contains:
        raise SystemExit("extract-bin: provide at least one --pattern or --contains")
    out = args.output or _default_output(args.input, ".extract.bin")
    t_min, record_min = _resolve_tail_args(args.input, args)
    n_cols, n_rows = extract_to_bin(
        args.input,
        out,
        args.pattern,
        regex=args.regex,
        contains=args.contains,
        chunk_records=args.chunk_records,
        t_min=t_min,
        record_min=record_min,
    )
    print(
        f"Wrote {n_cols} variable(s) × {n_rows} record(s) to {out}",
        file=sys.stderr,
    )
    return 0


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Inspect and extract columns from Dynawo .bin curve files.",
    )
    sub = p.add_subparsers(dest="command", required=True)

    names = sub.add_parser(
        "names",
        help="Dump every variable name to a text file (one per line).",
    )
    names.add_argument("input", help="Path to the .bin file")
    names.add_argument(
        "-o", "--output",
        help="Output .txt file (default: <input>.names.txt)",
    )
    names.set_defaults(func=_cmd_names)

    count = sub.add_parser(
        "count",
        help="Print the number of records in a .bin file (header-only cost). "
             "Output goes to stdout for shell scripting.",
    )
    count.add_argument("input", help="Path to the .bin file")
    count.set_defaults(func=_cmd_count)

    extract = sub.add_parser(
        "extract",
        help="Stream-extract columns matching one or more patterns to a CSV.",
    )
    extract.add_argument("input", help="Path to the .bin file")
    extract.add_argument(
        "-p", "--pattern",
        action="append",
        default=[],
        help="fnmatch-style pattern (can be repeated; OR-combined). "
             "Examples: '*voltage*', 'bus?_V'. Use --regex for regular expressions.",
    )
    extract.add_argument(
        "-c", "--contains",
        action="append",
        default=[],
        help="Plain substring filter (can be repeated; OR-combined with --pattern). "
             "Selects every variable whose name contains the given string. "
             "Example: -c generator.",
    )
    extract.add_argument(
        "--regex",
        action="store_true",
        help="Treat --pattern values as Python regular expressions (re.search). "
             "Has no effect on --contains.",
    )
    extract.add_argument(
        "-o", "--output",
        help="Output .csv file (default: <input>.extract.csv)",
    )
    extract.add_argument(
        "--chunk-bytes",
        type=int,
        default=DEFAULT_CHUNK_BYTES,
        help=f"Streaming chunk budget in bytes (default: {DEFAULT_CHUNK_BYTES}).",
    )
    extract.add_argument(
        "--fmt",
        default="%.7g",
        help="printf-style format for numeric values (default: %%.7g).",
    )
    extract.add_argument(
        "--t-min",
        type=float,
        default=None,
        help="Keep only records with time >= this value (inclusive).",
    )
    extract_tail = extract.add_mutually_exclusive_group()
    extract_tail.add_argument(
        "--tail",
        type=int,
        default=None,
        metavar="N",
        help="Keep only the last N records of the trajectory.",
    )
    extract_tail.add_argument(
        "--tail-time",
        type=float,
        default=None,
        metavar="DT",
        help="Keep only records within the last DT seconds of the trajectory "
             "(uses the last record's time as the reference end).",
    )
    extract.set_defaults(func=_cmd_extract)

    extract_bin = sub.add_parser(
        "extract-bin",
        help="Like 'extract' but writes a Dynawo .bin file instead of CSV. "
             "Much faster on large inputs because no text formatting is done; "
             "the output is itself a valid .bin file.",
    )
    extract_bin.add_argument("input", help="Path to the .bin file")
    extract_bin.add_argument(
        "-p", "--pattern",
        action="append",
        default=[],
        help="fnmatch-style pattern (can be repeated; OR-combined). "
             "Examples: '*voltage*', 'bus?_V'. Use --regex for regular expressions.",
    )
    extract_bin.add_argument(
        "-c", "--contains",
        action="append",
        default=[],
        help="Plain substring filter (can be repeated; OR-combined with --pattern).",
    )
    extract_bin.add_argument(
        "--regex",
        action="store_true",
        help="Treat --pattern values as Python regular expressions (re.search).",
    )
    extract_bin.add_argument(
        "-o", "--output",
        help="Output .bin file (default: <input>.extract.bin)",
    )
    extract_bin.add_argument(
        "--chunk-records",
        type=int,
        default=1_000_000,
        help="Records processed per mmap slice (default: 1,000,000). "
             "Bounds the transient memory of the column-slice copy.",
    )
    extract_bin.add_argument(
        "--t-min",
        type=float,
        default=None,
        help="Keep only records with time >= this value (inclusive).",
    )
    extract_bin_tail = extract_bin.add_mutually_exclusive_group()
    extract_bin_tail.add_argument(
        "--tail",
        type=int,
        default=None,
        metavar="N",
        help="Keep only the last N records of the trajectory.",
    )
    extract_bin_tail.add_argument(
        "--tail-time",
        type=float,
        default=None,
        metavar="DT",
        help="Keep only records within the last DT seconds of the trajectory "
             "(uses the last record's time as the reference end).",
    )
    extract_bin.set_defaults(func=_cmd_extract_bin)

    return p


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = _build_parser().parse_args(list(argv) if argv is not None else None)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
