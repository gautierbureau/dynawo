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

The module exposes three things:

* ``load_bin(data)``        — parse a whole file given as ``bytes`` and return
                              a pandas DataFrame. Used by the Streamlit app.
* ``read_header(f)``        — parse just the header of an open file. Cheap
                              even on 5 GB files because the record section
                              is skipped.
* a command-line interface with two sub-commands:

      python binary_curves.py names   <input.bin> [-o names.txt]
      python binary_curves.py extract <input.bin> -p PATTERN ... [-o out.csv]

The ``extract`` command streams the record section in fixed-size chunks so it
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
DEFAULT_CHUNK_BYTES = 64 * 1024 * 1024  # 64 MiB working set per chunk


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
    patterns: Sequence[str],
    *,
    regex: bool = False,
) -> List[int]:
    """Return the indices of ``names`` that match at least one pattern.

    Patterns are fnmatch-style globs (``*``, ``?``, ``[seq]``) by default, or
    full regular expressions when ``regex=True``. Order of results follows
    ``names`` order, not pattern order; duplicates are removed.
    """
    if not patterns:
        return []
    if regex:
        compiled = [re.compile(p) for p in patterns]
        return [i for i, n in enumerate(names) if any(c.search(n) for c in compiled)]
    return [
        i
        for i, n in enumerate(names)
        if any(fnmatch.fnmatchcase(n, p) for p in patterns)
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
    patterns: Sequence[str],
    *,
    regex: bool = False,
    chunk_bytes: int = DEFAULT_CHUNK_BYTES,
    fmt: str = "%.7g",
) -> tuple[int, int]:
    """Extract columns matching ``patterns`` to ``csv_path``.

    The file is streamed chunk-by-chunk so files larger than RAM are handled
    without issue. Each row starts with ``time`` followed by the matching
    variables in their original order.

    Returns ``(n_matched_columns, n_records_written)``.
    """
    with open(bin_path, "rb") as f:
        header = read_header(f)
        matched = select_indices(header.names, patterns, regex=regex)
        if not matched:
            raise SystemExit(
                "No variable names matched the given patterns.\n"
                "Use the 'names' subcommand to list what's available."
            )
        # +1 shift because column 0 in a record row is time.
        selected_cols = np.array([0] + [i + 1 for i in matched], dtype=np.intp)
        out_header = ["time"] + [header.names[i] for i in matched]

        n_written = 0
        # ``np.savetxt`` in append mode: open once, write header, then write
        # chunks via savetxt so formatting stays vectorised.
        with open(csv_path, "wb") as out:
            out.write((",".join(out_header) + "\n").encode("utf-8"))
            for chunk in iter_chunks(f, header, chunk_bytes=chunk_bytes):
                sub = chunk[:, selected_cols]
                np.savetxt(out, sub, fmt=fmt, delimiter=",")
                n_written += sub.shape[0]

    return len(matched), n_written


# ── Command-line interface ───────────────────────────────────────────────────

def _default_output(input_path: str, new_ext: str) -> str:
    base, _ = os.path.splitext(input_path)
    return base + new_ext


def _cmd_names(args: argparse.Namespace) -> int:
    out = args.output or _default_output(args.input, ".names.txt")
    count = write_variable_names(args.input, out)
    print(f"Wrote {count} variable names to {out}", file=sys.stderr)
    return 0


def _cmd_extract(args: argparse.Namespace) -> int:
    out = args.output or _default_output(args.input, ".extract.csv")
    n_cols, n_rows = extract_to_csv(
        args.input,
        out,
        args.pattern,
        regex=args.regex,
        chunk_bytes=args.chunk_bytes,
        fmt=args.fmt,
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

    extract = sub.add_parser(
        "extract",
        help="Stream-extract columns matching one or more patterns to a CSV.",
    )
    extract.add_argument("input", help="Path to the .bin file")
    extract.add_argument(
        "-p", "--pattern",
        action="append",
        default=[],
        required=True,
        help="fnmatch-style pattern (can be repeated; OR-combined). "
             "Examples: '*voltage*', 'bus?_V'. Use --regex for regular expressions.",
    )
    extract.add_argument(
        "--regex",
        action="store_true",
        help="Treat --pattern values as Python regular expressions (re.search).",
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
    extract.set_defaults(func=_cmd_extract)

    return p


def main(argv: Optional[Iterable[str]] = None) -> int:
    args = _build_parser().parse_args(list(argv) if argv is not None else None)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
