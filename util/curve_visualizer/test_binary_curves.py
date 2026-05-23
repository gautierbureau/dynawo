"""Tests for util/curve_visualizer/binary_curves.py — focused on the
extract_to_bin path and its equivalence with the CSV extractor.

Run with::

    pytest util/curve_visualizer/test_binary_curves.py
"""
from __future__ import annotations

import os
import struct
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import binary_curves as bc  # noqa: E402


# ── Helpers ──────────────────────────────────────────────────────────────────

def _write_bin(path: Path, names, times, values):
    """Build a synthetic DYNB file. ``values`` is shape (n_records, n_vars)."""
    values = np.asarray(values, dtype="<f8")
    times = np.asarray(times, dtype="<f8")
    assert values.shape == (len(times), len(names))
    with open(path, "wb") as f:
        f.write(bc.MAGIC)
        f.write(struct.pack("<I", bc.VERSION))
        f.write(struct.pack("<I", len(names)))
        for n in names:
            b = n.encode("utf-8")
            f.write(struct.pack("<I", len(b)))
            f.write(b)
        # Row-major: time then values, per record.
        row = np.empty(1 + len(names), dtype="<f8")
        for i, t in enumerate(times):
            row[0] = t
            row[1:] = values[i]
            row.tofile(f)


def _read_bin(path: Path):
    """Parse a DYNB file and return (names, times, values)."""
    with open(path, "rb") as f:
        header = bc.read_header(f)
        remaining = os.path.getsize(path) - header.data_offset
        arr = np.frombuffer(
            f.read(remaining), dtype="<f8"
        ).reshape(-1, 1 + header.n_vars)
    return header.names, arr[:, 0].copy(), arr[:, 1:].copy()


# ── Tests ────────────────────────────────────────────────────────────────────

def test_extract_bin_round_trip_subset(tmp_path):
    """A subset extract is itself a valid DYNB file and preserves the
    selected columns bit-for-bit."""
    src = tmp_path / "src.bin"
    names = ["alpha", "beta", "gamma", "delta"]
    times = np.linspace(0.0, 1.0, 5)
    values = np.arange(20, dtype="<f8").reshape(5, 4)
    _write_bin(src, names, times, values)

    out = tmp_path / "out.bin"
    n_cols, n_rows = bc.extract_to_bin(
        str(src), str(out), patterns=["beta", "delta"]
    )
    assert (n_cols, n_rows) == (2, 5)

    out_names, out_times, out_vals = _read_bin(out)
    assert out_names == ["beta", "delta"]
    np.testing.assert_array_equal(out_times, times)
    # Columns 1 (beta) and 3 (delta) of the source.
    np.testing.assert_array_equal(out_vals, values[:, [1, 3]])


def test_extract_bin_preserves_source_column_order(tmp_path):
    """Even when patterns are given in reverse order, output columns follow
    the source order (matches extract_to_csv behaviour)."""
    src = tmp_path / "src.bin"
    names = ["a", "b", "c"]
    times = np.array([0.0, 0.1])
    values = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    _write_bin(src, names, times, values)

    out = tmp_path / "out.bin"
    bc.extract_to_bin(str(src), str(out), patterns=["c", "a"])
    out_names, _, out_vals = _read_bin(out)
    assert out_names == ["a", "c"]
    np.testing.assert_array_equal(out_vals, values[:, [0, 2]])


def test_extract_bin_matches_extract_csv(tmp_path):
    """Extract to .bin then to CSV from that .bin equals extract original to
    CSV directly. This pins the semantics: the binary extract is a lossless
    pre-stage."""
    src = tmp_path / "src.bin"
    names = ["x", "y", "z"]
    times = np.linspace(0, 2, 10)
    values = np.random.default_rng(0).standard_normal((10, 3))
    _write_bin(src, names, times, values)

    direct_csv = tmp_path / "direct.csv"
    bc.extract_to_csv(str(src), str(direct_csv), patterns=["x", "z"])

    intermediate_bin = tmp_path / "mid.bin"
    bc.extract_to_bin(str(src), str(intermediate_bin), patterns=["x", "z"])
    via_bin_csv = tmp_path / "via_bin.csv"
    # Re-extract from the subset; pattern '*' would match both columns.
    bc.extract_to_csv(str(intermediate_bin), str(via_bin_csv), patterns=["*"])

    assert direct_csv.read_bytes() == via_bin_csv.read_bytes()


def test_extract_bin_empty_data_section(tmp_path):
    """Source with header only (zero records) writes a header-only output."""
    src = tmp_path / "src.bin"
    _write_bin(src, ["a", "b"], times=np.array([]), values=np.empty((0, 2)))

    out = tmp_path / "out.bin"
    n_cols, n_rows = bc.extract_to_bin(str(src), str(out), patterns=["a"])
    assert (n_cols, n_rows) == (1, 0)
    out_names, out_times, out_vals = _read_bin(out)
    assert out_names == ["a"]
    assert out_times.size == 0
    assert out_vals.shape == (0, 1)


def test_extract_bin_chunk_boundary(tmp_path):
    """Records spanning multiple chunk_records slices are emitted contiguously
    with no duplication or loss."""
    src = tmp_path / "src.bin"
    names = ["only"]
    n = 1000
    times = np.arange(n, dtype="<f8")
    values = (times * 10.0).reshape(n, 1)
    _write_bin(src, names, times, values)

    out = tmp_path / "out.bin"
    # chunk_records=137 forces ~8 chunks with an awkward boundary.
    bc.extract_to_bin(
        str(src), str(out), patterns=["only"], chunk_records=137
    )
    out_names, out_times, out_vals = _read_bin(out)
    assert out_names == ["only"]
    np.testing.assert_array_equal(out_times, times)
    np.testing.assert_array_equal(out_vals.ravel(), values.ravel())


def test_extract_bin_no_match(tmp_path):
    src = tmp_path / "src.bin"
    _write_bin(src, ["foo"], np.array([0.0]), np.array([[1.0]]))
    out = tmp_path / "out.bin"
    with pytest.raises(SystemExit):
        bc.extract_to_bin(str(src), str(out), patterns=["does_not_match"])


def test_extract_bin_requires_a_filter(tmp_path):
    src = tmp_path / "src.bin"
    _write_bin(src, ["a"], np.array([0.0]), np.array([[1.0]]))
    out = tmp_path / "out.bin"
    with pytest.raises(ValueError):
        bc.extract_to_bin(str(src), str(out))


def test_extract_bin_contains_filter(tmp_path):
    src = tmp_path / "src.bin"
    names = ["bus_a_V", "bus_a_Theta", "load_P"]
    times = np.array([0.0, 1.0])
    values = np.array([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    _write_bin(src, names, times, values)

    out = tmp_path / "out.bin"
    n_cols, _ = bc.extract_to_bin(str(src), str(out), contains=["bus_a"])
    assert n_cols == 2
    out_names, _, out_vals = _read_bin(out)
    assert out_names == ["bus_a_V", "bus_a_Theta"]
    np.testing.assert_array_equal(out_vals, values[:, [0, 1]])


def test_extract_bin_full_precision_preserved(tmp_path):
    """Values that the CSV exporter would round are preserved bit-for-bit
    by the binary extract — that's the whole point of staying binary."""
    src = tmp_path / "src.bin"
    names = ["x"]
    # A value that does not survive %.7g rounding.
    raw = 1234.123456789012
    times = np.array([0.123456789012345])
    values = np.array([[raw]])
    _write_bin(src, names, times, values)

    out = tmp_path / "out.bin"
    bc.extract_to_bin(str(src), str(out), patterns=["x"])
    _, out_times, out_vals = _read_bin(out)
    # Exact equality, not allclose: binary path is lossless.
    assert out_times[0] == times[0]
    assert out_vals[0, 0] == raw


# ── Time-window tests ────────────────────────────────────────────────────────

def _windowed_fixture(tmp_path):
    """Source with 10 evenly-spaced records (times 0.0, 0.1, …, 0.9)."""
    src = tmp_path / "src.bin"
    names = ["a", "b"]
    times = np.round(np.arange(10) * 0.1, 8)
    values = np.column_stack([np.arange(10) * 1.0, np.arange(10) * 10.0])
    _write_bin(src, names, times, values)
    return src, names, times, values


def test_extract_bin_t_max_only(tmp_path):
    src, _, times, values = _windowed_fixture(tmp_path)
    out = tmp_path / "out.bin"
    n_cols, n_rows = bc.extract_to_bin(
        str(src), str(out), patterns=["a", "b"], t_max=0.4
    )
    # Inclusive: times 0.0, 0.1, 0.2, 0.3, 0.4 -> 5 records.
    assert (n_cols, n_rows) == (2, 5)
    _, out_times, out_vals = _read_bin(out)
    np.testing.assert_array_equal(out_times, times[:5])
    np.testing.assert_array_equal(out_vals, values[:5])


def test_extract_bin_t_min_only(tmp_path):
    src, _, times, values = _windowed_fixture(tmp_path)
    out = tmp_path / "out.bin"
    n_cols, n_rows = bc.extract_to_bin(
        str(src), str(out), patterns=["a"], t_min=0.7
    )
    # Inclusive: times 0.7, 0.8, 0.9 -> 3 records.
    assert (n_cols, n_rows) == (1, 3)
    _, out_times, out_vals = _read_bin(out)
    np.testing.assert_array_equal(out_times, times[7:])
    np.testing.assert_array_equal(out_vals.ravel(), values[7:, 0])


def test_extract_bin_t_window(tmp_path):
    src, _, times, values = _windowed_fixture(tmp_path)
    out = tmp_path / "out.bin"
    n_cols, n_rows = bc.extract_to_bin(
        str(src), str(out), patterns=["a"], t_min=0.3, t_max=0.6
    )
    # Inclusive on both sides: 0.3, 0.4, 0.5, 0.6 -> 4 records.
    assert (n_cols, n_rows) == (1, 4)
    _, out_times, _ = _read_bin(out)
    np.testing.assert_array_equal(out_times, times[3:7])


def test_extract_bin_window_outside_data(tmp_path):
    """A window entirely outside the recorded range writes header only."""
    src, _, _, _ = _windowed_fixture(tmp_path)
    out = tmp_path / "out.bin"
    n_cols, n_rows = bc.extract_to_bin(
        str(src), str(out), patterns=["a"], t_min=10.0
    )
    assert (n_cols, n_rows) == (1, 0)
    out_names, out_times, _ = _read_bin(out)
    assert out_names == ["a"]
    assert out_times.size == 0


def test_extract_bin_inverted_window(tmp_path):
    """t_min > t_max yields an empty extract, not an error."""
    src, _, _, _ = _windowed_fixture(tmp_path)
    out = tmp_path / "out.bin"
    n_cols, n_rows = bc.extract_to_bin(
        str(src), str(out), patterns=["a"], t_min=0.8, t_max=0.2
    )
    assert (n_cols, n_rows) == (1, 0)


def test_extract_csv_t_window(tmp_path):
    """CSV path honours the same window semantics; result matches a manual
    slice + savetxt of the source."""
    src, _, times, values = _windowed_fixture(tmp_path)
    out_csv = tmp_path / "out.csv"
    bc.extract_to_csv(
        str(src), str(out_csv), patterns=["b"], t_min=0.2, t_max=0.5
    )
    got = out_csv.read_text().splitlines()
    assert got[0] == "time,b"
    # Times 0.2, 0.3, 0.4, 0.5 -> 4 rows.
    assert len(got) == 5
    # Each row is "time,b" with the same %.7g formatting.
    for row, t, v in zip(got[1:], times[2:6], values[2:6, 1]):
        expected_t = ("%.7g" % t)
        expected_v = ("%.7g" % v)
        assert row == f"{expected_t},{expected_v}"


def test_extract_bin_cli_with_time_window(tmp_path):
    src, _, times, _ = _windowed_fixture(tmp_path)
    out = tmp_path / "out.bin"
    rc = bc.main([
        "extract-bin", str(src), "-p", "a",
        "--t-min", "0.4", "--t-max", "0.7",
        "-o", str(out),
    ])
    assert rc == 0
    _, out_times, _ = _read_bin(out)
    np.testing.assert_array_equal(out_times, times[4:8])


def test_extract_bin_cli(tmp_path, monkeypatch, capsys):
    """End-to-end check of the extract-bin CLI subcommand."""
    src = tmp_path / "src.bin"
    _write_bin(
        src, ["a", "b"], np.array([0.0, 1.0]),
        np.array([[1.0, 2.0], [3.0, 4.0]]),
    )
    out = tmp_path / "subset.bin"
    rc = bc.main([
        "extract-bin", str(src), "-p", "b", "-o", str(out),
    ])
    assert rc == 0
    out_names, _, out_vals = _read_bin(out)
    assert out_names == ["b"]
    np.testing.assert_array_equal(out_vals.ravel(), [2.0, 4.0])
