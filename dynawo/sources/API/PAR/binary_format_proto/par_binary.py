#!/usr/bin/env python3
"""Reference encoder, decoder and inspector for the Dynawo binary PAR format.

See ``FORMAT.md`` for the format spec.

Subcommands::

    par_binary.py xml2bin  input.par   output.par.bin
    par_binary.py bin2xml  input.par.bin output.par
    par_binary.py inspect  input.par.bin   [--full]

The intent is to give Dynawo users a "swiss army knife" so that they can
keep working with XML even if the production toolchain switches to the
binary format: dump it, eyeball it, convert it back.
"""

from __future__ import annotations

import argparse
import io
import struct
import sys
from typing import BinaryIO, Iterable
from xml.sax.saxutils import escape

# --- format constants ------------------------------------------------------

MAGIC = b"DYNP"
VERSION_MAJOR = 1
VERSION_MINOR = 0
HEADER_SIZE = 32

PTYPE_BOOL = 0
PTYPE_INT = 1
PTYPE_DOUBLE = 2
PTYPE_STRING = 3

PTYPE_NAMES = {
    PTYPE_BOOL: "BOOL",
    PTYPE_INT: "INT",
    PTYPE_DOUBLE: "DOUBLE",
    PTYPE_STRING: "STRING",
}
PTYPE_FROM_NAME = {v: k for k, v in PTYPE_NAMES.items()}

ORIGDATA_IIDM = 0
ORIGDATA_PAR = 1
ORIGDATA_NAMES = {ORIGDATA_IIDM: "IIDM", ORIGDATA_PAR: "PAR"}
ORIGDATA_FROM_NAME = {v: k for k, v in ORIGDATA_NAMES.items()}

TAG_PAR = 0x10
TAG_PARTABLE = 0x11
TAG_REFERENCE = 0x12
TAG_MACROPARSET = 0x13

NAMESPACE = "http://www.rte-france.com/dynawo"

# --- low-level helpers -----------------------------------------------------


def _write_varint(buf: bytearray, value: int) -> None:
    if value < 0:
        raise ValueError("varint must be non-negative")
    while value >= 0x80:
        buf.append((value & 0x7F) | 0x80)
        value >>= 7
    buf.append(value)


def _read_varint(stream: BinaryIO) -> int:
    shift = 0
    result = 0
    while True:
        byte = stream.read(1)
        if not byte:
            raise EOFError("truncated varint")
        b = byte[0]
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result
        shift += 7
        if shift > 63:
            raise ValueError("varint too long")


def _read_exact(stream: BinaryIO, n: int) -> bytes:
    data = stream.read(n)
    if len(data) != n:
        raise EOFError(f"expected {n} bytes, got {len(data)}")
    return data


# --- string-table builder --------------------------------------------------


class StringTable:
    """Insertion-ordered, deduplicated string table.

    Index 0 is reserved for the empty string (used for missing optional
    string fields), per the spec.
    """

    def __init__(self) -> None:
        self._index: dict[str, int] = {"": 0}
        self._items: list[str] = [""]

    def intern(self, s: str | None) -> int:
        if s is None:
            return 0
        idx = self._index.get(s)
        if idx is not None:
            return idx
        idx = len(self._items)
        self._items.append(s)
        self._index[s] = idx
        return idx

    @property
    def items(self) -> list[str]:
        return self._items

    def encode(self) -> bytes:
        buf = bytearray()
        for s in self._items:
            data = s.encode("utf-8")
            _write_varint(buf, len(data))
            buf.extend(data)
        return bytes(buf)


# --- in-memory data model (mirrors the XML schema) -------------------------


class Par:
    __slots__ = ("ptype", "name", "value")

    def __init__(self, ptype: int, name: str, value):
        self.ptype = ptype
        self.name = name
        self.value = value


class Cell:
    __slots__ = ("row", "column", "value")

    def __init__(self, row: int, column: int, value):
        self.row = row
        self.column = column
        self.value = value


class ParTable:
    __slots__ = ("ptype", "name", "cells")

    def __init__(self, ptype: int, name: str, cells: list[Cell]):
        self.ptype = ptype
        self.name = name
        self.cells = cells


class Reference:
    __slots__ = ("type", "name", "orig_data", "orig_name",
                 "component_id", "par_id", "par_file")

    def __init__(self, type_: str, name: str, orig_data: int, orig_name: str,
                 component_id: str | None = None,
                 par_id: str | None = None,
                 par_file: str | None = None):
        self.type = type_
        self.name = name
        self.orig_data = orig_data
        self.orig_name = orig_name
        self.component_id = component_id
        self.par_id = par_id
        self.par_file = par_file


class MacroParSetRef:
    __slots__ = ("id",)

    def __init__(self, id_: str):
        self.id = id_


class Set:
    __slots__ = ("id", "items")

    def __init__(self, id_: str, items: list):
        self.id = id_
        self.items = items


class MacroParameterSet:
    __slots__ = ("id", "items")

    def __init__(self, id_: str, items: list):
        self.id = id_
        self.items = items


class Document:
    __slots__ = ("macros", "sets")

    def __init__(self) -> None:
        self.macros: list[MacroParameterSet] = []
        self.sets: list[Set] = []


# --- XML import ------------------------------------------------------------

# Pure xml.etree usage; this is the reference implementation, not the hot
# path. The Dynawo C++ reader will be the production fast path.
import xml.etree.ElementTree as ET


def _strip_ns(tag: str) -> str:
    return tag.split("}", 1)[1] if tag.startswith("{") else tag


def _parse_value(ptype: int, raw: str):
    if ptype == PTYPE_BOOL:
        return raw.strip().lower() == "true"
    if ptype == PTYPE_INT:
        return int(raw)
    if ptype == PTYPE_DOUBLE:
        return float(raw)
    return raw  # STRING


def _xml_to_par(elem) -> Par:
    ptype = PTYPE_FROM_NAME[elem.attrib["type"]]
    return Par(ptype, elem.attrib["name"], _parse_value(ptype, elem.attrib["value"]))


def _xml_to_par_table(elem) -> ParTable:
    ptype = PTYPE_FROM_NAME[elem.attrib["type"]]
    cells: list[Cell] = []
    for child in elem:
        if _strip_ns(child.tag) != "par":
            continue
        cells.append(Cell(
            int(child.attrib["row"]),
            int(child.attrib["column"]),
            _parse_value(ptype, child.attrib["value"]),
        ))
    return ParTable(ptype, elem.attrib["name"], cells)


def _xml_to_reference(elem) -> Reference:
    return Reference(
        type_=elem.attrib["type"],
        name=elem.attrib["name"],
        orig_data=ORIGDATA_FROM_NAME[elem.attrib["origData"]],
        orig_name=elem.attrib["origName"],
        component_id=elem.attrib.get("componentId"),
        par_id=elem.attrib.get("parId"),
        par_file=elem.attrib.get("parFile"),
    )


def _xml_to_items(parent, allow_table: bool, allow_macropar: bool) -> list:
    items: list = []
    for child in parent:
        tag = _strip_ns(child.tag)
        if tag == "par":
            items.append(_xml_to_par(child))
        elif tag == "parTable":
            if not allow_table:
                raise ValueError("parTable not allowed inside macroParameterSet")
            items.append(_xml_to_par_table(child))
        elif tag == "reference":
            items.append(_xml_to_reference(child))
        elif tag == "macroParSet":
            if not allow_macropar:
                raise ValueError("macroParSet not allowed inside macroParameterSet")
            items.append(MacroParSetRef(child.attrib["id"]))
        # unknown tags are silently skipped (forward-compat with future XSDs)
    return items


def parse_xml(path: str) -> Document:
    tree = ET.parse(path)
    root = tree.getroot()
    if _strip_ns(root.tag) != "parametersSet":
        raise ValueError(f"unexpected root element {root.tag!r}")
    doc = Document()
    for child in root:
        tag = _strip_ns(child.tag)
        if tag == "set":
            doc.sets.append(Set(
                id_=child.attrib["id"],
                items=_xml_to_items(child, allow_table=True, allow_macropar=True),
            ))
        elif tag == "macroParameterSet":
            doc.macros.append(MacroParameterSet(
                id_=child.attrib["id"],
                items=_xml_to_items(child, allow_table=False, allow_macropar=False),
            ))
    return doc


# --- binary encoder --------------------------------------------------------


def _encode_value(buf: bytearray, ptype: int, value, st: StringTable) -> None:
    if ptype == PTYPE_BOOL:
        buf.append(1 if value else 0)
    elif ptype == PTYPE_INT:
        buf.extend(struct.pack("<i", int(value)))
    elif ptype == PTYPE_DOUBLE:
        buf.extend(struct.pack("<d", float(value)))
    elif ptype == PTYPE_STRING:
        _write_varint(buf, st.intern(str(value)))
    else:
        raise ValueError(f"bad ptype {ptype}")


def _encode_item(buf: bytearray, item, st: StringTable) -> None:
    if isinstance(item, Par):
        buf.append(TAG_PAR)
        buf.append(item.ptype)
        _write_varint(buf, st.intern(item.name))
        _encode_value(buf, item.ptype, item.value, st)
    elif isinstance(item, ParTable):
        buf.append(TAG_PARTABLE)
        buf.append(item.ptype)
        _write_varint(buf, st.intern(item.name))
        _write_varint(buf, len(item.cells))
        for cell in item.cells:
            buf.extend(struct.pack("<ii", cell.row, cell.column))
            _encode_value(buf, item.ptype, cell.value, st)
    elif isinstance(item, Reference):
        buf.append(TAG_REFERENCE)
        _write_varint(buf, st.intern(item.type))
        _write_varint(buf, st.intern(item.name))
        buf.append(item.orig_data)
        _write_varint(buf, st.intern(item.orig_name))
        flags = 0
        if item.component_id is not None:
            flags |= 0x01
        if item.par_id is not None:
            flags |= 0x02
        if item.par_file is not None:
            flags |= 0x04
        buf.append(flags)
        if flags & 0x01:
            _write_varint(buf, st.intern(item.component_id))
        if flags & 0x02:
            _write_varint(buf, st.intern(item.par_id))
        if flags & 0x04:
            _write_varint(buf, st.intern(item.par_file))
    elif isinstance(item, MacroParSetRef):
        buf.append(TAG_MACROPARSET)
        _write_varint(buf, st.intern(item.id))
    else:
        raise TypeError(f"unknown item {item!r}")


def write_binary(doc: Document, path: str) -> None:
    st = StringTable()
    body = bytearray()

    # Pre-intern set ids in document order so the body's first stref values
    # are tiny — a marginal but free optimization.
    for m in doc.macros:
        st.intern(m.id)
    for s in doc.sets:
        st.intern(s.id)

    for m in doc.macros:
        _write_varint(body, st.intern(m.id))
        _write_varint(body, len(m.items))
        for it in m.items:
            _encode_item(body, it, st)
    for s in doc.sets:
        _write_varint(body, st.intern(s.id))
        _write_varint(body, len(s.items))
        for it in s.items:
            _encode_item(body, it, st)

    string_table = st.encode()
    body_offset = HEADER_SIZE + len(string_table)

    header = bytearray()
    header.extend(MAGIC)
    header.extend(struct.pack("<HH", VERSION_MAJOR, VERSION_MINOR))
    header.extend(struct.pack("<I", 0))                     # flags
    header.extend(struct.pack("<I", len(st.items)))          # num_strings
    header.extend(struct.pack("<I", len(doc.macros)))        # num_macroparset
    header.extend(struct.pack("<I", len(doc.sets)))          # num_sets
    header.extend(struct.pack("<Q", body_offset))            # body_offset
    assert len(header) == HEADER_SIZE

    with open(path, "wb") as f:
        f.write(header)
        f.write(string_table)
        f.write(body)


# --- binary decoder --------------------------------------------------------


class _Header:
    __slots__ = ("version_major", "version_minor", "flags", "num_strings",
                 "num_macroparset", "num_sets", "body_offset")


def _read_header(stream: BinaryIO) -> _Header:
    raw = _read_exact(stream, HEADER_SIZE)
    if raw[:4] != MAGIC:
        raise ValueError("not a Dynawo binary PAR file (bad magic)")
    h = _Header()
    (h.version_major, h.version_minor) = struct.unpack("<HH", raw[4:8])
    if h.version_major != VERSION_MAJOR:
        raise ValueError(
            f"unsupported major version {h.version_major} (expected {VERSION_MAJOR})"
        )
    (h.flags,) = struct.unpack("<I", raw[8:12])
    (h.num_strings,) = struct.unpack("<I", raw[12:16])
    (h.num_macroparset,) = struct.unpack("<I", raw[16:20])
    (h.num_sets,) = struct.unpack("<I", raw[20:24])
    (h.body_offset,) = struct.unpack("<Q", raw[24:32])
    return h


def _read_string_table(stream: BinaryIO, num_strings: int) -> list[str]:
    out: list[str] = []
    for _ in range(num_strings):
        n = _read_varint(stream)
        out.append(_read_exact(stream, n).decode("utf-8"))
    return out


def _read_value(stream: BinaryIO, ptype: int, strings: list[str]):
    if ptype == PTYPE_BOOL:
        return _read_exact(stream, 1)[0] != 0
    if ptype == PTYPE_INT:
        return struct.unpack("<i", _read_exact(stream, 4))[0]
    if ptype == PTYPE_DOUBLE:
        return struct.unpack("<d", _read_exact(stream, 8))[0]
    if ptype == PTYPE_STRING:
        return strings[_read_varint(stream)]
    raise ValueError(f"bad ptype {ptype}")


def _read_item(stream: BinaryIO, strings: list[str]):
    tag = _read_exact(stream, 1)[0]
    if tag == TAG_PAR:
        ptype = _read_exact(stream, 1)[0]
        name = strings[_read_varint(stream)]
        return Par(ptype, name, _read_value(stream, ptype, strings))
    if tag == TAG_PARTABLE:
        ptype = _read_exact(stream, 1)[0]
        name = strings[_read_varint(stream)]
        n = _read_varint(stream)
        cells = []
        for _ in range(n):
            r, c = struct.unpack("<ii", _read_exact(stream, 8))
            cells.append(Cell(r, c, _read_value(stream, ptype, strings)))
        return ParTable(ptype, name, cells)
    if tag == TAG_REFERENCE:
        rtype = strings[_read_varint(stream)]
        rname = strings[_read_varint(stream)]
        orig_data = _read_exact(stream, 1)[0]
        orig_name = strings[_read_varint(stream)]
        flags = _read_exact(stream, 1)[0]
        component_id = strings[_read_varint(stream)] if flags & 0x01 else None
        par_id = strings[_read_varint(stream)] if flags & 0x02 else None
        par_file = strings[_read_varint(stream)] if flags & 0x04 else None
        return Reference(rtype, rname, orig_data, orig_name,
                         component_id, par_id, par_file)
    if tag == TAG_MACROPARSET:
        return MacroParSetRef(strings[_read_varint(stream)])
    raise ValueError(f"unknown item tag 0x{tag:02x}")


def read_binary(path: str) -> Document:
    with open(path, "rb") as f:
        header = _read_header(f)
        strings = _read_string_table(f, header.num_strings)
        # Spec invariant: we should be at body_offset now.
        if f.tell() != header.body_offset:
            raise ValueError(
                f"string table size mismatch: at {f.tell()}, "
                f"expected body at {header.body_offset}"
            )
        doc = Document()
        for _ in range(header.num_macroparset):
            id_ = strings[_read_varint(f)]
            n = _read_varint(f)
            items = [_read_item(f, strings) for _ in range(n)]
            doc.macros.append(MacroParameterSet(id_, items))
        for _ in range(header.num_sets):
            id_ = strings[_read_varint(f)]
            n = _read_varint(f)
            items = [_read_item(f, strings) for _ in range(n)]
            doc.sets.append(Set(id_, items))
    return doc


# --- XML export ------------------------------------------------------------


def _format_value(ptype: int, value) -> str:
    if ptype == PTYPE_BOOL:
        return "true" if value else "false"
    if ptype == PTYPE_INT:
        return str(int(value))
    if ptype == PTYPE_DOUBLE:
        # repr() is round-trippable; trim it lightly for human readability.
        s = repr(float(value))
        return s
    return str(value)


def write_xml(doc: Document, path: str) -> None:
    out: list[str] = []
    out.append("<?xml version='1.0' encoding='UTF-8'?>\n")
    out.append(f'<parametersSet xmlns="{NAMESPACE}">\n')
    for m in doc.macros:
        out.append(f'  <macroParameterSet id="{escape(m.id)}">\n')
        for it in m.items:
            _emit_xml_item(out, it, indent="    ")
        out.append('  </macroParameterSet>\n')
    for s in doc.sets:
        out.append(f'  <set id="{escape(s.id)}">\n')
        for it in s.items:
            _emit_xml_item(out, it, indent="    ")
        out.append('  </set>\n')
    out.append('</parametersSet>\n')
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(out)


def _emit_xml_item(out: list[str], item, indent: str) -> None:
    if isinstance(item, Par):
        out.append(
            f'{indent}<par type="{PTYPE_NAMES[item.ptype]}" '
            f'name="{escape(item.name)}" '
            f'value="{escape(_format_value(item.ptype, item.value))}"/>\n'
        )
    elif isinstance(item, ParTable):
        out.append(
            f'{indent}<parTable type="{PTYPE_NAMES[item.ptype]}" '
            f'name="{escape(item.name)}">\n'
        )
        for cell in item.cells:
            out.append(
                f'{indent}  <par row="{cell.row}" column="{cell.column}" '
                f'value="{escape(_format_value(item.ptype, cell.value))}"/>\n'
            )
        out.append(f'{indent}</parTable>\n')
    elif isinstance(item, Reference):
        attrs = (
            f'type="{escape(item.type)}" '
            f'name="{escape(item.name)}" '
            f'origData="{ORIGDATA_NAMES[item.orig_data]}" '
            f'origName="{escape(item.orig_name)}"'
        )
        if item.component_id is not None:
            attrs += f' componentId="{escape(item.component_id)}"'
        if item.par_id is not None:
            attrs += f' parId="{escape(item.par_id)}"'
        if item.par_file is not None:
            attrs += f' parFile="{escape(item.par_file)}"'
        out.append(f'{indent}<reference {attrs}/>\n')
    elif isinstance(item, MacroParSetRef):
        out.append(f'{indent}<macroParSet id="{escape(item.id)}"/>\n')
    else:
        raise TypeError(f"unknown item {item!r}")


# --- inspector -------------------------------------------------------------


def inspect(path: str, full: bool) -> None:
    doc = read_binary(path)
    # Re-read the raw header so we can show it independently of the parsed model.
    with open(path, "rb") as f:
        header = _read_header(f)
        strings = _read_string_table(f, header.num_strings)
    n_par = n_table = n_ref = n_macro_ref = 0
    for s in doc.sets:
        for it in s.items:
            if isinstance(it, Par):
                n_par += 1
            elif isinstance(it, ParTable):
                n_table += 1
            elif isinstance(it, Reference):
                n_ref += 1
            elif isinstance(it, MacroParSetRef):
                n_macro_ref += 1
    for m in doc.macros:
        for it in m.items:
            if isinstance(it, Par):
                n_par += 1
            elif isinstance(it, Reference):
                n_ref += 1

    import os
    size = os.path.getsize(path)
    string_bytes = sum(len(s.encode("utf-8")) for s in strings)
    print(f"file:           {path}")
    print(f"size:           {size:,} bytes")
    print(f"version:        {header.version_major}.{header.version_minor}")
    print(f"flags:          0x{header.flags:08x}")
    print(f"strings:        {header.num_strings:,} (raw bytes: {string_bytes:,})")
    print(f"macroParSet:    {header.num_macroparset:,}")
    print(f"sets:           {header.num_sets:,}")
    print(f"par items:      {n_par:,}")
    print(f"parTable items: {n_table:,}")
    print(f"reference items:{n_ref:,}")
    print(f"macroParSet refs:{n_macro_ref:,}")

    if full:
        print()
        print("string table (first 32):")
        for i, s in enumerate(strings[:32]):
            print(f"  [{i:5d}] {s!r}")
        print()
        print("first 3 sets:")
        for s in doc.sets[:3]:
            print(f"  set id={s.id!r}, {len(s.items)} items")
            for it in s.items[:5]:
                print(f"    {it.__class__.__name__}: {_describe(it)}")
            if len(s.items) > 5:
                print(f"    ... +{len(s.items) - 5} more")


def _describe(item) -> str:
    if isinstance(item, Par):
        return f"name={item.name!r}, type={PTYPE_NAMES[item.ptype]}, value={item.value!r}"
    if isinstance(item, ParTable):
        return (f"name={item.name!r}, type={PTYPE_NAMES[item.ptype]}, "
                f"{len(item.cells)} cells")
    if isinstance(item, Reference):
        return (f"name={item.name!r}, type={item.type!r}, "
                f"origData={ORIGDATA_NAMES[item.orig_data]}, "
                f"origName={item.orig_name!r}")
    if isinstance(item, MacroParSetRef):
        return f"id={item.id!r}"
    return repr(item)


# --- CLI -------------------------------------------------------------------


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p_x2b = sub.add_parser("xml2bin", help="convert XML par file to binary")
    p_x2b.add_argument("input")
    p_x2b.add_argument("output")

    p_b2x = sub.add_parser("bin2xml", help="convert binary par file to XML")
    p_b2x.add_argument("input")
    p_b2x.add_argument("output")

    p_ins = sub.add_parser("inspect", help="dump a binary par file's contents")
    p_ins.add_argument("input")
    p_ins.add_argument("--full", action="store_true",
                       help="also dump the first strings and sets")

    args = parser.parse_args(argv[1:])
    if args.cmd == "xml2bin":
        doc = parse_xml(args.input)
        write_binary(doc, args.output)
    elif args.cmd == "bin2xml":
        doc = read_binary(args.input)
        write_xml(doc, args.output)
    elif args.cmd == "inspect":
        inspect(args.input, args.full)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
