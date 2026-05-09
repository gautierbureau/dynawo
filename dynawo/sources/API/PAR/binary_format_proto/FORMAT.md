# Dynawo binary PAR format — draft v1

Status: **prototype / under investigation**.
Goal: a compact, fast-to-load binary alternative to the current XML PAR
format defined in `dynawo/sources/API/PAR/xsd/parameters.xsd`. This document
is the contract that both the Dynawo C++ importer and the Java writer in
powsybl-dynawo will implement.

The binary format is a 1:1 representation of the XML data model. It holds
exactly the same information, no more, no less, so any binary file can be
losslessly converted back to XML and vice versa.

---

## 1. Conventions

* All multi-byte integers are **little-endian**.
* `u8`, `u16`, `u32`, `u64` are unsigned fixed-width integers.
* `i32` is a signed 32-bit two's complement integer.
* `f64` is an IEEE-754 binary64 little-endian double.
* `varint` is unsigned LEB128 (each byte: bit7=continuation, bits0..6=payload).
* `stref` is a `varint` index into the file's string table.
* Strings in the string table are UTF-8, **without** a trailing NUL.

## 2. File layout

```
+------------------+
| Header           |  fixed 32 bytes
+------------------+
| String table     |  variable
+------------------+
| Body             |  variable: macroParameterSets then sets
+------------------+
```

The string table comes before the body so that a streaming reader can
resolve every `stref` immediately, with one forward pass.

## 3. Header (32 bytes)

| offset | size | field             | value / meaning                         |
| -----: | ---: | ----------------- | --------------------------------------- |
|      0 |    4 | `magic`           | ASCII `"DYNP"` (0x44 0x59 0x4E 0x50)    |
|      4 |    2 | `version_major`   | `1`                                     |
|      6 |    2 | `version_minor`   | `0`                                     |
|      8 |    4 | `flags`           | bitfield, currently 0; reserved         |
|     12 |    4 | `num_strings`     | number of entries in the string table   |
|     16 |    4 | `num_macroparset` | number of `macroParameterSet` records   |
|     20 |    4 | `num_sets`        | number of `set` records                 |
|     24 |    8 | `body_offset`     | absolute byte offset of body start      |

`body_offset` is redundant with the string table size but lets a reader
mmap the file and jump straight to the body once string indices are
resolved on demand.

## 4. String table

```
for i in 0 .. num_strings - 1:
    len  : varint   # byte length of the UTF-8 payload
    data : len bytes
```

The empty string at index 0 is **reserved** (`""`), so producers should
emit it as the first entry. Missing optional string fields use index 0.

The string table holds: every `set`/`macroParameterSet` `id`, every
parameter `name`, every reference `type`/`origName`/`componentId`/
`parId`/`parFile`, and every STRING-typed parameter `value`. Names like
`"tb"`, `"Ca"`, `"Cb"` that repeat across thousands of sets pay the
encoding cost only once.

## 5. Body

The body is just a back-to-back sequence of records, in this order:

1. `num_macroparset` `MacroParameterSet` records.
2. `num_sets` `Set` records.

No padding, no separators. There is no per-record byte length prefix:
records are self-delimiting because every field's size is fixed or
determined by an explicit count.

### 5.1 `MacroParameterSet`

```
id        : stref
num_items : varint
items     : num_items × Item        # tags 0x10 (par) or 0x12 (reference) only
```

### 5.2 `Set`

```
id        : stref
num_items : varint
items     : num_items × Item        # any of the four item tags
```

### 5.3 `Item`

```
tag : u8
payload depending on tag
```

Defined tags:

| tag    | meaning       | allowed in `Set` | allowed in `MacroParameterSet` |
| ------ | ------------- | :--------------: | :----------------------------: |
| `0x10` | `par`         |        yes       |               yes              |
| `0x11` | `parTable`    |        yes       |               no               |
| `0x12` | `reference`   |        yes       |               yes              |
| `0x13` | `macroParSet` |        yes       |               no               |

Any other tag is a hard error.

#### 5.3.1 `par` (tag `0x10`)

```
ptype : u8                          # 0=BOOL 1=INT 2=DOUBLE 3=STRING
name  : stref
value : depends on ptype
        ptype=0  → u8   (0=false, 1=true)
        ptype=1  → i32
        ptype=2  → f64
        ptype=3  → stref
```

`ptype` values match the order of `parameters::Parameter::ParameterType`
in `PARParameter.h` (`BOOL`, `INT`, `DOUBLE`, `STRING`).

#### 5.3.2 `parTable` (tag `0x11`)

```
ptype     : u8                      # same encoding as par
name      : stref
num_cells : varint
cells     : num_cells × Cell
```

```
Cell:
    row    : i32
    column : i32
    value  : depends on ptype       # same encoding as par.value
```

#### 5.3.3 `reference` (tag `0x12`)

```
type      : stref                   # free-form per current XML schema
name      : stref
origData  : u8                      # 0=IIDM, 1=PAR
origName  : stref
opt_flags : u8                      # bit0: has componentId
                                    # bit1: has parId
                                    # bit2: has parFile
                                    # bits3..7: reserved, must be 0
componentId : stref   if opt_flags & 0x01
parId       : stref   if opt_flags & 0x02
parFile     : stref   if opt_flags & 0x04
```

#### 5.3.4 `macroParSet` (tag `0x13`)

```
id : stref
```

## 6. Versioning rules

* Major version bump for any incompatible change (new mandatory field,
  changed tag semantics).
* Minor version bump for additive, backward-compatible changes (e.g.
  defining an unused bit in `opt_flags`).
* Readers must reject files with a higher major version, and tolerate
  files with a higher minor version provided no unknown bit/tag is hit.

## 7. Notes for implementers

* The format is intentionally **not** self-compressing. If on-disk size
  matters more than parse speed, gzip-wrap the binary; the binary still
  parses ~order-of-magnitude faster than XML even after `gunzip`.
* Doubles are written as raw `f64`, not as decimal text. This is exact
  and 8 bytes regardless of precision, vs. up to ~24 bytes in XML.
* String dedup is the single biggest win on real RTE par files where
  the same parameter names recur across thousands of sets.
