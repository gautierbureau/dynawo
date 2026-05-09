# Binary PAR format — investigation prototype

This directory is an **experimental sandbox** for evaluating whether a
binary alternative to the current XML PAR format
(`dynawo/sources/API/PAR/xsd/parameters.xsd`) can speed up parameter
loading on large simulations (~5 MB par files).

Nothing here is wired into the Dynawo build — it lives outside
`CMakeLists.txt` on purpose.

## Layout

| file                     | role                                                    |
| ------------------------ | ------------------------------------------------------- |
| `FORMAT.md`              | draft spec of the binary format (v1.0)                  |
| `gen_sample.py`          | generate a synthetic ~5 MB XML par file with mixed sets |
| `par_binary.py`          | reference Python encoder / decoder / inspector          |
| `cpp_bench/bench.cpp`    | minimal self-contained C++ load-time benchmark          |
| `cpp_bench/Makefile`     | builds `bench` against system Expat                     |

## Reproduce the numbers

```bash
cd dynawo/sources/API/PAR/binary_format_proto

# 1. Generate a 5 MB XML par file with realistic shape
python3 gen_sample.py sample.par 5

# 2. Convert it to the proposed binary format
python3 par_binary.py xml2bin sample.par sample.par.bin

# 3. Inspect (header + summary + first sets)
python3 par_binary.py inspect sample.par.bin --full

# 4. Round-trip back to XML for users who want to keep editing in XML
python3 par_binary.py bin2xml sample.par.bin sample.roundtrip.par

# 5. Build and run the C++ load-time benchmark
make -C cpp_bench
./cpp_bench/bench sample.par sample.par.bin 20
```

## Measured results (Ubuntu 24.04, g++ 13.3, `-O2`, in-cache reads)

Sample file: 3 914 sets (mostly 30 params, some 5 params), 2
`macroParameterSet`, 323 `parTable`, 1 599 `reference`, 97 501 scalar
`par` items.

### Size

|                       |   bytes | ratio vs XML |
| --------------------- | ------: | -----------: |
| XML (`.par`)          | 5 360 778 |       1.00× |
| **Binary (`.par.bin`)**       | **1 190 534** |       **4.50× smaller** |
| XML gzip -9           |   533 727 |      10.04× |
| Binary gzip -9        |   574 723 |       9.33× |

Take-away: the binary format alone gets ~⅕ the on-disk footprint, and
once gzipped it is in the same league as gzipped XML — which is
reassuring because **what we really care about is parse speed, not disk
size**.

### Parse time (C++ benchmark, 20 iterations, file in cache)

|                  | ms / load | speedup |
| ---------------- | --------: | ------: |
| XML via Expat SAX |   54.32  |   1.00× |
| **Binary mmap + decode** |    **0.77**  |   **70.93×** |

The XML path is using Expat, the same SAX-style parser family Dynawo's
`PARXmlImporter` uses under the hood, so this is a fair lower bound on
what can be achieved on the XML side without changing the algorithm.
The binary path mmaps the file, walks the string table, and consumes
every record exactly once.

The Python encoder (`par_binary.py xml2bin`) is unoptimised reference
code: it uses `xml.etree.ElementTree`, so it's slow (~2 s on this
sample). The production writer will be the Java side in
powsybl-dynawo, where the format is naturally streamed.

## Design highlights

A short summary; the authoritative spec is `FORMAT.md`.

* `DYNP` magic + 32-byte header, then a length-prefixed UTF-8 string
  table, then back-to-back records.
* String table dedups every set id, parameter name, reference type,
  componentId etc. On a real 5 MB file the table holds ~5 000 unique
  strings totalling ~50 KB — vs millions of repeated bytes in XML.
* Doubles are stored as raw IEEE-754 binary64 (8 bytes), exact and
  short. INT is `i32`, BOOL is one byte.
* All counts and string indices use unsigned LEB128 (varint), which
  keeps small set ids and small item counts at one byte.
* No padding, no per-record length prefixes — the format is fully
  self-delimiting from the tag byte and explicit element counts.

## What this prototype deliberately does *not* do

* No C++ importer wired into Dynawo. Adding one means a new
  `PARBinaryImporter` next to `PARXmlImporter.cpp` plus a sniff in
  `Importer::Importer()` that switches on `DYNP` magic vs. `<?xml`.
* No Java writer in powsybl-dynawo. The Java side will need a small
  serializer mirroring `par_binary.py:write_binary`. With the spec in
  `FORMAT.md` this is a couple of hundred lines.
* No streaming reader API. The current C++ benchmark walks the whole
  file in one pass; that's already <1 ms on a 5 MB file, so it's
  almost certainly fine, but a chunked reader is straightforward to
  add later if needed.

## Suggested next steps

1. Validate the spec on a couple of *real* RTE par files (5 MB and
   larger) — the hot path is parameter-name dedup, so realistic name
   distributions matter.
2. If numbers hold, add `PARBinaryImporter` (and `PARBinaryExporter`
   for tooling) inside `dynawo/sources/API/PAR/`, sniffing the format
   from the file's first 4 bytes.
3. Mirror the encoder in
   [powsybl-dynawo](https://github.com/powsybl/powsybl-dynawo)
   (`com.powsybl.dynawo.parameters` writer) so end users can pick the
   binary format from Java.
4. Keep `par_binary.py` shipped as the user-facing escape hatch
   (`bin2xml`) so existing XML tooling and editors keep working.
