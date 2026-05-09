#!/usr/bin/env python3
"""Generate a synthetic ~5 MB Dynawo XML PAR file for binary-format experiments.

The file contains a realistic mix of set shapes:

* the bulk of sets have 30 parameters (typical generator/regulator block)
* a smaller fraction have 5 parameters (small device blocks)
* a sprinkle of `parTable` entries and `reference` entries
* a couple of `macroParameterSet` definitions referenced from many sets

Usage::

    python3 gen_sample.py [output.par] [target_mb]

Defaults: ``sample.par`` and ``5`` MB.
"""

from __future__ import annotations

import os
import random
import sys
from xml.sax.saxutils import escape

NAMESPACE = "http://www.rte-france.com/dynawo"

# Realistic-looking parameter name pool (kept short, biased toward repetition
# across sets — that is the realistic case in large RTE par files).
NAME_POOL_30 = [
    "tFilter", "KGover", "KAVR", "Vmin", "Vmax", "tIntegral", "tDeriv",
    "Tr", "Td", "Tq", "Xd", "Xq", "Xl", "H", "D", "R", "X", "B", "G",
    "P0Pu", "Q0Pu", "U0Pu", "UPhase0", "SNom", "PNom", "QNom", "VNom",
    "iLim", "uBlock", "tBlock",
]
assert len(NAME_POOL_30) == 30
NAME_POOL_5 = ["KGain", "tDelay", "vRef", "fRef", "active"]
TABLE_NAMES = ["LookupV", "LookupI", "LookupT"]


def _rand_double(rng: random.Random) -> float:
    # Mix of small, large, and "round" numbers — what real par files look like.
    pick = rng.random()
    if pick < 0.15:
        return 0.0
    if pick < 0.30:
        return float(rng.randint(1, 100))
    return rng.uniform(-10.0, 10.0)


def _rand_value(rng: random.Random, ptype: str) -> str:
    if ptype == "DOUBLE":
        return f"{_rand_double(rng):.6g}"
    if ptype == "INT":
        return str(rng.randint(0, 10))
    if ptype == "BOOL":
        return "true" if rng.random() < 0.5 else "false"
    return f"mode_{rng.randint(0, 4)}"


def _emit_par(out, ptype: str, name: str, value: str) -> None:
    out.append(
        f'    <par type="{ptype}" name="{escape(name)}" value="{escape(value)}"/>\n'
    )


def _emit_set_30(out, set_id: str, rng: random.Random) -> None:
    out.append(f'  <set id="{escape(set_id)}">\n')
    # 30 typed scalar parameters; type chosen to roughly mirror real files
    # (mostly DOUBLE, with a smattering of INT/BOOL/STRING).
    for name in NAME_POOL_30:
        roll = rng.random()
        if roll < 0.80:
            ptype = "DOUBLE"
        elif roll < 0.92:
            ptype = "INT"
        elif roll < 0.98:
            ptype = "BOOL"
        else:
            ptype = "STRING"
        _emit_par(out, ptype, name, _rand_value(rng, ptype))
    # ~10% of large sets carry a small lookup table
    if rng.random() < 0.10:
        tname = rng.choice(TABLE_NAMES)
        out.append(f'    <parTable type="DOUBLE" name="{escape(tname)}">\n')
        for r in range(1, 4):
            for c in range(1, 3):
                out.append(
                    f'      <par row="{r}" column="{c}" '
                    f'value="{_rand_double(rng):.6g}"/>\n'
                )
        out.append('    </parTable>\n')
    # ~50% of large sets attach a reference and a macroParSet
    if rng.random() < 0.50:
        out.append(
            '    <reference type="DOUBLE" name="P0Pu" origData="IIDM" '
            'origName="p_pu" componentId="GEN_'
            f'{rng.randint(0, 9999)}"/>\n'
        )
    if rng.random() < 0.50:
        macro = rng.choice(["GeneratorMacro", "RegulatorMacro"])
        out.append(f'    <macroParSet id="{macro}"/>\n')
    out.append('  </set>\n')


def _emit_set_5(out, set_id: str, rng: random.Random) -> None:
    out.append(f'  <set id="{escape(set_id)}">\n')
    for name in NAME_POOL_5:
        ptype = "DOUBLE" if name not in ("active",) else "BOOL"
        _emit_par(out, ptype, name, _rand_value(rng, ptype))
    out.append('  </set>\n')


def _emit_macros(out) -> None:
    out.append('  <macroParameterSet id="GeneratorMacro">\n')
    for name in ("KGover", "KAVR", "tFilter"):
        out.append(
            f'    <par type="DOUBLE" name="{name}" value="1.0"/>\n'
        )
    out.append(
        '    <reference type="DOUBLE" name="SNom" origData="IIDM" '
        'origName="ratedS"/>\n'
    )
    out.append('  </macroParameterSet>\n')
    out.append('  <macroParameterSet id="RegulatorMacro">\n')
    for name in ("Vmin", "Vmax", "tDeriv"):
        out.append(
            f'    <par type="DOUBLE" name="{name}" value="0.5"/>\n'
        )
    out.append('  </macroParameterSet>\n')


def generate(path: str, target_mb: float = 5.0, seed: int = 42) -> None:
    target_bytes = int(target_mb * 1024 * 1024)
    rng = random.Random(seed)

    chunks: list[str] = []
    chunks.append("<?xml version='1.0' encoding='UTF-8'?>\n")
    chunks.append(f'<parametersSet xmlns="{NAMESPACE}">\n')
    _emit_macros(chunks)

    # Roughly ~80% large sets, ~20% small sets — vary as requested.
    written = sum(len(c) for c in chunks)
    set_idx = 0
    while written < target_bytes:
        set_id = f"SET_{set_idx:06d}"
        if rng.random() < 0.80:
            _emit_set_30(chunks, set_id, rng)
        else:
            _emit_set_5(chunks, set_id, rng)
        # Update size estimate cheaply by summing only the new tail.
        written = sum(len(c) for c in chunks[-8:]) + written
        set_idx += 1
        # Periodically flush sum to avoid drift
        if set_idx % 200 == 0:
            written = sum(len(c) for c in chunks)

    chunks.append('</parametersSet>\n')

    with open(path, "w", encoding="utf-8") as f:
        f.writelines(chunks)


def main(argv: list[str]) -> int:
    out = argv[1] if len(argv) > 1 else "sample.par"
    mb = float(argv[2]) if len(argv) > 2 else 5.0
    generate(out, mb)
    size = os.path.getsize(out)
    print(f"wrote {out}: {size:,} bytes ({size / (1024 * 1024):.2f} MiB)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
