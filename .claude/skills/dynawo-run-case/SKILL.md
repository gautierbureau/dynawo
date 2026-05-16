---
name: dynawo-run-case
description: Run a Dynawo simulation case and validate it against the reference. Use when executing an NRT or example .jobs case, identifying which black-box models a case needs, or comparing simulation curves/timeline/lostEquipments against the committed reference outputs.
---

# Running and validating a Dynawo case

## Locate the case

NRT cases: `nrt/data/<Network>/<Group>/<CaseName>/`. Example cases:
`examples/`. A case directory has `<name>.jobs`, `.dyd`, `.iidm`, `.par`,
`.crv`, `solvers.par`, and a `reference/` directory.

## Identify required models

Each `<dyn:blackBoxModel ... lib="X">` in the `.dyd` needs model `X`.
Classify each `lib`:

- Has `PreassembledModels/X.xml` → **preassembled Modelica model**, must be
  built (`clean-build-models X` or `build-dynawo-models`).
- Otherwise (e.g. `DYNModelOmegaRef`) → **C++ model**, already built by
  `build-minimal`.

The `NETWORK` model (from `<dyn:network>` in the `.jobs`) and the solver
(`solvers.par`, e.g. `dynawo_SolverIDA`) are C++ — covered by `build-minimal`.

```sh
# build any missing preassembled models, e.g.:
./myEnvDynawo.sh clean-build-models LoadAlphaBeta EventSetPointBoolean
```

## Run

```sh
cd <case-dir>
/path/to/myEnvDynawo.sh jobs <name>.jobs
```

Exit 0 and `job '...' succeeded` mean it ran. Outputs go to `outputs/`:
`curves/curves.csv`, `logs/dynawo.log`, `timeLine/`, `lostEquipments/`,
`initValues/`.

## IIDM cases need a solved load flow

A case using `<dyn:network iidmFile="...">` needs the IIDM to carry a
load-flow solution: bus `v`/`angle` and injector `p`/`q`. The dynamic
models read their start values from it via
`<reference origData="IIDM" origName="p_pu|q_pu|v_pu|angle_pu"/>` in the
`.par`. Without it the initialization is inconsistent. If the IIDM has no
load flow, compute one with pypowsybl — see the `dynawo-iidm-loadflow`
skill.

## Validate against reference

`lostEquipments.xml` should match `reference/outputs/lostEquipments/`.

Curves must be compared **aligned by the time column** — the variable-step
solver produces differing row counts, so a row-index diff is meaningless:

```python
import csv
def load(p):
    with open(p) as f: return list(csv.reader(f, delimiter=';'))
r = load('reference/outputs/curves/curves.csv')
c = load('outputs/curves/curves.csv')
hdr = r[0]
rd = {float(x[0]): x for x in r[1:]}
cd = {float(x[0]): x for x in c[1:]}
md, mc = 0, None
for t in sorted(set(rd) & set(cd)):
    for j in range(1, len(hdr)):
        try:
            d = abs(float(rd[t][j]) - float(cd[t][j]))
            if d > md: md, mc = d, (t, hdr[j])
        except ValueError:
            pass
print('max abs diff', md, 'at', mc)
```

A max absolute difference around `1e-4` or below is within NRT tolerance.
