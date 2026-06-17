---
name: dynawo-iidm-loadflow
description: Prepare an IIDM network file for a Dynawo case by computing its load flow with pypowsybl. Use when a Dynawo NRT/example uses an iidmFile that lacks bus voltages/angles and injector p/q, when adding a new IIDM-based case, or whenever an IIDM needs a solved steady state before a time-domain simulation.
---

# Preparing an IIDM for a Dynawo case (pypowsybl load flow)

A Dynawo case that uses `<dyn:network iidmFile="...">` needs the IIDM to
carry a **solved load flow**: every `<bus>` must have `v` and `angle`, and
every injector (`generator`, `load`, ...) must have `p` and `q`. Dynawo's
dynamic models read their start values from these via `<reference>` tags
in the `.par` (see "Wiring" below). An IIDM with only `targetP`/`p0`
and no bus state cannot initialize a simulation consistently.

`pypowsybl` is the tool to compute that load flow.

## Install

```sh
pip install pypowsybl
```

## Compute the load flow

```python
import pypowsybl as pp

n = pp.network.load('network.xiidm')
results = pp.loadflow.run_ac(n)
print(results[0].status_text)          # expect "Converged"

n.save('network_solved.xiidm', format='XIIDM',
       parameters={'iidm.export.xml.version': '1.5'})
```

`run_ac` uses distributed slack by default — the loss/imbalance mismatch
is spread over the generators, so each generator's solved `p` differs
slightly from its `targetP`. That is the intended, consistent result;
the dynamic models take the solved `p` as their operating point.

Inspect before saving:

```python
print(n.get_buses()[['v_mag', 'v_angle']])
print(n.get_generators()[['target_p', 'p', 'q']])
print(n.get_lines()[['p1', 'q1', 'p2', 'q2']])
```

## Format gotchas (Dynawo vs pypowsybl)

- **Namespace prefix** — pypowsybl only reads IIDM with the `iidm:`-prefixed
  namespace (`<iidm:network xmlns:iidm="...">`). A file written with a
  default namespace (`<network xmlns="...">`) fails with "Unsupported file
  format". Re-serialize with a prefix first:
  ```python
  import xml.etree.ElementTree as ET
  NS = 'http://www.powsybl.org/schema/iidm/1_5'
  ET.register_namespace('iidm', NS)
  ET.parse('in.xiidm').write('out.xiidm', xml_declaration=True, encoding='UTF-8')
  ```
- **`--` in XML comments** — illegal XML; ElementTree and pypowsybl reject
  it (Dynawo's reader is more lenient). Strip comments before parsing:
  `re.sub(r'<!--.*?-->', '', txt, flags=re.DOTALL)`.
- **Export version** — pypowsybl's native format is recent (iidm 1_16).
  Export with `iidm.export.xml.version` set to a version Dynawo reads
  (`1.5` works; Dynawo also reads `1.0`). Prefixed namespace is fine for
  Dynawo — the standard NRT IIDMs use it.
- **Extension blocks** — the export adds `<iidm:extension>` blocks for
  `slackTerminal` / `referenceTerminals`. Dynawo does not need them;
  remove the blocks and their `xmlns:slt` / `xmlns:reft` declarations.

## Wiring the IIDM into the Dynawo case

In the `.dyd`, each dynamic model gets a `staticId` linking it to the IIDM
component:
`<dyn:blackBoxModel id="GEN1" lib="GeneratorClassical" ... staticId="GEN1"/>`.

In the `.par`, the model's `*0` start values are pulled from the IIDM with
`<reference>` (not `<par>`):

```xml
<reference name="generator_P0Pu" origData="IIDM" origName="p_pu" type="DOUBLE"/>
<reference name="generator_Q0Pu" origData="IIDM" origName="q_pu" type="DOUBLE"/>
<reference name="generator_U0Pu" origData="IIDM" origName="v_pu" type="DOUBLE"/>
<reference name="generator_UPhase0" origData="IIDM" origName="angle_pu" type="DOUBLE"/>
```

`origName` values: `p_pu`, `q_pu` (injector power, base SnRef), `v_pu`
(bus voltage / nominalV), `angle_pu` (bus angle). The `.par` then only
needs the genuinely dynamic parameters (H, reactances, ...).

The `<dyn:network>` element itself takes a `.par` set of C++ network-model
parameters (`load_alpha`, `line_currentLimit_maxTimeOperation`, ...); copy
one from an existing IIDM case (e.g. `nrt/data/IEEE14/.../IEEE14.par`).
