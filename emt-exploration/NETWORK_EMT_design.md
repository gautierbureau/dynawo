# Scoping note — a native `DYNModelNetworkEMT` (Option A)

Status: design / scoping only. Option B (pure-Modelica live IIDM-driven init, see
`INIT_from_IIDM.md` + `iidm/smib_iidm.*`) is proven and is the right path for
correctness and small networks. This note scopes the **performance / first-class**
step: a native abc C++ network that mirrors the phasor `DYNModelNetwork`, so the
grid is solved in vectorised C++ instead of element-by-element Modelica.

## Why a separate model (not a mutation of `DYNModelNetwork`)

The phasor `DYNModelNetwork` is positive-sequence throughout: every `ModelBus`
carries a complex `(ur, ui)`; `ModelLine` / `ModelTwoWindingsTransformer` /
`ModelShuntCompensator` / `ModelSwitch` / `ModelGenerator` / `ModelLoad`
contribute complex injections into a complex nodal system. Threading an
abc/phasor switch through every component's `evalF`/`evalJt` would risk the
proven positive-sequence path. A standalone **`DYNModelNetworkEMT`** (its own
component family) is cleaner and carries no regression risk; a case picks one.

## Model selection — from the jobs file (decided)

The two network models stay **fully separate** and a case selects which one to
build **in the `.jobs`**, mirroring how the solver is already chosen there. No
runtime flag is threaded through the components, no shared abc/phasor branches:

```xml
<!-- phasor (default, unchanged) -->
<dyn:network iidmFile="grid.iidm" parFile="net.par" parId="Network"/>
<!-- native EMT network -->
<dyn:network iidmFile="grid.iidm" parFile="net.par" parId="Network"
             lib="dynawo_DYNModelNetworkEMT"/>
```

The modeler reads the optional `lib` attribute when it builds the network model:
absent → the built-in phasor `DYNModelNetwork`; set → load that model factory
instead (exactly like `<solver lib="dynawo_SolverTRAP"/>`). Both are compiled
independently and never reference each other; the IIDM, the `.par` network set,
and the `staticId`/`<reference>` init wiring are identical for both, so a case
switches domains by that one attribute. This keeps the wiring trivial and the two
networks genuinely decoupled — the design choice that makes Option A tractable
alongside the proven phasor path.

## What changes vs the phasor network

The C++ `NetworkComponent` interface is reused unchanged — the same virtuals
already carry everything EMT needs:

```
instantiateVariables / init / getY0      defineElements / evalStaticYType ...
evalNodeInjection                        evalDerivatives(cj) / evalDerivativesPrim
evalF(type)                              evalJt(cj, rowOffset, Jt) / evalJtPrim
evalG / evalZ                            evalCalculatedVars
```

The difference is the *state layout and the residuals*, not the framework:

| | phasor `DYNModelNetwork` | `DYNModelNetworkEMT` |
|---|---|---|
| bus unknowns | `(ur, ui)` — 2 algebraic | `(va, vb, vc)` [+ `v0`] — instantaneous |
| branch state | algebraic complex `I = Y·U` | **differential** `L·di/dt = v − R·i` (3 phases) |
| KCL at a node | `Σ Ire = 0`, `Σ Iim = 0` | `Σ ia = 0`, `Σ ib = 0`, `Σ ic = 0` |
| `evalDerivatives` | mostly unused (algebraic) | **core** — every L/C element has `der()` |
| capacitive node | shunt B in the admittance | `C·dv/dt = Σ i` (node voltage is a state) |
| `yType` | all `ALGEBRAIC` | mix of `DIFFERENTIAL` (L,C states) and `ALGEBRAIC` |

So the EMT network is effectively a **differential-algebraic nodal model**: node
voltages are states (each node needs a shunt C, exactly the `EmtBus` artifact from
Option B — or a small parasitic C), branch currents are states, and `evalF`
assembles per-phase KCL + each element's per-phase v–i relation. The Modelica EMT
models already encode every one of these equations (`SeriesRL`, `Capacitor`,
`TransformerYgYg`, the machine) — the C++ port transcribes them into
`evalF`/`evalJt`/`evalDerivatives`.

## Component classes to build (mirror the phasor set)

- `ModelBusEMT` — abc node; accumulates per-phase current injections
  (`iaAdd/ibAdd/icAdd`, replacing `irAdd/iiAdd`); holds the node-voltage states
  with `C·dv/dt = Σ i`.
- `ModelLineEMT` / `ModelQuadripoleEMT` — series R+L per phase (`L·di/dt = Δv −
  R·i`), optional mutual coupling and shunt half-C at each end.
- `ModelTwoWindingsTransformerEMT` — YgYg / YgD with the ratio and phase shift
  (the `TransformerYgYg` / `TransformerYgD` equations already derived).
- `ModelShuntEMT`, `ModelSwitchEMT`, fault injection (per-phase, for SLG/LL/3φ).
- Loads / sources as per-phase injectors.
- Synchronous machines stay **Modelica** (as in the phasor world — `ModelGenerator`
  in C++ is only a simple injector; detailed machines are Modelica), connected to
  the C++ EMT network through an **abc terminal** (see boundary below).

`DYNModelNetworkEMT` orchestrates them and a `ModelBusContainerEMT` assembles the
sparse Jacobian, built from the IIDM via the DataInterface — the same wiring as
today, plus a balanced→abc expansion of each element at build time and
`balancedAbcInit` from the load flow for `getY0`.

## The one genuinely new piece: the abc connector boundary

Phasor Modelica models reach the C++ network through `@NODE@_ACPIN` (a complex
`ACPower` terminal). EMT needs an **abc terminal macro** — three instantaneous
`v`/`i` pairs — so Modelica EMT models (the synchronous machine, power-electronic
converters) connect to `DYNModelNetworkEMT` at a node. This is the abc analogue of
`ACPIN`; it is the only new interface concept, everything else is a transcription
of equations Option B already validated.

## IIDM, init, events

- **IIDM** is balanced / positive-sequence (no per-phase data). Fine: expand each
  balanced element into three coupled phases at build, init from the balanced load
  flow (`balancedAbcInit`, exactly Option B's `<reference>` values), and let
  unbalance develop dynamically (faults). The shunt-cap-vs-load-flow consistency
  refinement noted in `INIT_from_IIDM.md` is handled natively here by giving each
  node its parasitic C and seeding `v(0)` from the load flow.
- **Events** (faults, switches) reuse `evalG`/`evalZ` per phase.
- **Solver**: stiff at the network's electrical time constants → the fixed small
  step `SolverTRAP` (`hMin=hMax=2e-5`) already used by the Modelica EMT cases;
  the C++ network just makes each step far cheaper than N Modelica elements.

## Effort and phasing

Large but bounded — a first-class feature, comparable to adding a new C++ model
family. Suggested phases, each independently testable against the Option-B
Modelica cases (which become the reference oracle):

1. `ModelBusEMT` + `ModelLineEMT` + a voltage source → RLC line energisation
   (cross-check `Examples.ThreePhaseRLCircuit`).
2. Add `ModelTwoWindingsTransformerEMT` + the abc terminal boundary; connect the
   Modelica `SynchronousMachine` → reproduce `iidm/smib_iidm` (now with a flat
   start, the shunt-C consistency handled natively).
3. Add fault/switch components → reproduce the fault NRTs.
4. IIDM instantiation + `<network>` selection of the EMT network model.

Until then, **Option B is the supported path** — it already delivers live
IIDM-driven init and the full phasor-control reuse, in pure Modelica.
