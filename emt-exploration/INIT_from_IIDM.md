# Design note — IIDM-driven initialization for EMT (and the network-model question)

Status: design / scoping only (no code). Captures the assessment of how to give
the EMT models a proper, automatic initialization and an IIDM-based authoring
workflow, instead of the current full-`dyd`-with-no-IIDM cases.

## The core insight

An IIDM file already contains a **solved load flow** — bus voltages `V∠θ` and
injector `P/Q` at every node. That is *exactly* the steady-state phasor operating
point we currently hand-propagate with `Functions.balancedAbcInit` (see
`INIT.md`). So if an EMT case were driven from an IIDM, its initialization would
come **for free** from the load flow: no manual propagation, no residual
pre-fault settling.

Two questions are separable, with very different cost/benefit:

1. **Where does the initialization come from?**  (IIDM load flow)
2. **Where do the dynamics live?**  (C++ network vs Modelica EMT models)

## How Dynawo wires IIDM → init today (phasor)

In an IIDM-based dyd, a dynamic model is bound to a static IIDM element by
`staticId`, and Dynawo feeds that model's `_INIT` the node's load-flow values.
From `nrt/data/SMIB/.../SMIB_1_StepPm_IIDM`:

```xml
<dyn:blackBoxModel id="sm" lib="GeneratorSynchronous..." staticId="SM" .../>
<dyn:connect id1="sm" var1="generator_terminal" id2="NETWORK" var2="@SM@@NODE@_ACPIN"/>
```

- `staticId="SM"` → the generator's `_INIT` receives `P0/Q0/U0/UPhase0` from the
  IIDM load flow.
- `@SM@@NODE@_ACPIN` → the dynamic model connects to the C++ `DYNModelNetwork` at
  the IIDM node (a complex `ACPower`/`ACPIN` terminal).

This is the machinery to reuse. `Dynawo.Electrical.EMT.SynchronousMachine_INIT`
already follows this exact pattern (inputs `P0Pu/Q0Pu/U0Pu/UPhase0`).

## Option A — a native abc-EMT C++ network model (heavy)

This can be done two ways, and **it does not have to be an extension of the
existing `DYNModelNetwork`**. Mutating the current phasor network in place means
threading an abc/phasor switch through every component's `evalF`/`evalJt`, which
risks the proven positive-sequence path. The cleaner — and likely easier — route
is a **brand-new, separate `DYNModelNetworkEMT`** (its own `ModelBus`/`ModelLine`/
… family) that is purely abc, built from the same IIDM but never sharing code
paths with the phasor network. The phasor `DYNModelNetwork` stays completely
untouched, the abc model is free to use whatever state layout suits EMT, and the
two coexist (a case picks one). Either way the cost below is the same order of
magnitude; the from-scratch model just avoids the regression risk of editing a
mature, widely-used component.

- The C++ network is **positive-sequence phasor throughout**: every `ModelBus`
  carries `(ur, ui)`; `ModelLine` / `Model2WT` / `ModelLoad` / `ModelGenerator` /
  `ModelShunt` / `ModelSwitch` / … contribute complex injections into a complex
  nodal system. abc-EMT means **3 instantaneous phase voltages per bus (+ neutral)
  and `der()` currents**, i.e. rewriting `evalF`/`evalJt` for *every* component.
  That is effectively a **parallel network model**, not an extension.
- The existing **`line_isDynamic` is not a stepping stone**: it stays complex
  `(ur,ui)` and merely un-neglects `L·di/dt` and `C·dv/dt` — *dynamic-phasor /
  shifted-frequency* EMT, in the rotating frame. No harmonics, unbalance or
  waveforms.
- The **IIDM is balanced / positive-sequence** (no per-phase data). That is fine:
  an abc network expands each balanced element into three coupled phases, inits
  from the balanced load flow (the `balancedAbcInit` step, automatic), and lets
  **unbalance develop dynamically** (faults). Init balanced, simulate unbalanced
  — consistent with what we already do.
- Upside if done: native, fast, IIDM-driven topology + parameters + init,
  integrates with switches/events/curves. Cost: a major, first-class-feature
  sized C++ project.

## Option B — IIDM for *init only*, EMT dynamics stay in Modelica (light) — RECOMMENDED

Keep the EMT components as the Modelica models already built; make them
IIDM-aware by giving each a `staticId` and an `_INIT` companion that receives the
node load-flow `P/Q/V/θ` and reconstructs the abc/dq start values (the
`SynchronousMachine_INIT` pattern, generalised to line / transformer / cap /
load).

- The init problem **disappears**: IIDM load flow → `_INIT` → abc start values,
  reusing Dynawo's whole `staticId` / `_INIT` / macro-connect machinery. No hand
  propagation, no settling.
- Cases become **IIDM + dyd** — removing the "full-dyd, no IIDM" limitation.
- Effort is **bounded and C++-free**: `_INIT` companions for the remaining EMT
  components + a small mapping convention.

### The one real wrinkle: the connector boundary

Phasor models speak complex `ACPower`/`ACPIN`; EMT models speak abc
`EmtTerminal`. They cannot connect directly. Therefore:

- A **fully-EMT case** (every element EMT, no `DYNModelNetwork` for dynamics) is
  clean: abc terminals throughout, each element bound to its IIDM element *only
  for init*.
- A **hybrid case** (an EMT island on a phasor grid) additionally needs a
  phasor↔abc **boundary model** — itself a co-simulation topic. At `t = 0` the
  boundary is balanced, so the conversion is exact; the live coupling is the
  research part.

Also: binding an EMT branch model to an IIDM line/transformer means declaring
that element **controlled by a dynamic model** (the standard IIDM override) and
providing an `_INIT` that reconstructs the branch current from the two endpoint
load-flow voltages — exactly what the phasor line `_INIT` does, so the mechanism
is proven; it only has to be written for the abc form.

## Recommendation and scope

- **Now (Option B).** IIDM-driven init via `staticId` + `_INIT` companions on the
  Modelica EMT models. Biggest payoff (kills the manual init *and* unlocks the
  IIDM workflow) for the least work, and it is the idiomatic Dynawo path.
  Scope:
  1. ✅ `_INIT` companions for `SeriesRL`/`CoupledRL` (line), `TransformerYgYg`/`YgD`,
     `Capacitor` — reconstruct `I0`/`U0` from endpoint load-flow phasors
     (`SynchronousMachine_INIT` also done, reworked to an explicit salient-pole
     construction so the init never goes singular). A load `_INIT` is the only
     remaining companion.
  2. ✅ A small convention for the abc init (`Functions.balancedAbcInit` from the
     scalar IIDM phasor magnitude/angle, reconstructed in the dynamic model).
  3. ✅ A fully-EMT reference case — `Examples.SMIBLoadedInit` (run case under
     `examples/SMIBLoadedInit`) — proves the flow end-to-end: every start value is
     computed by `SMIBLoadedInit_INIT` and transferred by Dynawo's name-matching
     `_INIT` convention, with no hand-set constants. Pre-fault is exactly flat and
     it matches the hand-initialised `SMIBLoaded` to ~1e-4.

  **Remaining for the true IIDM workflow.** `SMIBLoadedInit` still seeds its
  `_INIT` from one operating point declared in the model, not from an `iidmFile`
  via `staticId`. The transfer mechanism is now proven; the next step is a
  *per-component* composite preassembled model (one `unitDynamicModel` +
  `initName` per element, wired by `dyn:connect` for the dynamics and
  `dyn:initConnect` for the phasor seeds) bound to an IIDM, so the load flow
  itself supplies each node voltage. The passive companions above already expose
  their current/voltage phasors (`IMagPu`/`IAngleRad`, `IcMagPu`, …) for exactly
  this `initConnect` chaining. The one open design point is the node-current
  balance at a bus with >2 elements, which `initConnect` (point-to-point) cannot
  express directly — a real IIDM sidesteps it by giving every node voltage from
  the load flow, so each companion stays independent.
- **Later (Option A), only if EMT becomes first-class.** A native abc network
  built as a **separate `DYNModelNetworkEMT`** (IIDM → EMT component instances),
  **not** by mutating the phasor `DYNModelNetwork`, accepting it is a large C++
  project. A standalone model is likely easier than an in-place extension and
  carries no regression risk for the phasor path. The
  dynamic-phasor `line_isDynamic` remains the lighter, already-existing C++
  "EMT-ish" path when waveform fidelity is not required.

## Empirical findings (IIDM prototype iteration)

Hands-on testing with a pypowsybl-solved SMIB IIDM nailed down the mechanics and
the boundary precisely:

1. **The init values come from the `.par`, via `<reference>`, keyed by `staticId`.**
   A dynamic model bound to an IIDM element pulls its `*0` start values with, e.g.
   `<reference name="machine_P0Pu" origData="IIDM" origName="p_pu" type="DOUBLE"/>`
   (`origName` ∈ `p_pu`, `q_pu` — receptor, base SnRef; `v_pu` — RMS, base UNom;
   `angle_pu` — rad). The C++ DataInterface resolves them from the load flow.
2. **The EMT machine `_INIT` is now IIDM-native.** Reworked to Dynawo's universal
   convention — `s0Pu = Complex(P0Pu, Q0Pu)` *receptor* on SnRef, `PGen0Pu = -P0Pu`,
   voltage RMS on UNom (peak abc reconstructed internally as `√2·V`). So
   `machine_{P0Pu,Q0Pu,U0Pu,UPhase0}` map straight onto IIDM `p_pu/q_pu/v_pu/angle_pu`.
   Operating point unchanged (existing cases bit-identical machine-side).
3. **`<network iidmFile=…>` always instantiates the C++ phasor network for every
   IIDM element that is *not* overridden by a dynamic model.** Binding only the
   machine (`staticId="SM"`) and leaving the EMT caps/line/tfo as parallel black
   boxes left the IIDM's buses + tfo + line + infinite-bus generator live in the
   C++ network, decoupled from the EMT island → `131 variables ≠ 129 equations`.
   The `<reference>` machinery cannot be used "for init only" while ignoring the
   rest of the IIDM network.
4. **The phasor pattern that resolves this is full override — and it is pure
   Modelica, no C++ network.** `nrt/.../SMIB_1_StepPm_IIDM/baseCase` has **zero**
   `NETWORK`/`ACPIN` references: every IIDM element is replaced by a Modelica black
   box (`Bus`×2, `Line`×2, `TransformerFixedRatio`, `InfiniteBus`, generator), each
   init-bound by `staticId`, connected to the others through **`Bus` models as the
   nodes**. `Electrical.Buses.Bus` is trivial — an `ACPower` node with
   `terminal.i = 0` that contributes no equation; it exists only to be the node and
   to carry the IIDM bus voltage to the connected `_INIT`s.

**Consequence for EMT — now demonstrated.** A fully-EMT, live-IIDM-driven case
works **in pure Modelica (Option B), no C++ EMT network** — see `iidm/smib_iidm.*`
(machine + buses start exactly at the load-flow point via `<reference>`:
`UStatorPu(0)=0.920`, `PGenPu(0)=0.600`). It was built by:
  - adding an **EMT `Bus` model** (`Electrical.EMT.Bus`) — the abc analogue of
    `Electrical.Buses.Bus`: a small shunt-capacitance node `staticId`-bound to the
    IIDM bus, init `Upu`/`Theta_pu` → abc (bus origNames are `Upu`/`Theta_pu`;
    generator origNames are `p_pu/q_pu/v_pu/angle_pu`), and
  - giving **every** IIDM element an EMT override (`staticId`) so the C++ network
    is left empty (as in `baseCase`); the infinite-bus slack generator is dropped
    from the Dynawo IIDM and the `EmtVoltageSource` overrides the `INF_BUS` bus.

  One refinement remains for a perfectly flat start: the EMT shunt caps draw an AC
  current the (cap-free) load flow does not see, leaving a ~0.4% startup swing.
  Modelling the caps as IIDM shunt compensators, or reconstructing the branch
  references to include the cap current, closes it. The live-init mechanism is
  exact; this is network consistency, not the IIDM binding.

A native **`DYNModelNetworkEMT` (Option A)** is therefore the *performance / first-
class* solution (a Modelica per-element network does not scale like the vectorised
C++ one), **not** a feasibility prerequisite — Option B can prove live IIDM-driven
EMT init first and cheaply.

## One-line summary

The IIDM is a free load flow; the cheapest way to use it is to let the existing
`staticId`/`_INIT` machinery seed the Modelica EMT models (Option B) — which the
phasor `baseCase` proves works with the network *fully overridden in Modelica*
(an EMT `Bus` model is the one missing piece), keeping the C++ `DYNModelNetwork`
empty and reserving a native abc `DYNModelNetworkEMT` (Option A) for first-class,
at-scale EMT.
