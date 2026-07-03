# Exploring EMT support in Dynawo

*Exploration notes — 2026-06-27. Scope: can Dynawo simulate full three-phase
(EMT) signals, and how would we set up examples? The reference point is the
classical EMTP family of tools (instantaneous abc, nodal companion models).*

---

## 1. Two different ways to pose the problem

### Dynawo — phasor network, global implicit DAE

Dynawo represents the AC network with **complex phasors** (RMS,
positive-sequence, single base frequency). A connector carries a complex
voltage `V` and current `i`; the network equations are **algebraic** in those
phasors. The PI line (`dynawo/.../Electrical/Lines/Line.mo`) is literally:

```modelica
ZPu * (terminal2.i - YPu * terminal2.V) = terminal2.V - terminal1.V;
ZPu * (terminal1.i - YPu * terminal1.V) = terminal1.V - terminal2.V;
```

No `der(...)` on network quantities — the line is **quasi-static**. Machine
flux dynamics, controls, etc. add the *differential* part. The whole plant is
assembled into one large **DAE** `F(y, y', t) = 0` mixing differential and
algebraic states, and solved implicitly:

- `SolverIDA` — variable-step, variable-order BDF (SUNDIALS IDA).
- `SolverSIM` / `SolverTRAP` — fixed-step (SUNDIALS KINSOL each step).

This is excellent for **stability / RMS** time scales (ms–minutes) where the
network is assumed to ride a sinusoid at `f₀`. It does **not** represent
intra-cycle waveforms, DC offsets, harmonics, or wave propagation.

### EMT (EMTP family) — instantaneous abc, nodal companion models

EMT tools solve for **instantaneous three-phase voltages/currents** `v(t)`,
`i(t)` — the actual waveform. Two things characterise the classic EMTP method:

1. **Instantaneous branch equations** with real `der()`, per phase. A multiphase
   R-L branch is literally:
   ```modelica
   v = R.*i + L.*der(i);   // v,i are length-m (=3) instantaneous vectors
   ```
2. **Companion / Norton discretisation.** EMTP does *not* hand a DAE to a
   general integrator. It applies the **trapezoidal rule to each L and C
   locally**, turning every element into a **resistor in parallel with a
   history current source** that depends only on past values. The resulting
   network is purely resistive at each step, so one **real nodal-admittance
   solve** `G·v = i_history` advances the whole system. A Norton companion branch
   is exactly this:
   ```modelica
   v = RN*(i + iN);   // RN = companion resistance, iN = history current input
   ```
   and a constant-parameter (Bergeron) **traveling-wave line** is built from two
   Norton ends + a history term (Clarke transform + transport delay `tau`).

So the user's intuition is correct: **EMT tools do not formulate a global
implicit DAE.** They formulate a sequence of linear nodal solves with
per-element companion models, at a small fixed step (typically 1–50 µs).

### Side-by-side

| | Dynawo (phasor) | EMT (EMTP) |
|---|---|---|
| Unknown | complex phasor `V`, `i` (RMS, +seq) | instantaneous `v(t)`, `i(t)`, per phase abc |
| Network model | algebraic, quasi-static | differential (`L di/dt`, `C dv/dt`) or traveling-wave |
| Line | PI, complex `Z`,`Y` | PI with `der`, or Bergeron/wideband companion |
| Assembly | one global DAE `F(y,y',t)=0` | per-element companion → real `G·v=i_hist` |
| Solve | implicit Newton (KINSOL) / BDF (IDA) on full DAE | one linear nodal solve per step |
| Step | variable (µs–s) or fixed | fixed, small (~µs) |
| Captures | RMS envelope, controls, stability | waveforms, DC offset, harmonics, wave propagation |
| Time scale | ms – minutes | µs – ms |

---

## 2. What Dynawo *already* has that is relevant

### 2a. The trapezoidal solver — same math as EMT, different architecture

`dynawo/sources/Solvers/FixedTimeStep/SolverTRAP` is a fixed-step
**trapezoidal** integrator. Its core (`DYNSolverTRAP.cpp::computeYP`) is:

```cpp
// Yp = (2/h)*(y - y_prev) - Yp_prev
vectorYp_[i] = (2./h_) * (yy[i] - vectorYSave_[i]) - vectorYpSave_[i];
```

That is the trapezoidal rule `y_{n+1} = y_n + (h/2)(y'_n + y'_{n+1})` solved
for `y'_{n+1}`. **This is the identical integration rule EMTP applies to its
L and C elements.** The difference is purely *where* it is applied:

- **EMTP**: substitutes the rule into each element → builds a real `G` matrix →
  one linear solve per step.
- **Dynawo TRAP**: substitutes the rule into the **global symbolic DAE** →
  solves the (generally nonlinear) residual with KINSOL Newton each step.

**Consequence (the important one):** Dynawo's engine is *mathematically capable
of EMT*. If you feed it Modelica models written with **instantaneous abc states
and `der()`** (the classical EMTP style) instead of phasor algebraic
equations, and run them under `TRAP` (or `SIM`) at a small fixed step, you get a
genuine EMT time-domain solve. Dynawo would do it as a global implicit DAE
rather than nodal companion models — slower per step, but correct, and with the
benefit of a general nonlinear solver.

### 2b. "Dynamic in lines" — Dynawo already ships this (in the C++ network)

The shipped **Modelica** `Line.mo` is quasi-static, but the **C++ network
model** `dynawo/sources/Models/CPP/ModelNetwork/DYNModelLine.cpp` contains a
genuine **dynamic line** mode, gated by the per-line boolean parameter
**`line_isDynamic`** (set in the network parameter set). It is documented in
`documentation/functionalDoc/functionalDoc.tex` (§ on the line):

> *"the line can also be dynamic. In this case, the derivative terms
> (L·di/dt and C·dv/dt) are no longer neglected ... activated by setting the
> parameter `line_isDynamic` at true ... Please notice the dynamic line model
> has not been as extensively used and validated as the other network
> components."*

When enabled, the line **branch current becomes a differential state**
(`IbRe`/`IbIm`, marked `DIFFERENTIAL` / `DIFFERENTIAL_EQ`) and the equation
carries the `der` term (`DYNModelLine.cpp::evalF`, with `yp_` = derivative):

```cpp
f_[0] = -X·yp_[IbRe] - ωnom·(R·y_[IbRe] - X·ωref·y_[IbIm]) + ωnom·(ur1 - ur2);
f_[1] = -X·yp_[IbIm] - ωnom·(R·y_[IbIm] + X·ωref·y_[IbRe]) + ωnom·(ui1 - ui2);
```

i.e. `(X/ωnom)·dIb/dt = (V1−V2) − R·Ib + jX·ωref·Ib` — a **dynamic-phasor RL
branch in the rotating reference frame** (the `jX·ωref·Ib` term is the frame
rotation). Simultaneously the connected **buses get differential voltages**
(`modelBus_->setHasDifferentialVoltages(true)`): the shunt half-susceptance
becomes a real **capacitor** with `C·dv/dt` dynamics — see `ModelBus::urp()`
returning `yp_[urNum_]`, and the `suscept·urp/ωnom` injection term in
`evalNodeInjection`. So with `line_isDynamic=true` the network is solved as a
dynamic-phasor **RLC** system, not a quasi-static one.

This is part of the **DynaWave** initiative (grid-forming converters + dynamic
lines, built to reproduce the MIGRATE 3-node converter-interaction test case —
see `documentation/functionalDoc`). The grid-forming converter controls
(`Electrical/Controls/Converters/*`) and `Sources/Converter.mo` are the dynamic
sources designed to be used with it.

**What it is and isn't:** this is **dynamic-phasor / shifted-frequency EMT** —
it captures the network's electromagnetic transients (the *envelope* of the
fast L/C dynamics) in a single-frequency rotating frame. It is **not**
abc-waveform broadband EMT: it does not represent harmonics, phase unbalance,
or per-phase waveforms. It is the natural middle ground between RMS phasor and
full abc-EMT, and it already runs on the existing `IDA`/`TRAP` solvers.

**Limitation:** only fully-closed lines are supported — a partially-connected
dynamic line (`CLOSED_1`/`CLOSED_2`) throws `DynamicLineStatusNotSupported`
(`DYNModelLine.cpp`, exercised by `test/TestLine.cpp::ModelNetworkDynamicLine`).
No NRT/example case ships with `line_isDynamic=true`, so there is no committed
reference case — but the C++ unit test covers the dynamic path.

### 2c. Frames Dynawo already uses

Dynawo's converters work in a **dq / dynamic-phasor** frame. That is the
natural *middle ground* between RMS phasor and full abc-EMT: it captures fast
electrical dynamics (and can be extended to unbalanced/abc) while avoiding the
µs steps of waveform EMT. Worth keeping in mind when choosing fidelity.

---

## 3. Why we built native primitives instead of reusing an external EMT library

The obvious shortcut is to drop an existing MSL-based EMT Modelica library
straight into Dynawo. It was ruled out, for reasons that all point the same way:

1. **MSL `Electrical.*` dependency.** Such libraries `extends
   Modelica.Electrical.MultiPhase.*` / `Analog.*` and use SI units (kV, H, Ω).
   Dynawo's own library intentionally avoids MSL `Electrical.*` — it uses only a
   small Modelica subset (`Blocks`, `ComplexMath`, `SIunits`, `Constants`) and
   works in **per-unit**. Even though the bundled OpenModelica ships full MSL
   3.2.3 (so such models *flatten* fine standalone), **Dynawo's model-build
   pipeline** loads only the Dynawo subset with **no MSL `Electrical.*`**, so an
   MSL-electrical model cannot build into a Dynawo preassembled model without
   patching the build to inject MSL's `Electrical` sources.
2. **Licensing / provenance.** A third-party library with no clear license is not
   appropriate to merge into RTE's MPL-2.0 source tree with RTE headers.
3. **No interoperability anyway.** MSL `Pin`/`Plug` connectors cannot `connect()`
   to Dynawo's phasor `ACPower`; it could only ever be a self-contained EMT
   island — the *same* limitation the from-scratch primitives have, but without
   the per-unit / convention-compliant / MSL-free / license-clean properties.
4. **Companion-model dead weight.** Classical EMTP lines carry companion
   machinery (history terms, transport delays) for *nodal* solution —
   unnecessary in Dynawo's global DAE, where plain `der()` suffices.

So the from-scratch primitives (`Dynawo.Electrical.EMT`, per-unit, MSL-free) were
the right call: the low-risk way to *prove the engine works* (done — see the
Path B STATUS below), and the basis everything since was built on. See
`EMT_methods.tex` for the maths of the DAE vs companion approaches.

---

## 4. How we could simulate full three-phase signals — options

Several realistic paths, increasing integration effort:

### Path 0 — Use Dynawo's existing dynamic-phasor network (no new models)
Build a case with **`line_isDynamic=true`** in the network parameter set, driven
by the grid-forming **converter** models (DynaWave). This needs **only a new
`.dyd`/`.par`/`.jobs` + IIDM** — no model authoring or rebuild — and exercises
the `L·di/dt` / `C·dv/dt` network dynamics on the shipped `IDA`/`TRAP` solvers.
It gives **dynamic-phasor (shifted-frequency EMT)** results, not abc waveforms,
but it is the **fastest honest "EMT-ish in Dynawo" demonstrator** and the right
first experiment. Caveat: lightly validated upstream, fully-closed lines only.
The MIGRATE 3-node grid-forming case is the upstream reference to mirror.

### Path A — Run a reference EMT circuit in OpenModelica (fastest to first abc waveform)
Install OpenModelica + MSL 3.2.3 and simulate a standard multiphase EMT circuit
(source + RLC + a travelling-wave line) in plain MSL. This gives reference
three-phase waveforms immediately, **with no Dynawo involvement** — good for a
numerical oracle, but it does **not** answer "EMT *in Dynawo*".

### Path B — Port a minimal EMT kernel into Dynawo's Modelica subset (recommended PoC)
Re-author a *small* set of EMT primitives **in Dynawo's own Modelica subset**
(no MSL dependency), then run them through `SolverTRAP`/`SolverSIM` at a small
fixed step. Minimum viable set:

1. A 3-phase instantaneous connector (3 real `v`, 3 real flow `i`) — Dynawo
   uses flow-connectors already, so this is a small new `Connector`.
2. `EmtSource` — `v[k] = Vm*cos(2πf t + φ_k)`, `k∈{a,b,c}`.
3. `EmtRL` — `L·der(i) = v - R·i` (the standard instantaneous R-L branch, abc).
4. `EmtCapacitor` — `C·der(v) = i`.
5. A grounded star / load.

Wire source → RL → RLC load, a `.jobs` with `solver lib="dynawo_SolverTRAP"`,
`timeStep` ~ 1e-5 s, and curves on `i[a]/i[b]/i[c]`. This is the **honest
proof that Dynawo's DAE+TRAP engine produces EMT three-phase waveforms.**

> **STATUS — built, run & VALIDATED (2026-06-27).** Dynawo was built from
> source in this environment, the EMT primitives compiled cleanly through
> `omcDynawo` (array connector + `flow` + `der()` + `connect()` all accepted),
> and the demo ran on `SolverTRAP` at a 10 µs step over 0.2 s. Steady state
> matches the analytic divider to ≤0.1%: source peak √2 pu, line-current peak
> 0.6998 vs 0.700, balanced a+b+c=0, 50 Hz, with the expected energisation DC
> offset (iₐ 1.044→0.700). Plot: `examples/ThreePhaseRLC/emt_waveforms.png`.
> Two practical findings: (1) build the example as a **preassembled model**
> because `build-minimal` doesn't install the Modelica library sources for
> on-the-fly compilation; (2) set the fixed-step solver's
> `minimalAcceptableStep` below the EMT step (default 0.1 s aborts a µs-step
> run). Path B is implemented:
> - Primitive library `Dynawo.Electrical.EMT` under
>   `dynawo/sources/Models/Modelica/Dynawo/Electrical/EMT/`: `EmtTerminal`
>   (3-phase abc connector with `flow` current), `BaseEmtTwoTerminal`,
>   `Resistor`, `Inductor`, `Capacitor`, `SeriesRL`, `VoltageSource`, `Ground`
>   — all instantaneous abc with real `der()`, no MSL dependency. Registered in
>   the parent `CMakeLists.txt` / `package.order`.
> - Demo circuit `EMT.Examples.ThreePhaseRLCircuit` + runnable case
>   `emt-exploration/examples/ThreePhaseRLC/` (jobs/dyd/par/crv, `SolverTRAP`,
>   10 µs step).
> - **Not yet built/run** — the container has no compiled Dynawo and building is
>   the gated heavy step (`BUILD_NOTES.md`). Validation plan: analytic
>   steady-state divider check (oracle), then optional OpenModelica MSL
>   waveform cross-check (Path A).

### Path C — Full library port / converter (largest effort)
Systematically build out the EMT primitive set (RLC, sources, switches,
CP/WB lines, transformers, SM) on Dynawo primitives, expressing every branch as a
plain `der()` DAE equation (no MSL `Electrical.*`, no companion-model
interfaces). This is a real
project; only worth it once Path B validates fidelity and performance at the
step sizes / network sizes of interest.

---

## 5. Open questions to decide before building

1. **Target fidelity**: true abc-waveform EMT (µs steps), or dq/dynamic-phasor
   (the frame Dynawo converters already use, ms steps)? They imply very
   different model sets and step sizes.
2. **Why EMT in Dynawo specifically** (vs. EMTP/OpenModelica): hybrid
   phasor↔EMT co-simulation? Reusing Dynawo controls/IIDM/IO? Single-tool
   workflow? This shapes whether Path B or C is worthwhile.
3. **Network size & step**: EMT at µs over a large network is the performance
   question for a *global implicit DAE* engine; a small PoC tells us a lot.

## 6. Recommendation

Decide the target first, because Dynawo already covers one of the two meanings
of "EMT":

- **If "fast network electromagnetic transients in phasor frame" is enough**,
  start with **Path 0** — it is already in Dynawo (`line_isDynamic` + converter
  models) and needs only a case, no new models. This is the quickest win and
  worth doing regardless, to establish a baseline dynamic-network case (none
  ships today).
- **If you specifically need full three-phase abc waveforms** (harmonics,
  unbalance, per-phase signals — what a classical EMT tool produces), then **Path B**: a
  handful of abc EMT primitives in Dynawo's Modelica subset, run under `TRAP`,
  validated against an OpenModelica run of the same circuit (**Path A** as the
  reference oracle). This answers "can Dynawo produce true three-phase
  waveforms" with a runnable example, before committing to a full port
  (**Path C**).

Concretely I'd suggest: (1) build a `line_isDynamic` demonstrator case now
[Path 0], and in parallel (2) stand up OpenModelica to get abc reference curves
from a standard MSL EMT circuit [Path A]; then choose between Path B and C based on
whether the dynamic-phasor fidelity of Path 0 is sufficient.
