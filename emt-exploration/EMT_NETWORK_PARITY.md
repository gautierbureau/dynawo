# EMT network parity — status & remaining work

Scope of this document: the native abc **EMT** network model
(`dynawo/sources/Models/CPP/ModelNetworkEMT/`) versus the phasor network it is
ported from (`dynawo/sources/Models/CPP/ModelNetwork/`). It records what is at
parity, what is intentionally *not* a gap, and scopes the work that genuinely
remains so it can be picked up later.

The phasor model is the reference: "parity" means the EMT network exposes the
same capability, with abc (three-phase instantaneous) physics instead of a
single-phase phasor. Differences that are deliberate (the EMT folds several
phasor concepts into fewer methods) are called out as **non-gaps**.

Validation harness: the eight EMT-network cases were promoted out of this
investigation folder into the standard NRT suite as `nrt/data/EMT/EmtNetwork*`
(`EmtNetworkSMIB`, `EmtNetworkSMIBNodeFault`, `EmtNetworkIEEE14`,
`EmtNetworkIEEE14LineDisconnection`, `EmtNetworkIEEE14Fault`,
`EmtNetworkShuntReclosing`, `EmtNetworkLineFaultDisconnection`,
`EmtNetworkTapChanger`), each with a committed `reference/outputs/curves/curves.csv`.
Every change below must keep these green under `nrt.py` (or regenerate a reference
with a documented reason, as was done once for the line-disconnection case).

---

## 1. At parity (done)

| Capability | EMT status | Commit |
|---|---|---|
| Per-component structure (`instantiateVariables`/`defineElements`, offsets) | modular, in-block ACPIN flows | `bfde8be`, `4fdb806` |
| Switchable shunt compensator (connection-state z) | `ModelShuntCompensatorEMT`; inductive bank = reactor-to-ground | `0fbca2f` |
| Event-driven disconnection (line/switch/load/tfo/shunt) | **fixed** — was a silent no-op (wrong z convention) | `0fbca2f` |
| Internal-variable dump/restore (checkpoint/restart) | node C/G/faultG, connection flags, tap positions | `aeb5b6f` |
| Residual & root equation naming (`setFequations`/`setGequations`) | every abc residual / root labelled | `6a0e9dc` |
| Silent-z optimization (`collectSilentZ`) | connection-state z flagged `NotUsedInDiscreteEquations` | `cd3c9b7` |
| Tap-changer action delays (`t1st`/`tNext`/`tolV` from PAR) | `applyTapTiming`, VHV/HV selection | `d19f1eb` |
| Bus short-circuit capability flag (`_hasShortCircuitCapabilities`) | accepted as a synonym of `_hasConnection` (same injection terminal) | (this change) |
| Node (bus) fault | already present: built-in `fault_*` automaton **and** external `EmtFault` on a bus ACPIN | pre-existing |

The simulation-critical contract — variables, elements, residuals, Jacobian,
discrete events, calculated variables, checkpoint/restore, equation naming — is
complete and regression-validated.

---

## 2. Non-gaps (intentional, do NOT implement)

- **Voltage-/frequency-dependent and restorative loads, voltage-dependent PQ
  generators.** These are quasi-static / RMS constructs, NOT classical EMT
  models, so they do not belong in the abc network as C++ primitives. Their
  defining variable is the RMS voltage `U` (and P/Q): the phasor `ModelLoad` /
  `ModelGenerator` express `P = P0*(U/U0)^alpha`, a Tp/Tq recovery, etc. directly
  because the phasor formulation is quasi-static — `U` is a state and `P = U*I`
  is an algebraic identity. In EMT none of that holds: `U` is not a state (only
  the instantaneous `v_a,v_b,v_c` are), and `P`/`Q` are not instantaneous (a
  constant-power draw needs `i = P/v`, singular at voltage zero-crossings and
  only meaningful averaged over a cycle). Realising them in EMT therefore
  requires an RMS/dq measurement filter plus a controlled current source — a
  "dynamic phasor inside EMT" that (a) adds a measurement lag/parameters absent
  from the phasor model, (b) is valid only on timescales slower than a cycle
  (~20 ms), i.e. the regime where one would use phasor anyway, and (c) is
  meaningless for the sub-cycle transients EMT exists to capture. The classical
  split, which this network already follows, is: the C++ network holds the
  passive/algebraic primitive (the constant-impedance load — present; the
  constant-PQ injector — present as the same simplification the phasor makes),
  and any voltage-/frequency-dependent or restorative behaviour is a detailed
  **Modelica** EMT model on the bus ACPIN, with an explicit measurement. So these
  are non-gaps for the network in the same sense as the 3WT / SVC / HVDC: the
  phasor has them as network primitives because it is quasi-static; the EMT
  network should not. (This retires the former "remaining work" items R1 and R2.)
- **Three-winding transformer.** The phasor `ModelThreeWindingsTransformer` is a
  pure no-op placeholder (`sizeY = 0`, every electrical method empty): the C++
  `ThreeWTransformerInterface` exposes only the three bus interfaces and
  connection flags, no leg R/X/ratedU/ratio. A detailed 3WT is an external
  Modelica model bound to the three bus ACPINs. Documented in
  `DYNModelNetworkEMT.cpp` (`initializeFromData`). A *physical* EMT 3WT (star +
  3 RL legs) is possible but would require extending the DataInterface to surface
  the leg impedances — see item **R5** below.
- **`evalState`** — folded into `evalZ` (the EMT reports state/topology changes
  through `evalZ`'s return code).
- **`evalYMat` / `evalDerivatives` / `evalDerivativesPrim`** — phasor
  admittance-matrix plumbing; the EMT assembles residuals directly in `evalF` /
  `evalJt`, so there is nothing to port.
- **Per-component `defineNonGenericParameters`** — the EMT derives all physical
  data (R, L, C, ratios) from the IIDM in `initializeFromData` and passes it to
  component constructors; it does not route physical data through the PAR. The
  *only* phasor per-component PAR parameters are behavioural/control knobs (not
  impedances), and they are needed only for the feature models in section 3.
  When one of those features is implemented, that component gains its own
  `defineNonGenericParameters` + `setSubModelParameters` at the same time.
- **SVC frozen at load-flow B**, **HVDC as decoupled PQ injections** — these are
  the phasor's *own* simplified forms (detailed versions are Modelica), so the
  EMT matches the phasor, not a gap.

---

## 3. Remaining work (feature models, not interface parity)

These are genuine capability gaps, but each is a **new behavioural model**, not a
missing interface method. Effort: S ≈ <1 day, M ≈ 1–3 days, L ≈ >3 days.
Risk is to the regression unless isolated behind "absent parameter → current
behaviour kept".

> **R1 and R2 retired.** Voltage-/frequency-dependent & restorative loads (R1)
> and voltage-dependent PQ generators (R2) were originally listed here. They are
> NOT classical EMT models — they are quasi-static / RMS constructs that would
> require a measurement-based "dynamic phasor inside EMT" overlay, valid only on
> slower-than-a-cycle timescales. They belong on the Modelica-on-ACPIN side, not
> in the C++ network. See section 2 for the full rationale. The only genuinely
> EMT-appropriate remaining item is R5 (and it is optional).

### R3 — Shunt reclosing delay — DONE
`ModelShuntCompensatorEMT` now honours `<id>_no_reclosing_delay` (the abc
analogue of the phasor's `_no_reclosing_delay`): on a trip it records the opening
time, owns a "reclosing permitted" root (`evalGReclose`), and a reclose commanded
during the lockout window is **deferred** (`pendingClose_`) until the delay
expires (`evalZReclose`) — slightly stronger than the phasor, which only exposes
an advisory availability flag. Wired through the network's g pass / evalG / evalZ
like the line current-limit roots; `tLastOpening_`/`pendingClose_` are
checkpointed. Validated by `nrt/data/EMT/EmtNetworkShuntReclosing`:
open the bus-9 bank at 0.1, command reclose at 0.2, delay 0.4 -> the reclose is
deferred to t=0.5 (vs t=0.2 with delay 0). No section change yet (the EMT shunt
still fixes its active section at the initial value); scope that separately if
needed.

### R4 — Bus short-circuit capabilities — DONE (was mis-scoped)
Earlier draft called this "short-circuit calculation tooling, no time-domain
dynamics." That was wrong. In the phasor `ModelBus`, `_hasShortCircuitCapabilities`
is used only in `(hasConnection_ || hasShortCircuitCapabilities_)` — the two flags
gate the IDENTICAL machinery (the bus gains its `ir`/`ii` injection terminal,
`sizeY` 2→4, and the injected current enters the KCL). It is the bus-terminal flag
a node-fault / short-circuit model attaches through, not a calculation feature, and
the EMT already had it as `hasConnection` (`enableExternalConnection`, `nbY` 3→6).
`_hasShortCircuitCapabilities` is now accepted as a synonym of `_hasConnection`
(verified: opening a fault bus with either flag is bit-identical). Node faults
themselves already work two ways — the built-in `fault_*` automaton (`smibnetflt`)
and an external `EmtFault` on a bus ACPIN (`ieee14_mfault`). The only genuine
remaining sliver here is **multiple simultaneous built-in faults** (the `fault_*`
automaton holds a single `faultBus_`); use multiple `EmtFault` black-boxes for that
today, or generalise `fault_*` to a list if a built-in multi-fault is wanted (S).

### R5 — Physical three-winding transformer (L, exceeds phasor)
Build a real EMT 3WT: a fictitious star `ModelBusEMT` + three switchable RL leg
branches (reusing `ModelTwoWindingsTransformerEMT` / `ModelLineEMT` per leg).
**Blocked** on extending `DYNThreeWTransformerInterface` (DataInterface layer) to
expose per-leg R/X/ratedU/ratio — without that the network has no data. This
would make the EMT *better* than the phasor (which is a no-op), so it is a
product decision, not a parity requirement.
- Files: new `DYNModelThreeWindingsTransformerEMT.*` (or reuse 2WT legs),
  `DYNModelNetworkEMT.cpp`, **and** the DataInterface/IIDM importer.
- Test: new NRT case with a 3WT; compare to a Modelica reference.
- Risk: high (touches the data layer).

### R6 — Runtime validation of tap-changer timing — DONE
Added `nrt/data/EMT/EmtNetworkTapChanger` (a regulating LTC transformer, an
EmtInfiniteBus source, targetV deliberately off the operating point, PAR
`transformer_t1st_HT`/`tNext_HT`/`tolV`) to the regression harness with a golden
reference, so `applyTapTiming` runs at runtime. Building this test exposed and
fixed a real bug: the tap-timing parameters were never DECLARED in
`defineParameters`, so `hasParameterDynamic()` was always false and the
`d19f1eb` timing port was dead code. With the declaration fixed, the tap cadence
follows the PAR (first tap deferred to t≈`t1st`, subsequent steps ≈`tNext`
apart) instead of the construction default. Locked in by the harness.

---

## 4. Status of the original items

- **R3** (shunt reclosing delay) — DONE.
- **R4** (bus short-circuit capability) — DONE (`_hasShortCircuitCapabilities`
  is now a synonym of `_hasConnection`; node faults already work). Its only
  leftover, a built-in *multi*-fault list, is optional and listed inside R4.
- **R6** (tap-changer-timing runtime test) — DONE (and fixed a dead-code bug).
- **R1, R2** (voltage-dependent / restorative load, voltage-dependent generator)
  — RETIRED: quasi-static / RMS constructs, not classical EMT; belong in
  Modelica-on-ACPIN, not the C++ network (section 2).
- **R5** (physical three-winding transformer) — the only genuinely
  EMT-appropriate item left, and itself optional (the phasor 3WT is a no-op).
  Blocked on extending the DataInterface to expose leg impedances; do it only if
  you want the EMT to exceed the phasor here.

**Bottom line:** the interface-level parity is complete, and every network-
appropriate behavioural item is done. The remaining theoretical work (R5) is an
EMT-fidelity *enhancement* beyond the phasor, not a parity requirement; R1/R2 are
out of scope for the network by physics. So the EMT network can be considered at
parity with — and in places ahead of — the phasor.

---

## 5. DAE EMT formulation — implemented equations vs classical EMTP

This section records the actual equations the EMT models contribute, and contrasts
the **Dynawo formulation** (a continuous-time differential-algebraic system solved
by a general implicit integrator) with the **classical EMTP / Dommel formulation**
(fixed-step trapezoidal *companion models* assembled into a nodal admittance
system). They describe the *same physics*; they differ in how integration and the
unknown set are organised.

### 5.0 The two formulations in one paragraph

- **Dynawo (what we implemented).** Each component contributes a residual
  `f(y, y', t) = 0` and its Jacobian. The unknown vector `y` holds **node abc
  voltages AND branch abc currents** (an augmented / sparse-tableau set, close to
  Modified Nodal Analysis with explicit current states). Inductor/capacitor
  constitutive laws keep their derivatives `der(·)`; the **solver** (here the
  variable-step `SolverTRAP`, trapezoidal; or IDA/BDF) does the time discretisation
  and a Newton iteration per step. No companion model is ever formed.
- **Classical EMTP (Dommel).** Each L and C is first *discretised by the
  trapezoidal rule* into a **companion model** = a fixed conductance in parallel
  with a **history current source** that depends only on the previous step. Branch
  currents are eliminated; the only unknowns are **node voltages**, found from one
  linear nodal solve `G·v(t) = i_hist(t) + i_src(t)` per fixed step `Δt`.

Notation: receptor convention, instantaneous **peak-pu** abc quantities
(`v_k = √2·Vrms·cos(θ − 2πk/3)`), `k = a,b,c`. From the IIDM: `R = r/Zbase`,
`L = x/(Zbase·ω_N)`, `C = b·Zbase/ω_N`, `ω_N = 2π·f_N`. `v_k`/`i_k` below are the
component's node-voltage / branch-current entries in `y`; `ẏ ≡ der(y)`.

### 5.1 Node (bus) — `DYNModelBusEMT`

A bus is a small shunt capacitance `C` (the snubber / physical stray C, plus any
shunt-compensator C) to ground, into which every incident branch injects its
current (KCL), with shunt conductance `G` (loads) and an optional fault
conductance `Gf`:

- **Dynawo residual** (per phase `k`):
  `f_k = Σ_branches(±i_k) − (G_k + Gf_k)·v_k − C·v̇_k = 0`
  i.e. `C·v̇_k = Σi_k − (G_k+Gf_k)·v_k`. Differential whenever `C > 0`; the
  network forces `C ≥ 1e-4` (snubber) so every node stays a (stiff) **index-1**
  differential state — this is the DAE regularisation that replaces EMTP's
  always-present `2C/Δt` term. Switched-off (dead island): `f_k = v_k` (`v=0`).
- **Classical EMTP**: the capacitor becomes `G_C = 2C/Δt` ∥ `I_histC`, and the KCL
  at the node is the nodal row `(G_C + ΣG)·v_k(t) = I_histC + Σ i_branch_hist`.
  The node is always solvable (no snubber needed) because `2C/Δt` is finite by
  construction; a *C-less* node is simply algebraic (a pure conductance row).

### 5.2 Series R–L branch (line, cable, reactor) — `DYNModelLineEMT`

- **Dynawo residual** (current `i_k` is a state):
  `f_k = v_from,k − v_to,k − R·i_k − L·i̇_k = 0`,
  and the branch injects `−i_k` into `from`, `+i_k` into `to` (KCL of 5.1). Open:
  `f_k = i_k` (`i=0`). The current is **differential** (carries `L·di/dt`).
- **Classical EMTP**: trapezoidal companion of the R–L branch,
  `i_k(t) = G_RL·(v_from − v_to) + I_hist`, with `G_RL = 1/(R + 2L/Δt)` and
  `I_hist = (1 + (R − 2L/Δt)·G_RL)... ` carried from `t−Δt`. The current is *not*
  a state — it is back-substituted after the nodal solve.

The contrast is the whole story: **Dynawo keeps `i` as an unknown with a
`der(i)` residual; EMTP eliminates `i` into a node conductance + history source.**

### 5.3 Switch / breaker — `DYNModelSwitchEMT`

- **Dynawo**: closed → `f_k = v1,k − v2,k = 0` (ideal zero-impedance equality, the
  current `i_k` is the unknown injected ±1 into the two nodes); open → `f_k = i_k`.
  An ideal closed switch is thus a pure *algebraic* constraint on `y`.
- **Classical EMTP**: an ideal switch is awkward (zero impedance → singular
  nodal row), so EMTP typically uses a small `R_on`/large `R_off` resistance, or
  node merging. Dynawo's augmented set represents the ideal constraint directly
  (the current unknown absorbs it), which is cleaner for an ideal breaker.

### 5.4 Two-winding transformer (R–L + ratio + phase shift) — `DYNModelTwoWindingsTransformerEMT`

With turns ratio `rT = N1/N2` and phase shift `α` (an abc rotation `R(α)`):

- **Dynawo residual** (primary current `i_k` state):
  `f_k = v_pri,k − rT·[R(α)·v_sec]_k − R·i_k − L·i̇_k = 0`,
  where `[R(α)·v]_k = cosα·v_k + (sinα/√3)·(v_{k+2} − v_{k+1})`; primary gets
  `−i_k`, secondary gets the power-conserving `rT·[R(−α)·i]_k`. Reduces to the R–L
  branch (5.2) for `rT=1, α=0`.
- **Classical EMTP**: an ideal-transformer companion (turns-ratio constraint) in
  series/parallel with the R–L companion of 5.2, again eliminating the current.
  The phase shift is the same `R(α)` rotation; the difference is only the
  companion-vs-DAE treatment of the leakage R–L.

### 5.5 Default load and PQ generator — `DYNModelLoadEMT`, `DYNModelGeneratorEMT`

- **Load** (constant-impedance): no own state; registers a per-phase conductance
  `G = 1/R` into its node (5.1). Pure algebraic shunt — identical in both
  formulations (a conductance row / a `G` companion with no history).
- **PQ generator** (default): a *constant balanced current source*
  `i_k(t) = √2·I·cos(ω_N t + φ − 2πk/3)` injected into the node — the same in both
  formulations (a prescribed current source). (Voltage-dependent / restorative
  variants are deliberately NOT here; see section 2.)

### 5.6 Synchronous machine — `Electrical/EMT/GeneratorSynchronous.mo`

The detailed machine is Modelica, attached to its bus through the abc ACPIN. It is
a **Park (dq0)** model that, unlike the phasor, **keeps the stator electromagnetic
transients** `der(λ)/ω_N`:

- **Park projection of the (EMT) terminal**:
  `u_d = C·Σ_k sin(θ−2πk/3)·v_k`, `u_q = C·Σ_k cos(θ−2πk/3)·v_k`,
  `u_0 = (Σ_k v_k)/√3`; the stator current is the inverse map
  `i_k = Ki·Cdq·[sin(θ−2πk/3)·i_d + cos(θ−2πk/3)·i_q + i_0/√2]` injected into the node.
- **Stator voltage equations WITH transients** (the EMT term the phasor drops):
  `u_d = (Ra+RTfo)·i_d + der(λ_d)/ω_N − ω·λ_q`
  `u_q = (Ra+RTfo)·i_q + der(λ_q)/ω_N + ω·λ_d`
  `u_0 = Ra·i_0 + der(λ_0)/ω_N`
- **Rotor windings** (field `f`, dampers `D,Q1,Q2`), e.g.
  `u_f = Rf·i_f + der(λ_f)/ω_N`, `0 = RD·i_D + der(λ_D)/ω_N`, …
- **Flux–current linkage** (saturable mutuals `Md_sat/Mq_sat`, Canay `Mrc`):
  `λ_d = (Md_sat+Ld+XTfo)·i_d + Md_sat·i_f + Md_sat·i_D`, … `λ_0 = L_0·i_0`.
- **Mechanical / swing** (absolute angle):
  `der(θ) = ω·ω_N`, `2H·der(ω) = cm·PNomTurb/SNom − ce − DPu·(ω − ω_ref)`,
  `ce = λ_q·i_d − λ_d·i_q`.
- **Classical EMTP machine**: the universal-machine / Park model is integrated the
  same way *in principle*, but classically it is interfaced to the network by a
  **Norton/Thevenin companion at the stator terminal** (a predicted internal EMF
  behind a subtransient impedance, updated each fixed step, often with a
  prediction/interpolation of `θ`). Dynawo instead solves the machine fluxes and
  the network abc voltages **simultaneously** as one DAE (the ACPIN couples
  `terminal.v ↔ node v`, `terminal.i ↔ branch/KCL`), so there is no
  predictor/companion lag at the machine–network interface.

### 5.7 Synthesis — what differs

| Aspect | Dynawo EMT (implemented) | Classical EMTP (Dommel) |
|---|---|---|
| Unknowns | node abc voltages **and** branch abc currents (`der(i)`, `der(v)` states) | node voltages only |
| L, C | constitutive `der(·)` kept; solver discretises | trapezoidal **companion** (`G_eq ∥ I_hist`), Δt baked in |
| Time step | **variable**, error-controlled (SolverTRAP / IDA) | **fixed** Δt |
| Per step | Newton on the full nonlinear DAE residual + Jacobian | one linear nodal solve (+ iteration only for nonlinearities) |
| Nonlinearity (saturation) | handled natively by the DAE Newton | piecewise / predictor / interpolation |
| Ideal switch / breaker | exact algebraic constraint (current unknown absorbs it) | needs R_on/R_off or node merge |
| Machine–network coupling | one simultaneous DAE (ACPIN), no interface lag | Norton/Thevenin companion, predicted internal EMF |
| Node solvability | snubber `C ≥ 1e-4` makes every node index-1 differential | `2C/Δt` always present; C-less node stays algebraic |

Both reproduce the same waveforms; the Dynawo DAE form trades EMTP's fixed-step
companion bookkeeping for a solver-agnostic residual system that integrates with a
variable step and resolves the whole network + machine implicitly each step.
