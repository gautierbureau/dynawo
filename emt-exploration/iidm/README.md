# IIDM-driven init for the EMT SMIB — load flow + findings

> **The runnable cases moved to the NRT suite.** The composite IIDM-driven SMIB is now
> `nrt/data/EMT/EmtCompositeSmib` and its snubber-seeded variant is
> `nrt/data/EMT/EmtCompositeSmibSnubberSeed`; the C++-network seed-correction demos are
> `nrt/data/EMT/EmtNetwork{Svc,Hvdc,DanglingLine,IEEE14}SteadyStateSeed`. What stays
> here is the prototype *tooling* (`solve.py`, `solve_snub.py`, `snub_seed_internal.py`)
> and this findings write-up. Where the text below says "run `smib_iidm.jobs`", the
> case now lives under `nrt/data/EMT/`; the scripts here still regenerate its seeds.

`smib.xiidm` is the SMIB power-system description (gen bus, step-up transformer,
HV bus, line, infinite bus). `solve.py` runs the **pypowsybl** AC load flow
(slack forced at the infinite bus so power flows gen → tfo → line → infinite bus)
and derives the per-node EMT start values; `smib_solved.xiidm` is the saved
solved network.

```
pip install pypowsybl && python3 solve.py
```

## Why this matters

An IIDM already carries a solved load flow — the exact steady operating point the
EMT models need to start flat. The `SynchronousMachine_INIT` is now **IIDM-native**
(receptor power on SnRef, RMS voltage on UNom — Dynawo's universal convention), so
its `P0Pu/Q0Pu/U0Pu/UPhase0` map straight onto the IIDM references
`p_pu/q_pu/v_pu/angle_pu`.

## The boundary finding (see ../INIT_from_IIDM.md)

`<network iidmFile=…>` instantiates the C++ **phasor** network for every IIDM
element not overridden by a dynamic model — so binding only the machine, with the
rest of the grid as parallel EMT black boxes, leaves the IIDM's buses/tfo/line in
the C++ network and over-determines the system (`131 variables ≠ 129 equations`).

The phasor `nrt/.../SMIB_1_StepPm_IIDM/baseCase` shows the resolution: **fully
override** every IIDM element with Modelica black boxes (using `Bus` models as the
nodes), leaving the C++ network empty. A fully-EMT live-IIDM case is therefore
feasible in **pure Modelica** once an EMT `Bus` model (abc node) and per-element
`staticId`/`<reference>` init are added — no C++ EMT network required for
feasibility (that is the later performance/first-class step).

## Live IIDM-driven init — working prototype (Option B)

`smib_iidm.{jobs,dyd,par}` is a **fully-EMT case driven live by the IIDM**, pure
Modelica, no C++ network:

```
./myEnvDynawo.sh jobs smib_iidm.jobs
```

Every IIDM element is overridden by an EMT black box bound with `staticId`, so the
C++ network is left empty (the `baseCase` pattern, in abc):

| IIDM element | EMT override (`staticId`) | init |
|---|---|---|
| generator `SM`   | `GeneratorEmtSynchronous` | **live** `<reference>` `p_pu/q_pu/v_pu/angle_pu` |
| bus `GEN_BUS`    | `EmtBus`                  | **live** `<reference>` `Upu/Theta_pu` |
| bus `HV_BUS`     | `EmtBus`                  | **live** `<reference>` `Upu/Theta_pu` |
| bus `INF_BUS`    | `EmtVoltageSource`        | baked peak/phase (defines the node) |
| transformer `TR` | `EmtTransformerYgYg`      | **live** `<reference>` `r_pu/x_pu` (impedance); endpoint voltages + `I0` from this load flow |
| line `LINE`      | `EmtSeriesRL`             | **live** `<reference>` `r_pu/x_pu` (impedance); endpoint voltages + `I0` from this load flow |

`EmtBus` is the keystone — the abc analogue of `Electrical.Buses.Bus`: a small
shunt-capacitance node whose `UMag0Pu`/`UPhase0` bind to the IIDM bus `Upu`/
`Theta_pu` (note: **bus** origNames are `Upu`/`Theta_pu`; **generator** origNames
are `p_pu/q_pu/v_pu/angle_pu`).

**Result:** the machine and buses start exactly at the load-flow point pulled live
from the IIDM — `machine_UStatorPu(0) = 0.920000` (GEN_BUS `v_pu`),
`machine_PGenPu(0) = 0.600000` (the solved `p_pu`), bus phase-a peaks =
`√2·v·cos(angle)`. This proves live IIDM-driven init for a fully-EMT network in
pure Modelica.

## Referencing r/x from the IIDM (per-unit convention bridge)

The branch impedances are now pulled **live** from the IIDM via `<reference>`
`r_pu`/`x_pu` (like the standard phasor `LinesAndTfosReferences` pattern), so the
EMT model impedances can't drift from the network. There is one subtlety, handled
in `solve.py`:

- pypowsybl/IIDM stores `r`/`x` in **single-line** per-unit (`P = V·I`).
- the EMT abc models use the **three-phase instantaneous** convention
  (`P = Σ vₖiₖ = (3/2)·V̂·Î = 3·Vrms·Irms` — the same `1.5·V̂` factor `solve.py`'s
  `branch_I` uses for the seed currents).

For an identical voltage drop the EMT per-phase current is **1/3** of the
single-line current, so the **EMT per-unit impedance is exactly 3× the single-line
one**. `solve.py` therefore scales the line/transformer `r`/`x` by 3 when it writes
`smib_dynawo.xiidm` (the Dynawo-ready export only — `smib.xiidm`/`smib_solved.xiidm`
keep the physical single-line values). The references then resolve directly to the
EMT-base values (`tfo_XPu=0.3`, `line_XPu=0.6`, `line_RPu=0.06`). The seed is now
KCL-consistent at t=0: machine terminal current = TFO primary current = 0.31 peak.

## Open: ~350 Hz snubber-LC startup ringing (machine init is NOT the cause)

A residual transient remains in the first ~10 ms: `UStator` starts exactly at 0.92
but then swings (≈0.82…1.06) at **~352 Hz** before settling. Investigated to a firm
conclusion:

- **The machine init is a perfect equilibrium.** For the 2-axis `SynchronousMachine`
  used here, all flux derivatives are zero at the seed
  (`der(Phid)=der(Phiq)=der(Phifd)=der(Phiq1)=der(dw)=0`, verified analytically), and
  the dynamic Park projection at t=0 reproduces the init's `Vd/Vq` exactly. The rotor
  (`omega`) stays flat at 1.0 through the swing. Adding the full machine parameter set
  changes nothing (and note: this case uses the *reduced 2-axis* `SynchronousMachine`,
  not the full 6-winding `GeneratorSynchronous` — full-machine params like `LdPu`/`MdPu`
  don't exist on it and are silently ignored).
- **It is snubber-network LC ringing.** ~352 Hz is far too fast for electromechanical
  (~1 Hz) or field (~s) dynamics — it is the resonance of the per-node snubber caps
  (`bus_CPu = 1e-4`) with the line/transformer inductances. The snubber caps are an EMT
  modelling addition the pypowsybl load flow does not contain, so the bare-load-flow
  branch-current seeds (`i = (Vp−Vn)/Z`) omit each node's small snubber shunt current
  (~0.04 pu, ~13% of the 0.31 branch current). That KCL imbalance at t=0 kicks the LC
  modes.

**Fix when we return to it:** seed the branch currents (and node voltages) from a
network solve that **includes the snubber shunt admittances**, so KCL closes exactly
at t=0 — the same approach the smibcmp comparison used (a 2-terminal solve carrying
`Ybus`), which is why they start perfectly flat. This is a `solve.py`-side
computation (augment the load-flow result with the snubber `jωC` shunts, then emit
shunt-consistent branch currents), not a model or machine-init change. Alternatively,
model the snubber caps as IIDM shunt compensators so the load flow itself accounts for
them. The live IIDM-driven init mechanism (v/angle/p/q/r/x all referenced) is exact;
this is the remaining network-consistency step.

## Prototype result — snubber-aware seed (`solve_snub.py`)

`solve_snub.py` implements the second variant of the fix above: before the load
flow it adds, at each `EmtBus` node, a shunt compensator matching the snubber, runs
the LF, then exports `smib_dynawo_snub.xiidm` with the snubber-consistent v/angle
and **strips the shunts back out** so the EMT topology is unchanged.
`smib_iidm_snub.{par,jobs}` reseed every branch/bus from that solve.

**The pu bridge matters — and was the whole story.** The EMT snubber susceptance is
`b_emt = ω·CPu ≈ 0.0314` in the *3-phase-peak* pu the EMT models use. But pypowsybl's
load flow is *single-line* pu, and — exactly like the ×3 that `solve.py` applies to
series `r`/`x`, only **inverted for a shunt** — the single-line susceptance the LF
needs is `b_single = 3·b_emt ≈ 0.0942`. A first cut used `b_emt` directly and
injected only **1/3** of the real snubber current: the branch seeds differed by
`tfo I0[a] − line I0[a] = −0.00196` where the model actually draws `−0.00586`
(ratio 2.99). With the ×3 correction the seeds carry the *full* snubber current
(`tfo I0[a] = 0.30389`, `line I0[a] = 0.30974`, difference `−0.00586`, matching the
model) and KCL closes at t=0.

**Result — first-cycle `UStator` ringing (0–30 ms p2p), see `snub_seed_effect.png`:**

| seed | p2p | vs baseline |
|---|---|---|
| baseline (no snubber in LF) | 0.2466 | — |
| snub, `b_emt` (1/3 current, first cut) | 0.1981 | −20 % |
| **snub, `3·b_emt` (correct pu bridge)** | **0.1232** | **−50 %** |

Getting the pu factor right *doubled* the improvement. The blue trace now sits
tightly around the 0.92 set point on every cycle at the same ~350 Hz mode.

**On the machine init (the "seed the flux derivatives" idea).** It has nothing to
bite on: `SynchronousMachine` is written in the **rotor dq0 frame**, whose states
(`Phid, Phiq, Phi0, Phifd, Phiq1, dw, dtheta`) are *constant* at steady state, and
`SynchronousMachine_INIT` already computes `Theta0` and all four flux starts from
the operating point, so `der(Φ)=0` at t=0 by construction (verified: `omegaPu` is
flat to ±4e-5 through the whole swing). The machine is a true equilibrium; the
residual ring is entirely the network snubber-LC, which the corrected seed halves.
The remaining ~0.12 p2p is the part of the LC transient the seed can't remove
(the seed fixes t=0 KCL and `der(v)`; higher-order consistency of every branch
`der(i)` simultaneously is what a full `Ybus` companion solve would add).

## Usable within Dynawo — internal nodal correction, **no re-LF** (`snub_seed_internal.py`)

`solve_snub.py` proved the effect but is **not** a usable mechanism: it *adds shunt
components to the IIDM and re-runs the AC load flow*, then strips them out. Dynawo
receives an IIDM that already carries a classical, snubber-unaware load flow and
cannot modify the network and re-solve a load flow. So the question is whether the
same seed is reachable from the **classical LF values alone** — and it is.

The snubber current at each node is the analytic quantity `i = jωC·V`, fully known
from the classical LF voltage and the model's own `CPu`. Closing KCL at t=0 is
therefore a small **internal nodal correction**: solve the branch network (`Ybus`)
plus the snubber shunts for the voltage shift that supplies `i_snub`, starting from
the classical LF point — one linear/Newton solve using only data Dynawo already has
at init (network admittances + `CPu`). **No external load flow, no network edit.**

`snub_seed_internal.py` runs exactly that correction on the classical SMIB LF (the
voltages `smib_dynawo.xiidm` carries) and reproduces the re-LF prototype's operating
point **to 5–6 significant figures** — the seeds it prints (`tfo_UpPhaseRad =
0.214979`, `tfo_UsMagPu = 1.299394`, `HV |V| = 0.918810`) are byte-identical to what
`solve_snub.py` obtained by re-running the load flow. Same seed ⇒ same −50 % ring.
This proves the strategy is portable to Dynawo's init.

**Where it lives in Dynawo.** The correction needs the network `Ybus` and the
snubber `CPu` at init time:

- *C++ EMT network* (`DYNModelNetworkEMT`): it already assembles the nodal system,
  so this is its natural home — one linear correction of the LF-seeded node voltages
  with the snubber shunts included, then reseed branch `i0`/node `v` from the
  corrected point. This is "seed from the EMT steady state (snubbers included)"
  rather than "copy the phasor LF verbatim".
- *Modelica per-component case* (this `smib_iidm` bundle): a self-correcting init here
  would be ideal — but **it was tried and Dynawo's init architecture does not allow
  it** (see next section). The correction has to be *computed* (the nodal solve) and
  *supplied as start values*, either offline (reseed the par, as `snub_seed_internal.py`
  does) or by the C++ network init above.

### Tested and ruled out: self-correcting init via Modelica `initial equation`

The tempting "clean" fix was to stop pinning the states and instead impose the
balanced AC steady-state condition `der(x) = -ω·quad(x)` on every storage element
(bus `v`, branch `i`), so the coupled init would reproduce the phasor network *with*
snubbers and land on the corrected point by itself — `quad` of a balanced abc triple
being the algebraic `quad(x)[k] = (x[k+1]-x[k+2])/√3`. It was implemented on `Bus`,
`SeriesRL`, `TransformerYgYg` (opt-in flag), built, and run. **It changed nothing:
byte-identical to baseline.**

Diagnosis (decisive test): the `Bus` `initial equation` was replaced with a blatantly
wrong pin `v = 0.5·U0Pu`, rebuilt, and run — `UStator(0)` was *still* `0.92`, output
still byte-identical. So **Dynawo does not use a dynamic model's `initial equation` to
set its differential-state starts.** Its init reads each state's *start value* (from
the `_INIT` transfer / `start` attribute), runs `solverKINAlgRestoration_` to restore
only the **algebraic** variables holding the differential states fixed, then
`solverKINYPrimInit_` for `y'`. There is no coupled "free the states and re-solve"
step that a per-component steady-state condition could hook into. The experiment was
reverted; the models are back to pinning `v = U0Pu` / `i = I0Pu`.

**Conclusion.** Improving the init from the classical LF *is* possible and worth
~50 % of the startup ring, but it must be done by **computing the snubber nodal
correction and feeding it as start values**, not by per-component init equations. The
right home is the **C++ EMT network init** (`DYNModelNetworkEMT`), which owns the
`Ybus`: run the linear correction from the classical LF there and set the corrected
node-voltage / branch-current starts. `snub_seed_internal.py` is the reference
implementation of that correction; porting it into the network init is the
production step.

**It is *not* a `getY0`/derivative-init change.** Dynawo already makes `y'` consistent
with `y` via `solverKINYPrimInit`; the gap is the `y`-seed. And it is *not* a machine
change — the dq0 machine is already a constant-state equilibrium (above). The lever is
the network init seeding the EMT steady state from the classical LF via the snubber
nodal correction, **using the ×3 single-line↔3-phase shunt bridge**.

Reproduce: `python3 snub_seed_internal.py` (portable, no re-LF) prints the seeds;
`python3 solve_snub.py && ./myEnvDynawo.sh jobs smib_iidm_snub.jobs` runs the case
(committed: `solve_snub.py`, `snub_seed_internal.py`, `smib_iidm_snub.{par,jobs}`,
`smib_dynawo_snub.xiidm`; run outputs `outputs_snub*/` are git-ignored).

### Landed in the C++ network: `steady_state_init_seed` (`DYNModelNetworkEMT`)

The production version of the correction is implemented directly in the C++ EMT
network as an **opt-in PAR flag** `steady_state_init_seed` (default `false`, so every
committed case is byte-identical — the 8 EMT-network NRT cases `nrt/data/EMT/EmtNetwork*`
pass at `maxRelErr ≤ 4e-19`). When on, `ModelNetworkEMT::applySnubberSeedCorrection()` runs
once at `init()`, using only data the network already holds (branch `r/l`, each node's
snubber `CPu` + shunt `G`, and the load-flow bus voltages it now records via
`setLoadFlowVoltage`): it assembles the complex nodal matrix over the **free**
(non-external, live) buses and solves

  `(Y_branch + diag(G_sh + jωC)) · ΔV = -(G_emt + jωC_emt) · V_LF`

(the snubber shunts give every node an admittance to ground, so it is non-singular
with no slack; external-connection buses are dropped from the rows/cols and held at
`V_LF`, so no slack/identity-row handling is needed), then rewrites each internal
node's abc voltage seed and every branch's abc current seed to that snubber-consistent
AC steady state. The matrix is **sparse** (each bus touches only its neighbours) and is
factorised/solved with **KLU (SuiteSparse)** — the same sparse solver Dynawo uses in
its algebraic solvers, via `klu_z_*` for the complex system — so the correction scales
to a real grid rather than the O(n³)/O(n²) a dense solve would cost. New hooks:
`NetworkComponentEMT::initSeedBranch` / `setInitSeedCurrent` (lines + transformers
report `r/l/ratio` and take the corrected current), `ModelBusEMT::phasorLF`.

**Result (IEEE14, `ieee14_ss.{par,jobs}`, `ss_seed_ieee14.png`).** The internal load
buses seed at 0 in the baseline (the robust-meshed-init strategy) and ring *up* to
their operating voltage; with the flag they start at the right value — a **near-flat
start**. First-30 ms `Urms` p2p:

| bus | baseline | `steady_state_init_seed` | change |
|---|---|---|---|
| BUS_9 (internal load) | 1.612 (starts at 0) | 0.042 | **−97 %** |
| BUS_4 (internal load) | 1.292 (starts at 0) | 0.051 | **−96 %** |
| BUS_14 (internal load) | 1.541 | 0.030 | −98 % |
| BUS_1 (gen, held fixed) | 1.023 | 0.022 | −98 % |

Settle p2p is 0.0000 and the settle value matches baseline — same steady state, only
a better start.

**Component coverage (`svc_ss` / `hvdc_ss` / `dangling_ss` (now `nrt/data/EMT/EmtNetwork{Svc,Hvdc,DanglingLine}SteadyStateSeed`)).** All
the phase-4 components are covered, each validated on its toy case (every one settles
to the same steady state — a correct init, not just a quieter one):
- **SVC** — capacitive banks fold into the node `cPu` (already on the diagonal),
  inductive banks are reactor-to-ground branches (stamped by the reactor handling).
  `svc` B1 ring 1.468 → 0.441 (−70 %).
- **HVDC** — modelled as two decoupled PQ current injections; injections **cancel** in
  the `ΔV` form (`I_ext` drops out), so nothing to stamp and the converter buses still
  get their snubber correction. `hvdc` B1 1.218 → 0.910 (−25 %; small toy case, B1 is
  an external terminal).
- **Dangling line** — its fictitious boundary node had no load-flow voltage (stayed 0
  and mis-seeded); it is now seeded by the voltage divider `V_f = V_r·y/(y+g_load)`
  through the dangling branch onto the boundary load. `dangling` boundary node
  `DL1_fict` 0.825 → 0.204 and branch current `DL1_Irms` 0.165 → 0.041 (both −75 %).

Only **three-winding transformers** remain unstamped — the network has no C++ 3WT
model at all (the leg impedances aren't surfaced by the DataInterface; a detailed 3WT
is an external Modelica model on the three buses), so there is nothing to stamp; those
buses fall back to the baseline seed.

**Getting the physical/EMT shunt split right was decisive.** A first cut put the
*total* node shunt (`shuntGPhase + jωcPu`) on both the matrix and the RHS; that
wrongly charged the RHS with physical load-G and physical shunt-C (both already in
the load flow), leaving the load buses at ~0.27–0.39 p2p. The correct RHS carries
**only the EMT-added shunts** — the `1e-4` snubber-C regularisation and the machine-bus
`1e-2` damping G, which the load flow never saw — recorded per bus via `addEmtShunt`
(`emtShuntY`), while the matrix diagonal keeps the *total* shunt. Reactor-to-ground
branches (inductive loads / reactor banks, one end at ground) are now stamped as a
shunt admittance and their `i0` reseeded to `V/(jωL)` instead of 0. With that split
the load buses drop to ~0.03–0.05 p2p (a true flat start).

**Scope / limitation (important).** The correction holds **external-connection buses
fixed** (`hasExternalConnection`): a machine terminal or an infinite bus is driven by
its own external Modelica model that seeds that node to the load flow, and moving it
makes the two inits fight (KINSOL restoration diverges — observed, then fixed by the
fixed-boundary constraint). So the correction reseeds only *internal* nodes and the
branch currents. Consequence: the pure 2-bus SMIB (`smibnet`), whose only ringing node
is the **machine terminal itself**, is a no-op — its ring lives on an external-model
node and must be closed by the machine's own init, not the network. The network fix is
for **meshed grids with internal snubber buses** (IEEE14 and up), where it is a large,
correct win. Enable with `<par type="BOOL" name="steady_state_init_seed" value="true"/>`
in the `Network` set.
