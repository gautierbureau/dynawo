# EMT initialisation infrastructure

How to start an EMT network in exact steady state (no pre-fault settling), and
the reusable pieces that make it repeatable for future cases.

## Why EMT init is different

At steady state the machine dq flux linkages are **constant**, but every abc
network quantity (capacitor voltages, inductor currents, source voltages) is a
**50 Hz sinusoid** — its derivative is not zero. So a proper EMT init cannot
just impose `der = 0`; it needs the steady-state **phasor (load-flow) solution**
of the whole network, evaluated at `t = 0` for each abc state.

## The two reusable building blocks

1. **`Dynawo.Electrical.EMT.SynchronousMachine_INIT`** — a Dynawo-style `_INIT`
   model (uses `Modelica.ComplexMath`, no MSL electrical). Inputs: the stator
   terminal operating point `P0Pu, Q0Pu, U0Pu, UPhase0` (peak phasor). It solves
   the salient-pole generator equations for the start values
   `Phid0Pu, Phiq0Pu, Phifd0Pu, Phiq10Pu, Theta0, Ifd0, Pm`, and also emits the
   stator **output-current phasor** `IgMagPu, IgAngleRad` — the start of the
   network propagation. In a dyd-assembled case it binds to the dynamic machine
   via `initName`.

2. **`Dynawo.Electrical.EMT.Functions.balancedAbcInit(magPu, angleRad)`** — turns
   a balanced positive-sequence phasor into the instantaneous `{a, b, c}` values
   at `t = 0`. Use it to set every capacitor `U0Pu` and every inductor/line/
   transformer `I0Pu` from its load-flow phasor — no triplet is hand-written.

## Recipe (as used by `Examples.SMIBLoaded`)

1. Pick the machine operating point; get the machine fluxes/angle/Pm and the
   terminal voltage & output-current phasors (`SynchronousMachine_INIT`, or the
   equivalent forward calc).
2. **Propagate** the phasors through the network with ordinary complex circuit
   relations (series `Z = R + jX`, shunt `Y = jωC` or `1/R`, transformer ratio):
   machine terminal → caps/dampers → transformer → caps/dampers → line → bus.
   This yields the phasor at every node/branch **and** the consistent infinite-
   bus voltage (it is *determined by* the operating point, not free).
3. Set each state from its phasor:
   - caps: `U0Pu = Functions.balancedAbcInit(Vmag, Vangle)`
   - inductors / line / transformer: `I0Pu = Functions.balancedAbcInit(Imag, Iangle)`
   - infinite bus: `UPeakPu = Vmag`, `Phase0 = Vangle`
   - machine: the `*0Pu` flux/angle/Pm start values.

Result: rotor speed `1.000000`, angle constant, torque `= Pm` until the fault —
no settling. The propagation in step 2 is currently done alongside (a few lines
of complex arithmetic); folding it into a Modelica network-`_INIT` that emits all
the phasors is the natural next step, and `SynchronousMachine_INIT` already
provides the chain-start (`IgMagPu/IgAngleRad`).
