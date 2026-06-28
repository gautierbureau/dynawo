# Reusing the phasor control library on the EMT machine

The EMT `SynchronousMachine` exposes the same **signal interface** as the phasor
`GeneratorSynchronous`, so the existing `Controls.Machines` models (voltage
regulators, governors, power-system stabilisers) can be bundled onto it unchanged
through a preassembled model -- no EMT-specific control code.

## The interface

Inputs the machine consumes:
- `efdPu` — field voltage (normalised so `Efd = Ifd` at steady state)
- `PmPu`  — mechanical power

Outputs the controls measure:
- `UStatorPu` — stator terminal voltage magnitude (RMS pu)
- `omegaPu`   — rotor speed
- `IRotorPu`  — field (rotor) current — **needed by rotor-current exciters** (St3a, St4b, AC/ST families)
- `PGenPu`    — generated active power — **needed by power-feedback governors and PSSs** (GovSteam*, Pss1aPGen, Pss2b)

`SynchronousMachine_INIT` emits the matching steady seeds `Efd0Pu`, `UStator0Pu`,
`Pm0Pu`, `IRotor0Pu`, `PGen0Pu`, which the control `_INIT` companions consume via
`initConnect` (bumpless start), exactly as in the phasor generator bundles.

## Bundles provided

| Preassembled | Controls (reused phasor models) | Signals used |
|---|---|---|
| `GeneratorEmtSynchronousGoverPropVRPropInt`    | `VRProportionalIntegral` + `GoverProportional` | `UStatorPu`, `omegaPu` |
| `GeneratorEmtSynchronousGoverPropVRPropIntPss1aOmega` | + `Pss1aOmega` (speed-input PSS, tuned) | adds `omegaPu` → PSS → `voltageRegulator.deltaUsRefPu` |

`GeneratorEmtSynchronous` is the uncontrolled bundle (machine +
`FixedExcitationMechanicalPower`, the constant field/power feed). All three wire
the controls with `initConnect`/`connect` identical to the phasor bundles
(e.g. `GeneratorSynchronousFourWindingsGovSteam1St4bPss2b.xml`).

## What works, and two honest caveats

- **Slow regulators reuse cleanly.** The PI voltage regulator and the droop
  governor bundle on, initialise bumplessly, and behave correctly through the
  SLG fault (AVR field to ceiling, governor trims `Pm`). See
  `examples/ControlledSMIB`.
- **The fast PSS, once tuned, demonstrably adds damping.** A PSS only helps when
  its gain/sign and lead-lag phase compensation match the machine's
  electromechanical mode — with untuned gains it does **not** improve (and can
  worsen) the swing. `GeneratorEmtSynchronousGoverPropVRPropIntPss1aOmega` was tuned for this machine and now
  damps the rotor oscillation cleanly:
  - Scenario (`examples/ControlledSMIBPss`, NRT `EmtControlledSMIBPss`): a
    high-gain fast AVR (`Gain = 100`, `tIntegral = 0.5`) erodes the damping of the
    ~1.25 Hz local mode (the textbook negative-damping-from-fast-excitation
    setup); a mild **balanced** three-phase fault (0.40–0.45 s) excites the mode.
  - Tuning: two identical lead-lag stages `t1 = t3 = 0.12 s`, `t2 = t4 = 0.02 s`
    (≈ +72° to compensate the exciter+field lag at 7.9 rad/s), gain `Ks = 4`,
    washout `t5 = 3 s`, output limit ±0.1 pu. Positive `Ks` damps; negative `Ks`
    de-damps — the sign and phase were swept on `.par` (no recompile) to the
    optimum.
  - Result: post-fault settling energy (∫|Δδ| over 1–5 s) **−61 %** and peak rotor
    swing **−56 %** versus the same case with the PSS removed, with **no**
    steady-state offset (bumpless). See `examples/ControlledSMIBPss/pss_compare.png`.
- **The speed input sidesteps the EMT-ripple caveat.** `PGenPu` and `UStatorPu`
  are true *instantaneous* quantities, so under **unbalanced** conditions they
  carry second-harmonic ripple the phasor controls were not written for. The PSS
  here therefore uses **`Pss1aOmega`** — the rotor speed is a single mechanical
  state, ripple-free regardless of balance — which is also the textbook
  delta-omega stabilizer. (A `PGenPu`-input PSS still binds and is fine under
  balanced disturbances, but would want a filter / positive-sequence extraction
  under unbalance.)

Net: the architecture lets the whole phasor control library bind to the EMT
machine, and a fast control (the PSS) was tuned to a measurable damping
improvement on it — confirming the reuse is genuine, not just structural.
