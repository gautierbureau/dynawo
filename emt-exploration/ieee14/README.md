# IEEE14 with detailed EMT machines (drop-in) — findings

> The runnable case moved to `nrt/data/EMT/EmtNetworkIEEE14` (and its variants
> `EmtNetworkIEEE14Fault`, `…LineDisconnection`, `…SteadyStateSeed`). What remains in
> this folder is this findings write-up plus the `steady_state_init_seed` before/after
> plots `ss_seed_ieee14.png` (RMS envelope) and `ss_seed_ieee14_abc.png` (abc waveforms),
> which show the internal load buses starting at their operating voltage instead of 0.

The case runs the IEEE14 IIDM through the C++ EMT network (DYNModelNetworkEMT) with a
**detailed EMT synchronous machine dropped onto every generator bus** -- the phase-5
external-machine connection in action.

## Setup
- `ieee14.dyd`: one `GeneratorEmtSynchronousMachineFull` per generator
  (buses 1, 2, 3, 6, 8), each connected `machine_terminal <-> @<staticId>@@NODE@_ACPIN`.
  No OMEGA_REF -- the EMT machine runs in the absolute frame.
- `ieee14.par`: each machine carries its SNom (matching the phasor IEEE14 case)
  and pulls its operating point from the IIDM load flow by `<reference>`
  (`machine_P0Pu/Q0Pu/U0Pu/UPhase0` <- IIDM `p_pu/q_pu/v_pu/angle_pu`); the
  reference feeds the `SynchronousMachineFull_INIT` model, which computes the
  machine start values. Other physical parameters keep the model defaults.
- the network skips its own internal model for a generator that carries a
  dynamic model (`hasDynamicModel`) and exposes that bus as `@...@@NODE@_ACPIN`.

## Result
The whole 14-bus, two-voltage-level (69 kV / 13.8 kV) grid runs end to end and
holds its operating point:
- bus voltages match the load flow to ~1-2 % (BUS_1 1.062, BUS_4 1.031 vs 1.018,
  BUS_9 1.057 vs 1.056);
- every machine sits at its set point (GEN_1 PGenPu = 2.34 = 232 MW, omega ~ 1.0,
  drift < 2e-4 over 0.5 s -- the small snubber-damping draw, as on smibfull/smibnet);
- the machine buses flat-start at their load-flow voltage; the regular load buses
  ramp from 0 to their load-flow voltage within ~20 ms (the surgical flat-start
  keeps the global init robust on the meshed grid), then hold steady.
- 0.5 s simulates in ~3 s wall time.

## What made it work (network-model fixes, this branch)
- **transformer per-unit ratio**: couple the two voltage levels by the per-unit
  ratio (1 for a nominal transformer), not the physical turns ratio -- without
  this the 13.8 kV buses sat at 0.37 / 1.85 pu.
- **reactive loads**: the load draws Q0 as well as P0, so the grid can reproduce
  its load flow (IEEE14 has ~80 MVAr of load Q).
- **current-base bridge + terminal snubber**: the machine's power-invariant Park
  current is bridged to the network's standard abc convention (factor 3) and the
  machine bus carries a small snubber C + damping G.

## Next
- exercise the basic events (line disconnect, fault) on top of this base case;
- a detailed AVR/governor (the EMT machine here has fixed excitation / mechanical
  power) for voltage/frequency regulation under disturbance.
