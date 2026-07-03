# EMT exploration for Dynawo

This directory collects material for exploring **electromagnetic-transient (EMT)
/ three-phase instantaneous** simulation in (and alongside) Dynawo, which is
otherwise a **phasor / RMS quasi-static** tool built on a full DAE formulation.

## Where the cases went

The validated, runnable EMT cases that used to live here have been **promoted into
the standard NRT suite** under [`nrt/data/EMT/`](../nrt/data/EMT) (grouped by a single
aggregating `cases.py`), each with a committed `reference/outputs/curves/curves.csv`.
That is now the home for the EMT non-regression cases — the network cases
(`EmtNetwork*`, on the C++ `DYNModelNetworkEMT`), the Modelica black-box / example
cases (`EmtSMIB*`, `EmtBreakerOpening`, `EmtFaultAndClearing`, `EmtTransformer*`, …),
the composite IIDM-driven SMIB (`EmtComposite*`), and the `steady_state_init_seed`
variants that lock in the snubber-consistent init correction.

What remains here is genuine **investigation material**: design/findings write-ups,
the snubber-seed prototype tooling, seed-correction demo plots, and the cases that
were not promoted (ones that don't run cleanly, curve-only duplicates, or the
phasor-side comparison baseline).

## Contents

- `FINDINGS.md` — the exploration write-up: how EMT formulation differs from
  Dynawo's phasor-DAE, what Dynawo already has that is relevant (the trapezoidal
  `TRAP` solver, line models), the concrete blockers, and a staged plan.
- `ARCHITECTURE_NOTES.md`, `NETWORK_EMT_design.md`, `EMT_NETWORK_PARITY.md` — the
  C++ abc network model design and its parity/remaining-work tracker vs the phasor
  network.
- `INIT.md`, `INIT_from_IIDM.md`, `CONTROL_REUSE.md` — init-from-load-flow and
  phasor-control-reuse notes.
- `EMT_MISSING_MODELS.md` — catalog of classical EMT models not yet in Dynawo.
- `EMT_methods.tex` — standalone LaTeX paper: the Dynawo DAE EMT formulation vs the
  classical EMTP companion/nodal method.
- `iidm/` — the snubber-consistent init-seed prototype tooling (`solve.py`,
  `solve_snub.py`, `snub_seed_internal.py`) with the solved `.xiidm` operating points,
  its README, and the ringing-comparison plot.
- `ieee14/` — the `steady_state_init_seed` before/after plots (`ss_seed_ieee14*.png`).

The runnable cases themselves now live under `../nrt/data/EMT/`; only the design
notes, prototype tooling, and plots remain here.
