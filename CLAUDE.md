# CLAUDE.md

Guidance for working in the Dynawo repository.

## What this is

Dynawo — a hybrid C++/Modelica time-domain simulation tool for power system
stability. The C++ core handles simulation, solvers, and I/O; physical
component behavior is written in Modelica and compiled to shared libraries.

## Repository layout

- `dynawo/sources/` — Dynawo C++ code: `API`, `Common`, `Launcher`,
  `Modeler`, `ModelicaCompiler`, `Models`, `RT`, `Simulation`, `Solvers`.
- `dynawo/sources/Models/CPP/` — C++ models (e.g. `DYNModelOmegaRef`, the
  network model).
- `dynawo/sources/Models/Modelica/Dynawo/` — the Modelica model library,
  organized by domain: `Electrical/Machines`, `Electrical/Loads`,
  `Electrical/HVDC`, `Connectors`, `NonElectrical`, etc.
- `dynawo/sources/Models/Modelica/PreassembledModels/` — one `.xml` per
  preassembled (black-box) model, plus `CMakeLists.txt` listing them.
- `dynawo/3rdParty/` — toolchains to download/patch/build external libraries.
- `nrt/data/` — non-regression test cases (also serve as usage examples).
- `examples/` — DynaFlow / DynaSwing / DynaWaltz functional test cases.
- `documentation/` — LaTeX docs. `advancedDoc/advancedDoc.tex` covers adding
  models; `documentation/resources/exampleLibrary/` is the canonical
  new-model template (`FrequencyLoad`).
- `util/envDynawo.sh` — the build/run entry point.

## Building (this web environment)

See `BUILD_NOTES.md` for full setup. Short version: run everything through
`./myEnvDynawo.sh <command>` (committed wrapper around `util/envDynawo.sh`).

- `./myEnvDynawo.sh build-minimal` — C++ core + C++ models + solvers (run
  once; does NOT build Modelica preassembled models).
- `./myEnvDynawo.sh clean-build-models <Name> ...` — clean + rebuild one or
  more preassembled models. **This is the iterative model-dev command.**
- `./myEnvDynawo.sh build-dynawo-target <target>` — build one CMake target.
- `./myEnvDynawo.sh build-dynawo-models` — build all preassembled models.
- `./myEnvDynawo.sh list-models` — list valid preassembled model names.
- `./myEnvDynawo.sh jobs <case>.jobs` — run a simulation.

Key constraint: `adept-2.1.1.tar.gz` must be present in
`dynawo/3rdParty/adept/` before `build-minimal` — the network policy blocks
its only host. Details in `BUILD_NOTES.md`.

## How a Modelica model is structured

A dynamic model that needs initialization from load flow is made of:

1. `<Name>.mo` — dynamic behavior. `extends` a base class; uses the
   **receptor convention** (`i > 0` entering the device).
2. `<Name>_INIT.mo` — initialization model: computes start values
   (`*0`-suffixed parameters) from `P0Pu, Q0Pu, U0Pu, UPhase0`. Usually
   `extends` a base such as `BaseClasses_INIT.BaseGeneratorParameters_INIT`.
3. `<Name>.extvar` — XML declaring external variables (inputs/outputs wired
   to other models, e.g. `omegaRefPu`, `terminal.V.re/im`, switch-off signals).
   Only needed if the model is not square (has pending equations).
4. `PreassembledModels/<Name>.xml` — binds the dynamic model to its init
   model via `<dyn:unitDynamicModel name="..." initName="..."/>`.

Registration (easy to forget — the build silently ignores unlisted files):
- Add `.mo`/`.extvar` to the directory's `CMakeLists.txt` `MODEL_FILES`.
- Add the model name to that directory's `package.order`.
- Add `<Name>.xml` to `PreassembledModels/CMakeLists.txt`.

OmegaRef-coupled machines (`Electrical/Machines/OmegaRef/`) consume an input
`omegaRefPu` and expose an output `omegaPu`; the C++ `DYNModelOmegaRef`
computes the frequency barycenter across synchronous machines.

## Conventions

- Every source file starts with the MPL-2.0 copyright header (copy from a
  sibling file).
- `.mo` files end with `annotation(preferredView = "text");`.
- Don't commit build artifacts: `build/`, `install/`, `OpenModelica/`,
  `myEnvDynawo.sh`, downloaded 3rd-party tarballs are git-ignored
  (`myEnvDynawo.sh` was force-added intentionally).

## Testing a model

Build the model(s), then run an NRT or example `.jobs` case and compare
`outputs/curves/curves.csv` against `reference/outputs/curves/curves.csv`
**aligned by the time column** (variable-step solver → differing row counts).
NRT cases live under `nrt/data/`.
