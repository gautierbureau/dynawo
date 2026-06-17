---
name: dynawo-add-model
description: Add a new Modelica model to the Dynawo library and build it. Use when creating, registering, or compiling a new preassembled (black-box) Modelica model — dynamic .mo, initialization _INIT.mo, .extvar external variables, and the PreassembledModels .xml — including the CMakeLists.txt and package.order registration the build silently depends on.
---

# Adding a Modelica model to Dynawo

A preassembled (black-box) model usable in a `.dyd` file via
`lib="<Name>"` requires up to four source files plus three registration
edits. Reference template: `documentation/resources/exampleLibrary/`
(`FrequencyLoad`). Canonical machine examples:
`dynawo/sources/Models/Modelica/Dynawo/Electrical/Machines/OmegaRef/`.

## 1. Files to create

Place model files under the relevant domain directory in
`dynawo/sources/Models/Modelica/Dynawo/` (e.g. `Electrical/Machines/OmegaRef/`).

- **`<Name>.mo`** — dynamic model. Start with the MPL-2.0 header (copy from a
  sibling), `within <Package>;`, `extends` an appropriate base class, end
  with `annotation(preferredView = "text");`. Receptor convention
  (`terminal.i > 0` entering the device).
- **`<Name>_INIT.mo`** — initialization model. Computes `*0` start-value
  parameters from load-flow inputs `P0Pu, Q0Pu, U0Pu, UPhase0`. Typically
  `extends Dynawo.Electrical.Machines.BaseClasses_INIT.BaseGeneratorParameters_INIT`
  (provides those inputs, `u0Pu`, `s0Pu`, `i0Pu`, and the
  receptor→generator convention flip).
- **`<Name>.extvar`** — XML listing external variables (inputs/outputs wired
  to other models). Needed when the model is not square. Common entries:
  `terminal.V.re`, `terminal.V.im`, `omegaRefPu` (or `omegaRefPu.value`),
  `switchOffSignal1/2/3.value`. Mark optional ones `optional="true"`.
- **`PreassembledModels/<Name>.xml`** — binds dynamic + init model:
  ```xml
  <dyn:dynamicModelsArchitecture xmlns:dyn="http://www.rte-france.com/dynawo">
    <dyn:modelicaModel id="<Name>">
      <dyn:unitDynamicModel id="<unit>" name="Dynawo.<Pkg>.<Name>"
                            initName="Dynawo.<Pkg>.<Name>_INIT"/>
    </dyn:modelicaModel>
  </dyn:dynamicModelsArchitecture>
  ```

## 2. Registration (the build ignores unlisted files)

- Add `<Name>.mo`, `<Name>_INIT.mo`, `<Name>.extvar` to the **`MODEL_FILES`**
  list in that directory's `CMakeLists.txt`.
- Add `<Name>` and `<Name>_INIT` to that directory's `package.order`.
- Add `<Name>.xml` to `PreassembledModels/CMakeLists.txt`.

## 3. Build and verify

```sh
./myEnvDynawo.sh build-minimal          # once, if not already built
./myEnvDynawo.sh clean-build-models <Name>
```

Success prints `Built target <Name>`. Artifacts land in
`install/.../shared/dynawo/ddb/<Name>.so` and `<Name>.desc.xml`.
After editing model source, re-run `clean-build-models <Name>` (it discards
stale generated code under `build/.../M/M/P/<Name>*`).

Inspect the compiled model's parameters/variables:
`dynawo dump-model -m <Name>.so -o <Name>.desc.xml`.

## Notes

- OmegaRef machines: input `omegaRefPu`, output `omegaPu`; the C++
  `DYNModelOmegaRef` computes the synchronous-frequency barycenter.
- If unsure which base class to extend, read siblings in the target
  directory — e.g. `BaseGeneratorSimplified`, `BaseGeneratorSynchronous`.
