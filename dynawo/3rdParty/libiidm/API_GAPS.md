# iidm-bridge-cpp — API gaps for Dynawo migration

Migration target: replace `powsybl-iidm4cpp` (v1.5.1-rc2) with
[`gautierbureau/iidm4cpp`](https://github.com/gautierbureau/iidm4cpp)
(iidm-bridge-cpp, GraalVM/JNI wrapper over Java `powsybl-core`).

Pinned upstream SHA: `7a277e2bfdb51d55632e33f31b5a927c315cd947`
(includes PR #7 "Add Switch component API (C++, GraalVM, JNI)").

Scope surveyed: `dynawo/sources/Modeler/DataInterface/PowSyblIIDM/` — 28 `.cpp`
files. Of those, 8 are trivial (header-only / no IIDM calls) and 20 require
per-file adaptation. This document lists the remaining **upstream** gaps — APIs
Dynawo needs that iidm-bridge does not yet expose. Dynawo-side changes
(namespace flip `powsybl::iidm::` → `iidm::`, header extension `.hpp` → `.h`,
extension access `findExtension<T>()` → `hasXxx()`/`getXxx()`) are tracked in
the migration branch and are not listed here.

## Gap table

Priority is "H" when the gap blocks more than three Dynawo files, "M" for two
or three, "L" for one.

| #  | Prio | API gap                                                                                                        | Suggested upstream signature                                                                                | Blast radius (Dynawo files)                                                                                  |
|----|------|----------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------|
| 1  | H    | `Terminal::getVoltageLevel()` — required for nominal-V lookups at every connectable                            | `VoltageLevel Terminal::getVoltageLevel() const;`                                                           | DataInterface, Generator, Shunt, TwoWT, FictTwoWT, Line, Battery, Load, SVC                                  |
| 2  | H    | `Switch::getNode1()/getNode2()` — node-breaker topology reconstruction                                         | `int Switch::getNode1() const; int Switch::getNode2() const;`                                               | DataInterface, VoltageLevel                                                                                  |
| 3  | H    | `VoltageLevel::NodeBreakerView::getInternalConnections()` (+ iterator yielding `{node1,node2}` pairs)          | `std::vector<InternalConnection> NodeBreakerView::getInternalConnections() const;`                          | DataInterface, VoltageLevel                                                                                  |
| 4  | M    | `Generator::getRegulatingTerminal()` (or at minimum `getRegulatingTerminalId()` — Dynawo only reads the ID)    | `std::optional<Terminal> Generator::getRegulatingTerminal() const;` or `std::string getRegulatingTerminalId() const;` | Generator, ServiceManager                                                                                    |
| 5  | L    | `StaticVarCompensator::getRegulatingTerminal()` / `…Id()`                                                      | mirror of #4                                                                                                | ServiceManager                                                                                               |
| 6  | M    | `ShuntCompensator::getRegulatingTerminal()` / `…Id()`, plus `isVoltageRegulatorOn()`, `getTargetV()`           | mirror of #4 + two boolean/double getters                                                                   | Shunt, ServiceManager                                                                                        |
| 7  | M    | `HvdcLine::getConverterStation1Id()/getConverterStation2Id()` — Dynawo only needs the IDs                      | `std::string HvdcLine::getConverterStation1Id() const;` (mirror for 2)                                      | DataInterface, HvdcLine                                                                                      |
| 8  | M    | `RatioTapChanger::getRegulationTerminalId()` + `getTargetDeadband()`                                           | `std::string getRegulationTerminalId() const; double getTargetDeadband() const;`                            | DataInterface, RatioTapChanger, FictTwoWT                                                                    |
| 9  | L    | `PhaseTapChanger::getRegulationTerminalId()` + `getTargetDeadband()`                                           | mirror of #8                                                                                                | PhaseTapChanger                                                                                              |
| 10 | L    | `Network::writeXml(path, opts)` + `ExportOptions` + `FakeAnonymizer`                                           | `void Network::writeXml(const std::string& path, const ExportOptions& = {}) const;`                         | DataInterface (optional; Dynawo only writes diagnostic dumps)                                                |
| 11 | H    | Built-in extensions used by Dynawo's custom plugin: `GeneratorActivePowerControl` (Dynawo variant), `StaticVarCompensatorInterface`, `ActiveSeason`, `CurrentLimitsPerSeason` | `has<Ext>()/get<Ext>()` pair on the owning component, mirroring existing `ActivePowerControl` etc.          | IIDMExtensions, IIDMExtensionsTraits                                                                         |
| 12 | M    | **Alternative to #11**: generic back-door to let downstream plugins attach typed attributes without upstream changes — e.g. `Connectable::getAttribute<T>(name)` or an `ExtensionProvider` registry | `template<class T> std::optional<T> Connectable::getAttribute(const std::string&) const;`                   | IIDMExtensions, IIDMExtensionsTraits (removes the need for #11 entirely if adopted)                          |
| 13 | M    | Branch current-limits: `getCurrentLimits()` on `Line`, `TwoWindingsTransformer`, `ThreeWindingsTransformer::Leg`; iteration over temporary limits | `std::optional<CurrentLimits> getCurrentLimits() const;` (mirror on each Leg + both sides of branches)      | CurrentLimit, TwoWT, ThreeWT, FictTwoWT                                                                      |

## Suggested upstream PR ordering

1. **Gap #1** first — it is the single biggest unblock (nine Dynawo files) and
   is a trivial accessor forwarded straight to the Java object.
2. **Gaps #2 and #3** second — they unblock node-breaker topology end-to-end;
   both are straightforward JNI/GraalVM wrappers over existing Java getters.
3. **Gap #11 or #12** third — decide between extending the built-in extension
   list (#11, closed set, stronger typing) or a generic attribute back-door
   (#12, open set, looser typing). Gap #12 would also make future custom
   extensions possible without recompiling iidm-bridge, which matches Dynawo's
   current `DYNAWO_IIDM_EXTENSION` dlopen plugin story.
4. **Gaps #4–#9, #13** afterwards — each is scoped to a single component or
   tap-changer and can land independently.
5. **Gap #10** last — optional; Dynawo only uses `writeXml` for diagnostics and
   can be guarded with a compile-time flag in the meantime.

## Dynawo-side file bucketing

Survey of the 28 files under
`dynawo/sources/Modeler/DataInterface/PowSyblIIDM/`:

### Not actually blocked by any gap (zero `powsybl::iidm::` usage)
- `DYNFictBusInterfaceIIDM.{h,cpp}` — pure Dynawo-side synthetic bus
- `DYNFictVoltageLevelInterfaceIIDM.{h,cpp}` — pure Dynawo-side synthetic VL

### Needs method additions only (non-blocking, trackable per-file)
- `DYNFictTwoWTransformerInterfaceIIDM` — wraps `ThreeWindingsTransformer::Leg`;
  blocked on gaps #1, #8, #13 only
- `DYNServiceManagerInterfaceIIDM` — blocked on gaps #4, #5, #6

### Blocked on larger upstream stories
- `DYNDataInterfaceIIDM` — gaps #1, #2, #3, #7, #8, #10 and the
  extension-plugin story (#11 / #12)
- `DYNIIDMExtensions.{hpp,cpp}` — needs #11 or #12 to have any path forward
- `DYNIIDMExtensionsTraits.{hpp,cpp}` — likewise

### Straight namespace/header migration (no upstream gap)
The remaining ~16 files (Bus, Battery, Load, Line, LccConverter, VscConverter,
Svc, DanglingLine, BusBar, Calculated*, Component*, etc.) require only:
- `powsybl::iidm::` → `iidm::`
- `.hpp` → `.h`
- `findExtension<T>()` → `hasXxx()/getXxx()` where relevant

## Notes on Dynawo's custom-extension plugin

`DYNIIDMExtensions` implements a `dlopen`-based plugin loader (env var
`DYNAWO_IIDM_EXTENSION`) that expects downstream `.so` libraries to export
`create<Name>`/`destroy<Name>` symbols for four Dynawo-specific extensions:

| Dynawo extension                          | Attaches to                       |
|-------------------------------------------|-----------------------------------|
| `StaticVarCompensatorInterfaceIIDMExtension` | `powsybl::iidm::StaticVarCompensator` |
| `ActiveSeasonIIDMExtension`               | `powsybl::iidm::Connectable`      |
| `CurrentLimitsPerSeasonIIDMExtension`     | `powsybl::iidm::Connectable`      |
| `GeneratorActivePowerControlIIDMExtension` | `powsybl::iidm::Generator`       |

iidm-bridge currently exposes the six upstream `powsybl-core` extensions
(`ActivePowerControl`, `CoordinatedReactiveControl`,
`HvdcAngleDroopActivePowerControl`, `HvdcOperatorActivePowerRange`,
`SlackTerminal`, `VoltagePerReactivePowerControl`) via flat
`hasXxx()/getXxx()` pairs on the owning component — but has no registry for
third-party extensions. Gap #11 (add the four Dynawo extensions upstream) or
Gap #12 (generic attribute accessor) is required before any of Dynawo's four
custom extensions can be read from iidm-bridge.
