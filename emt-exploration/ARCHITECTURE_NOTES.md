# EMT work — open architectural threads

## Next TODOs (recorded)
1. INIT models for Modelica EMT components (IN PROGRESS -- see emt-exploration/init):
   the INIT models exist but the preassembled wrappers don't bind them, and binding
   reveals a dynamic+INIT var/eq imbalance to fix. Hand-crafted init values are not a
   long-term solution.
2. Line fault (the bus fault is done).
3. InfiniteBus as a Modelica model (vs synthesised from the generator in the C++ net).
4. Reproduce a phasor SMIB NRT case in full Modelica EMT.


## Convention: switch-off / disconnection capability
Every EMT Modelica model whose phasor counterpart can be switched off must gain the
SAME capability the SAME way -- by extending the matching phasor switch-off base
class (`Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOff{Line,Transformer,
Generator,Load,Shunt,...}`). Those bases are domain-agnostic (Boolean `BPin`
signals + a `running` flag), so an EMT model extends them directly -- no EMT-specific
base is needed. The model then gates its constitutive law on `running.value`
(`if running.value then [normal abc equations] else i = 0`), exactly as the phasor
Line/Transformer do. The two `switchOffSignal{1,2}.value` are declared `discrete`
(defaultValue=false) in the model's `.extvar`, so they default to closed and an
external event (`EventSetPointBoolean` -> `<unit>_switchOffSignal2`) or a node's
disconnection (`switchOffSignal1`) can trip it. DONE (branch models): `SeriesRL`, `LineFault` (validated: `iidm-network/linefault_trip`
opens the line at t=0.2 -> both terminal currents go to 0, the islanded load bus B1
collapses to 0), and `Inductor`, `CoupledRL`, `TransformerYgYg`, `TransformerYgD`
(the transformers extend `SwitchOffTransformer` and open both windings; `Inductor`/
`CoupledRL` extend `SwitchOffLine`). All transparent when running: `smibfull`/`smiblf`
unchanged (0.9319 flat start) and `ieee14` still runs, because the signals default to
closed.

DONE (shunts): `Resistor`, `Capacitor` extend `SwitchOffShunt`; open (i = 0) when not
running. DONE (machines, interface): `SynchronousMachineFull`, `GeneratorSynchronous`
(extend `SwitchOffGenerator`) and `SynchronousMachine` (extends `SwitchOffGenerator`)
gate their stator coupling on `running.value` -- when off the stator is open
(id = iq = i0 = 0, terminal.i = 0) and the field/damper/rotor dynamics continue.
All transparent when running: `smibfull` (Resistor + Capacitor + Full machine) is
unchanged at 0.9319, and `ieee14` (5 detailed machines + transformers) still runs,
because the signals default to closed. All three machines and both shunts compile
(EmtResistor, EmtCapacitor, GeneratorEmtSynchronousMachineFull, GeneratorEmtSynchronous,
GeneratorEmtSynchronousGoverPropVRPropInt rebuilt).

CAVEAT (machine HARD trip): an instantaneous machine disconnection (running -> false
while loaded) hits a real EMT barrier -- forcing the stator currents to zero
over-constrains the machine FLUX states (lambda = L*i; the fluxes cannot jump), so the
algebraic restoration at the event fails (KINSOL). This is the synchronous-machine
analogue of interrupting an inductor current. In EMT a machine is properly disconnected
by a SERIES breaker opening at a current zero (a Breaker between the stator terminal and
the bus), not by the machine model zeroing its own stator current; the machine's
`switchOffSignal1` (node disconnection) is for propagating a dead bus, where the current
is already ~0. The switch-off INTERFACE is in place and transparent when running; a clean
loaded trip needs the series-breaker path. Same class as the line-fault-clearing and
bus-fault-clearing caveats above.

Running notes on differences between the new EMT path and the phasor reference,
and what still needs porting. Captured from review discussion.

## 1. Bus fault (done, in full Modelica EMT)
The bus fault for SMIB tests is the Modelica `Dynawo.Electrical.EMT.Fault` (a
per-phase shunt resistance to ground, active over [tBegin, tEnd], with
single-line / two-line / three-phase selection). It is already wired into
`emt-exploration/machine/smibfull` (the full-Modelica-EMT SMIB: machine + EMT
grid + EMT Fault), connected to the transformer-secondary node.

Verified response (single-line-to-ground, tBegin=0.4, tEnd=0.5, RfPu=0.005):
flat start (omega=1.0, UStator=0.914) -> fault (UStator dips to 0.34, machine
accelerates to omega=1.0025, field current spikes to 1.15) -> clear -> the rotor
swings and damps back to steady state by ~2.5 s. Physically correct.

This is the analogue of the phasor bus `Fault.mo` used in the phasor SMIB NRT
cases, now in full EMT.

## 2. Bus fault on the C++ network (DYNModelNetworkEMT) — DONE (native), incl. unbalanced
Implemented as a NATIVE C++ bus fault (no connector, the network's own standard
abc convention, so no per-unit bridge): a per-phase shunt-to-ground on a chosen
bus, switched over [tBegin, tEnd], read from the network PAR
(`fault_bus / fault_RfPu / fault_tBegin / fault_tEnd / fault_faultedPhase_{0,1,2}_`).
Supports UNBALANCED faults via the per-phase select. Demonstrated in
`emt-exploration/machine/smibnetflt` (single-line-to-ground: |i_a|=2.84 vs
|i_b|=0.65/|i_c|=0.37, 2-omega RMS ripple) — see SMIBNETFLT_STATUS.md.

Chose the native route over the Modelica `Fault` via ACPIN because the latter
hits a convention clash: the machine's terminal current is power-invariant Park
(Cdq = sqrt(2)/3), bridged to standard abc by the factor-3 term in the bus KCL;
the `Fault`'s Ohm's-law current is already standard -> it must NOT get the
bridge, but the bridge sits on the summed FLOW. (Long-term, move the factor-3 to
the machine side so any model can attach to any bus uniformly — see thread 5.)

REFINEMENT: fault CLEARING (finite tEnd) abruptly interrupts the inductive fault
current against the tiny terminal snubber C -> a ~6 pu switching overvoltage.
Needs a breaker/commutation model (open at current zero) or a larger damped
terminal snubber. The demo uses a sustained fault to show inception cleanly.

## 2b. Line fault (Modelica model + C++ bridge for real networks) — DONE
A fault at a point ALONG a line, the EMT counterpart of the phasor
`Events.LineFault`. `Dynawo.Electrical.EMT.LineFault` is a Modelica composite:
two `SeriesRL` segments (D and 1-D of the line) with a `Fault` shunt-to-ground at
the junction, plus a small junction snubber `Capacitor`+`Ground` (the abc node
regularisation) so fault CLEARING is well-posed -- without the snubber, removing
the shunt from an inductor-only junction leaves the node voltage undefined at the
switching instant. Exposed as preassembled `EmtLineFault`; `terminal1/terminal2`
external. Validated all-Modelica in `emt-exploration/machine/smiblf` (smibfull with
the LINE replaced by a mid-line SLG fault: inception at 0.4, clearing at 0.5,
machine swings and recovers).

To work on REAL (IIDM) networks the C++ network has the bridge the phasor network
uses: in `DYNModelNetworkEMT`, a line carrying its own dynamic model
(`line->hasDynamicModel()`) is NOT built as an internal `ModelLineEMT` -- the
external Modelica `LineFault` replaces the branch and drives BOTH end buses through
their ACPIN connectors (each end bus is exposed for the external connection, seeded
to its load-flow voltage; no machine damping G). Validated in
`emt-exploration/iidm-network/linefault` (an `EmtLineFault` bound by staticId to the
IIDM `LINE`: the faulted-phase B1 voltage collapses to ~0 during the SLG fault and
recovers to its 1.0 pu peak after clearing).

The state-forwarding half of the phasor `NetworkBridgeQuadripole` is also ported:
`BridgedLineEMT` (a network component with NO electrical contribution -- the external
model carries the current) holds the bridged line's connection-state z, links its two
end buses in the connectivity graph while CLOSED, and on a change reports
`NC_TOPO_CHANGE` so the network recomputes its islands. The state is exposed as the
usual connectable `<lineId>_state_value` (structured `<id>_state{value}`), so an
external event drives the trip exactly like a real line: validated in
`iidm-network/linefault_disc` (an `EventQuadripoleDisconnection` connected to
`LINE_state_value` opens the bridged line at t=0.2; the topology change fires and the
run completes). NOTE: two effects are bounded by the rest of the design -- (a) the
actual current interruption is the Modelica `LineFault`'s job, and the EMT `LineFault`
does not yet carry a switch-off (so opening only the bridge state does not by itself
zero the current); (b) switching off a resulting unreferenced island is moot here
because every EMT node carries a snubber C and is therefore always "referenced". The
bridge is the correct, complete network-side mechanism; giving the Modelica
`LineFault` an electrical switch-off is the remaining piece for a fully observable trip.

## 3. InfiniteBus: a Modelica model, NOT synthesised by the C++ network — DONE
Originally the C++ EMT network turned every voltage-regulating generator into an
internal `ModelInfiniteBusEMT` (it replaced the generator's bus). The phasor world
does NOT do this: the infinite bus is a *separate Modelica model* (`InfiniteBus`)
bound to a bus, the network stays passive. The EMT network now matches.

**Modelica side:** `Dynawo.Electrical.EMT.InfiniteBus` is the EMT counterpart of the
phasor `Buses.InfiniteBus` — a *single-terminal* model: a balanced abc EMF
`e[k] = sqrt(2)*UPu*cos(2*pi*FNom*time + UPhase + shift[k])` behind a small series
resistance `RsPu`, so `terminal.i[k] = (terminal.v[k] - e[k]) / RsPu`. `UPu` keeps
its phasor meaning (RMS module pu), `UPhase` the angle. The tiny `RsPu` (default
1e-3, the abc analogue of the phasor `InfiniteBusWithImpedance`) lets it attach to
ANY node — inductive (terminal.v ≈ e) or capacitive (the node's own state, driven
toward e) — whereas an ideal source fixing a capacitive node's voltage is index-
singular. `terminal.v` is external (`InfiniteBus.extvar`). Neutral is ground, so no
separate ground component. Exposed as preassembled `EmtInfiniteBus`. Validated on
`smibinf.*` (smibfull with INFBUS as a single `EmtInfiniteBus`): same physics as the
`EmtVoltageSource`+ground version (t=0 |i| 0.9319, omega within 3e-4, identical
fault response).

**C++ side:** `ModelInfiniteBusEMT` is deleted. The network NEVER imposes a node
voltage. Generator handling:
  - a generator bound to an external dynamic model (machine OR InfiniteBus) →
    its bus is exposed for the ACPIN connection (pass 1, `hasDynamicModel()`);
  - a non-regulating generator with no dynamic model → constant (PQ) current
    injection (pass 3b);
  - a voltage-regulating generator with no dynamic model → nothing; the user
    attaches a Modelica `EmtInfiniteBus` to its bus (as in the phasor world).
The internal 0 V ground (shunt-reactor / inductive-SVC return) is now a dedicated
`ModelGroundEMT` node, not an infinite bus with uPeak = 0. Validated: `ieee14` (all
detailed machines, unaffected) and `smibnet` (its B1 voltage-regulating generator
re-modelled as an `EmtInfiniteBus` bound by staticId → a proper SMIB: B0 at 1.0 pu,
omega synchronous, PGen 0.495). The `iidm-network/*` pass-validation cases were
migrated the same way (an `EmtInfiniteBus` on the former slack bus); 11/12 run.
KNOWN LIMITATION: `rlc_sw` (a *closed* switch enforcing `v(B1)=v(B2)` plus an
islanded bus behind an open switch) fails the init algebraic restoration with the
external InfiniteBus — a structural index issue of that topology, independent of
the source impedance value.

## 4. Reproduce a classical phasor SMIB NRT in full Modelica EMT — DONE
A full-Modelica EMT SMIB (machine + InfiniteBus + transformer + line + bus Fault)
is compared one-for-one against the phasor model on the SAME machine (internal-
parameter 6-winding, reusing `GeneratorSynchronousInt_INIT`), grid, operating point
and fault -- only the domain differs. Two preassembled twins were added for this:
`GeneratorSynchronousFixedInternalParameters` (phasor `OmegaRef.GeneratorSynchronous`
+ `FixedExcitationMechanicalPower`) and the symmetric EMT
`GeneratorEmtSynchronousProportionalRegulationsInternalParameters`. Cases and the full
result table are in `emt-exploration/smibcmp/` (`smibcmp_emt` vs `smibcmp_ph`).

Result: across a balanced bus fault [0.4, 0.5] s, the rotor-speed swing of the EMT
SMIB tracks the phasor SMIB to within **~0.0017 pu (0.17 %)** -- same flat start, same
acceleration (~+0.6-0.7 %), same swing period and damping. The residual is exactly the
EMT content the phasor model omits (stator electromagnetic transients + the 50 Hz abc
ripple at switching). The slow electromechanical envelope matches the phasor reference.
The phasor NRT `nrt/data/SMIB/SMIB_Nordic/SMIB_PmConstVRNordic` was also run as an
independent phasor SMIB reference (omega peaks +0.51 % at clearing, damps over ~10 s --
the same qualitative swing). This closes the EMT-vs-phasor validation at the full-SMIB
level (the machine alone was already validated in the machine work).

### prop-regulation EMT twin NaN — root cause pinned, NaN removed (init caveat open)
`GeneratorEmtSynchronousProportionalRegulationsInternalParameters` (full six-winding
`EMT.GeneratorSynchronous` + `VRProportional`) compiled and locally inited (an exact EMT
fixed point), but failed **global** init with a NaN in the MACHINE Jacobian. A diagnostic
patch in `DYNModelMulti::evalJt/evalJtPrim` printed the offending sparse entries and
pinned it to two stacked causes, both now fixed in `EMT/GeneratorSynchronous.mo`:
  1. `∂F/∂y'` NaN at `udPu`/`uqPu`: the `if running then udPu=Park(v) else id=0` construct
     compiled to `udPu = (...)/(if running then 1 else 0)`, whose Jacobian `/denom^2` term
     is non-finite once an exciter makes `efdPu` free. Fix: define `udPu/uqPu/u0Pu`
     unconditionally; gate the current injection `terminal.i` by `running` instead.
  2. `∂F/∂(terminal.v)` NaN: `UStatorPu = sqrt(ud^2+uq^2)` pulled the dq voltages (with
     their `der(lambda)` coupling) into the exciter loop. Fix: sense `UStatorPu` from the
     abc node voltages directly, `sqrt((va^2+vb^2+vc^2)/3)` (+ tiny guard), like the 2-axis
     machine.
Ruled out along the way (all still NaN): implicit-vs-explicit flux->current, Shackshaft
saturation (incl. frozen `MdSat=const`), VR submodel/type, excitation base, loop gain.
RESULT: the full machine now RUNS with both a proportional and a PI voltage regulator
(`smibvr`, `smibvrpi`); `#26` reproduces bit-for-bit (regression-safe). Bundles:
`GeneratorEmtSynchronousProportionalRegulationsInternalParameters` (P) and the new
`GeneratorEmtSynchronousFullGoverPropVRPropInt` (PI). OPEN CAVEAT: global init now
converges but to a spurious collapsed-voltage root (singular global-init Jacobian at the
physical point), so the VR cases carry a startup recovery transient before settling --
not yet clean for an immediate disturbance. The 2-axis `GeneratorEmtSynchronousGoverPropVRPropInt`
(`EmtControlledSMIB`) hits the same NaN/collapse here too, so this is common to EMT
voltage regulation, not specific to the six-winding machine. MITIGATION (in the demo
cases): `calculateIC` is an algebraic restoration that holds differential vars fixed and
solves algebraic ones; a direct-feedthrough exciter makes `efdPu` algebraic (free during
restoration), which is what lets KINSOL walk to the collapsed root. Giving the exciter a
small physical lag (`voltageRegulator_LagEfdMin/Max = 0.01 s`) makes `efdPu` a state, so
the time-domain solver snaps back to the operating point in < 0.1 s -- settled well before
the 0.4 s fault, with a physically correct AVR fault response (efd to ceiling during the
fault, backing off after). So the recommended setup for a voltage-regulated EMT machine is
a nonzero exciter lag; the clean global-init fix remains the open follow-up. Full detail in
`emt-exploration/smibcmp/README.md`. Does NOT affect the #26 validation (constant-excitation
pair).

## 5. Connectable interface ported from the phasor C++ network — DONE
- per-bus ACPIN: a per-bus `"<busId>_hasConnection"` BOOL PAR input (the phasor's
  mechanism) now exposes that bus's abc ACPIN, so any external Modelica model
  (fault, load, injector) can attach to any flagged bus -- the EMT network is now
  one-to-one with the phasor's per-bus connection interface.
- the factor-3 Park bridge is now PER-BUS (`flowScale_`): 3 for an external-machine
  bus (power-invariant Park), 1 for a hasConnection bus (standard abc). So a
  standard-convention Modelica model attaches correctly to a flagged bus.
- demonstrated in `emt-exploration/ieee14/ieee14_mfault`: a Modelica `EmtFault`
  on flagged BUS_4 gives a single-line-to-ground fault (phase a collapses to 0.91,
  b/c stay at 1.43/1.45) -- a Modelica bus Fault via the per-bus port.
- component connection states are exposed (`<id>_state_value`, used by
  EventQuadripoleDisconnection); other event hooks follow the same pattern.
## 6. Single abc convention across all EMT models — DONE
The machine's terminal current was the only EMT model not in the network's standard
abc convention: its inverse Park used Cdq = sqrt(2)/3, so for the same physical
current its abc waveform was 1/3 the size (peak (sqrt(2)/3)*Irms instead of
sqrt(2)*Irms), which needed the per-bus factor-3 bridge. Fixed by rendering the
terminal current with Cdq = sqrt(2) (standard abc) in BOTH machine models
(SynchronousMachineFull, GeneratorSynchronous); the internal dq dynamics are
unchanged (Cabc stays sqrt(2)/3 so lambda is still RMS-pu for saturation), and the
reported PGenPu is taken from the dq product ud*id+uq*iq (invariant to Cdq) instead
of sum(terminal.v*terminal.i).

Result: every EMT model (C++ network components, Modelica machine, Modelica fault)
now uses ONE abc convention, so flowScale_ is 1 everywhere -- a machine and any other
model share a bus by simple summation, exactly like the phasor bus. Validated:
smibnet/IEEE14 (C++ network) flat-start clean with flowScale_=1 and the machine
abc current now equals the network line current (0.700 peak); the 12 regression
cases pass. The hand-built Modelica-grid rigs (smibfull/smibgs) settle to the same
steady state but show a small startup transient from their hardcoded init currents,
which were scaled x3 for smibfull but are inherently approximate (a separate hand-
tuning / proper-INIT task).
