within Dynawo.Electrical.EMT.Examples;

/*
* Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.
*/

model SMIBLoadedInit "EMT loaded single-machine infinite-bus, fully initialised by its _INIT companion (no hand-set start values)"

  /*
    Identical topology and physics to Examples.SMIBLoaded, but every start value
    comes from the SMIBLoadedInit_INIT companion (bound through the preassembled
    model's initName) instead of being hand-written. The init seed is the
    generator-terminal operating point (P0/Q0/U0/UPhase0); the init propagates the
    load flow through the whole network and fills the scalar phasor parameters
    below by name. The abc t=0 triplets are then reconstructed here with
    Functions.balancedAbcInit. This is the Path B demonstration: IIDM-style
    load-flow init driving the Modelica EMT models. A single-line-to-ground fault
    on phase a at the HV bus (0.4-0.5 s) is the disturbance.
  */

  // --- Scalar start values filled by SMIBLoadedInit_INIT (matched by name) ---
  parameter Real machinePhid0Pu "Machine initial d-axis flux linkage in pu";
  parameter Real machinePhiq0Pu "Machine initial q-axis flux linkage in pu";
  parameter Real machinePhifd0Pu "Machine initial field flux linkage in pu";
  parameter Real machinePhiq10Pu "Machine initial q-damper flux linkage in pu";
  parameter Real machinePm "Machine mechanical power in pu";
  parameter Real machineIfd0 "Machine field current in pu";
  parameter Real machineTheta0 "Machine initial rotor angle in rad";
  parameter Real capGenUMagPu "Generator-node voltage magnitude (peak) in pu";
  parameter Real capGenUAngleRad "Generator-node voltage angle in rad";
  parameter Real capBusUMagPu "HV-bus voltage magnitude (peak) in pu";
  parameter Real capBusUAngleRad "HV-bus voltage angle in rad";
  parameter Real tfoIMagPu "Transformer primary current magnitude (peak) in pu";
  parameter Real tfoIAngleRad "Transformer primary current angle in rad";
  parameter Real lineIMagPu "Line current magnitude (peak) in pu";
  parameter Real lineIAngleRad "Line current angle in rad";
  parameter Real infiniteBusUPeakPu "Infinite-bus peak voltage in pu";
  parameter Real infiniteBusPhase0 "Infinite-bus phase-a angle at t = 0 in rad";

  // --- abc t=0 triplets reconstructed from the scalar phasors above ---
  final parameter Real capGenU0Pu[3] = Dynawo.Electrical.EMT.Functions.balancedAbcInit(capGenUMagPu, capGenUAngleRad) "Generator-node capacitor voltages at t = 0";
  final parameter Real capBusU0Pu[3] = Dynawo.Electrical.EMT.Functions.balancedAbcInit(capBusUMagPu, capBusUAngleRad) "HV-bus capacitor voltages at t = 0";
  final parameter Real tfoI0Pu[3] = Dynawo.Electrical.EMT.Functions.balancedAbcInit(tfoIMagPu, tfoIAngleRad) "Transformer primary currents at t = 0";
  final parameter Real lineI0Pu[3] = Dynawo.Electrical.EMT.Functions.balancedAbcInit(lineIMagPu, lineIAngleRad) "Line currents at t = 0";

  Dynawo.Electrical.EMT.SynchronousMachine machine(
    Theta0 = machineTheta0, Efd0Pu = machineIfd0, Pm0Pu = machinePm,
    Phid0Pu = machinePhid0Pu, Phiq0Pu = machinePhiq0Pu, Phifd0Pu = machinePhifd0Pu, Phiq10Pu = machinePhiq10Pu) "Loaded generator";
  Dynawo.Electrical.EMT.FixedExcitationMechanicalPower fixedControl(Efd0Pu = machineIfd0, Pm0Pu = machinePm) "Constant field voltage and mechanical power (uncontrolled)";
  Dynawo.Electrical.EMT.Capacitor capGen(CPu = 1e-4, U0Pu = capGenU0Pu) "Generator-terminal shunt capacitance";
  Dynawo.Electrical.EMT.Resistor dampGen(RPu = 100) "Light damping at the generator terminal";
  Dynawo.Electrical.EMT.TransformerYgYg tfo(rTfoPu = 1.0, RPu = 0.0, LPu = 3.183e-4, I0Pu = tfoI0Pu) "Step-up transformer (X = 0.1 pu)";
  Dynawo.Electrical.EMT.Capacitor capBus(CPu = 1e-4, U0Pu = capBusU0Pu) "HV-bus shunt capacitance / fault node";
  Dynawo.Electrical.EMT.Resistor dampBus(RPu = 100) "Light damping at the HV bus";
  Dynawo.Electrical.EMT.Fault fault(RfPu = 0.005, tBegin = 0.4, tEnd = 0.5, faultedPhase = {true, false, false}) "Single-line-to-ground fault on phase a, 100 ms";
  Dynawo.Electrical.EMT.SeriesRL line(RPu = 0.02, LPu = 6.366e-4, I0Pu = lineI0Pu) "Transmission line (X = 0.2 pu)";
  Dynawo.Electrical.EMT.VoltageSource infiniteBus(UPeakPu = infiniteBusUPeakPu, FNom = 50, Phase0 = infiniteBusPhase0) "Infinite bus";
  Dynawo.Electrical.EMT.Ground gGen;
  Dynawo.Electrical.EMT.Ground gBus;
  Dynawo.Electrical.EMT.Ground gInf;

  Real vBusA = capBus.p.v[1] "HV-bus phase-a voltage in pu (faulted phase)";
  Real vBusB = capBus.p.v[2] "HV-bus phase-b voltage in pu";
  Real vBusC = capBus.p.v[3] "HV-bus phase-c voltage in pu";
  Real iGenA = machine.terminal.i[1] "Generator phase-a current in pu";
  Real iGenB = machine.terminal.i[2] "Generator phase-b current in pu";
  Real iGenC = machine.terminal.i[3] "Generator phase-c current in pu";
  Real Wr = machine.Wr "Rotor speed in pu";
  Real dtheta = machine.dtheta "Rotor angle deviation in rad";
  Real Te = machine.Te "Electromagnetic torque in pu";

equation
  connect(fixedControl.efdPu, machine.efdPu);
  connect(fixedControl.PmPu, machine.PmPu);
  connect(machine.terminal, capGen.p);
  connect(capGen.n, gGen.terminal);
  connect(machine.terminal, dampGen.p);
  connect(dampGen.n, gGen.terminal);
  connect(machine.terminal, tfo.primary);
  connect(tfo.secondary, capBus.p);
  connect(capBus.n, gBus.terminal);
  connect(tfo.secondary, dampBus.p);
  connect(dampBus.n, gBus.terminal);
  connect(tfo.secondary, fault.terminal);
  connect(tfo.secondary, line.p);
  connect(line.n, infiniteBus.p);
  connect(infiniteBus.n, gInf.terminal);

  annotation(preferredView = "text");
end SMIBLoadedInit;
