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

model SMIBLoaded "EMT loaded single-machine infinite-bus with an unbalanced (single-line-to-ground) HV-bus fault"

  // --- EMT initialisation from the steady-state load-flow phasors (peak, pu / rad) ---
  // Node/branch phasors from the full-network propagation (machine -> caps/dampers ->
  // transformer -> caps/dampers -> line -> infinite bus). The abc t=0 values are then
  // obtained with Functions.balancedAbcInit, so no triplet is hand-written.
  final parameter Real capGenU0Pu[3] = Dynawo.Electrical.EMT.Functions.balancedAbcInit(1.29300, -0.071687) "Generator-terminal voltage at t=0";
  final parameter Real capBusU0Pu[3] = Dynawo.Electrical.EMT.Functions.balancedAbcInit(1.28806, -0.094782) "HV-bus voltage at t=0";
  final parameter Real tfoI0Pu[3]    = Dynawo.Electrical.EMT.Functions.balancedAbcInit(0.30210, -0.247453) "Transformer primary current at t=0";
  final parameter Real lineI0Pu[3]   = Dynawo.Electrical.EMT.Functions.balancedAbcInit(0.29849, -0.388473) "Line current at t=0";

  /*
    Like SMIB, but the generator delivers real power pre-fault (Pm = 0.60 pu) and
    is started exactly at the loaded operating point. A full-network steady-state
    phasor solution (load flow over machine + caps + dampers + transformer + line)
    gives EVERY initial condition: the machine fluxes/angle/Pm, the two capacitor
    voltages and the transformer/line currents at t=0, and the consistent infinite
    bus voltage/phase (1.266 pu, -0.139 rad -- NOT 1.3/0, which was the residual
    settling source). The network therefore starts in exact steady state. External reactance
    X = 0.30 pu (X_tfo = 0.1, X_line = 0.2). The fault is a single-line-to-ground
    fault on phase a at the HV bus (0.3-0.4 s): an UNBALANCED fault, so the three
    phases respond differently, the machine sees negative-sequence currents and a
    second-harmonic torque ripple, and the loaded rotor accelerates and swings.
  */

  Dynawo.Electrical.EMT.SynchronousMachine machine(
    Theta0 = 1.919862, Efd0Pu = 1.05, Pm0Pu = 0.602702,
    Phid0Pu = 1.447180, Phiq0Pu = 0.647660, Phifd0Pu = 1.880368, Phiq10Pu = 0.545398) "Loaded generator (P0 = 0.60 pu)";
  Dynawo.Electrical.EMT.FixedExcitationMechanicalPower fixedControl(Efd0Pu = 1.05, Pm0Pu = 0.602702) "Constant field voltage and mechanical power (uncontrolled)";
  Dynawo.Electrical.EMT.Capacitor capGen(CPu = 1e-4, U0Pu = capGenU0Pu) "Generator-terminal shunt capacitance";
  Dynawo.Electrical.EMT.Resistor dampGen(RPu = 100) "Light damping at the generator terminal";
  Dynawo.Electrical.EMT.TransformerYgYg tfo(rTfoPu = 1.0, RPu = 0.0, LPu = 3.183e-4, I0Pu = tfoI0Pu) "Step-up transformer (X = 0.1 pu)";
  Dynawo.Electrical.EMT.Capacitor capBus(CPu = 1e-4, U0Pu = capBusU0Pu) "HV-bus shunt capacitance / fault node";
  Dynawo.Electrical.EMT.Resistor dampBus(RPu = 100) "Light damping at the HV bus";
  Dynawo.Electrical.EMT.Fault fault(RfPu = 0.005, tBegin = 0.4, tEnd = 0.5, faultedPhase = {true, false, false}) "Single-line-to-ground fault on phase a, 100 ms";
  Dynawo.Electrical.EMT.SeriesRL line(RPu = 0.02, LPu = 6.366e-4, I0Pu = lineI0Pu) "Transmission line (X = 0.2 pu)";
  Dynawo.Electrical.EMT.VoltageSource infiniteBus(UPeakPu = 1.26628, FNom = 50, Phase0 = -0.138557) "Infinite bus";
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
  Real Te = machine.Te "Electromagnetic torque in pu (2nd-harmonic ripple under the unbalanced fault)";

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
end SMIBLoaded;
