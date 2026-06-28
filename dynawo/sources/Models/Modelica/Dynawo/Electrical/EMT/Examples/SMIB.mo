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

model SMIB "EMT single-machine infinite-bus: generator -> step-up transformer -> HV bus -> line -> infinite bus, with an HV-bus fault"

  /*
    EMT counterpart of the phasor SMIB cases (nrt/data/SMIB): the same topology
    (generator, step-up transformer, line, infinite bus, fault at the HV bus) but
    solved in instantaneous abc. Reactances mirror SMIB_Nordic (X_tfo = 0.0068,
    X_line = 0.0225 pu). A small shunt capacitance sits at the generator terminal
    and at the HV bus: it interfaces the current-behaviour machine to the
    fixed-step solver and removes the transformer/line series-inductor index
    problem; the HV-bus capacitor also hosts the fault node. The machine field
    (Ifd0) and angle (Theta0 = pi/2, cosine reference) are set so its open-circuit
    EMF matches the infinite bus, minimising the pre-fault circulating current.

    t = 0.2 s : three-phase bolted fault at the HV bus.
    t = 0.27 s: fault cleared (70 ms, as in the phasor case).
    One run shows both the fast EMT fault transient and the slow electromechanical
    rotor swing.
  */

  Dynawo.Electrical.EMT.SynchronousMachine machine(Theta0 = 1.5707963, Phid0Pu = 1.5929, Phifd0Pu = 1.9342491, Efd0Pu = 0.937, Pm0Pu = 0) "Generator";
  Dynawo.Electrical.EMT.FixedExcitationMechanicalPower fixedControl(Efd0Pu = 0.937, Pm0Pu = 0) "Constant field voltage and mechanical power (uncontrolled)";
  Dynawo.Electrical.EMT.Capacitor capGen(CPu = 2e-3, U0Pu = {1.3, -0.65, -0.65}) "Generator-terminal shunt capacitance";
  Dynawo.Electrical.EMT.TransformerYgYg tfo(rTfoPu = 1.0, RPu = 0.0, LPu = 2.15e-5) "Step-up transformer (X = 0.0068 pu)";
  Dynawo.Electrical.EMT.Capacitor capBus(CPu = 1e-3, U0Pu = {1.3, -0.65, -0.65}) "HV-bus shunt capacitance / fault node";
  Dynawo.Electrical.EMT.Fault fault(RfPu = 0.002, tBegin = 0.2, tEnd = 0.27, faultedPhase = {true, true, true}) "HV-bus three-phase bolted fault, 70 ms";
  Dynawo.Electrical.EMT.Resistor dampGen(RPu = 15) "Light damping load at the generator terminal";
  Dynawo.Electrical.EMT.Resistor dampBus(RPu = 15) "Light damping load at the HV bus";
  Dynawo.Electrical.EMT.SeriesRL line(RPu = 0.001, LPu = 7.16e-5) "Transmission line (X = 0.0225 pu)";
  Dynawo.Electrical.EMT.VoltageSource infiniteBus(UPeakPu = 1.3, FNom = 50, Phase0 = 0) "Infinite bus";
  Dynawo.Electrical.EMT.Ground gGen "Generator-cap neutral";
  Dynawo.Electrical.EMT.Ground gBus "HV-bus-cap neutral";
  Dynawo.Electrical.EMT.Ground gInf "Infinite-bus neutral";

  Real vGenA = machine.terminal.v[1] "Generator terminal phase-a voltage in pu";
  Real iGenA = machine.terminal.i[1] "Generator stator phase-a current in pu";
  Real iGenB = machine.terminal.i[2] "Generator stator phase-b current in pu";
  Real iGenC = machine.terminal.i[3] "Generator stator phase-c current in pu";
  Real vBusA = capBus.p.v[1] "HV-bus phase-a voltage in pu";
  Real iLineA = line.p.i[1] "Line phase-a current in pu";
  Real Wr = machine.Wr "Rotor speed in pu";
  Real dtheta = machine.dtheta "Rotor angle deviation from the synchronous frame in rad";
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
end SMIB;
