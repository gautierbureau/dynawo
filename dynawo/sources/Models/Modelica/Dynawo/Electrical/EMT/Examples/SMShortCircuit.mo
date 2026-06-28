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

model SMShortCircuit "EMT: synchronous machine with a terminal shunt capacitance and load, then a three-phase fault"

  /*
    machine --+-- shunt C --+-- R load -- ground
              |             |
              +----------- 3-phase fault (t = 0.12 s)

    The small terminal capacitance makes the node voltage a state, which is the
    standard way to interface a current-behaviour machine model to a fixed-step
    EMT solver (it removes the stiff algebraic v<->i loop). The capacitor is
    initialised to the machine open-circuit EMF so the run starts near steady
    state; at t = 0.12 s a near-bolted three-phase fault collapses the terminal
    and drives the stator short-circuit current.
  */

  Dynawo.Electrical.EMT.SynchronousMachine machine "Synchronous machine (field + q-damper)";
  Dynawo.Electrical.EMT.FixedExcitationMechanicalPower fixedControl "Constant field voltage and mechanical power (uncontrolled, open-circuit defaults)";
  Dynawo.Electrical.EMT.Capacitor shunt(CPu = 2e-3, U0Pu = {0.0, -0.707, 0.707}) "Terminal shunt capacitance (~open-circuit EMF initial)";
  Dynawo.Electrical.EMT.Resistor load(RPu = 5) "Resistive load";
  Dynawo.Electrical.EMT.Fault fault(RfPu = 0.002, tBegin = 0.12, faultedPhase = {true, true, true}) "Three-phase fault at 0.12 s";
  Dynawo.Electrical.EMT.Ground gShunt "Shunt-capacitor neutral";
  Dynawo.Electrical.EMT.Ground gLoad "Load neutral";

  Real vTermA = machine.terminal.v[1] "Terminal phase-a voltage in pu";
  Real vTermB = machine.terminal.v[2] "Terminal phase-b voltage in pu";
  Real vTermC = machine.terminal.v[3] "Terminal phase-c voltage in pu";
  Real iStatorA = machine.terminal.i[1] "Stator phase-a current in pu";
  Real iStatorB = machine.terminal.i[2] "Stator phase-b current in pu";
  Real iStatorC = machine.terminal.i[3] "Stator phase-c current in pu";
  Real Wr = machine.Wr "Rotor speed in pu";
  Real Te = machine.Te "Electromagnetic torque in pu";

equation
  connect(fixedControl.efdPu, machine.efdPu);
  connect(fixedControl.PmPu, machine.PmPu);
  connect(machine.terminal, shunt.p);
  connect(shunt.n, gShunt.terminal);
  connect(machine.terminal, load.p);
  connect(load.n, gLoad.terminal);
  connect(machine.terminal, fault.terminal);

  annotation(preferredView = "text");
end SMShortCircuit;
