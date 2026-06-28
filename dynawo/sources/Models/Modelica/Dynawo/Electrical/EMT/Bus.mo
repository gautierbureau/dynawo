within Dynawo.Electrical.EMT;

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
model Bus "EMT bus: a small shunt-capacitance three-phase node, IIDM-bindable for init (the abc analogue of Electrical.Buses.Bus)"
  extends Dynawo.Electrical.EMT.BaseEmtTwoTerminal;

  /*
    A capacitive node: the small shunt capacitance gives the node a voltage state
    (the standard EMT nodal formulation) and lets the node be initialised directly
    from a load flow. The init interface is Dynawo's universal convention so it
    binds straight to an IIDM bus: UMag0Pu is the RMS voltage magnitude (base UNom,
    = IIDM v_pu) and UPhase0 the angle in rad (= IIDM angle_pu); the peak abc
    voltages at t = 0 are reconstructed internally. Connect p to the node and n to
    a ground, exactly like a shunt Capacitor.
  */

  parameter Real CPu = 1e-4 "Shunt capacitance per phase in pu (small; gives the node a voltage state)";
  parameter Real UMag0Pu = 1.0 "Initial node voltage magnitude in pu (RMS, base UNom; = IIDM bus v_pu)";
  parameter Real UPhase0 = 0.0 "Initial node voltage angle in rad (= IIDM bus angle)";

  final parameter Real U0Pu[3] = Dynawo.Electrical.EMT.Functions.balancedAbcInit(sqrt(2.0) * UMag0Pu, UPhase0) "Per-phase node voltages at t = 0 (peak) in pu";

initial equation
  v = U0Pu;

equation
  for k in 1:3 loop
    p.i[k] = CPu * der(v[k]);
  end for;

  annotation(preferredView = "text");
end Bus;
