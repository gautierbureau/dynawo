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

model TransformerStepDown "EMT: source -> YgYg 2:1 transformer -> resistive load (voltage ratio and current scaling)"

  Dynawo.Electrical.EMT.VoltageSource source(UPeakPu = sqrt(2), FNom = 50, Phase0 = 0) "Balanced 1 pu RMS, 50 Hz source";
  Dynawo.Electrical.EMT.TransformerYgYg tfo(rTfoPu = 2.0, RPu = 0.005, LPu = 3.2e-4) "Grounded-wye/grounded-wye 2:1 transformer";
  Dynawo.Electrical.EMT.Resistor load(RPu = 1.0) "Secondary resistive load";
  Dynawo.Electrical.EMT.Ground gPrimary "Primary neutral / source return";
  Dynawo.Electrical.EMT.Ground gSecondary "Secondary neutral";

  Real vPrimA = tfo.primary.v[1] "Primary phase-a voltage in pu";
  Real vSecA = tfo.secondary.v[1] "Secondary phase-a voltage in pu (expect ~ vPrim/2)";
  Real iPrimA = tfo.primary.i[1] "Primary phase-a current in pu";
  Real iSecA = tfo.secondary.i[1] "Secondary phase-a current in pu (expect ~ -2*iPrim)";

equation
  connect(source.p, tfo.primary);
  connect(source.n, gPrimary.terminal);
  connect(tfo.secondary, load.p);
  connect(load.n, gSecondary.terminal);

  annotation(preferredView = "text");
end TransformerStepDown;
