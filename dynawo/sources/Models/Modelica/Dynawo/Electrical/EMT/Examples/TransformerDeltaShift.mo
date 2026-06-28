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

model TransformerDeltaShift "EMT: source -> YgD transformer -> grounded resistive load (shows the 30 deg phase shift)"

  Dynawo.Electrical.EMT.VoltageSource source(UPeakPu = sqrt(2), FNom = 50, Phase0 = 0) "Balanced 1 pu RMS, 50 Hz source";
  Dynawo.Electrical.EMT.TransformerYgD tfo(rTfoPu = 1.0, RPu = 0.005, LPu = 3.2e-4) "Grounded-wye / delta transformer";
  Dynawo.Electrical.EMT.Resistor load(RPu = 3.0) "Secondary grounded-wye resistive load (references the delta)";
  Dynawo.Electrical.EMT.Ground gPrimary "Primary neutral / source return";
  Dynawo.Electrical.EMT.Ground gSecondary "Secondary load neutral";

  Real vPrimA = tfo.primary.v[1] "Primary phase-a voltage in pu";
  Real vSecA = tfo.secondary.v[1] "Secondary phase-a voltage in pu (30 deg shifted vs primary)";
  Real vSecB = tfo.secondary.v[2] "Secondary phase-b voltage in pu";
  Real vSecC = tfo.secondary.v[3] "Secondary phase-c voltage in pu";

equation
  connect(source.p, tfo.primary);
  connect(source.n, gPrimary.terminal);
  connect(tfo.secondary, load.p);
  connect(load.n, gSecondary.terminal);

  annotation(preferredView = "text");
end TransformerDeltaShift;
