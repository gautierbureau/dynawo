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

model LineToLineFaultDemo "EMT: source -> line -> load with a phase-to-phase (a-b) fault, to exercise LineToLineFault"

  Dynawo.Electrical.EMT.VoltageSource source(UPeakPu = sqrt(2), FNom = 50) "Balanced 1 pu RMS source";
  Dynawo.Electrical.EMT.SeriesRL line(RPu = 0.05, LPu = 5e-4) "Series R-L line";
  Dynawo.Electrical.EMT.Resistor load(RPu = 2.0) "Resistive load";
  Dynawo.Electrical.EMT.LineToLineFault fault(faultPhase1 = 1, faultPhase2 = 2, RfPu = 0.01, tBegin = 0.1, tEnd = 0.16) "Phase a-b fault, 60 ms";
  Dynawo.Electrical.EMT.Ground ground;

  Real vLoadA = load.p.v[1] "Load phase-a voltage in pu";
  Real vLoadB = load.p.v[2] "Load phase-b voltage in pu";
  Real vLoadC = load.p.v[3] "Load phase-c voltage in pu (unfaulted)";
  Real iLineA = line.p.i[1] "Line phase-a current in pu";
  Real iLineB = line.p.i[2] "Line phase-b current in pu";
  Real iLineC = line.p.i[3] "Line phase-c current in pu (unfaulted)";

equation
  connect(source.p, line.p);
  connect(line.n, load.p);
  connect(line.n, fault.terminal);
  connect(load.n, ground.terminal);
  connect(source.n, ground.terminal);

  annotation(preferredView = "text");
end LineToLineFaultDemo;
