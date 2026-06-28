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

model VoltageSource "Ideal balanced three-phase cosine voltage source (positive sequence a-b-c)"
  extends Dynawo.Electrical.EMT.BaseEmtTwoTerminal;

  parameter Real UPeakPu = sqrt(2) "Peak phase-to-ground voltage in pu (sqrt(2) gives 1 pu RMS)";
  parameter Real FNom = 50 "Source frequency in Hz";
  parameter Real Phase0 = 0 "Angle of phase a at t = 0 in rad";

  constant Real PI = 3.141592653589793 "Pi";
  final parameter Real Omega = 2 * PI * FNom "Angular frequency in rad/s";
  final parameter Real PhaseShift[3] = {0, -2 * PI / 3, 2 * PI / 3} "Per-phase angular shifts for balanced positive sequence";

equation
  for k in 1:3 loop
    v[k] = UPeakPu * cos(Omega * time + Phase0 + PhaseShift[k]);
  end for;

  annotation(preferredView = "text");
end VoltageSource;
