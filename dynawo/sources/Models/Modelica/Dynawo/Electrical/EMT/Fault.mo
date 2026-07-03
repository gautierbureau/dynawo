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

model Fault "Three-phase-to-ground fault: per-phase shunt resistance to ground, active during [tBegin, tEnd]"

  /*
    Shunt fault element (one terminal connected to the faulted node). A fault in
    EMT is simply a switch-to-ground through a fault resistance:

      faulted phase, fault active : terminal.v = Rf * terminal.i   (clamped to ground through Rf)
      otherwise                   : terminal.i = terminal.v / Rinf (negligible shunt current)

    faultedPhase selects single-line-to-ground (e.g. {true,false,false}),
    line-to-line-to-ground, or three-phase faults. Dynawo-native logic, no MSL.
  */

  Dynawo.Electrical.EMT.EmtTerminal terminal "Faulted node terminal";

  parameter Real RfPu = 0.02 "Fault (on-state) resistance to ground per phase in pu";
  parameter Real RinfPu = 1e6 "Off-state resistance to ground per phase in pu";
  parameter Real tBegin = 0.1 "Fault inception time in s";
  parameter Real tEnd = 1e9 "Fault clearing time in s (large value => permanent fault)";
  parameter Boolean faultedPhase[3] = {true, true, true} "Phases shorted to ground during the fault (a, b, c)";

  Boolean faultOn(start = false, fixed = true) "Fault active state";

equation
  when time >= tBegin then
    faultOn = true;
  elsewhen time >= tEnd then
    faultOn = false;
  end when;

  for k in 1:3 loop
    if faultOn and faultedPhase[k] then
      terminal.v[k] = RfPu * terminal.i[k];
    else
      terminal.i[k] = terminal.v[k] / RinfPu;
    end if;
  end for;

  annotation(preferredView = "text");
end Fault;
