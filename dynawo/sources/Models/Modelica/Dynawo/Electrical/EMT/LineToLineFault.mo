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

model LineToLineFault "Phase-to-phase (line-to-line) fault: two phases shorted through Rf during [tBegin, tEnd], no ground"

  /*
    Unbalanced fault complementing Fault (which is phase-to-ground). A current iF
    flows from faultPhase1 to faultPhase2 through the fault resistance while the
    fault is active; the third phase is left essentially open. This is a pure
    line-to-line fault (no zero-sequence / ground path).
  */

  Dynawo.Electrical.EMT.EmtTerminal terminal "Faulted node terminal";

  parameter Integer faultPhase1 = 1 "First faulted phase (1=a, 2=b, 3=c)";
  parameter Integer faultPhase2 = 2 "Second faulted phase (1=a, 2=b, 3=c)";
  parameter Real RfPu = 0.002 "Fault (on-state) resistance between the two phases in pu";
  parameter Real RinfPu = 1e6 "Off-state / healthy-phase resistance to ground in pu";
  parameter Real tBegin = 0.1 "Fault inception time in s";
  parameter Real tEnd = 1e9 "Fault clearing time in s";

  final parameter Integer faultPhase3 = 6 - faultPhase1 - faultPhase2 "Unfaulted phase";
  Boolean faultOn(start = false, fixed = true) "Fault active state";
  Real iF "Fault current flowing from phase1 to phase2 in pu";

equation
  when time >= tBegin then
    faultOn = true;
  elsewhen time >= tEnd then
    faultOn = false;
  end when;

  if faultOn then
    terminal.v[faultPhase1] - terminal.v[faultPhase2] = RfPu * iF;
  else
    iF = (terminal.v[faultPhase1] - terminal.v[faultPhase2]) / RinfPu;
  end if;
  terminal.i[faultPhase1] = iF;
  terminal.i[faultPhase2] = -iF;
  terminal.i[faultPhase3] = terminal.v[faultPhase3] / RinfPu;

  annotation(preferredView = "text");
end LineToLineFault;
