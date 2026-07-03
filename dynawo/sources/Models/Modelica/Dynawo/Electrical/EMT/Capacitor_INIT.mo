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

model Capacitor_INIT "Initialisation model for the EMT Capacitor: abc start voltages from the node load-flow phasor"

  /*
    Dynawo _INIT companion for Capacitor. Given the steady-state voltage phasor at
    the node (peak magnitude / angle, from the operating point), it produces the
    three instantaneous phase voltages at t = 0 that the dynamic Capacitor starts
    from (its U0Pu[3] parameter, matched by name). The only unknowns are U0Pu[3]
    with exactly three equations -- a balanced, purely-parameter-driven init model
    (the global init assembles the per-component INITs).
  */

  constant Real PI = 3.141592653589793 "Pi";

  // Node operating point (peak phasor, pu / rad) -- from the operating point
  parameter Real UMagPu = 0.0 "Node voltage magnitude (peak) in pu (default 0: flat start unless the case provides the operating point)";
  parameter Real UPhaseRad = 0.0 "Node voltage angle in rad";

  // Output: the dynamic Capacitor's start values (transferred by name)
  Real U0Pu[3] "Per-phase capacitor voltages at t = 0 in pu";

equation
  // instantaneous a, b, c at t = 0 of the balanced positive-sequence node voltage phasor
  U0Pu[1] = UMagPu * cos(UPhaseRad);
  U0Pu[2] = UMagPu * cos(UPhaseRad - 2 * PI / 3);
  U0Pu[3] = UMagPu * cos(UPhaseRad + 2 * PI / 3);

  annotation(preferredView = "text");
end Capacitor_INIT;
