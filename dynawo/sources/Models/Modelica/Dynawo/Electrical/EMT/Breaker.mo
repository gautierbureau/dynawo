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

model Breaker "Three-phase ideal breaker: starts closed; each pole opens at the first current zero at or after its opening time"

  /*
    Three-phase ideal breaker in Dynawo logic: the pole receives an opening
    command at tOpening and actually interrupts at the next current zero (real
    breakers cannot chop an inductive current). Expressed directly with when /
    pre, no MSL dependency.

      closed pole : v = RonPu * i   (near-zero voltage drop)
      open   pole : i = GoffPu * v  (near-zero current; lets the recovery / TRV
                                     voltage build across the open contacts)
  */

  extends Dynawo.Electrical.EMT.BaseEmtTwoTerminal;

  parameter Real RonPu = 1e-5 "Closed-state series resistance per phase in pu";
  parameter Real GoffPu = 1e-6 "Open-state shunt conductance per phase in pu";
  parameter Real tOpening[3] = {1e9, 1e9, 1e9} "Per-phase opening-command time in s (large value keeps the pole closed)";

  // State is tracked as "open" (latched), whose default start value false is
  // honoured reliably; a "closed" boolean starting true is not, in this engine.
  Boolean open[3](each start = false, each fixed = true) "Pole interrupted (open) state; starts closed";

protected
  Boolean armed[3](each start = false, each fixed = true) "Opening command issued, waiting for a current zero";

equation
  for k in 1:3 loop
    when time >= tOpening[k] then
      armed[k] = true;
    end when;
    // Fire on every current zero crossing (both directions); latch open once armed.
    when {p.i[k] <= 0, p.i[k] >= 0} then
      open[k] = pre(armed[k]) or pre(open[k]);
    end when;
    if not open[k] then
      v[k] = RonPu * p.i[k];
    else
      p.i[k] = GoffPu * v[k];
    end if;
  end for;

  annotation(preferredView = "text");
end Breaker;
