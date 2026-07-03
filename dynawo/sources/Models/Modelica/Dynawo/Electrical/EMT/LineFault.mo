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

model LineFault "Three-phase EMT line with a shunt-to-ground fault at a point along its length"
  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffLine;

  /*
    EMT counterpart of Dynawo.Electrical.Events.LineFault. A series R-L line split
    at a fraction D of its length into two segments,

      terminal1 --[ D*(R + L*der) ]--*--[ (1-D)*(R + L*der) ]-- terminal2
                                     |
                                  Fault (shunt to ground over [tBegin, tEnd])

    The junction node carries the per-phase shunt-to-ground Fault (single-line-to-
    ground, line-to-line-to-ground or three-phase via faultedPhase). Composed from
    the tested EMT SeriesRL (each segment) and Fault components, so the abc / per-
    unit conventions are inherited unchanged. D in (0, 1); D -> 0 is a fault at
    terminal1, D -> 1 at terminal2.

    The junction (between the two segment inductors) carries a small shunt
    capacitance to ground (CPu, the abc analogue of the network's per-node EMT
    snubber): it makes the junction voltage a state, so removing the fault shunt at
    tEnd is well-posed -- without it, clearing a fault on an inductor-only junction
    leaves the node voltage undefined at the switching instant.

    Like the phasor LineFault it extends SwitchOffLine, so the whole line can be
    tripped (a node disconnection on switchOffSignal1, or a user event on
    switchOffSignal2): the switch-off is propagated to both segments, which open
    (i = 0), so both terminal currents go to zero. The snubber capacitances at the
    junction and the end buses bound the di/dt transient of the interruption.
  */

  Dynawo.Electrical.EMT.EmtTerminal terminal1 "Line terminal 1 (sending)";
  Dynawo.Electrical.EMT.EmtTerminal terminal2 "Line terminal 2 (receiving)";

  parameter Real RPu = 0.0 "Total series resistance per phase in pu (base ZNom)";
  parameter Real LPu = 6.366e-4 "Total series inductance per phase in pu (L = X / omegaNom)";
  parameter Real D = 0.5 "Fault location as a fraction of the line length from terminal1 (0 < D < 1)";
  parameter Real RfPu = 0.02 "Fault (on-state) resistance to ground per phase in pu";
  parameter Real RinfPu = 1e6 "Off-state resistance to ground per phase in pu";
  parameter Real tBegin = 0.1 "Fault inception time in s";
  parameter Real tEnd = 1e9 "Fault clearing time in s (large value => permanent fault)";
  parameter Boolean faultedPhase[3] = {true, true, true} "Phases shorted to ground during the fault (a, b, c)";
  parameter Real CPu = 1e-4 "Junction snubber capacitance to ground per phase in pu";
  parameter Real I0Pu[3] = {0, 0, 0} "Initial per-phase line currents in pu";
  parameter Real Um0Pu[3] = {0, 0, 0} "Initial per-phase junction (fault-point) voltages in pu";

  Dynawo.Electrical.EMT.SeriesRL line1(RPu = D * RPu, LPu = D * LPu, I0Pu = I0Pu) "Segment terminal1 -> fault point";
  Dynawo.Electrical.EMT.SeriesRL line2(RPu = (1 - D) * RPu, LPu = (1 - D) * LPu, I0Pu = I0Pu) "Segment fault point -> terminal2";
  Dynawo.Electrical.EMT.Fault fault(RfPu = RfPu, RinfPu = RinfPu, tBegin = tBegin, tEnd = tEnd, faultedPhase = faultedPhase) "Shunt-to-ground fault at the junction";
  Dynawo.Electrical.EMT.Capacitor snubber(CPu = CPu, U0Pu = Um0Pu) "Junction snubber capacitance (EMT node regularisation)";
  Dynawo.Electrical.EMT.Ground gnd "Ground reference for the junction snubber";

equation
  // propagate the line's switch-off to both segments (they open -> i = 0 -> both
  // terminal currents zero), exactly as the phasor LineFault drives its sub-lines
  line1.switchOffSignal1.value = switchOffSignal1.value;
  line1.switchOffSignal2.value = switchOffSignal2.value;
  line2.switchOffSignal1.value = switchOffSignal1.value;
  line2.switchOffSignal2.value = switchOffSignal2.value;

  connect(terminal1, line1.p);
  connect(line1.n, line2.p);
  connect(line1.n, fault.terminal);
  connect(line1.n, snubber.p);
  connect(snubber.n, gnd.terminal);
  connect(line2.n, terminal2);

  annotation(preferredView = "text");
end LineFault;
