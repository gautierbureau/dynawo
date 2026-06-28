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

model InfiniteBus "Three-phase EMT infinite bus: imposes a constant-amplitude balanced abc voltage at its single terminal; the terminal current is set by the network"

  /*
    EMT counterpart of Dynawo.Electrical.Buses.InfiniteBus. The phasor model
    imposes a constant complex voltage terminal.V = UPu*exp(j*UPhase) and lets the
    terminal current float. In the time domain the bus is a balanced positive-
    sequence EMF of fixed amplitude rotating at FNom,

      e[k] = sqrt(2)*UPu*cos(2*pi*FNom*time + UPhase + shift[k]),

    behind a small series (source) resistance RsPu, so the terminal current is

      terminal.i[k] = (terminal.v[k] - e[k]) / RsPu.

    UPu keeps its phasor meaning (RMS voltage module in pu) and UPhase the angle of
    phase a at t = 0; the peak EMF is sqrt(2)*UPu. The tiny RsPu (a few per-mille,
    the abc analogue of the phasor InfiniteBusWithImpedance) makes the source
    connectable to ANY node -- an inductive node (terminal.v is then algebraically
    e minus the small RsPu drop) or a capacitive node (terminal.v is the node's own
    state, driven toward e through RsPu): an IDEAL source fixing a capacitive node's
    voltage would be index-singular. Unlike the two-terminal VoltageSource it has a
    single terminal (the neutral is ground), so no explicit ground component is
    needed; terminal.v is external (declared in the .extvar), the shared node
    potential. Dynawo-native, no MSL.
  */

  Dynawo.Electrical.EMT.EmtTerminal terminal "Bus terminal";

  parameter Real UPu = 1.0 "Constant voltage module (RMS) in pu";
  parameter Real UPhase = 0.0 "Constant voltage angle of phase a at t = 0 in rad";
  parameter Real FNom = 50 "Bus frequency in Hz";
  parameter Real RsPu = 1e-3 "Series (source) resistance per phase in pu";

  constant Real PI = 3.141592653589793 "Pi";
  final parameter Real Omega = 2 * PI * FNom "Angular frequency in rad/s";
  final parameter Real PhaseShift[3] = {0, -2 * PI / 3, 2 * PI / 3} "Per-phase angular shifts for balanced positive sequence";

equation
  for k in 1:3 loop
    terminal.i[k] = (terminal.v[k] - sqrt(2) * UPu * cos(Omega * time + UPhase + PhaseShift[k])) / RsPu;
  end for;

  annotation(preferredView = "text");
end InfiniteBus;
