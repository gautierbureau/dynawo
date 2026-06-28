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

model ThreePhaseRLCircuit "EMT demo: balanced three-phase source feeding a series R-L line into a parallel R-C load to ground"

  /*
    Topology (per phase, x3 for a, b, c):

        node1            node2
      o---[ source ]---o---[ R-L line ]---o-----+-----+
      |                                   |     |     |
      |                                 [Rload][Cload]
      |                                   |     |     |
      +-----------------------------------+-----+-----+----o ground (v = 0)

    States: line current i[3] (from L) and load voltage v[3] (from C).
    Energising at t = 0 from i = 0, v = 0 excites the classic EMT
    transient (decaying DC offset on the phase currents).
    Per-unit element values are illustrative, on a 50 Hz base.
  */

  Dynawo.Electrical.EMT.VoltageSource source(UPeakPu = sqrt(2), FNom = 50, Phase0 = 0) "Balanced 1 pu RMS, 50 Hz source";
  Dynawo.Electrical.EMT.SeriesRL line(RPu = 0.05, LPu = 5e-4) "Series R-L line (X ~ 0.157 pu at 50 Hz)";
  Dynawo.Electrical.EMT.Resistor load(RPu = 2.0) "Resistive load";
  Dynawo.Electrical.EMT.Capacitor cap(CPu = 2e-4) "Shunt capacitor at the load node (B ~ 0.063 pu at 50 Hz)";
  Dynawo.Electrical.EMT.Ground ground "Neutral / ground reference";

  // Scalar output aliases for curve export (phases a, b, c)
  Real vSourceA = source.v[1] "Source phase-a voltage in pu";
  Real vSourceB = source.v[2] "Source phase-b voltage in pu";
  Real vSourceC = source.v[3] "Source phase-c voltage in pu";
  Real iLineA = line.p.i[1] "Line phase-a current in pu";
  Real iLineB = line.p.i[2] "Line phase-b current in pu";
  Real iLineC = line.p.i[3] "Line phase-c current in pu";
  Real vLoadA = cap.v[1] "Load-node phase-a voltage in pu";
  Real vLoadB = cap.v[2] "Load-node phase-b voltage in pu";
  Real vLoadC = cap.v[3] "Load-node phase-c voltage in pu";

equation
  connect(source.p, line.p);
  connect(line.n, load.p);
  connect(line.n, cap.p);
  connect(source.n, ground.terminal);
  connect(load.n, ground.terminal);
  connect(cap.n, ground.terminal);

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>
<p>Square, self-contained EMT test circuit. Simulate with a fixed-step solver
(<code>SIM</code> or <code>TRAP</code>) at a small step (e.g. 1e-5 s) over a few
cycles. Expected curves: balanced three-phase sinusoids on
<code>vSourceA/B/C</code>, and <code>iLineA/B/C</code> showing a decaying DC
offset during energisation before settling to balanced steady state.</p>
</body></html>"));
end ThreePhaseRLCircuit;
