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

model BreakerOpening "EMT: balanced source feeding a coupled R-L load through a breaker that opens around t = 0.1 s"

  /*
    source --[breaker]--+--[coupled R-L load]--+-- ground
                        |                      |
    Each breaker pole interrupts at the first current zero at/after t = 0.1 s.
    Exercises the ported CoupledRL (mutual 3x3 inductance) and Breaker (current-
    zero interruption). Expected: per-phase load currents drop to ~0 staggered at
    their zero crossings, and a recovery (transient recovery) voltage builds
    across the open breaker poles.
  */

  Dynawo.Electrical.EMT.VoltageSource source(UPeakPu = sqrt(2), FNom = 50, Phase0 = 0) "Balanced 1 pu RMS, 50 Hz source";
  Dynawo.Electrical.EMT.Breaker breaker(tOpening = {0.1, 0.1, 0.1}, RonPu = 1e-5, GoffPu = 1e-6) "Three-phase breaker opening at the first current zero after 0.1 s";
  Dynawo.Electrical.EMT.CoupledRL load(
    RPu = [1.0, 0.0, 0.0; 0.0, 1.0, 0.0; 0.0, 0.0, 1.0],
    LPu = [5e-4, 1e-4, 1e-4; 1e-4, 5e-4, 1e-4; 1e-4, 1e-4, 5e-4]) "Mutually-coupled R-L load";
  Dynawo.Electrical.EMT.Ground ground "Neutral / ground reference";

  // Scalar output aliases for curve export (phases a, b, c)
  Real iLoadA = breaker.p.i[1] "Load / breaker phase-a current in pu";
  Real iLoadB = breaker.p.i[2] "Load / breaker phase-b current in pu";
  Real iLoadC = breaker.p.i[3] "Load / breaker phase-c current in pu";
  Real vBreakerA = breaker.v[1] "Voltage across breaker pole a in pu (recovery voltage once open)";
  Real vBreakerB = breaker.v[2] "Voltage across breaker pole b in pu (recovery voltage once open)";
  Real vBreakerC = breaker.v[3] "Voltage across breaker pole c in pu (recovery voltage once open)";

equation
  connect(source.p, breaker.p);
  connect(breaker.n, load.p);
  connect(load.n, ground.terminal);
  connect(source.n, ground.terminal);

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>
<p>Self-contained EMT switching test. Simulate with a fixed-step solver
(<code>SIM</code>/<code>TRAP</code>) at a small step (e.g. 1e-5 s) over ~0.2 s,
with the solver parameter <code>minimalAcceptableStep</code> set below the step.
Expected: balanced load currents until ~0.1 s, then each phase current
interrupts at its next zero crossing, and the breaker-pole voltages recover
towards the source voltage.</p>
</body></html>"));
end BreakerOpening;
