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

model FaultAndClearing "EMT protection sequence: single-line-to-ground fault inception, then breaker clearing"

  /*
    source --[breaker]--[coupled R-L line]--+--[R load]-- ground
                                            |
                                         [SLG fault on phase a]

    t = 0.10 s : phase-a-to-ground fault appears at the load bus (current surges).
    t = 0.13 s : breaker opens; each pole clears at its next current zero,
                 isolating the source from the permanent fault.
    Uses the ported Breaker, CoupledRL and the new Fault, all Dynawo-native.
  */

  Dynawo.Electrical.EMT.VoltageSource source(UPeakPu = sqrt(2), FNom = 50, Phase0 = 0) "Balanced 1 pu RMS, 50 Hz source";
  Dynawo.Electrical.EMT.Breaker breaker(tOpening = {0.13, 0.13, 0.13}, RonPu = 1e-5, GoffPu = 1e-6) "Breaker, opens at 0.13 s to clear the fault";
  Dynawo.Electrical.EMT.CoupledRL line(
    RPu = [0.05, 0.0, 0.0; 0.0, 0.05, 0.0; 0.0, 0.0, 0.05],
    LPu = [5e-4, 1e-4, 1e-4; 1e-4, 5e-4, 1e-4; 1e-4, 1e-4, 5e-4]) "Mutually-coupled R-L line";
  Dynawo.Electrical.EMT.Resistor load(RPu = 2.0) "Resistive load at the fault bus";
  Dynawo.Electrical.EMT.Fault fault(RfPu = 0.02, tBegin = 0.1, faultedPhase = {true, false, false}) "Single-line-to-ground fault on phase a, from 0.1 s";
  Dynawo.Electrical.EMT.Ground ground "Neutral / ground reference";

  // Scalar output aliases for curve export
  Real iSrcA = breaker.p.i[1] "Source-side phase-a current in pu";
  Real iSrcB = breaker.p.i[2] "Source-side phase-b current in pu";
  Real iSrcC = breaker.p.i[3] "Source-side phase-c current in pu";
  Real iFaultA = fault.terminal.i[1] "Fault current to ground, phase a, in pu";
  Real vBusA = fault.terminal.v[1] "Fault-bus phase-a voltage in pu";
  Real vBusB = fault.terminal.v[2] "Fault-bus phase-b voltage in pu";
  Real vBusC = fault.terminal.v[3] "Fault-bus phase-c voltage in pu";

equation
  connect(source.p, breaker.p);
  connect(breaker.n, line.p);
  connect(line.n, load.p);
  connect(line.n, fault.terminal);
  connect(load.n, ground.terminal);
  connect(source.n, ground.terminal);

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>
<p>Self-contained EMT protection study. Fixed-step solver (SIM/TRAP) at ~1e-5 s
over 0.2 s, with minimalAcceptableStep below the step. Expected: balanced
operation, a phase-a fault-current surge at 0.1 s with the phase-a bus voltage
collapsing, then interruption of the source-side currents at their zeros after
the 0.13 s breaker command.</p>
</body></html>"));
end FaultAndClearing;
