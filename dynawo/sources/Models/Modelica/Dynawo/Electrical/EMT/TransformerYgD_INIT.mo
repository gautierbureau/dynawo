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

model TransformerYgD_INIT "Initialisation model for the EMT TransformerYgD: abc start primary currents, accounting for the delta 30 deg shift"

  /*
    Dynawo _INIT companion for TransformerYgD (grounded-wye / delta). The primary
    winding k spans line k to ground while the secondary delta winding k spans
    secondary lines k and k+1, so per phase the dynamic model satisfies
      primary.v[k] = rTfoPu*(secondary.v[k] - secondary.v[next(k)]) + (R + L*der)*primary.i[k].
    For a balanced positive-sequence operating point the line-line difference of the
    secondary equals sqrt(3)*e^{j30deg} times the secondary phase phasor, so
      Ip = (Vprimary - rTfoPu*sqrt(3)*e^{j30deg}*Vsecondary) / (R + j*Omega*L).
    Every intermediate is a final parameter, so the only unknowns are I0Pu[3] (the
    dynamic model's start currents, matched by name) with exactly three equations.
  */

  constant Real PI = 3.141592653589793 "Pi";

  // Terminal operating points (peak phasor, pu / rad), phase-a positive sequence
  parameter Real UpMagPu = 1.0 "Primary (wye) voltage magnitude (peak) in pu";
  parameter Real UpPhaseRad = 0.0 "Primary (wye) voltage angle in rad";
  parameter Real UsMagPu = 1.0 "Secondary (delta-line) voltage magnitude (peak) in pu";
  parameter Real UsPhaseRad = 0.0 "Secondary (delta-line) voltage angle in rad";

  // Transformer parameters (must match the dynamic TransformerYgD)
  parameter Real rTfoPu = 1.0 "Turns ratio N1/N2 (primary winding / secondary winding)";
  parameter Real RPu = 0.005 "Leakage resistance referred to primary in pu";
  parameter Real LPu = 3.2e-4 "Leakage inductance referred to primary in pu (L = X / omegaNom)";
  parameter Real FNom = 50 "Nominal frequency in Hz";

  // Primary current phasor Ip = (Vp - rTfo*sqrt(3)e^{j30}*Vs) / (R + j*Omega*L), all from parameters.
  // deltaFactor = sqrt(3)*e^{j30deg} = 1.5 + j*sqrt(3)/2 (wye-line / delta-winding factor).
  final parameter Real Omega = 2 * PI * FNom "Nominal angular frequency in rad/s";
  final parameter Real Xl = Omega * LPu "Leakage reactance referred to primary in pu";
  final parameter Real Dre = 1.5 "Re(deltaFactor)";
  final parameter Real Dim = sqrt(3.0) / 2.0 "Im(deltaFactor)";
  final parameter Real Vsre = UsMagPu * cos(UsPhaseRad) "Re(Vs)";
  final parameter Real Vsim = UsMagPu * sin(UsPhaseRad) "Im(Vs)";
  final parameter Real DVsre = Dre * Vsre - Dim * Vsim "Re(deltaFactor*Vs)";
  final parameter Real DVsim = Dre * Vsim + Dim * Vsre "Im(deltaFactor*Vs)";
  final parameter Real DVre = UpMagPu * cos(UpPhaseRad) - rTfoPu * DVsre "Re(Vp - rTfo*deltaFactor*Vs)";
  final parameter Real DVim = UpMagPu * sin(UpPhaseRad) - rTfoPu * DVsim "Im(Vp - rTfo*deltaFactor*Vs)";
  final parameter Real Zden = RPu * RPu + Xl * Xl "|R + jX|^2";
  final parameter Real Ire = (DVre * RPu + DVim * Xl) / Zden "Re(Ip)";
  final parameter Real Iim = (DVim * RPu - DVre * Xl) / Zden "Im(Ip)";
  final parameter Real IpMagPu = sqrt(Ire * Ire + Iim * Iim) "Primary current magnitude (peak) in pu";
  final parameter Real IpAngleRad = atan2(Iim, Ire) "Primary current angle in rad";

  // Output: the dynamic TransformerYgD's start values (transferred by name)
  Real I0Pu[3] "Per-phase primary currents at t = 0 in pu";

equation
  // instantaneous a, b, c at t = 0 of the balanced positive-sequence primary current phasor
  I0Pu[1] = IpMagPu * cos(IpAngleRad);
  I0Pu[2] = IpMagPu * cos(IpAngleRad - 2 * PI / 3);
  I0Pu[3] = IpMagPu * cos(IpAngleRad + 2 * PI / 3);

  annotation(preferredView = "text");
end TransformerYgD_INIT;
