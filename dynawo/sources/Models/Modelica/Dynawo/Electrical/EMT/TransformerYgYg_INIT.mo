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

model TransformerYgYg_INIT "Initialisation model for the EMT TransformerYgYg: abc start primary currents from the two terminal load-flow phasors"

  /*
    Dynawo _INIT companion for TransformerYgYg. The dynamic model satisfies
      primary.v = rTfoPu*secondary.v + (R + L*der)*primary.i,
    so at steady state (der -> j*Omega) the primary current phasor is
      Ip = (Vprimary - rTfoPu*Vsecondary) / (R + j*Omega*L),
    reconstructed from the two terminal voltage phasors (peak, from the operating
    point). It produces the three instantaneous primary phase currents at t = 0
    (the dynamic model's I0Pu[3] parameter, matched by name). Every intermediate is
    a final parameter, so the only unknowns are I0Pu[3] with exactly three equations
    (the global init assembles the per-component INITs).
  */

  constant Real PI = 3.141592653589793 "Pi";

  // Terminal operating points (peak phasor, pu / rad) -- from the operating point
  parameter Real UpMagPu = 1.0 "Primary voltage magnitude (peak) in pu";
  parameter Real UpPhaseRad = 0.0 "Primary voltage angle in rad";
  parameter Real UsMagPu = 1.0 "Secondary voltage magnitude (peak) in pu";
  parameter Real UsPhaseRad = 0.0 "Secondary voltage angle in rad";

  // Transformer parameters (must match the dynamic TransformerYgYg)
  parameter Real rTfoPu = 1.0 "Turns ratio N1/N2 (primary/secondary)";
  parameter Real RPu = 0.0 "Leakage resistance referred to primary in pu";
  parameter Real LPu = -1 "Leakage inductance referred to primary in pu (L = X / omegaNom; legacy, used only if XPu < 0)";
  parameter Real XPu = -1 "Leakage reactance referred to primary in pu (IIDM-native; if >= 0 it is used directly)";
  parameter Real FNom = 50 "Nominal frequency in Hz";

  // Primary current phasor Ip = (Vp - rTfo*Vs) / (R + j*Xl), all from parameters
  final parameter Real Omega = 2 * PI * FNom "Nominal angular frequency in rad/s";
  final parameter Real Xl = if XPu >= 0 then XPu else Omega * LPu "Leakage reactance referred to primary in pu";
  final parameter Real DVre = UpMagPu * cos(UpPhaseRad) - rTfoPu * UsMagPu * cos(UsPhaseRad) "Re(Vp - rTfo*Vs)";
  final parameter Real DVim = UpMagPu * sin(UpPhaseRad) - rTfoPu * UsMagPu * sin(UsPhaseRad) "Im(Vp - rTfo*Vs)";
  final parameter Real Zden = RPu * RPu + Xl * Xl "|R + jX|^2";
  final parameter Real Ire = (DVre * RPu + DVim * Xl) / Zden "Re(Ip)";
  final parameter Real Iim = (DVim * RPu - DVre * Xl) / Zden "Im(Ip)";
  final parameter Real IpMagPu = sqrt(Ire * Ire + Iim * Iim) "Primary current magnitude (peak) in pu";
  final parameter Real IpAngleRad = atan2(Iim, Ire) "Primary current angle in rad";

  // Output: the dynamic TransformerYgYg's start values (transferred by name)
  Real I0Pu[3] "Per-phase primary currents at t = 0 in pu";

equation
  // instantaneous a, b, c at t = 0 of the balanced positive-sequence primary current phasor
  I0Pu[1] = IpMagPu * cos(IpAngleRad);
  I0Pu[2] = IpMagPu * cos(IpAngleRad - 2 * PI / 3);
  I0Pu[3] = IpMagPu * cos(IpAngleRad + 2 * PI / 3);

  annotation(preferredView = "text");
end TransformerYgYg_INIT;
