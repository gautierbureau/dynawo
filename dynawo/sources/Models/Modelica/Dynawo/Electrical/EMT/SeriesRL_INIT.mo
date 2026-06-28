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

model SeriesRL_INIT "Initialisation model for the EMT SeriesRL branch: abc start currents from the two endpoint load-flow phasors"

  /*
    Dynawo _INIT companion for SeriesRL. Given the steady-state voltage phasors at
    the two ends (peak magnitude / angle, as a load flow provides at each bus), it
    reconstructs the branch current phasor I = (Vp - Vn) / (R + j*Omega*L) and
    produces the three instantaneous phase currents at t = 0 that the dynamic
    SeriesRL starts from (its I0Pu[3] parameter, matched by name). Every
    intermediate is a final parameter (evaluated from the input parameters), so
    the only unknowns are I0Pu[3] with exactly three equations -- a balanced,
    purely-parameter-driven init model.
  */

  constant Real PI = 3.141592653589793 "Pi";

  // Endpoint operating points (peak phasor, pu / rad) -- e.g. from a load flow
  parameter Real UpMagPu = 1.0 "Terminal-p voltage magnitude (peak) in pu";
  parameter Real UpPhaseRad = 0.0 "Terminal-p voltage angle in rad";
  parameter Real UnMagPu = 1.0 "Terminal-n voltage magnitude (peak) in pu";
  parameter Real UnPhaseRad = 0.0 "Terminal-n voltage angle in rad";

  // Branch parameters (must match the dynamic SeriesRL)
  parameter Real RPu = 0.0 "Series resistance per phase in pu";
  parameter Real LPu = -1 "Series inductance per phase in pu (L = X / omegaNom; legacy, used only if XPu < 0)";
  parameter Real XPu = -1 "Series reactance per phase in pu (IIDM-native; if >= 0 it is used directly)";
  parameter Real FNom = 50 "Nominal frequency in Hz";

  // Branch current phasor I = (Vp - Vn) / (R + j*Xl), all from parameters
  final parameter Real Omega = 2 * PI * FNom "Nominal angular frequency in rad/s";
  final parameter Real Xl = if XPu >= 0 then XPu else Omega * LPu "Series reactance per phase in pu";
  final parameter Real DVre = UpMagPu * cos(UpPhaseRad) - UnMagPu * cos(UnPhaseRad) "Re(Vp - Vn)";
  final parameter Real DVim = UpMagPu * sin(UpPhaseRad) - UnMagPu * sin(UnPhaseRad) "Im(Vp - Vn)";
  final parameter Real Zden = RPu * RPu + Xl * Xl "|R + jX|^2";
  final parameter Real Ire = (DVre * RPu + DVim * Xl) / Zden "Re(I)";
  final parameter Real Iim = (DVim * RPu - DVre * Xl) / Zden "Im(I)";
  final parameter Real IMagPu = sqrt(Ire * Ire + Iim * Iim) "Branch current magnitude (peak) in pu";
  final parameter Real IAngleRad = atan2(Iim, Ire) "Branch current angle in rad";

  // Output: the dynamic SeriesRL's start values (transferred by name)
  Real I0Pu[3] "Per-phase branch currents at t = 0 in pu";

equation
  // instantaneous a, b, c at t = 0 of the balanced positive-sequence current phasor
  I0Pu[1] = IMagPu * cos(IAngleRad);
  I0Pu[2] = IMagPu * cos(IAngleRad - 2 * PI / 3);
  I0Pu[3] = IMagPu * cos(IAngleRad + 2 * PI / 3);

  annotation(preferredView = "text");
end SeriesRL_INIT;
