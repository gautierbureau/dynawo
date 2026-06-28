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

model CoupledRL_INIT "Initialisation model for the EMT CoupledRL branch: abc start currents from the two endpoint load-flow phasors"

  /*
    Dynawo _INIT companion for CoupledRL (a mutually-coupled R-L branch). A balanced
    positive-sequence operating point only sees the POSITIVE-SEQUENCE impedance
      Z1 = (Rself - Rmutual) + j*Omega*(Lself - Lmutual)
    (symmetric 3x3 matrices: diagonal = self, off-diagonal = mutual), so the branch
    current phasor is I = (Vp - Vn)/Z1, reconstructed from the two endpoint voltage
    phasors (peak). Every intermediate is a final parameter, so the only unknowns are
    I0Pu[3] (the dynamic model's start currents, matched by name) with exactly three
    equations. Assumes balanced (symmetric) coupling, the only case a positive-
    sequence operating point can seed.
  */

  constant Real PI = 3.141592653589793 "Pi";

  // Endpoint operating points (peak phasor, pu / rad) -- from the operating point
  parameter Real UpMagPu = 1.0 "Terminal-p voltage magnitude (peak) in pu";
  parameter Real UpPhaseRad = 0.0 "Terminal-p voltage angle in rad";
  parameter Real UnMagPu = 1.0 "Terminal-n voltage magnitude (peak) in pu";
  parameter Real UnPhaseRad = 0.0 "Terminal-n voltage angle in rad";

  // Branch parameters (must match the dynamic CoupledRL; symmetric coupling assumed)
  parameter Real RselfPu = 0.0 "Self resistance per phase in pu (RPu[1,1])";
  parameter Real RmutualPu = 0.0 "Mutual resistance per phase in pu (RPu[1,2])";
  parameter Real LselfPu = 6.366e-4 "Self inductance per phase in pu (LPu[1,1])";
  parameter Real LmutualPu = 0.0 "Mutual inductance per phase in pu (LPu[1,2])";
  parameter Real FNom = 50 "Nominal frequency in Hz";

  // Branch current I = (Vp - Vn) / Z1, Z1 = (Rself-Rmutual) + j*Omega*(Lself-Lmutual), all params
  final parameter Real Omega = 2 * PI * FNom "Nominal angular frequency in rad/s";
  final parameter Real R1 = RselfPu - RmutualPu "Positive-sequence resistance in pu";
  final parameter Real X1 = Omega * (LselfPu - LmutualPu) "Positive-sequence reactance in pu";
  final parameter Real DVre = UpMagPu * cos(UpPhaseRad) - UnMagPu * cos(UnPhaseRad) "Re(Vp - Vn)";
  final parameter Real DVim = UpMagPu * sin(UpPhaseRad) - UnMagPu * sin(UnPhaseRad) "Im(Vp - Vn)";
  final parameter Real Zden = R1 * R1 + X1 * X1 "|Z1|^2";
  final parameter Real Ire = (DVre * R1 + DVim * X1) / Zden "Re(I)";
  final parameter Real Iim = (DVim * R1 - DVre * X1) / Zden "Im(I)";
  final parameter Real IMagPu = sqrt(Ire * Ire + Iim * Iim) "Branch current magnitude (peak) in pu";
  final parameter Real IAngleRad = atan2(Iim, Ire) "Branch current angle in rad";

  // Output: the dynamic CoupledRL's start values (transferred by name)
  Real I0Pu[3] "Per-phase branch currents at t = 0 in pu";

equation
  // instantaneous a, b, c at t = 0 of the balanced positive-sequence current phasor
  I0Pu[1] = IMagPu * cos(IAngleRad);
  I0Pu[2] = IMagPu * cos(IAngleRad - 2 * PI / 3);
  I0Pu[3] = IMagPu * cos(IAngleRad + 2 * PI / 3);

  annotation(preferredView = "text");
end CoupledRL_INIT;
