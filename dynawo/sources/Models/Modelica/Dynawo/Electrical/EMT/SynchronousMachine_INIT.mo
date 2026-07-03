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

model SynchronousMachine_INIT "Initialisation model for SynchronousMachine: flux start values plus the Efd0/UStator0/Pm0 seeds the bundled exciter and governor need"

  /*
    Like SynchronousMachine_INIT (explicit salient-pole construction from the
    terminal operating point P0/Q0/U0/UPhase0), but it also exposes the steady
    seeds the control init companions consume through initConnect in the
    preassembled bundle:
      Efd0Pu     = Ifd0           (field-voltage seed for the AVR),
      UStator0Pu = U0Pu / sqrt(2) (stator RMS voltage seed for the AVR),
      Pm0Pu      = Pm             (mechanical-power seed for the governor).
    The flux / angle outputs transfer to the machine parameters by name.
  */

  import Modelica.ComplexMath;

  constant Real PI = 3.141592653589793 "Pi";

  parameter Real P0Pu = -0.602 "Active power at the stator terminal in pu (base SnRef, receptor convention; = IIDM generator p_pu)";
  parameter Real Q0Pu = -0.024 "Reactive power at the stator terminal in pu (base SnRef, receptor convention; = IIDM generator q_pu)";
  parameter Real U0Pu = 0.9142993715318843 "Stator terminal voltage magnitude in pu (RMS, base UNom; = IIDM bus v_pu)";
  parameter Real UPhase0 = -0.071687 "Stator terminal voltage angle in rad (= IIDM bus angle)";

  parameter Real Ra = 0.005, Lmd = 1.7, Lmq = 1.6, Ld = 2.0, Lq = 1.9, Lff = 2.0643, LQQ = 2.3273;
  final parameter Real sq23 = sqrt(2.0 / 3.0);

  Real Id, Iq, Ifd, Theta0, Vd, Vq;
  Complex Vt, Ig, Eq, rot, IdMinusjIq, VdMinusjVq;

  // Flux / angle start values (transfer to the machine by name)
  Real Phid0Pu, Phiq0Pu, Phifd0Pu, Phiq10Pu;
  // Control seeds (consumed by the exciter / governor init via initConnect; also the machine input starts)
  Real Efd0Pu "Field-voltage seed (= Ifd0)";
  Real Pm0Pu "Mechanical-power seed";
  Real UStator0Pu "Stator RMS voltage seed";
  Real IRotor0Pu "Field-current seed (= Ifd0)";
  Real PGen0Pu "Generated-power seed (= P0Pu)";

equation
  // IIDM supplies the RMS phasor (base UNom); the abc waveforms use peak amplitudes
  Vt = Complex(sqrt(2.0), 0) * ComplexMath.fromPolar(U0Pu, UPhase0);
  // Receptor -> generator power; peak three-phase: S = 1.5 * Vpeak * conj(Ipeak)
  Ig = ComplexMath.conj(Complex(-P0Pu, -Q0Pu)) / (Complex(1.5, 0) * ComplexMath.conj(Vt));
  Eq = Vt + Complex(Ra, Lq) * Ig;
  Theta0 = atan2(Eq.im, Eq.re) + 1.5707963267948966;
  rot = ComplexMath.exp(Complex(0, Theta0));
  IdMinusjIq = Ig / (Complex(sq23, 0) * rot);
  Id = IdMinusjIq.re;
  Iq = -IdMinusjIq.im;
  VdMinusjVq = Vt / (Complex(sq23, 0) * rot);
  Vd = VdMinusjVq.re;
  Vq = -VdMinusjVq.im;
  Ifd = (Vq + Ra * Iq - Ld * Id) / Lmd;

  Phid0Pu = Ld * Id + Lmd * Ifd;
  Phiq0Pu = Lq * Iq;
  Phifd0Pu = Lmd * Id + Lff * Ifd;
  Phiq10Pu = Lmq * Iq;

  Efd0Pu = Ifd;
  Pm0Pu = Phid0Pu * Iq - Phiq0Pu * Id;
  UStator0Pu = U0Pu;
  IRotor0Pu = Ifd;
  PGen0Pu = -P0Pu;

  annotation(preferredView = "text");
end SynchronousMachine_INIT;
