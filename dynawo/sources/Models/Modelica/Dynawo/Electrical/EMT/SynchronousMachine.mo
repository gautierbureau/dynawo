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

model SynchronousMachine "EMT synchronous machine with the Dynawo control signal interface (field voltage / mechanical power inputs, stator voltage / speed outputs) so phasor-domain exciters and governors can drive it"

  /*
    Same electromagnetic / mechanical model as SynchronousMachine, but the field
    voltage and mechanical power are signal INPUTS and the machine exposes the
    same signal OUTPUTS as the phasor GeneratorSynchronous, so the existing
    phasor voltage regulators and governors (pure signal models) can be bundled
    onto it unchanged through a preassembled model:
      inputs  : efdPu     (normalised field voltage, = Ifd at steady state),
                PmPu      (mechanical power, base SNom).
      outputs : UStatorPu (stator terminal voltage magnitude, RMS pu),
                omegaPu   (rotor speed in pu).
    Field equation: der(Phifd) = -Wbase*Rf*(Ifd - efdPu)  =>  efdPu0 = Ifd0.
    This is the single inputs-driven machine; an uncontrolled bundle just feeds
    the inputs with constants.
  */

  import Modelica.Constants.pi;

  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffGenerator;

  Dynawo.Electrical.EMT.EmtTerminal terminal "Stator three-phase terminal (pu voltages/currents)";
  Modelica.Blocks.Interfaces.RealInput efdPu(start = Efd0Pu) "Normalised field voltage input (= Ifd at steady state)";
  Modelica.Blocks.Interfaces.RealInput PmPu(start = Pm0Pu) "Mechanical power input in pu (base SNom)";
  Modelica.Blocks.Interfaces.RealOutput UStatorPu(start = UStator0Pu) "Stator terminal voltage magnitude in pu (RMS)";
  Modelica.Blocks.Interfaces.RealOutput omegaPu(start = 1) "Rotor speed in pu";
  Modelica.Blocks.Interfaces.RealOutput IRotorPu(start = IRotor0Pu) "Field (rotor) current in pu (same base as efdPu, = Ifd)";
  Modelica.Blocks.Interfaces.RealOutput PGenPu(start = PGen0Pu) "Generated active power in pu (generator convention)";

  parameter Real Ra = 0.005 "Stator resistance in pu";
  parameter Real Lmd = 1.7 "d-axis magnetising inductance in pu";
  parameter Real Lmq = 1.6 "q-axis magnetising inductance in pu";
  parameter Real Ld = 2.0 "d-axis stator self-inductance in pu";
  parameter Real Lq = 1.9 "q-axis stator self-inductance in pu";
  parameter Real L0 = 0.3 "zero-sequence inductance in pu";
  parameter Real Lff = 2.0643 "field self-inductance in pu";
  parameter Real LQQ = 2.3273 "q-damper self-inductance in pu";
  parameter Real Rf = 7.8224e-4 "field resistance in pu";
  parameter Real RQ1 = 0.0088 "q-damper resistance in pu";
  parameter Real H = 5 "inertia constant in s";
  parameter Real FNom = 50 "nominal frequency in Hz";
  parameter Real Theta0 = 0 "initial rotor angle in rad";

  final parameter Real Wbase = 2 * pi * FNom "electrical base speed in rad/s";
  final parameter Real detD = Ld * Lff - Lmd * Lmd "d-block determinant";
  final parameter Real detQ = Lq * LQQ - Lmq * Lmq "q-block determinant";
  final parameter Real sq23 = sqrt(2.0 / 3.0);

  parameter Real Phid0Pu = 1.0 "Initial d-axis flux linkage in pu";
  parameter Real Phiq0Pu = 0 "Initial q-axis flux linkage in pu";
  parameter Real Phifd0Pu = 1.2142941176470588 "Initial field flux linkage in pu";
  parameter Real Phiq10Pu = 0 "Initial q-damper flux linkage in pu";
  parameter Real Efd0Pu = 0.5882352941176471 "Initial field voltage (= Ifd0) for the input start";
  parameter Real Pm0Pu = 0 "Initial mechanical power for the input start";
  parameter Real UStator0Pu = 1 "Initial stator voltage magnitude (RMS) for the output start";
  parameter Real IRotor0Pu = 0.5882352941176471 "Initial field current for the output start (= Ifd0)";
  parameter Real PGen0Pu = 0 "Initial generated active power for the output start";

  Real Phid(start = Phid0Pu, fixed = true) "d-axis flux linkage in pu";
  Real Phiq(start = Phiq0Pu, fixed = true) "q-axis flux linkage in pu";
  Real Phi0(start = 0, fixed = true) "zero-sequence flux linkage in pu";
  Real Phifd(start = Phifd0Pu, fixed = true) "field flux linkage in pu";
  Real Phiq1(start = Phiq10Pu, fixed = true) "q-damper flux linkage in pu";
  Real dw(start = 0, fixed = true) "speed deviation in pu";
  Real dtheta(start = Theta0, fixed = true) "rotor angle deviation from synchronous frame in rad";

  Real theta "electrical rotor angle in rad";
  Real Wr "rotor speed in pu";
  Real Id, Iq, I0, Ifd, Iq1 "winding currents in pu";
  Real Vd, Vq, V0 "Park-transformed stator voltages in pu";
  Real Te "electromagnetic torque in pu";

equation
  theta = dtheta + Wbase * time;
  Wr = 1 + dw;
  omegaPu = Wr;
  // RMS magnitude of the abc terminal voltage (peak/sqrt(2) = sqrt((va^2+vb^2+vc^2)/3))
  UStatorPu = sqrt((terminal.v[1] ^ 2 + terminal.v[2] ^ 2 + terminal.v[3] ^ 2) / 3.0);
  // Field current (same base as efdPu) and generated active power (instantaneous 3-phase,
  // = constant in balanced steady state, with 2nd-harmonic ripple under unbalance)
  IRotorPu = Ifd;
  PGenPu = -(terminal.v[1] * terminal.i[1] + terminal.v[2] * terminal.i[2] + terminal.v[3] * terminal.i[3]);

  // running: terminal voltage drives the dq voltages. Off: stator open (Id=Iq=I0=0,
  // terminal.i = 0); the terminal voltage floats and the rotor dynamics continue.
  if running.value then
    Vd = sq23 * (cos(theta) * terminal.v[1] + cos(theta - 2 * pi / 3) * terminal.v[2] + cos(theta + 2 * pi / 3) * terminal.v[3]);
    Vq = sq23 * (sin(theta) * terminal.v[1] + sin(theta - 2 * pi / 3) * terminal.v[2] + sin(theta + 2 * pi / 3) * terminal.v[3]);
    V0 = (terminal.v[1] + terminal.v[2] + terminal.v[3]) / sqrt(3.0);
  else
    Id = 0;
    Iq = 0;
    I0 = 0;
  end if;

  Id = (Lff * Phid - Lmd * Phifd) / detD;
  Ifd = (-Lmd * Phid + Ld * Phifd) / detD;
  Iq = (LQQ * Phiq - Lmq * Phiq1) / detQ;
  Iq1 = (-Lmq * Phiq + Lq * Phiq1) / detQ;
  I0 = Phi0 / L0;

  der(Phid) = -Wbase * (Ra * Id + Wr * Phiq + Vd);
  der(Phiq) = -Wbase * (Ra * Iq - Wr * Phid + Vq);
  der(Phi0) = -Wbase * (Ra * I0 + V0);
  der(Phifd) = -Wbase * Rf * (Ifd - efdPu);
  der(Phiq1) = -Wbase * (RQ1 * Iq1);

  Te = Phid * Iq - Phiq * Id;
  der(dw) = (PmPu / Wr - Te) / (2 * H);
  der(dtheta) = dw * Wbase;

  terminal.i[1] = -sq23 * (cos(theta) * Id + sin(theta) * Iq + I0 / sqrt(2.0));
  terminal.i[2] = -sq23 * (cos(theta - 2 * pi / 3) * Id + sin(theta - 2 * pi / 3) * Iq + I0 / sqrt(2.0));
  terminal.i[3] = -sq23 * (cos(theta + 2 * pi / 3) * Id + sin(theta + 2 * pi / 3) * Iq + I0 / sqrt(2.0));

  annotation(preferredView = "text");
end SynchronousMachine;
