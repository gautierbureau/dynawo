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

model SMIBLoadedInit_INIT "Initialisation model for SMIBLoadedInit: full load-flow propagation from the generator-terminal operating point"

  /*
    Single _INIT companion that computes EVERY start value of the loaded SMIB
    from one seed: the steady-state operating point at the generator stator
    terminal (peak voltage U0Pu/UPhase0 and output power P0Pu/Q0Pu, as an IIDM
    load flow would provide at that node).

    The machine salient-pole point is solved EXPLICITLY (no nonlinear loop, so
    no init Jacobian to go singular): the q-axis EMF Eq = VG + (Ra + j*Lq)*Ig
    locates the rotor (Theta0 = arg(Eq) + pi/2), then the dq currents/voltages
    and the field current follow by projection. The network is then propagated
    explicitly outward -- generator node -> shunt caps/dampers -> transformer
    -> HV bus -> line -> infinite bus -- giving the capacitor voltages, the
    transformer/line currents and the consistent infinite-bus voltage/phase.

    Outputs are SCALAR phasor magnitudes/angles (and machine *0 scalars); the
    dynamic SMIBLoadedInit turns each magnitude/angle into its abc t=0 triplet
    with Functions.balancedAbcInit. Each output matches a same-named parameter
    of SMIBLoadedInit, so the values transfer by Dynawo's _INIT convention.
    Only the Modelica.ComplexMath subset -- no MSL electrical.
  */

  import Modelica.ComplexMath;

  constant Real PI = 3.141592653589793 "Pi";

  // --- Seed: generator-terminal operating point (peak phasor, pu / rad) ---
  parameter Real U0Pu = 1.293 "Generator terminal voltage magnitude (peak) in pu";
  parameter Real UPhase0 = -0.0717 "Generator terminal voltage angle in rad";
  parameter Real P0Pu = 0.602 "Generator stator output active power in pu";
  parameter Real Q0Pu = 0.024 "Generator stator output reactive power in pu";

  // --- Network parameters (must match SMIBLoadedInit) ---
  parameter Real CPu = 1e-4 "Shunt capacitance per phase in pu (each node)";
  parameter Real RdampPu = 100 "Shunt damping resistance per phase in pu (each node)";
  parameter Real rTfoPu = 1.0 "Transformer turns ratio N1/N2";
  parameter Real RTfoPu = 0.0 "Transformer leakage resistance referred to primary in pu";
  parameter Real LTfoPu = 3.183e-4 "Transformer leakage inductance referred to primary in pu";
  parameter Real RLinePu = 0.02 "Line resistance per phase in pu";
  parameter Real LLinePu = 6.366e-4 "Line inductance per phase in pu";
  parameter Real FNom = 50 "Nominal frequency in Hz";
  final parameter Real Omega = 2 * PI * FNom "Nominal angular frequency in rad/s";

  // --- Machine parameters (must match SynchronousMachine / SMIBLoadedInit) ---
  parameter Real Ra = 0.005, Lmd = 1.7, Lmq = 1.6, Ld = 2.0, Lq = 1.9, Lff = 2.0643, LQQ = 2.3273;
  final parameter Real sq23 = sqrt(2.0 / 3.0);

  // Machine quantities, all computed EXPLICITLY (no nonlinear solve)
  Real Id, Iq, Ifd, Theta0, Vd, Vq;
  Complex VG "Generator-terminal voltage phasor (peak)";
  Complex Ig "Generator stator output-current phasor (peak), injected into node G";
  Complex Eq "q-axis EMF locating the rotor: Eq = VG + (Ra + j*Lq)*Ig";
  Complex rot "exp(j*Theta0)";
  Complex IdMinusjIq "Ig / (sqrt(2/3)*rot) = Id - j*Iq";
  Complex VdMinusjVq "VG / (sqrt(2/3)*rot) = Vd - j*Vq";

  // Network phasors (peak)
  Complex IcapG, IdampG, Itfo, VB, ItfoSecOut, IcapB, IdampB, Iline, VI;

  // --- Outputs: top-level start parameters of SMIBLoadedInit (transferred by name), all scalar ---
  Real machinePhid0Pu, machinePhiq0Pu, machinePhifd0Pu, machinePhiq10Pu, machinePm, machineIfd0, machineTheta0;
  Real capGenUMagPu, capGenUAngleRad "Generator-node voltage phasor (peak)";
  Real capBusUMagPu, capBusUAngleRad "HV-bus voltage phasor (peak)";
  Real tfoIMagPu, tfoIAngleRad "Transformer primary current phasor (peak)";
  Real lineIMagPu, lineIAngleRad "Line current phasor (peak)";
  Real infiniteBusUPeakPu, infiniteBusPhase0 "Infinite-bus voltage phasor (peak)";

equation
  // ---- Machine salient-pole steady state, solved EXPLICITLY ----
  VG = ComplexMath.fromPolar(U0Pu, UPhase0);
  Ig = ComplexMath.conj(Complex(P0Pu, Q0Pu)) / (Complex(1.5, 0) * ComplexMath.conj(VG));
  // q-axis EMF behind (Ra + j*Xq), Xq = Lq in pu at steady state, locates the rotor q-axis
  Eq = VG + Complex(Ra, Lq) * Ig;
  Theta0 = atan2(Eq.im, Eq.re) + PI / 2;
  rot = ComplexMath.exp(Complex(0, Theta0));
  // Project the (known) current and voltage phasors onto the rotor dq frame
  IdMinusjIq = Ig / (Complex(sq23, 0) * rot);
  Id = IdMinusjIq.re;
  Iq = -IdMinusjIq.im;
  VdMinusjVq = VG / (Complex(sq23, 0) * rot);
  Vd = VdMinusjVq.re;
  Vq = -VdMinusjVq.im;
  // Field current from the q-axis voltage equation Vq = Ld*Id + Lmd*Ifd - Ra*Iq
  Ifd = (Vq + Ra * Iq - Ld * Id) / Lmd;

  machinePhid0Pu = Ld * Id + Lmd * Ifd;
  machinePhiq0Pu = Lq * Iq;
  machinePhifd0Pu = Lmd * Id + Lff * Ifd;
  machinePhiq10Pu = Lmq * Iq;
  machinePm = machinePhid0Pu * Iq - machinePhiq0Pu * Id;
  machineIfd0 = Ifd;
  machineTheta0 = Theta0;

  // ---- Explicit network propagation: node G -> transformer -> node B -> line -> infinite bus ----
  // Node G (generator terminal): Ig (injected) = IcapG + IdampG + Itfo
  IcapG = Complex(0, Omega * CPu) * VG;
  IdampG = VG / Complex(RdampPu, 0);
  Itfo = Ig - IcapG - IdampG;
  // Transformer: VG = rTfo*VB + (R + j*Omega*L)*Itfo  ->  VB ; current delivered to node B = rTfo*Itfo
  VB = (VG - Complex(RTfoPu, Omega * LTfoPu) * Itfo) / Complex(rTfoPu, 0);
  ItfoSecOut = Complex(rTfoPu, 0) * Itfo;
  // Node B (HV bus): ItfoSecOut = IcapB + IdampB + Iline
  IcapB = Complex(0, Omega * CPu) * VB;
  IdampB = VB / Complex(RdampPu, 0);
  Iline = ItfoSecOut - IcapB - IdampB;
  // Line: VB - VI = (R + j*Omega*L)*Iline  ->  infinite-bus voltage VI
  VI = VB - Complex(RLinePu, Omega * LLinePu) * Iline;

  // ---- Scalar phasor magnitudes/angles handed to the dynamic model ----
  capGenUMagPu = U0Pu;
  capGenUAngleRad = UPhase0;
  capBusUMagPu = sqrt(VB.re ^ 2 + VB.im ^ 2);
  capBusUAngleRad = atan2(VB.im, VB.re);
  tfoIMagPu = sqrt(Itfo.re ^ 2 + Itfo.im ^ 2);
  tfoIAngleRad = atan2(Itfo.im, Itfo.re);
  lineIMagPu = sqrt(Iline.re ^ 2 + Iline.im ^ 2);
  lineIAngleRad = atan2(Iline.im, Iline.re);
  infiniteBusUPeakPu = sqrt(VI.re ^ 2 + VI.im ^ 2);
  infiniteBusPhase0 = atan2(VI.im, VI.re);

  annotation(preferredView = "text");
end SMIBLoadedInit_INIT;
