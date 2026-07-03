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

model SynchronousMachineFull "Full-detail EMT synchronous machine: the same physical equivalent circuit as the phasor GeneratorSynchronous (d/f/D and q/Q1/Q2 windings, Canay mutual, magnetic saturation) but formulated natively for full EMT - instantaneous abc terminal, stator electromagnetic transients kept, absolute rotor frame (no omegaRef), and saturation evaluated on the instantaneous air-gap flux"

  /*
    Physics identical in detail to the RMS machine, valid for full EMT:
      - windings d, q, 0, field f, d-damper D, q-dampers Q1 & Q2 (7 magnetic),
      - Canay's differential leakage MrcPu between field and d-damper,
      - Shackshaft cross-saturation, evaluated on the INSTANTANEOUS air-gap flux
        (in balanced steady state lambda_dq is constant, so this matches the RMS
        saturation exactly; under unbalance/faults it saturates instantaneously,
        which is the physical EMT behaviour). The 1/lambdaAirGap^2 weighting is
        singularity-guarded so instantaneous flux may pass through zero.
    Differences from the RMS model are exactly the EMT content:
      - stator voltage equations keep der(lambda)/omegaNom (no fundamental-freq
        approximation),
      - a zero-sequence winding (lambda0 = L0Pu*i0),
      - the terminal is the instantaneous three-phase EmtTerminal; the abc<->dq
        Park layer (d on sin, q on cos, factor Cabc = sqrt(2)/3) puts dq in RMS-pu
        so lambda is the physical flux the saturation curve expects,
      - the rotor frame is ABSOLUTE (der(theta) = omegaPu*omegaNom); there is no
        rotating reference frame and hence no omegaRef. The swing damping uses the
        slip from nominal speed, DPu*(omegaPu - 1).
    Parameters are the physical equivalent-circuit values directly (no plate-data
    Xd'/Td0' conversion; an optional converter can feed them from datasheet data).
  */

  import Modelica.Constants.pi;
  import Dynawo.Electrical.SystemBase;

  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffGenerator;

  Dynawo.Electrical.EMT.EmtTerminal terminal "Stator three-phase terminal (instantaneous pu voltages/currents)";

  // Input variables (no omegaRef: the EMT machine runs in the absolute frame)
  Modelica.Blocks.Interfaces.RealInput PmPu(start = Pm0Pu) "Mechanical power in pu (base SNom)";
  Modelica.Blocks.Interfaces.RealInput efdPu(start = Efd0Pu) "Normalised field voltage in pu (= field current at steady state)";

  // Output variables
  Modelica.Blocks.Interfaces.RealOutput omegaPu(start = 1) "Rotor speed in pu";
  Modelica.Blocks.Interfaces.RealOutput UStatorPu(start = UStator0Pu) "Stator voltage magnitude in pu (RMS, base UNom)";
  Modelica.Blocks.Interfaces.RealOutput IRotorPu(start = IRotor0Pu) "Field (rotor) current in pu (= efdPu base)";
  Modelica.Blocks.Interfaces.RealOutput PGenPu(start = PGen0Pu) "Generated active power in pu (base SnRef, generator convention)";

  // Physical equivalent-circuit parameters (base UNom, SNom) - supplied directly
  parameter Types.ApparentPowerModule SNom = 100 "Nominal apparent power in MVA";
  parameter Types.PerUnit RaPu = 0.003 "Armature resistance in pu";
  parameter Types.PerUnit LdPu = 0.15 "Direct axis stator leakage in pu";
  parameter Types.PerUnit MdPu = 1.66 "Direct axis magnetising (mutual) inductance in pu";
  parameter Types.PerUnit LfPu = 0.165 "Excitation winding leakage in pu";
  parameter Types.PerUnit RfPu = 0.0006 "Excitation winding resistance in pu";
  parameter Types.PerUnit LDPu = 0.16 "Direct axis damper leakage in pu";
  parameter Types.PerUnit RDPu = 0.03 "Direct axis damper resistance in pu";
  parameter Types.PerUnit MrcPu = 0 "Canay's mutual (differential field/damper) inductance in pu";
  parameter Types.PerUnit LqPu = 0.15 "Quadrature axis stator leakage in pu";
  parameter Types.PerUnit MqPu = 1.61 "Quadrature axis magnetising (mutual) inductance in pu";
  parameter Types.PerUnit LQ1Pu = 0.07 "Quadrature axis 1st damper leakage in pu";
  parameter Types.PerUnit RQ1Pu = 0.0062 "Quadrature axis 1st damper resistance in pu";
  parameter Types.PerUnit LQ2Pu = 0.21 "Quadrature axis 2nd damper leakage in pu";
  parameter Types.PerUnit RQ2Pu = 0.024 "Quadrature axis 2nd damper resistance in pu";
  parameter Types.PerUnit L0Pu = 0.15 "Zero-sequence inductance in pu";
  parameter Types.Time H = 3.5 "Inertia constant in s";
  parameter Types.PerUnit DPu = 0 "Swing-equation damping coefficient in pu (lumped; damper windings already provide electromagnetic damping)";
  parameter Types.AngularVelocity FNom = 50 "Nominal frequency in Hz";

  // Saturation parameters (Shackshaft); all zero => no saturation
  parameter Real md = 0 "Direct axis saturation parameter (0 = no saturation)";
  parameter Real mq = 0 "Quadrature axis saturation parameter (0 = no saturation)";
  parameter Real nd = 2 "Direct axis saturation exponent (kept > 0 to avoid a singular Jacobian when unsaturated)";
  parameter Real nq = 2 "Quadrature axis saturation exponent (kept > 0 to avoid a singular Jacobian when unsaturated)";

  final parameter Types.AngularVelocity omegaNom = 2 * pi * FNom "Nominal electrical speed in rad/s";

  // Start values produced by the init model (transferred by name through the preassembled model)
  parameter Types.PerUnit MsalPu "Constant difference between d and q saturated mutual inductances in pu";
  parameter Types.PerUnit Efd0Pu "Start value of normalised field voltage in pu (= If0)";
  parameter Types.PerUnit Pm0Pu "Start value of mechanical power in pu (base SNom)";
  parameter Types.Angle Theta0 "Start value of absolute electrical rotor angle in rad";
  parameter Types.PerUnit Id0Pu "Start value of d-axis current in pu";
  parameter Types.PerUnit Iq0Pu "Start value of q-axis current in pu";
  parameter Types.PerUnit If0Pu "Start value of excitation winding current in pu";
  parameter Types.PerUnit Lambdad0Pu "Start value of d-axis flux in pu";
  parameter Types.PerUnit Lambdaq0Pu "Start value of q-axis flux in pu";
  parameter Types.PerUnit LambdaD0Pu "Start value of d-axis damper flux in pu";
  parameter Types.PerUnit Lambdaf0Pu "Start value of excitation winding flux in pu";
  parameter Types.PerUnit LambdaQ10Pu "Start value of q-axis 1st damper flux in pu";
  parameter Types.PerUnit LambdaQ20Pu "Start value of q-axis 2nd damper flux in pu";
  parameter Types.PerUnit Ce0Pu "Start value of electrical torque in pu";
  parameter Types.PerUnit MdSat0PPu "Start value of d-axis saturated mutual inductance in pu";
  parameter Types.PerUnit MqSat0PPu "Start value of q-axis saturated mutual inductance in pu";
  parameter Types.VoltageModulePu UStator0Pu "Start value of stator voltage magnitude in pu (RMS)";
  parameter Types.PerUnit IRotor0Pu "Start value of field current in pu";
  parameter Types.ActivePowerPu PGen0Pu "Start value of generated active power in pu (base SnRef)";

  // d-q-0 axis variables (base UNom, SNom)
  Types.PerUnit udPu "Voltage of direct axis in pu";
  Types.PerUnit uqPu "Voltage of quadrature axis in pu";
  Types.PerUnit u0Pu "Zero-sequence voltage in pu";
  Types.PerUnit idPu(start = Id0Pu) "Current of direct axis in pu";
  Types.PerUnit iqPu(start = Iq0Pu) "Current of quadrature axis in pu";
  Types.PerUnit i0Pu(start = 0) "Zero-sequence current in pu";
  Types.PerUnit iDPu(start = 0) "Current of direct axis damper in pu";
  Types.PerUnit iQ1Pu(start = 0) "Current of quadrature axis 1st damper in pu";
  Types.PerUnit iQ2Pu(start = 0) "Current of quadrature axis 2nd damper in pu";
  Types.PerUnit ifPu(start = If0Pu) "Current of excitation winding in pu";
  Types.PerUnit ufPu(start = RfPu * If0Pu) "Voltage of exciter winding in pu";

  // Flux linkages (stator d/q/0 are differential states -> EMT transients)
  Types.PerUnit lambdadPu(start = Lambdad0Pu, fixed = true) "Flux of direct axis in pu";
  Types.PerUnit lambdaqPu(start = Lambdaq0Pu, fixed = true) "Flux of quadrature axis in pu";
  Types.PerUnit lambda0Pu(start = 0, fixed = true) "Zero-sequence flux in pu";
  Types.PerUnit lambdafPu(start = Lambdaf0Pu, fixed = true) "Flux of excitation winding in pu";
  Types.PerUnit lambdaDPu(start = LambdaD0Pu, fixed = true) "Flux of direct axis damper in pu";
  Types.PerUnit lambdaQ1Pu(start = LambdaQ10Pu, fixed = true) "Flux of quadrature axis 1st damper in pu";
  Types.PerUnit lambdaQ2Pu(start = LambdaQ20Pu, fixed = true) "Flux of quadrature axis 2nd damper in pu";

  // Mechanical / electrical
  Types.Angle theta(start = Theta0, fixed = true) "Absolute electrical rotor angle in rad";
  Types.PerUnit cePu(start = Ce0Pu) "Electrical torque in pu (base SNom)";

  // Saturation block (instantaneous; singularity-guarded)
  Types.PerUnit MdSatPPu(start = MdSat0PPu) "Direct axis saturated mutual inductance in pu";
  Types.PerUnit MqSatPPu(start = MqSat0PPu) "Quadrature axis saturated mutual inductance in pu";
  Types.PerUnit lambdaADPu "Common flux of direct axis in pu";
  Types.PerUnit lambdaAQPu "Common flux of quadrature axis in pu";
  Types.PerUnit lambdaAirGapPu "Total air gap flux in pu";
  Types.PerUnit mdsPu "Direct axis saturated mutual inductance, air gap flux on d, in pu";
  Types.PerUnit mqsPu "Quadrature axis saturated mutual inductance, air gap flux on q, in pu";
  Types.PerUnit cos2Eta "Direct axis contribution to the air gap flux";
  Types.PerUnit sin2Eta "Quadrature axis contribution to the air gap flux";
  Types.PerUnit miPu "Intermediate axis saturated mutual inductance in pu";

  // abc<->dq Park scaling. The FORWARD factor Cabc = sqrt(2)/3 keeps the dq quantities in RMS-pu
  // (so lambda is the physical flux the saturation curve expects). The INVERSE current factor Cdq
  // = sqrt(2) renders the dq current as a STANDARD abc waveform (peak = sqrt(2)*Irms, line current
  // = S/U pu) -- the same convention the C++ EMT network and the other Modelica EMT models use, so
  // a machine and any other model connect to a bus with NO per-unit bridge. (The internal dq
  // dynamics are unchanged; only the abc terminal rendering is rescaled. The electrical power is
  // taken from the dq product ud*id+uq*iq below, which is invariant to this scaling.)
  final parameter Real Cabc = sqrt(2.0) / 3.0 "abc->dq forward Park factor (RMS-pu)";
  final parameter Real Cdq = sqrt(2.0) "dq->abc inverse Park factor (standard abc: peak = sqrt(2)*Irms)";
  final parameter Real Ki = SNom / SystemBase.SnRef "machine (SNom) -> system (SnRef) current base conversion";
  final constant Real epsLambda = 1e-8 "Air-gap flux singularity guard";

equation
  // ---- abc -> dq0 (phasor dq convention: d on sin, q on cos), absolute rotor angle theta ----
  // When running, the terminal voltage drives the dq voltages. When switched off the stator
  // is open: the stator currents id/iq/i0 are zero (so terminal.i below is zero) and the
  // terminal voltage floats with the grid; the field/damper/rotor dynamics keep running.
  if running.value then
    udPu = Cabc * (sin(theta) * terminal.v[1] + sin(theta - 2 * pi / 3) * terminal.v[2] + sin(theta + 2 * pi / 3) * terminal.v[3]);
    uqPu = Cabc * (cos(theta) * terminal.v[1] + cos(theta - 2 * pi / 3) * terminal.v[2] + cos(theta + 2 * pi / 3) * terminal.v[3]);
    u0Pu = (terminal.v[1] + terminal.v[2] + terminal.v[3]) / sqrt(3.0);
  else
    idPu = 0;
    iqPu = 0;
    i0Pu = 0;
  end if;

  // ---- dq0 -> abc stator current (receptor convention, i > 0 entering), base SnRef ----
  terminal.i[1] = Ki * Cdq * (sin(theta) * idPu + cos(theta) * iqPu + i0Pu / sqrt(2.0));
  terminal.i[2] = Ki * Cdq * (sin(theta - 2 * pi / 3) * idPu + cos(theta - 2 * pi / 3) * iqPu + i0Pu / sqrt(2.0));
  terminal.i[3] = Ki * Cdq * (sin(theta + 2 * pi / 3) * idPu + cos(theta + 2 * pi / 3) * iqPu + i0Pu / sqrt(2.0));

  // ---- Flux linkages (physical equivalent circuit; magnetising inductance saturates) ----
  lambdadPu = (MdSatPPu + LdPu) * idPu + MdSatPPu * ifPu + MdSatPPu * iDPu;
  lambdafPu = MdSatPPu * idPu + (MdSatPPu + LfPu + MrcPu) * ifPu + (MdSatPPu + MrcPu) * iDPu;
  lambdaDPu = MdSatPPu * idPu + (MdSatPPu + MrcPu) * ifPu + (MdSatPPu + LDPu + MrcPu) * iDPu;
  lambdaqPu = (MqSatPPu + LqPu) * iqPu + MqSatPPu * iQ1Pu + MqSatPPu * iQ2Pu;
  lambdaQ1Pu = MqSatPPu * iqPu + (MqSatPPu + LQ1Pu) * iQ1Pu + MqSatPPu * iQ2Pu;
  lambdaQ2Pu = MqSatPPu * iqPu + MqSatPPu * iQ1Pu + (MqSatPPu + LQ2Pu) * iQ2Pu;
  lambda0Pu = L0Pu * i0Pu;

  // ---- Stator voltage equations in Park coordinates, WITH the EMT transients der(lambda)/omegaNom ----
  udPu = RaPu * idPu + der(lambdadPu) / omegaNom - omegaPu * lambdaqPu;
  uqPu = RaPu * iqPu + der(lambdaqPu) / omegaNom + omegaPu * lambdadPu;
  u0Pu = RaPu * i0Pu + der(lambda0Pu) / omegaNom;

  // ---- Rotor winding voltage equations ----
  ufPu = RfPu * ifPu + der(lambdafPu) / omegaNom;
  0 = RDPu * iDPu + der(lambdaDPu) / omegaNom;
  0 = RQ1Pu * iQ1Pu + der(lambdaQ1Pu) / omegaNom;
  0 = RQ2Pu * iQ2Pu + der(lambdaQ2Pu) / omegaNom;

  // ---- Excitation: efdPu is the normalised field voltage (= field current at steady state) ----
  ufPu = RfPu * efdPu;

  // ---- Mechanical equations (absolute frame; damping is the slip from nominal speed) ----
  der(theta) = omegaPu * omegaNom;
  2 * H * der(omegaPu) = PmPu / omegaPu - cePu - DPu * (omegaPu - 1);
  cePu = lambdaqPu * idPu - lambdadPu * iqPu;

  // ---- Mutual inductance saturation (Shackshaft), evaluated on the INSTANTANEOUS flux ----
  // EMT-native, loop-free: the saturation argument is the instantaneous stator flux state
  // (lambdad, lambdaq) rather than the MdSat*current air-gap flux. This keeps saturation
  // instantaneous but makes MdSat/MqSat an EXPLICIT function of the states, so the flux->
  // current inversion stays linear (no singular algebraic loop, unlike the phasor causality).
  // It differs from the air-gap flux only by the small leakage flux.
  lambdaADPu = lambdadPu;
  lambdaAQPu = lambdaqPu;
  lambdaAirGapPu = sqrt(lambdaADPu ^ 2 + lambdaAQPu ^ 2 + epsLambda);
  mdsPu = MdPu / (1 + md * lambdaAirGapPu ^ nd);
  mqsPu = MqPu / (1 + mq * lambdaAirGapPu ^ nq);
  cos2Eta = lambdaADPu ^ 2 / (lambdaAirGapPu ^ 2);
  sin2Eta = lambdaAQPu ^ 2 / (lambdaAirGapPu ^ 2);
  miPu = mdsPu * cos2Eta + mqsPu * sin2Eta;
  MdSatPPu = miPu + MsalPu * sin2Eta;
  MqSatPPu = miPu - MsalPu * cos2Eta;

  // ---- Output measurements ----
  UStatorPu = sqrt(udPu ^ 2 + uqPu ^ 2);
  IRotorPu = ifPu;
  // electrical power from the dq product (RMS-pu, base SnRef): invariant to the abc current scaling
  // (Cdq), unlike sum(terminal.v*terminal.i) which is in the abc base. Generator convention (>0 out).
  PGenPu = -(udPu * idPu + uqPu * iqPu + u0Pu * i0Pu);

  annotation(preferredView = "text");
end SynchronousMachineFull;
