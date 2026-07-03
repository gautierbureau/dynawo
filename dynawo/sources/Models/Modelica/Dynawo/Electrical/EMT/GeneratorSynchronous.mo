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

model GeneratorSynchronous "EMT synchronous machine matching the phasor OmegaRef.GeneratorSynchronous one-for-one: same windings (d, f, D / q, Q1, Q2), Canay mutual, full Shackshaft saturation, integrated step-up transformer and excitation pu, but with the EMT stator electromagnetic transients retained (der(lambda_stator)), a zero-sequence winding, and an instantaneous abc terminal"
  extends Dynawo.Electrical.Machines.OmegaRef.BaseClasses.GeneratorSynchronousParameters;
  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffGenerator;

  /*
    Relationship to the phasor model (Dynawo.Electrical.Machines.OmegaRef.BaseGeneratorSynchronous):
      - identical magnetic model: the same internal pu parameters (RaPPu, LdPPu,
        MdPPu, LDPPu, RDPPu, MrcPPu, LfPPu, RfPPu, LqPPu, MqPPu, LQ1PPu, RQ1PPu,
        LQ2PPu, RQ2PPu, MsalPu) and the same Shackshaft cross-saturation block,
        all produced by the SAME init (BaseGeneratorSynchronousInt_INIT, reused).
      - identical flux linkages, rotor (f, D, Q1, Q2) voltage equations,
        mechanical swing (DPu damping + omegaRef coupling), Kuf excitation pu.
      - DIFFERENCES (the EMT content):
          * stator voltage equations KEEP der(lambda)/omegaNom (the phasor drops
            it -> fundamental-frequency only),
          * a zero-sequence winding (lambda0Pu = L0Pu*i0Pu),
          * the terminal is the instantaneous three-phase EmtTerminal; the abc<->dq
            Park layer uses the phasor's dq convention (d on sin, q on cos) with the
            RMS-pu factor Cabc = sqrt(2)/3 so lambda is numerically the phasor flux
            (REQUIRED for the saturation curves to match).
    The rotor angle theta here is ABSOLUTE (der(theta) = omegaPu*omegaNom), unlike
    the phasor theta which is relative to the omegaRef port frame; theta(0) = Theta0.
  */

  import Modelica.Constants.pi;
  import Dynawo.Electrical.SystemBase;

  Dynawo.Electrical.EMT.EmtTerminal terminal(v(start = uTerminal0Pu)) "Stator three-phase terminal (instantaneous pu voltages/currents)";

  // Input variables (same signal interface as the phasor machine)
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference frequency in pu";
  Modelica.Blocks.Interfaces.RealInput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNomTurb)";
  Modelica.Blocks.Interfaces.RealInput efdPu(start = Efd0Pu) "Input voltage of exciter winding in pu (user-selected base)";

  // Output variables
  Modelica.Blocks.Interfaces.RealOutput omegaPu(start = SystemBase.omega0Pu) "Rotor speed in pu";
  Modelica.Blocks.Interfaces.RealOutput UStatorPu(start = UStator0Pu) "Stator voltage magnitude in pu (RMS, base UNom)";
  Modelica.Blocks.Interfaces.RealOutput IRotorPu(start = IRotor0Pu) "Rotor (field) current in pu (same base as efdPu)";
  Modelica.Blocks.Interfaces.RealOutput PGenPu(start = PGen0Pu) "Generated active power in pu (base SnRef, generator convention)";

  // Internal parameters of the synchronous machine in pu (base UNom, SNom), produced by the init
  parameter Types.PerUnit RaPPu "Armature resistance in pu";
  parameter Types.PerUnit LdPPu "Direct axis stator leakage in pu";
  parameter Types.PerUnit MdPPu "Direct axis mutual inductance in pu";
  parameter Types.PerUnit LDPPu "Direct axis damper leakage in pu";
  parameter Types.PerUnit RDPPu "Direct axis damper resistance in pu";
  parameter Types.PerUnit MrcPPu "Canay's mutual inductance in pu";
  parameter Types.PerUnit LfPPu "Excitation winding leakage in pu";
  parameter Types.PerUnit RfPPu "Excitation winding resistance in pu";
  parameter Types.PerUnit LqPPu "Quadrature axis stator leakage in pu";
  parameter Types.PerUnit MqPPu "Quadrature axis mutual inductance in pu";
  parameter Types.PerUnit LQ1PPu "Quadrature axis 1st damper leakage in pu";
  parameter Types.PerUnit RQ1PPu "Quadrature axis 1st damper resistance in pu";
  parameter Types.PerUnit LQ2PPu "Quadrature axis 2nd damper leakage in pu";
  parameter Types.PerUnit RQ2PPu "Quadrature axis 2nd damper resistance in pu";
  parameter Types.PerUnit MsalPu "Constant difference between direct and quadrature axis saturated mutual inductances in pu";
  parameter Types.PerUnit MdPPuEfd "Direct axis mutual inductance used to determine the excitation voltage in pu";
  parameter Types.PerUnit MdPPuEfdNom "Direct axis mutual inductance used to determine the excitation voltage in nominal conditions in pu";
  final parameter Types.PerUnit Kuf = if ExcitationPu == ExcitationPuType.Kundur then 1 elseif ExcitationPu == ExcitationPuType.UserBase then RfPPu / MdPPuEfd elseif ExcitationPu == ExcitationPuType.NoLoad then RfPPu / MdPPu elseif ExcitationPu == ExcitationPuType.NoLoadSaturated then RfPPu * (1 + md) / MdPPu else RfPPu / MdPPuEfdNom "Scaling factor for excitation pu voltage";

  // EMT-specific parameter (the phasor model is positive-sequence and has no zero-sequence inductance)
  parameter Types.PerUnit L0Pu = 0.15 "Zero-sequence inductance in pu";

  // Start values produced by the init model (transfer by name through the preassembled model)
  parameter Types.PerUnit Efd0Pu "Start value of input exciter voltage in pu (user-selected base)";
  parameter Types.PerUnit Pm0Pu "Start value of mechanical power in pu (base PNomTurb)";
  parameter Types.Angle Theta0 "Start value of absolute electrical rotor angle in rad";
  parameter Types.PerUnit Ud0Pu "Start value of d-axis voltage in pu";
  parameter Types.PerUnit Uq0Pu "Start value of q-axis voltage in pu";
  parameter Types.PerUnit Id0Pu "Start value of d-axis current in pu";
  parameter Types.PerUnit Iq0Pu "Start value of q-axis current in pu";
  parameter Types.PerUnit If0Pu "Start value of excitation winding current in pu";
  parameter Types.PerUnit Lambdad0Pu "Start value of d-axis flux in pu";
  parameter Types.PerUnit Lambdaq0Pu "Start value of q-axis flux in pu";
  parameter Types.PerUnit LambdaD0Pu "Start value of d-axis damper flux in pu";
  parameter Types.PerUnit Lambdaf0Pu "Start value of excitation winding flux in pu";
  parameter Types.PerUnit LambdaQ10Pu "Start value of q-axis 1st damper flux in pu";
  parameter Types.PerUnit LambdaQ20Pu "Start value of q-axis 2nd damper flux in pu";
  parameter Types.PerUnit Ce0Pu "Start value of electrical torque in pu (base SNom/omegaNom)";
  parameter Types.PerUnit Cm0Pu "Start value of mechanical torque in pu (base PNomTurb/omegaNom)";
  parameter Types.PerUnit MdSat0PPu "Start value of d-axis saturated mutual inductance in pu";
  parameter Types.PerUnit MqSat0PPu "Start value of q-axis saturated mutual inductance in pu";
  parameter Types.PerUnit LambdaAirGap0Pu "Start value of total air gap flux in pu";
  parameter Types.PerUnit LambdaAD0Pu "Start value of common d-axis flux in pu";
  parameter Types.PerUnit LambdaAQ0Pu "Start value of common q-axis flux in pu";
  parameter Types.PerUnit Mds0Pu "Start value of d-axis saturated mutual inductance, air gap flux on d in pu";
  parameter Types.PerUnit Mqs0Pu "Start value of q-axis saturated mutual inductance, air gap flux on q in pu";
  parameter Types.PerUnit Cos2Eta0 "Start value of d-axis contribution to the air gap flux";
  parameter Types.PerUnit Sin2Eta0 "Start value of q-axis contribution to the air gap flux";
  parameter Types.PerUnit Mi0Pu "Start value of intermediate axis saturated mutual inductance in pu";
  parameter Types.VoltageModulePu UStator0Pu "Start value of stator voltage magnitude in pu (RMS)";
  parameter Types.PerUnit IRotor0Pu "Start value of rotor current in pu";
  parameter Types.ActivePowerPu PGen0Pu "Start value of generated active power in pu (base SnRef)";
  parameter Types.PerUnit uTerminal0Pu[3] "Start value of the instantaneous abc terminal voltage at t = 0 in pu (transferred by name from the EMT init)";

  // d-q-0 axis variables (base UNom, SNom)
  Types.PerUnit udPu(start = Ud0Pu) "Voltage of direct axis in pu";
  Types.PerUnit uqPu(start = Uq0Pu) "Voltage of quadrature axis in pu";
  Types.PerUnit u0Pu(start = 0) "Zero-sequence voltage in pu";
  Types.PerUnit idPu(start = Id0Pu) "Current of direct axis in pu";
  Types.PerUnit iqPu(start = Iq0Pu) "Current of quadrature axis in pu";
  Types.PerUnit i0Pu(start = 0) "Zero-sequence current in pu";
  Types.PerUnit iDPu(start = 0) "Current of direct axis damper in pu";
  Types.PerUnit iQ1Pu(start = 0) "Current of quadrature axis 1st damper in pu";
  Types.PerUnit iQ2Pu(start = 0) "Current of quadrature axis 2nd damper in pu";
  Types.PerUnit ifPu(start = If0Pu) "Current of excitation winding in pu";
  Types.PerUnit ufPu(start = RfPPu * If0Pu) "Voltage of exciter winding in pu (Kundur base)";

  // Flux linkages (the stator d/q/0 ones are differential states -> EMT transients)
  Types.PerUnit lambdadPu(start = Lambdad0Pu, fixed = true) "Flux of direct axis in pu";
  Types.PerUnit lambdaqPu(start = Lambdaq0Pu, fixed = true) "Flux of quadrature axis in pu";
  Types.PerUnit lambda0Pu(start = 0, fixed = true) "Zero-sequence flux in pu";
  Types.PerUnit lambdafPu(start = Lambdaf0Pu, fixed = true) "Flux of excitation winding in pu";
  Types.PerUnit lambdaDPu(start = LambdaD0Pu, fixed = true) "Flux of direct axis damper in pu";
  Types.PerUnit lambdaQ1Pu(start = LambdaQ10Pu, fixed = true) "Flux of quadrature axis 1st damper in pu";
  Types.PerUnit lambdaQ2Pu(start = LambdaQ20Pu, fixed = true) "Flux of quadrature axis 2nd damper in pu";

  // Mechanical / electrical
  Types.Angle theta(start = Theta0, fixed = true) "Absolute electrical rotor angle in rad";
  Types.PerUnit cmPu(start = Cm0Pu) "Mechanical torque in pu (base PNomTurb/omegaNom)";
  Types.PerUnit cePu(start = Ce0Pu) "Electrical torque in pu (base SNom/omegaNom)";
  Types.PerUnit PePu(start = Ce0Pu * SystemBase.omega0Pu) "Electrical active power in pu (base SNom)";

  // Saturation block (identical to the phasor model)
  Types.PerUnit MdSatPPu(start = MdSat0PPu) "Direct axis saturated mutual inductance in pu";
  Types.PerUnit MqSatPPu(start = MqSat0PPu) "Quadrature axis saturated mutual inductance in pu";
  Types.PerUnit lambdaADPu(start = LambdaAD0Pu) "Common flux of direct axis in pu";
  Types.PerUnit lambdaAQPu(start = LambdaAQ0Pu) "Common flux of quadrature axis in pu";
  Types.PerUnit lambdaAirGapPu(start = LambdaAirGap0Pu) "Total air gap flux in pu";
  Types.PerUnit mdsPu(start = Mds0Pu) "Direct axis saturated mutual inductance, air gap flux on d, in pu";
  Types.PerUnit mqsPu(start = Mqs0Pu) "Quadrature axis saturated mutual inductance, air gap flux on q, in pu";
  Types.PerUnit cos2Eta(start = Cos2Eta0) "Direct axis contribution to the air gap flux";
  Types.PerUnit sin2Eta(start = Sin2Eta0) "Quadrature axis contribution to the air gap flux";
  Types.PerUnit miPu(start = Mi0Pu) "Intermediate axis saturated mutual inductance in pu";

  // abc<->dq Park scaling: RMS-pu so lambda matches the phasor flux (saturation must see the same air-gap flux)
  final parameter Real Cabc = sqrt(2.0) / 3.0 "abc->dq forward Park factor (RMS-pu convention)";
  final parameter Real Cdq = sqrt(2.0) "dq->abc inverse Park factor (standard abc: peak = sqrt(2)*Irms, one convention across all EMT models)";
  final parameter Real Ki = SNom / SystemBase.SnRef "machine (SNom) -> system (SnRef) current base conversion";

equation
  // ---- abc -> dq0 terminal-voltage projection (phasor dq convention: d on sin, q on cos) ----
  // udPu/uqPu/u0Pu are defined UNCONDITIONALLY as the Park projection of the terminal voltage.
  // (The earlier form put these inside `if running ... else id/iq/i0 = 0`, which the tool rewrites
  // into udPu = (...)/(if running then 1 else 0); the analytic Jacobian of that division carries a
  // /denom^2 term that goes non-finite when a terminal-voltage exciter makes efdPu a free variable
  // and pulls udPu/uqPu into the coupled global-init Jacobian -- the prop-VR NaN. Defining them
  // unconditionally and gating the current injection instead removes the division entirely.)
  udPu = Cabc * (sin(theta) * terminal.v[1] + sin(theta - 2 * pi / 3) * terminal.v[2] + sin(theta + 2 * pi / 3) * terminal.v[3]);
  uqPu = Cabc * (cos(theta) * terminal.v[1] + cos(theta - 2 * pi / 3) * terminal.v[2] + cos(theta + 2 * pi / 3) * terminal.v[3]);
  u0Pu = (terminal.v[1] + terminal.v[2] + terminal.v[3]) / sqrt(3.0);

  // ---- dq0 -> abc stator current (receptor convention, i > 0 entering), base SnRef ----
  // Gated by running: when the machine is switched off the stator terminal is open (no current
  // injected) and the terminal voltage floats; the internal fluxes/currents continue.
  if running.value then
    terminal.i[1] = Ki * Cdq * (sin(theta) * idPu + cos(theta) * iqPu + i0Pu / sqrt(2.0));
    terminal.i[2] = Ki * Cdq * (sin(theta - 2 * pi / 3) * idPu + cos(theta - 2 * pi / 3) * iqPu + i0Pu / sqrt(2.0));
    terminal.i[3] = Ki * Cdq * (sin(theta + 2 * pi / 3) * idPu + cos(theta + 2 * pi / 3) * iqPu + i0Pu / sqrt(2.0));
  else
    terminal.i[1] = 0;
    terminal.i[2] = 0;
    terminal.i[3] = 0;
  end if;

  // ---- Flux linkages (identical to the phasor; XTfoPu folds the step-up transformer reactance in) ----
  lambdadPu = (MdSatPPu + LdPPu + XTfoPu) * idPu + MdSatPPu * ifPu + MdSatPPu * iDPu;
  lambdafPu = MdSatPPu * idPu + (MdSatPPu + LfPPu + MrcPPu) * ifPu + (MdSatPPu + MrcPPu) * iDPu;
  lambdaDPu = MdSatPPu * idPu + (MdSatPPu + MrcPPu) * ifPu + (MdSatPPu + LDPPu + MrcPPu) * iDPu;
  lambdaqPu = (MqSatPPu + LqPPu + XTfoPu) * iqPu + MqSatPPu * iQ1Pu + MqSatPPu * iQ2Pu;
  lambdaQ1Pu = MqSatPPu * iqPu + (MqSatPPu + LQ1PPu) * iQ1Pu + MqSatPPu * iQ2Pu;
  lambdaQ2Pu = MqSatPPu * iqPu + MqSatPPu * iQ1Pu + (MqSatPPu + LQ2PPu) * iQ2Pu;
  lambda0Pu = L0Pu * i0Pu;

  // ---- Stator voltage equations in Park coordinates, WITH the EMT transients der(lambda)/omegaNom ----
  udPu = (RaPPu + RTfoPu) * idPu + der(lambdadPu) / SystemBase.omegaNom - omegaPu * lambdaqPu;
  uqPu = (RaPPu + RTfoPu) * iqPu + der(lambdaqPu) / SystemBase.omegaNom + omegaPu * lambdadPu;
  u0Pu = RaPPu * i0Pu + der(lambda0Pu) / SystemBase.omegaNom;

  // ---- Rotor winding voltage equations (identical to the phasor) ----
  ufPu = RfPPu * ifPu + der(lambdafPu) / SystemBase.omegaNom;
  0 = RDPPu * iDPu + der(lambdaDPu) / SystemBase.omegaNom;
  0 = RQ1PPu * iQ1Pu + der(lambdaQ1Pu) / SystemBase.omegaNom;
  0 = RQ2PPu * iQ2Pu + der(lambdaQ2Pu) / SystemBase.omegaNom;

  // ---- Mechanical equations (absolute rotor angle; DPu damping + omegaRef coupling) ----
  der(theta) = omegaPu * SystemBase.omegaNom;
  2 * H * der(omegaPu) = cmPu * PNomTurb / SNom - cePu - DPu * (omegaPu - omegaRefPu);
  cePu = lambdaqPu * idPu - lambdadPu * iqPu;
  PePu = cePu * omegaPu;
  PmPu = cmPu * omegaPu;

  // ---- Excitation voltage pu conversion (identical to the phasor) ----
  ufPu = efdPu * (Kuf * rTfoPu);

  // ---- Mutual inductance saturation (Shackshaft) ----
  // EMT-native, loop-free: the saturation argument is the instantaneous stator flux state
  // (lambdad, lambdaq) rather than the MdSat*current air-gap flux. In EMT causality the
  // fluxes are states, so the air-gap form closes a singular algebraic loop (MdSat ->
  // lambdaAirGap -> MdSat); the stator-flux form makes MdSat/MqSat explicit functions of
  // the states. Differs from the air-gap flux only by the small leakage flux.
  lambdaADPu = lambdadPu;
  lambdaAQPu = lambdaqPu;
  lambdaAirGapPu = sqrt(lambdaADPu ^ 2 + lambdaAQPu ^ 2 + 1e-8);
  mdsPu = MdPPu / (1 + md * lambdaAirGapPu ^ nd);
  mqsPu = MqPPu / (1 + mq * lambdaAirGapPu ^ nq);
  cos2Eta = lambdaADPu ^ 2 / lambdaAirGapPu ^ 2;
  sin2Eta = lambdaAQPu ^ 2 / lambdaAirGapPu ^ 2;
  miPu = mdsPu * cos2Eta + mqsPu * sin2Eta;
  MdSatPPu = miPu + MsalPu * sin2Eta;
  MqSatPPu = miPu - MsalPu * cos2Eta;

  // ---- Output measurements ----
  // Terminal-voltage magnitude for the AVR: instantaneous abc RMS taken DIRECTLY from the node
  // voltages, sqrt((va^2+vb^2+vc^2)/3) (numerically equal to sqrt(ud^2+uq^2) for a balanced set).
  // Routed through terminal.v rather than udPu/uqPu on purpose: when a voltage exciter consumes
  // UStatorPu, sensing via udPu/uqPu pulls the dq stator-voltage variables (which carry the
  // der(lambda) stator-transient coupling) into the exciter's global-init Jacobian, where the
  // derivative entry goes non-finite. Sensing from abc directly matches the 2-axis EMT machine.
  UStatorPu = sqrt((terminal.v[1] ^ 2 + terminal.v[2] ^ 2 + terminal.v[3] ^ 2) / 3.0 + 1e-8);
  IRotorPu = RfPPu / (Kuf * rTfoPu) * ifPu;
  // electrical power from the dq product (invariant to the abc current scaling Cdq), generator convention
  PGenPu = -(udPu * idPu + uqPu * iqPu + u0Pu * i0Pu);

  annotation(preferredView = "text");
end GeneratorSynchronous;
