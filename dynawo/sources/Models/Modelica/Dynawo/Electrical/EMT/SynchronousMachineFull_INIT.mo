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

model SynchronousMachineFull_INIT "Native EMT initialization for SynchronousMachineFull: steady-state operating point (balanced -> identical to the phasor) from the terminal P0/Q0/U0/UPhase0 and the physical parameters, reusing only the saturation/rotor-position math kernel"

  import Modelica.ComplexMath;
  import Dynawo.Electrical.SystemBase;
  import Dynawo.Electrical.Machines.OmegaRef.BaseClasses_INIT.RotorPositionEstimation;

  // Terminal operating point (from the load flow / IIDM, receptor convention)
  parameter Types.ActivePowerPu P0Pu = -0.7 "Active power at the terminal in pu (base SnRef, receptor convention)";
  parameter Types.ReactivePowerPu Q0Pu = -0.2 "Reactive power at the terminal in pu (base SnRef, receptor convention)";
  parameter Types.VoltageModulePu U0Pu = 1.0 "Terminal voltage magnitude in pu (RMS, base UNom)";
  parameter Types.Angle UPhase0 = 0 "Terminal voltage angle in rad";

  // Physical equivalent-circuit parameters (must match the dynamic model)
  parameter Types.ApparentPowerModule SNom = 100 "Nominal apparent power in MVA";
  parameter Types.PerUnit RaPu = 0.003 "Armature resistance in pu";
  parameter Types.PerUnit LdPu = 0.15 "Direct axis stator leakage in pu";
  parameter Types.PerUnit MdPu = 1.66 "Direct axis magnetising inductance in pu";
  parameter Types.PerUnit LfPu = 0.165 "Excitation winding leakage in pu";
  parameter Types.PerUnit RfPu = 0.0006 "Excitation winding resistance in pu";
  parameter Types.PerUnit LDPu = 0.16 "Direct axis damper leakage in pu";
  parameter Types.PerUnit MrcPu = 0 "Canay's mutual inductance in pu";
  parameter Types.PerUnit LqPu = 0.15 "Quadrature axis stator leakage in pu";
  parameter Types.PerUnit MqPu = 1.61 "Quadrature axis magnetising inductance in pu";
  parameter Real md = 0 "Direct axis saturation parameter";
  parameter Real mq = 0 "Quadrature axis saturation parameter";
  parameter Real nd = 0 "Direct axis saturation exponent";
  parameter Real nq = 0 "Quadrature axis saturation exponent";

  // Complex terminal voltage / current (base UNom, SnRef)
  Types.ComplexVoltagePu u0Pu "Complex terminal voltage in pu";
  Types.ComplexCurrentPu i0Pu "Complex terminal current in pu (receptor)";

  // Outputs transferred to the dynamic model by name
  Types.PerUnit MsalPu "Constant difference between d and q saturated mutual inductances in pu";
  Types.Angle Theta0 "Absolute electrical rotor angle in rad";
  Types.PerUnit Ud0Pu "d-axis voltage in pu";
  Types.PerUnit Uq0Pu "q-axis voltage in pu";
  Types.PerUnit Id0Pu "d-axis current in pu";
  Types.PerUnit Iq0Pu "q-axis current in pu";
  Types.PerUnit If0Pu "Excitation winding current in pu";
  Types.PerUnit Lambdad0Pu "d-axis flux in pu";
  Types.PerUnit Lambdaq0Pu "q-axis flux in pu";
  Types.PerUnit LambdaD0Pu "d-axis damper flux in pu";
  Types.PerUnit Lambdaf0Pu "Excitation winding flux in pu";
  Types.PerUnit LambdaQ10Pu "q-axis 1st damper flux in pu";
  Types.PerUnit LambdaQ20Pu "q-axis 2nd damper flux in pu";
  Types.PerUnit Ce0Pu "Electrical torque in pu";
  Types.PerUnit Efd0Pu "Normalised field voltage seed (= If0)";
  Types.PerUnit Pm0Pu "Mechanical power seed in pu (base SNom)";
  Types.VoltageModulePu UStator0Pu "Stator voltage magnitude seed in pu (RMS)";
  Types.PerUnit IRotor0Pu "Field current seed in pu";
  Types.ActivePowerPu PGen0Pu "Generated active power seed in pu (base SnRef)";
  // Saturation start values
  Types.PerUnit LambdaAD0Pu, LambdaAQ0Pu, LambdaAirGap0Pu, Mds0Pu, Mqs0Pu, Cos2Eta0, Sin2Eta0, Mi0Pu;
  Types.PerUnit MdSat0PPu "d-axis saturated mutual inductance in pu";
  Types.PerUnit MqSat0PPu "q-axis saturated mutual inductance in pu";

equation
  // Terminal phasor and current (RMS, base UNom). u0Pu carries UPhase0 so Theta0 is absolute.
  u0Pu = ComplexMath.fromPolar(U0Pu, UPhase0);
  i0Pu = ComplexMath.conj(Complex(P0Pu, Q0Pu) / u0Pu);

  // Rotor position + saturation operating point (identity transformer: rTfo = 1, RTfo = XTfo = 0)
  (MsalPu, Theta0, Ud0Pu, Uq0Pu, Id0Pu, Iq0Pu, LambdaAD0Pu, LambdaAQ0Pu, LambdaAirGap0Pu, Mds0Pu, Mqs0Pu, Cos2Eta0, Sin2Eta0, Mi0Pu, MdSat0PPu, MqSat0PPu)
    = RotorPositionEstimation(u0Pu, i0Pu, MdPu, MqPu, LdPu, LqPu, RaPu, 1.0, 0.0, 0.0, SNom, md, mq, nd, nq);

  // Stator fluxes from the steady-state stator equations (omega0 = 1)
  Lambdad0Pu = Uq0Pu - RaPu * Iq0Pu;
  Lambdaq0Pu = RaPu * Id0Pu - Ud0Pu;

  // Field current from the d-axis flux (dampers carry no current at steady state)
  If0Pu = (Lambdad0Pu - (MdSat0PPu + LdPu) * Id0Pu) / MdSat0PPu;

  // Remaining winding fluxes (iD0 = iQ10 = iQ20 = 0)
  Lambdaf0Pu = MdSat0PPu * Id0Pu + (MdSat0PPu + LfPu + MrcPu) * If0Pu;
  LambdaD0Pu = MdSat0PPu * Id0Pu + (MdSat0PPu + MrcPu) * If0Pu;
  LambdaQ10Pu = MqSat0PPu * Iq0Pu;
  LambdaQ20Pu = MqSat0PPu * Iq0Pu;

  // Torque, seeds and outputs
  Ce0Pu = Lambdaq0Pu * Id0Pu - Lambdad0Pu * Iq0Pu;
  Pm0Pu = Ce0Pu;
  Efd0Pu = If0Pu;
  IRotor0Pu = If0Pu;
  UStator0Pu = U0Pu;
  PGen0Pu = -P0Pu;

  annotation(preferredView = "text");
end SynchronousMachineFull_INIT;
