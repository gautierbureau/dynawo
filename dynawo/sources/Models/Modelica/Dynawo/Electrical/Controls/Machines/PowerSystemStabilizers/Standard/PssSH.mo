within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

/*
* Copyright (c) 2024, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model PssSH "Siemens H-infinity power system stabilizer (CGMES PssSH)"
  /*
    Multi-loop H-infinity controller with electrical power input. The input
    P is low-pass filtered (TD) into V1, and the leftmost subtractor
    computes V2 = V1 - P - V3 - V4 - V5 - V6 -- subtracting the raw P
    gives a built-in washout (V1 - P = -s*TD*P/(1+s*TD)) so the chain
    is excited only by transients in P. Four integrators 1/(s*Ti),
    i = 1..4, generate V3..V6 in a cascade from V2. The PSS output is
    V7 = K0*V2 + K1*V3 + K2*V4 + K3*V5 + K4*V6 (V1 does NOT appear in
    V7), scaled by the main gain K and clipped to [VSMIN, VSMAX]; this
    structure guarantees a zero steady-state output. Corresponds to the
    CGMES IEC 61970-302 PssSH model (NEPLAN PSSSH). Block-based: the
    equation section contains only connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit K "Main output gain";
  parameter Types.PerUnit K0 "Branch gain on V2";
  parameter Types.PerUnit K1 "Branch gain on V3";
  parameter Types.PerUnit K2 "Branch gain on V4";
  parameter Types.PerUnit K3 "Branch gain on V5";
  parameter Types.PerUnit K4 "Branch gain on V6";
  parameter Types.Time TD "Input low-pass time constant in s";
  parameter Types.Time T1 "First integrator time constant in s";
  parameter Types.Time T2 "Second integrator time constant in s";
  parameter Types.Time T3 "Third integrator time constant in s";
  parameter Types.Time T4 "Fourth integrator time constant in s";
  parameter Types.VoltageModulePu VSMAX "Maximum stabilizer output in pu (base UNom)";
  parameter Types.VoltageModulePu VSMIN "Minimum stabilizer output in pu (base UNom)";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power in MVA";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu PGen0Pu "Initial active power in pu (base SnRef) (generator convention)";

  final parameter Types.PerUnit PGen0PuSNom = PGen0Pu * SystemBase.SnRef / SNom "Initial active power in pu (base SNom)";

  // Inputs / output
  Blocks.Interfaces.RealInput PGenPu(start = PGen0Pu) "Active power in pu (base SnRef) (generator convention)";
  Blocks.Interfaces.RealOutput UPssPu(start = 0) "Stabilizer output voltage in pu (base UNom)";

  // Input conditioning
  Blocks.Math.Gain gainPGen(k = SystemBase.SnRef / SNom) "Conversion to machine base power";
  Blocks.Continuous.FirstOrder filterTD(T = if TD > 1e-6 then TD else 1e-5, y_start = PGen0PuSNom) "Input low-pass; output V1";

  // Feedback subtractor: V2 = V1 - P_machineBase - V3 - V4 - V5 - V6
  Blocks.Math.Sum sumV2(nin = 6, k = {1, -1, -1, -1, -1, -1}) "V2 = V1 - P - V3 - V4 - V5 - V6";

  // Chain of four integrators with gain 1/Ti
  Blocks.Continuous.Integrator integT1(k = 1 / T1) "1/(s*T1); output V3";
  Blocks.Continuous.Integrator integT2(k = 1 / T2) "1/(s*T2); output V4";
  Blocks.Continuous.Integrator integT3(k = 1 / T3) "1/(s*T3); output V5";
  Blocks.Continuous.Integrator integT4(k = 1 / T4) "1/(s*T4); output V6";

  // Output sum: V7 = K0*V2 + K1*V3 + K2*V4 + K3*V5 + K4*V6 (V1 does not enter)
  Blocks.Math.Sum sumV7(nin = 5, k = {K0, K1, K2, K3, K4}) "V7 = sum(Ki * V_(i+2))";

  // Main gain and output limiter
  Blocks.Math.Gain gainK(k = K) "Main gain";
  Blocks.Nonlinear.Limiter limiter(uMax = VSMAX, uMin = VSMIN, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Stabilizer output limiter";

equation
  connect(PGenPu, gainPGen.u);
  connect(gainPGen.y, filterTD.u);

  // Feedback: V2 = V1 - P_machineBase - V3 - V4 - V5 - V6
  connect(filterTD.y, sumV2.u[1]);
  connect(gainPGen.y, sumV2.u[2]);
  connect(integT1.y,  sumV2.u[3]);
  connect(integT2.y,  sumV2.u[4]);
  connect(integT3.y,  sumV2.u[5]);
  connect(integT4.y,  sumV2.u[6]);

  // Integrator chain: T1(V2) -> V3 -> T2 -> V4 -> T3 -> V5 -> T4 -> V6
  connect(sumV2.y, integT1.u);
  connect(integT1.y, integT2.u);
  connect(integT2.y, integT3.u);
  connect(integT3.y, integT4.u);

  // Output sum: V7 = K0*V2 + K1*V3 + K2*V4 + K3*V5 + K4*V6
  connect(sumV2.y,    sumV7.u[1]);
  connect(integT1.y,  sumV7.u[2]);
  connect(integT2.y,  sumV7.u[3]);
  connect(integT3.y,  sumV7.u[4]);
  connect(integT4.y,  sumV7.u[5]);

  connect(sumV7.y, gainK.u);
  connect(gainK.y, limiter.u);
  connect(limiter.y, UPssPu);

  annotation(preferredView = "text");
end PssSH;
