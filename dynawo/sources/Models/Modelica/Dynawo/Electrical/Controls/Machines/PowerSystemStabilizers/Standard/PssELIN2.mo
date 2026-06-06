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

model PssELIN2 "Austrian ELIN type-2 power system stabilizer (CGMES PssELIN2)"
  /*
    PSS typically associated with the ExcELIN2 exciter. The input active
    power is conditioned by two low-pass stages and three washouts (V5),
    and split into a proportional branch (KS1 * V5 = V6) and a first-order
    lag branch (KS2 / (1 + s*TS2) * V5 = V7); the two branches are then
    combined by a phase-selector function F(s) of the parameter PPSS
    (continuous in [0, 4], where PPSS = 0..2 gives positive sign and
    PPSS = 2..4 gives negative sign) and limited to +/- PSSLIM.
    Corresponds to the CGMES IEC 61970-302 PssELIN2 model (NEPLAN PSSELIN2).
    Block-based: the equation section contains only connect statements and
    the algebraic combiner equation for F(s).
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit APSS "Output coefficient";
  parameter Types.PerUnit KS1 "Proportional branch gain";
  parameter Types.PerUnit KS2 "Lag branch gain";
  parameter Types.PerUnit PPSS "Phase selector (in [0, 4]; integer phase angles 0,1,2,3 correspond to 0,-90,180,90 degrees)";
  parameter Types.VoltageModulePu PSSLIM "Output limit (symmetric +/-) in pu (base UNom)";
  parameter Types.Time TS1 "First low-pass time constant in s";
  parameter Types.Time TS2 "Lag-branch time constant in s";
  parameter Types.Time TS3 "Second low-pass time constant in s";
  parameter Types.Time TS4 "First washout time constant in s";
  parameter Types.Time TS5 "Second washout time constant in s";
  parameter Types.Time TS6 "Third washout time constant in s";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power in MVA";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu PGen0Pu "Initial active power in pu (base SnRef) (generator convention)";

  final parameter Types.PerUnit PGen0PuSNom = PGen0Pu * SystemBase.SnRef / SNom "Initial active power in pu (base SNom)";
  // F(s) sign and coefficients selected from PPSS, evaluated once as parameters
  final parameter Real signF = if PPSS <= 2 then 1.0 else -1.0 "Sign of F(s) selector";
  final parameter Real ppssEff = if PPSS <= 2 then PPSS else PPSS - 2 "Effective PPSS modulo 2";
  final parameter Real coefV6 = 1.0 - ppssEff "Combiner weight on the proportional (V6) branch";
  final parameter Real coefV7 = 1.0 - abs(ppssEff - 1.0) "Combiner weight on the lag (V7) branch";

  // Inputs / output
  Blocks.Interfaces.RealInput PGenPu(start = PGen0Pu) "Active power in pu (base SnRef) (generator convention)";
  Blocks.Interfaces.RealOutput UPssPu(start = 0) "Stabilizer output voltage in pu (base UNom)";

  // Input conditioning: two low-pass stages then three washouts (V5)
  Blocks.Math.Gain gainPGen(k = SystemBase.SnRef / SNom) "Conversion to machine base power";
  Blocks.Continuous.FirstOrder filter1(T = if TS1 > 1e-6 then TS1 else 1e-5, y_start = PGen0PuSNom) "First low-pass";
  Blocks.Continuous.FirstOrder filter3(T = if TS3 > 1e-6 then TS3 else 1e-5, y_start = PGen0PuSNom) "Second low-pass";
  Continuous.Washout washout4(tW = TS4, U0 = PGen0PuSNom) "First washout";
  Continuous.Washout washout5(tW = TS5) "Second washout";
  Continuous.Washout washout6(tW = TS6) "Third washout";

  // Two parallel branches: KS1 proportional and KS2 first-order lag
  Blocks.Math.Gain gainKS1(k = KS1) "Proportional branch gain";
  Blocks.Continuous.FirstOrder filterKS2(T = if TS2 > 1e-6 then TS2 else 1e-5, k = KS2) "Lag branch (KS2 / (1+sTS2))";

  // Combiner F(s) and output limiter
  Blocks.Math.Add combiner(k1 = signF * APSS * coefV6, k2 = signF * APSS * coefV7) "F(s) = sign * APSS * (coefV6*V6 + coefV7*V7)";
  Blocks.Nonlinear.Limiter limiter(uMax = PSSLIM, uMin = -PSSLIM, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Stabilizer output limiter";

equation
  connect(PGenPu, gainPGen.u);
  connect(gainPGen.y, filter1.u);
  connect(filter1.y, filter3.u);
  connect(filter3.y, washout4.u);
  connect(washout4.y, washout5.u);
  connect(washout5.y, washout6.u);

  // Branches
  connect(washout6.y, gainKS1.u);
  connect(washout6.y, filterKS2.u);

  // Combine and limit
  connect(gainKS1.y, combiner.u1);
  connect(filterKS2.y, combiner.u2);
  connect(combiner.y, limiter.u);
  connect(limiter.y, UPssPu);

  annotation(preferredView = "text");
end PssELIN2;
