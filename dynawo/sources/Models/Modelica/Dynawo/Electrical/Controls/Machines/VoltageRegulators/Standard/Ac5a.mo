within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite
* of simulation tools for power systems.
*/

model Ac5a "IEEE excitation system type AC5A model (2005/2016 standard, CGMES ExcIEEEAC5A)"
  /*
    IEEE Std 421.5 type AC5A simplified rotating brushless excitation system.
    Block diagram: terminal voltage filter -> error summation against UsRef
    with PSS bias -> gain-time-constant regulator (Ka / (1 + sTa)) with VrMin /
    VrMax limits -> exciter integrator (1 / sTe) with VeMin >= 0 -> exciter
    saturation (Ke + Se(Ve)) multiplied by Ve -> Vfe. There is no rectifier
    regulation correction (no Kc, Kd) and no rotor-current input. Rate
    feedback Kf * s / ((1 + sTf1)(1 + sTf2)(1 + sTf3)) taken from Vfe is
    subtracted at the input summer. Saturation is exponential:
    Se(Ve) = AEx * exp(BEx * Ve), with AEx and BEx fitted through the two
    operating points (E1, Se1) and (E2, Se2) supplied by the par file
    (computed externally; CGMES delivers them).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  // Regulation parameters
  parameter Types.PerUnit AEx "Gain of saturation function";
  parameter Types.PerUnit BEx "Exponential coefficient of saturation function";
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Types.PerUnit Ke "Exciter field resistance constant";
  parameter Types.PerUnit Kf "Exciter rate feedback gain";
  parameter Types.Time tA "Voltage regulator time constant in s";
  parameter Types.Time tE "Exciter field time constant in s";
  parameter Types.Time tF1 "Exciter rate feedback first time constant in s";
  parameter Types.Time tF2 "Exciter rate feedback second time constant in s";
  parameter Types.Time tF3 "Exciter rate feedback third time constant in s";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.VoltageModulePu VeMinPu = 0 "Minimum exciter output voltage in pu (user-selected base voltage)";
  parameter Types.VoltageModulePu VrMaxPu "Maximum field voltage in pu (user-selected base voltage)";
  parameter Types.VoltageModulePu VrMinPu "Minimum field voltage in pu (user-selected base voltage)";

  // Inputs
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0) "Power system stabilizer output voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-220, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-220, -20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu (base UNom)" annotation(
    Placement(visible = true, transformation(origin = {-220, 20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));

  // Output
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu (user-selected base voltage)" annotation(
    Placement(visible = true, transformation(origin = {290, 0}, extent = {{-10, -10}, {10, 10}}), iconTransformation(origin = {110, 0}, extent = {{-10, -10}, {10, 10}})));

  // Blocks
  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState) "Terminal voltage sensing filter";
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1) "Error: UsRef - filteredUs + UPss";
  Modelica.Blocks.Math.Feedback rateFbk "Error - rate feedback";
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VrMaxPu, YMin = VrMinPu) "Gain - time-constant regulator with VR limits";
  Modelica.Blocks.Math.Feedback excIn "Vr - Vfe";
  Modelica.Blocks.Continuous.LimIntegrator excIntegrator(k = 1 / tE, outMin = VeMinPu, outMax = Modelica.Constants.inf, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState) "Exciter time-constant integrator (1 / sTe)";
  Modelica.Blocks.Math.Add satSum(k1 = Ke) "Vfe = Ke*Ve + Ve*Se(Ve)";
  Modelica.Blocks.Math.Product satProd "Ve * Se(Ve)";
  Modelica.Blocks.Math.Gain expGain(k = AEx) "AEx";
  Modelica.Blocks.Math.Power expBex(base = exp(BEx), useExp = true) "exp(BEx*Ve)";
  Modelica.Blocks.Continuous.Derivative rateFbk1(k = Kf, T = tF1, x_start = Vfe0Pu, initType = Modelica.Blocks.Types.Init.SteadyState) "Kf*s / (1 + sTf1)";
  Modelica.Blocks.Continuous.FirstOrder rateFbk2(T = tF2, y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState) "1 / (1 + sTf2)";
  Modelica.Blocks.Continuous.FirstOrder rateFbk3(T = tF3, y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState) "1 / (1 + sTf3)";

  // Generator initial parameters (from generator init or par file)
  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage in pu (user-selected base voltage)";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage in pu (base UNom)";

  // Derived initial values (steady state: Vfe = Vr, Ve = Efd, rate fb = 0)
  final parameter Types.VoltageModulePu Vfe0Pu = Efd0Pu * (Ke + AEx * exp(BEx * Efd0Pu)) "Initial exciter field signal in pu";
  final parameter Types.VoltageModulePu Vr0Pu = Vfe0Pu "Initial regulator output in pu";
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / Ka + Us0Pu "Initial reference stator voltage in pu (base UNom)";

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rateFbk.u1);
  connect(rateFbk3.y, rateFbk.u2);
  connect(rateFbk.y, regulator.u);
  connect(regulator.y, excIn.u1);
  connect(satSum.y, excIn.u2);
  connect(excIn.y, excIntegrator.u);
  connect(excIntegrator.y, EfdPu);
  connect(excIntegrator.y, satSum.u1);
  connect(excIntegrator.y, expBex.u);
  connect(excIntegrator.y, satProd.u1);
  connect(expBex.y, expGain.u);
  connect(expGain.y, satProd.u2);
  connect(satProd.y, satSum.u2);
  connect(satSum.y, rateFbk1.u);
  connect(rateFbk1.y, rateFbk2.u);
  connect(rateFbk2.y, rateFbk3.u);

  annotation(preferredView = "diagram");
end Ac5a;
