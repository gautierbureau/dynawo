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

model Ac2a "IEEE excitation system type AC2A model (2005/2016 standard, CGMES ExcIEEEAC2A)"
  /*
    IEEE Std 421.5 type AC2A high-initial-response alternator-rectifier
    excitation system. Same alternator-rectifier topology as AC1A, with two
    additions for fast response: a minor-loop feedback Kh*Vfe taken from the
    exciter field signal and subtracted at the regulator output, and a gain
    Kb between the regulator output and the exciter field integrator. Both
    rectifier saturation (Kc, Kd) and exponential exciter saturation
    Se(Ve) = AEx*exp(BEx*Ve) are represented.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  // Regulation parameters
  parameter Types.PerUnit AEx "Gain of saturation function";
  parameter Types.PerUnit BEx "Exponential coefficient of saturation function";
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Types.PerUnit Kb "Second stage regulator gain";
  parameter Types.PerUnit Kc "Rectifier loading factor";
  parameter Types.PerUnit Kd "Demagnetizing factor";
  parameter Types.PerUnit Ke "Exciter field resistance constant";
  parameter Types.PerUnit Kf "Exciter rate feedback gain";
  parameter Types.PerUnit Kh "Exciter field current feedback gain";
  parameter Types.Time tA "Voltage regulator time constant in s";
  parameter Types.Time tB "Voltage regulator lag time constant in s";
  parameter Types.Time tC "Voltage regulator lead time constant in s";
  parameter Types.Time tE "Exciter field time constant in s";
  parameter Types.Time tF "Exciter rate feedback time constant in s";
  parameter Types.PerUnit TolLi = 0.0001 "Tolerance on limit crossing for the limited integrator";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.VoltageModulePu VaMaxPu "Maximum first-stage regulator output";
  parameter Types.VoltageModulePu VaMinPu "Minimum first-stage regulator output";
  parameter Types.VoltageModulePu VeMinPu = 0 "Minimum exciter output voltage";
  parameter Types.VoltageModulePu VfeMaxPu "Maximum exciter field current signal";
  parameter Types.VoltageModulePu VrMaxPu "Maximum field voltage";
  parameter Types.VoltageModulePu VrMinPu "Minimum field voltage";

  // Inputs
  Modelica.Blocks.Interfaces.RealInput IrPu(start = Ir0Pu) "Rotor current in pu (base SNom)";
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0) "PSS output voltage in pu (base UNom)";
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu (base UNom)";
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu (base UNom)";

  // Output
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu";

  // Blocks
  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1) "UsRef - UsFilt + UPss";
  Modelica.Blocks.Math.Feedback rateFbk "Error - rate feedback";
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction leadLag(a = {tB, 1}, b = {tC, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Va0Pu / Ka);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Va0Pu, YMax = VaMaxPu, YMin = VaMinPu);
  Modelica.Blocks.Math.Feedback fieldFbk "Va - Kh*Vfe";
  Modelica.Blocks.Math.Gain kbGain(k = Kb);
  Modelica.Blocks.Math.Gain khGain(k = Kh);
  Modelica.Blocks.Nonlinear.Limiter vrLim(uMax = VrMaxPu, uMin = VrMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.BaseClasses.AcRotatingExciter acRotatingExciter(
    AEx = AEx, BEx = BEx, Efd0Pu = Efd0Pu, Efe0Pu = Vr0Pu, Ir0Pu = Ir0Pu,
    Kc = Kc, Kd = Kd, Ke = Ke, tE = tE, TolLi = TolLi,
    Ve0Pu = Ve0Pu, VeMax0Pu = VeMax0Pu, VeMinPu = VeMinPu, VfeMaxPu = VfeMaxPu);
  Modelica.Blocks.Continuous.Derivative derivative(k = Kf, T = tF, x_start = Vfe0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);

  // Initial parameters
  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage";
  parameter Types.CurrentModulePu Ir0Pu "Initial rotor current";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage";
  parameter Types.VoltageModulePu Ve0Pu "Initial exciter output voltage";
  parameter Types.VoltageModulePu VeMax0Pu "Maximum exciter output voltage at initial conditions";

  // Derived initial values
  final parameter Types.VoltageModulePu Vfe0Pu = Kd * Ir0Pu + Ve0Pu * (Ke + AEx * exp(BEx * Ve0Pu)) "Initial exciter field signal";
  final parameter Types.VoltageModulePu Vr0Pu = Vfe0Pu "Initial integrator input";
  final parameter Types.VoltageModulePu Va0Pu = Vr0Pu / Kb + Kh * Vfe0Pu "Initial first-stage regulator output";
  final parameter Types.VoltageModulePu UsRef0Pu = Va0Pu / Ka + Us0Pu "Initial reference stator voltage";

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rateFbk.u1);
  connect(rateFbk.y, leadLag.u);
  connect(leadLag.y, regulator.u);
  connect(regulator.y, fieldFbk.u1);
  connect(acRotatingExciter.VfePu, khGain.u);
  connect(khGain.y, fieldFbk.u2);
  connect(fieldFbk.y, kbGain.u);
  connect(kbGain.y, vrLim.u);
  connect(vrLim.y, acRotatingExciter.EfePu);
  connect(IrPu, acRotatingExciter.IrPu);
  connect(acRotatingExciter.EfdPu, EfdPu);
  connect(acRotatingExciter.VfePu, derivative.u);
  connect(derivative.y, rateFbk.u2);

  annotation(preferredView = "diagram");
end Ac2a;
