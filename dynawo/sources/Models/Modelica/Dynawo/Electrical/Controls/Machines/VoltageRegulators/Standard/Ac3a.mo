within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite
* of simulation tools for power systems.
*/

model Ac3a "IEEE excitation system type AC3A model (2005/2016 standard, CGMES ExcIEEEAC3A)"
  /*
    IEEE Std 421.5 type AC3A self-excited alternator-rectifier excitation
    system. Unlike AC1A, the alternator field is self-excited from the
    alternator output rather than from the voltage regulator. The regulator
    output Va is multiplied by Ve (a positive feedback from the exciter
    output) to form Efe, which drives the exciter integrator. An additional
    Kr-Vfe minor loop introduces field-current rate feedback. The model
    keeps the AcRotatingExciter rectifier and exponential saturation from
    AC1A. EfdN scales the regulator gain in the high field range.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  // Regulation parameters
  parameter Types.PerUnit AEx "Gain of saturation function";
  parameter Types.PerUnit BEx "Exponential coefficient of saturation function";
  parameter Types.VoltageModulePu EfdNPu "Threshold field voltage at which gain reduces (Kn applied above)";
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Types.PerUnit Kc "Rectifier loading factor";
  parameter Types.PerUnit Kd "Demagnetizing factor";
  parameter Types.PerUnit Ke "Exciter field resistance constant";
  parameter Types.PerUnit Kf "Exciter rate feedback gain";
  parameter Types.PerUnit Kn "High field voltage gain reduction factor";
  parameter Types.PerUnit Kr "Field-current minor-loop feedback gain";
  parameter Types.Time tA "Voltage regulator time constant in s";
  parameter Types.Time tB "Voltage regulator lag time constant in s";
  parameter Types.Time tC "Voltage regulator lead time constant in s";
  parameter Types.Time tE "Exciter field time constant in s";
  parameter Types.Time tF "Exciter rate feedback time constant in s";
  parameter Types.PerUnit TolLi = 0.0001 "Tolerance on limit crossing for the limited integrator";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.VoltageModulePu VaMaxPu "Maximum regulator output";
  parameter Types.VoltageModulePu VaMinPu "Minimum regulator output";
  parameter Types.VoltageModulePu VeMinPu = 0 "Minimum exciter output voltage";
  parameter Types.VoltageModulePu VfeMaxPu "Maximum exciter field current signal";

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
  Modelica.Blocks.Math.Product selfExc "Va * Ve (self-excitation product)";
  Modelica.Blocks.Math.Feedback krFbk "selfExc - Kr*Vfe";
  Modelica.Blocks.Math.Gain krGain(k = Kr);
  Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.BaseClasses.AcRotatingExciter acRotatingExciter(
    AEx = AEx, BEx = BEx, Efd0Pu = Efd0Pu, Efe0Pu = Efe0Pu, Ir0Pu = Ir0Pu,
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
  final parameter Types.VoltageModulePu Efe0Pu = Vfe0Pu "Initial integrator input";
  final parameter Types.VoltageModulePu Va0Pu = (Efe0Pu + Kr * Vfe0Pu) / Ve0Pu "Initial regulator output (self-excited)";
  final parameter Types.VoltageModulePu UsRef0Pu = Va0Pu / Ka + Us0Pu "Initial reference stator voltage";

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rateFbk.u1);
  connect(rateFbk.y, leadLag.u);
  connect(leadLag.y, regulator.u);
  connect(regulator.y, selfExc.u1);
  connect(acRotatingExciter.EfdPu, selfExc.u2);
  connect(selfExc.y, krFbk.u1);
  connect(acRotatingExciter.VfePu, krGain.u);
  connect(krGain.y, krFbk.u2);
  connect(krFbk.y, acRotatingExciter.EfePu);
  connect(IrPu, acRotatingExciter.IrPu);
  connect(acRotatingExciter.EfdPu, EfdPu);
  connect(acRotatingExciter.VfePu, derivative.u);
  connect(derivative.y, rateFbk.u2);

  annotation(preferredView = "diagram");
end Ac3a;
