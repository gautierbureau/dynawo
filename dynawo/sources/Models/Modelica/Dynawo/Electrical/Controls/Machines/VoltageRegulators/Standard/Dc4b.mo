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

model Dc4b "IEEE excitation system type DC4B model (2005/2016 standard, CGMES ExcIEEEDC4B)"
  /*
    IEEE Std 421.5 type DC4B field-controlled dc commutator exciter with a
    PID controller (replaces the simple Ka/(1+sTa) gain of DC1A). The PID
    output feeds the Ka time-constant regulator, then the exciter
    integrator (1/sTe) with saturation Se(Efd) = AEx * exp(BEx * Efd) and
    Ke field resistance constant. Rate feedback Kf*s/(1+sTf) is taken from
    Efd. The selectable UEL takeover and OEL summation are modeled with the
    PositionUel and PositionOel discriminators.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  // Saturation parameters
  parameter Types.PerUnit AEx "Gain of saturation function";
  parameter Types.PerUnit BEx "Exponential coefficient of saturation function";

  // Regulator parameters
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Types.PerUnit KdPid "PID derivative gain";
  parameter Types.PerUnit Ke "Exciter field resistance constant";
  parameter Types.PerUnit Kf "Exciter rate feedback gain";
  parameter Types.PerUnit KiPid "PID integral gain";
  parameter Types.PerUnit KpPid "PID proportional gain";
  parameter Integer PositionOel = 0 "OEL location: 0 = none, 1 = at AVR input summation, 2 = takeover at AVR output";
  parameter Integer PositionUel = 0 "UEL location: 0 = none, 1 = at AVR input summation, 2 = takeover at AVR output";
  parameter Types.Time tA "Voltage regulator time constant in s";
  parameter Types.Time tDPid "PID derivative time constant in s";
  parameter Types.Time tE "Exciter time constant in s";
  parameter Types.Time tF "Exciter rate feedback time constant in s";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.VoltageModulePu VeMinPu = 0 "Minimum exciter output voltage";
  parameter Types.VoltageModulePu VrMaxPu "Maximum field voltage";
  parameter Types.VoltageModulePu VrMinPu "Minimum field voltage";

  // Inputs
  Modelica.Blocks.Interfaces.RealInput UOelPu(start = 0) "OEL output voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0) "PSS output voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UUelPu(start = 0) "UEL output voltage in pu";

  // Output
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu";

  // Blocks
  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Sum errSum(nin = 5) "Sum of UsRef -UsFilt + UPss + UUel(opt) + UOel(opt) - rateFb (rateFb in u5)";
  Modelica.Blocks.Math.Gain kpBranch(k = KpPid) "Proportional branch Kp";
  Modelica.Blocks.Continuous.Integrator kiBranch(k = KiPid, y_start = Vr0Pu / Ka, initType = Modelica.Blocks.Types.Init.NoInit) "PID integral branch Ki / s (NoInit: SteadyState would diverge when KiPid = 0 because k*u = 0 leaves u under-constrained)";
  Modelica.Blocks.Continuous.Derivative kdBranch(k = KdPid, T = tDPid, x_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState) "Derivative branch Kd*s / (1 + s*Td)";
  Modelica.Blocks.Math.Add3 pidSum "Kp*e + Ki/s*e + Kd*s/(1+sTd)*e";
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VrMaxPu, YMin = VrMinPu);
  Dynawo.NonElectrical.Blocks.NonLinear.Max3 oelMin "OEL combiner";
  Dynawo.NonElectrical.Blocks.NonLinear.Min3 uelMax "UEL combiner";
  Modelica.Blocks.Math.Feedback excIn "Vr - Vfe";
  Modelica.Blocks.Continuous.LimIntegrator excIntegrator(k = 1 / tE, outMin = VeMinPu, outMax = Modelica.Constants.inf, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add satSum(k1 = Ke) "Vfe = Ke*Ve + Ve*Se(Ve)";
  Modelica.Blocks.Math.Product satProd "Ve * Se(Ve)";
  Modelica.Blocks.Math.Gain expGain(k = AEx);
  Modelica.Blocks.Math.Power expBex(base = exp(BEx), useExp = true);
  Modelica.Blocks.Continuous.Derivative derivative(k = Kf, T = tF, x_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);

  // Initial parameters
  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage";

  // Derived initial values
  final parameter Types.VoltageModulePu Vfe0Pu = Efd0Pu * (Ke + AEx * exp(BEx * Efd0Pu)) "Initial exciter field signal";
  final parameter Types.VoltageModulePu Vr0Pu = Vfe0Pu "Initial regulator output";
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / (Ka * KpPid) + Us0Pu "Initial reference stator voltage";

equation
  if PositionUel == 1 then
    errSum.u[4] = UUelPu;
    uelMax.u[1] = regulator.y;
    uelMax.u[2] = uelMax.u[1];
    uelMax.u[3] = uelMax.u[1];
  elseif PositionUel == 2 then
    errSum.u[4] = 0;
    uelMax.u[1] = regulator.y;
    uelMax.u[2] = UUelPu;
    uelMax.u[3] = uelMax.u[1];
  else
    errSum.u[4] = 0;
    uelMax.u[1] = regulator.y;
    uelMax.u[2] = uelMax.u[1];
    uelMax.u[3] = uelMax.u[1];
  end if;
  if PositionOel == 1 then
    errSum.u[5] = -UOelPu;
    oelMin.u[1] = uelMax.y;
    oelMin.u[2] = oelMin.u[1];
    oelMin.u[3] = oelMin.u[1];
  elseif PositionOel == 2 then
    errSum.u[5] = 0;
    oelMin.u[1] = uelMax.y;
    oelMin.u[2] = UOelPu;
    oelMin.u[3] = oelMin.u[1];
  else
    errSum.u[5] = 0;
    oelMin.u[1] = uelMax.y;
    oelMin.u[2] = oelMin.u[1];
    oelMin.u[3] = oelMin.u[1];
  end if;

  // errSum.u[3] uses rate feedback via subtraction inside the sum (with k* = ... not available on Sum)
  // Workaround: feed positive sign and let derivative pre-invert via -Kf gain.
  errSum.u[1] = UsRefPu;
  errSum.u[2] = -uFilter.y + UPssPu;
  errSum.u[3] = -derivative.y;

  connect(UsPu, uFilter.u);
  connect(errSum.y, kpBranch.u);
  connect(errSum.y, kiBranch.u);
  connect(errSum.y, kdBranch.u);
  connect(kpBranch.y, pidSum.u1);
  connect(kiBranch.y, pidSum.u2);
  connect(kdBranch.y, pidSum.u3);
  connect(pidSum.y, regulator.u);
  connect(oelMin.y, excIn.u1);
  connect(satSum.y, excIn.u2);
  connect(excIn.y, excIntegrator.u);
  connect(excIntegrator.y, EfdPu);
  connect(excIntegrator.y, satSum.u1);
  connect(excIntegrator.y, expBex.u);
  connect(excIntegrator.y, satProd.u1);
  connect(expBex.y, expGain.u);
  connect(expGain.y, satProd.u2);
  connect(satProd.y, satSum.u2);
  connect(excIntegrator.y, derivative.u);

  annotation(preferredView = "diagram");
end Dc4b;
