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

model Dc3a "IEEE excitation system type DC3A model (2005/2016 standard, CGMES ExcIEEEDC3A)"
  /*
    IEEE Std 421.5 type DC3A non-continuously-acting dc commutator exciter.
    The voltage regulator is a motor-driven rheostat that is insensitive to
    voltage errors smaller than Kv (deadband half-width). Outside the
    deadband, the rheostat motor moves the field-voltage reference Vr at a
    constant rate (parameter Krm, typically 0.05 pu/s) until the error
    returns inside the deadband or until Vr hits the VMax / VMin limits.
    Block-view approximation: a DeadZone on the error generates a non-zero
    signal only outside the deadband; this signal is multiplied by Krm and
    integrated into Vr (limited integrator). The downstream exciter
    integrator (1/sTe), saturation Se(Efd) = AEx*exp(BEx*Efd), and Kf rate
    feedback are the same as in DC1A. ExcLim true clamps Ve >= 0.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  // Saturation parameters
  parameter Types.PerUnit AEx "Gain of saturation function";
  parameter Types.PerUnit BEx "Exponential coefficient of saturation function";

  // Regulator parameters
  parameter Boolean ExcLim = true "If true, clamp Ve >= 0 (exciter cannot reverse)";
  parameter Types.PerUnit Ke "Exciter field resistance constant";
  parameter Types.PerUnit Kf "Exciter rate feedback gain";
  parameter Types.PerUnit Krm = 0.05 "Motor-driven rheostat travel rate in pu/s (not in CGMES; default per IEEE 421.5 example)";
  parameter Types.VoltageModulePu KvPu "Deadband half-width on voltage error";
  parameter Types.Time tE "Exciter time constant in s";
  parameter Types.Time tF "Exciter rate feedback time constant in s";
  parameter Types.Time tRh "Stator voltage filter time constant in s (called trh in CGMES)";
  parameter Types.VoltageModulePu VeMinPu = 0 "Minimum exciter output voltage";
  parameter Types.VoltageModulePu VMaxPu "Maximum field voltage reference (rheostat upper bound)";
  parameter Types.VoltageModulePu VMinPu "Minimum field voltage reference (rheostat lower bound)";

  // Inputs
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu";

  // Output
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu";

  // Blocks
  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tRh, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Feedback err "UsRef - UsFilt";
  Modelica.Blocks.Math.Feedback errMinusRateFb "Error - rate feedback";
  Modelica.Blocks.Nonlinear.DeadZone deadZone(uMax = KvPu, uMin = -KvPu) "Insensitive zone";
  Modelica.Blocks.Nonlinear.Limiter motorLim(uMax = 1, uMin = -1) "Saturating relay output (+/-1)";
  Modelica.Blocks.Math.Gain motorRate(k = Krm) "Rheostat travel rate";
  Modelica.Blocks.Continuous.LimIntegrator rheostat(k = 1, outMin = VMinPu, outMax = VMaxPu, y_start = Vr0Pu, initType = Modelica.Blocks.Types.Init.SteadyState) "Motor-driven rheostat position = Vr";
  Modelica.Blocks.Math.Feedback excIn "Vr - Vfe";
  Modelica.Blocks.Continuous.LimIntegrator excIntegrator(k = 1 / tE, outMin = if ExcLim then VeMinPu else -Modelica.Constants.inf, outMax = Modelica.Constants.inf, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add satSum(k1 = Ke) "Vfe = Ke*Ve + Ve*Se(Ve)";
  Modelica.Blocks.Math.Product satProd "Ve * Se(Ve)";
  Modelica.Blocks.Math.Gain expGain(k = AEx);
  Modelica.Blocks.Math.Power expBex(base = exp(BEx), useExp = true);
  Modelica.Blocks.Continuous.Derivative derivative(k = Kf, T = tF, x_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);

  // Initial parameters
  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage";

  // Derived initial values (rheostat at rest inside the deadband => UsRef = Us0Pu)
  final parameter Types.VoltageModulePu Vfe0Pu = Efd0Pu * (Ke + AEx * exp(BEx * Efd0Pu)) "Initial exciter field signal";
  final parameter Types.VoltageModulePu Vr0Pu = Vfe0Pu "Initial rheostat position";
  final parameter Types.VoltageModulePu UsRef0Pu = Us0Pu "Initial reference stator voltage (rheostat settled inside the deadband)";

equation
  connect(UsPu, uFilter.u);
  connect(UsRefPu, err.u1);
  connect(uFilter.y, err.u2);
  connect(err.y, errMinusRateFb.u1);
  connect(derivative.y, errMinusRateFb.u2);
  connect(errMinusRateFb.y, deadZone.u);
  connect(deadZone.y, motorLim.u);
  connect(motorLim.y, motorRate.u);
  connect(motorRate.y, rheostat.u);
  connect(rheostat.y, excIn.u1);
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
end Dc3a;
