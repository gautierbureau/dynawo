within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model UnderexcLimX2 "Extended underexcitation limiter variant 2 (CGMES UnderexcLimX2)"
  /*
    Extended UEL variant 2 - identical topology to UnderexcLimIEEE2 minus
    the K1 / K2 / Kfb inner gains. Minimum-Q characteristic (P0, Q0) ->
    (P1, Q1); violation Qmin - QGen drives Kul + Kui/s integrator with
    Kuf voltage feedback. Output filtered through (Tu1/Tu2, Tu3/Tu4)
    lead-lag stages with clamped intermediate signal (Vuimax / Vuimin).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Ki "Integrator gain (alias for Kui)";
  parameter Types.PerUnit Kuf;
  parameter Types.PerUnit Kui;
  parameter Types.PerUnit Kul;
  parameter Types.PerUnit P0;
  parameter Types.PerUnit P1;
  parameter Types.PerUnit Q0;
  parameter Types.PerUnit Q1;
  parameter Types.Time tU1;
  parameter Types.Time tU2;
  parameter Types.Time tU3;
  parameter Types.Time tU4;
  parameter Types.VoltageModulePu VuiMaxPu;
  parameter Types.VoltageModulePu VuiMinPu;

  final parameter Real CharSlope = if abs(P1 - P0) > 1e-9 then (Q1 - Q0) / (P1 - P0) else 0;
  final parameter Real CharOffset = Q0 - CharSlope * P0;

  Modelica.Blocks.Interfaces.RealInput PGenPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput QGenPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = 1);
  Modelica.Blocks.Interfaces.RealOutput UUelPu(start = 0);

  Modelica.Blocks.Math.Gain pSlope(k = CharSlope);
  Modelica.Blocks.Sources.Constant offsetConst(k = CharOffset);
  Modelica.Blocks.Math.Add qMin;
  Modelica.Blocks.Math.Feedback violation;
  Modelica.Blocks.Math.Gain kulGain(k = Kul);
  Modelica.Blocks.Math.Gain kufGain(k = Kuf);
  Modelica.Blocks.Math.Feedback minusUf;
  Modelica.Blocks.Continuous.Integrator integ(k = Kui, y_start = 0, initType = Modelica.Blocks.Types.Init.NoInit) "UEL integrator (NoInit: SteadyState init diverges when Kui = 0 because k*u = 0 leaves u under-constrained)";
  Modelica.Blocks.Nonlinear.Limiter intLim(uMax = VuiMaxPu, uMin = VuiMinPu);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction stage1(a = {tU2, 1}, b = {tU1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction stage2(a = {tU4, 1}, b = {tU3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);

equation
  connect(PGenPu, pSlope.u);
  connect(pSlope.y, qMin.u1);
  connect(offsetConst.y, qMin.u2);
  connect(qMin.y, violation.u1);
  connect(QGenPu, violation.u2);
  connect(violation.y, kulGain.u);
  connect(UsPu, kufGain.u);
  connect(kulGain.y, minusUf.u1);
  connect(kufGain.y, minusUf.u2);
  connect(minusUf.y, integ.u);
  connect(integ.y, intLim.u);
  connect(intLim.y, stage1.u);
  connect(stage1.y, stage2.u);
  connect(stage2.y, UUelPu);

  annotation(preferredView = "text");
end UnderexcLimX2;
