within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model UnderexcLimIEEE2 "IEEE underexcitation limiter type 2 (CGMES UnderexcLimIEEE2)"
  /*
    IEEE Std 421.5 underexcitation limiter type 2. The minimum-Q
    characteristic is parameterised by two points (P0, Q0) and (P1, Q1),
    fitted as Qmin(P) = (Q0 - slope*P0) + slope*P with slope = (Q1-Q0)/
    (P1-P0). The violation v = Qmin - QGen (positive when in violation)
    goes through gains Kul + K1 + K2 - Kfb*v - Kuf*Us into a Kui
    integrator, output clamped to (Vuimin, Vuimax) and filtered through
    two lead-lag stages (Tu1/Tu2, Tu3/Tu4).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit K1;
  parameter Types.PerUnit K2;
  parameter Types.PerUnit Kfb;
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
  Modelica.Blocks.Math.Add qMin "Qmin = slope*P + offset";
  Modelica.Blocks.Math.Feedback violation "Qmin - QGen";
  Modelica.Blocks.Math.Gain kulGain(k = Kul);
  Modelica.Blocks.Math.Gain k1Gain(k = K1);
  Modelica.Blocks.Math.Gain k2Gain(k = K2);
  Modelica.Blocks.Math.Gain kfbGain(k = Kfb);
  Modelica.Blocks.Math.Gain kufGain(k = Kuf);
  Modelica.Blocks.Math.Sum gainSum(k = {1, 1, 1, -1, -1}, nin = 5);
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
  connect(kulGain.y, k1Gain.u);
  connect(violation.y, k2Gain.u);
  connect(violation.y, kfbGain.u);
  connect(UsPu, kufGain.u);
  connect(k1Gain.y, gainSum.u[1]);
  connect(k2Gain.y, gainSum.u[2]);
  connect(kulGain.y, gainSum.u[3]);
  connect(kfbGain.y, gainSum.u[4]);
  connect(kufGain.y, gainSum.u[5]);
  connect(gainSum.y, integ.u);
  connect(integ.y, intLim.u);
  connect(intLim.y, stage1.u);
  connect(stage1.y, stage2.u);
  connect(stage2.y, UUelPu);

  annotation(preferredView = "text");
end UnderexcLimIEEE2;
