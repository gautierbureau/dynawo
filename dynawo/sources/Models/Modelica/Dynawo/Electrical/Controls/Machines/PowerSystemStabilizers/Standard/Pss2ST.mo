within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model Pss2ST "Dual-input PSS with 2-stage torsional filter (CGMES Pss2ST)"
  /*
    Dual-input PSS with separate speed and active-power channels each
    routed through a washout-like filter and a series of lead-lag stages
    that include a torsional notch filter. The two channels (input 1 =
    speed deviation by default, input 2 = active power deviation by
    default) are gain-scaled by K1 / K2, lead-lag-filtered through
    (T1/T2, T3/T4) and (T5/T6, T7/T8), then summed and shaped by
    (1+sT9)/(1+sT10) before being clamped to (Lsmin, Lsmax).
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter String InputSignal1Type = "rotorAngularFrequencyDeviation";
  parameter String InputSignal2Type = "generatorElectricalPower";
  parameter Types.PerUnit K1 "Speed channel gain";
  parameter Types.PerUnit K2 "Power channel gain";
  parameter Types.Time t1; parameter Types.Time t2;
  parameter Types.Time t3; parameter Types.Time t4;
  parameter Types.Time t5; parameter Types.Time t6;
  parameter Types.Time t7; parameter Types.Time t8;
  parameter Types.Time t9; parameter Types.Time t10;
  parameter Types.VoltageModulePu LsMaxPu;
  parameter Types.VoltageModulePu LsMinPu;
  parameter Types.ApparentPowerModule SNom "Nominal apparent power in MVA";

  parameter Types.ActivePowerPu PGen0Pu "Initial active power in pu (base SnRef) (generator convention)";

  final parameter Types.PerUnit PGen0PuSNom = PGen0Pu * SystemBase.SnRef / SNom;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PGenPu(start = PGen0Pu);
  Modelica.Blocks.Interfaces.RealOutput UPssPu(start = 0);

  Modelica.Blocks.Math.Feedback dW;
  Modelica.Blocks.Math.Gain gainPGen(k = SystemBase.SnRef / SNom);
  Modelica.Blocks.Sources.Constant refPGen(k = PGen0PuSNom);
  Modelica.Blocks.Math.Feedback dP;
  Modelica.Blocks.Math.Gain k1Gain(k = K1);
  Modelica.Blocks.Math.Gain k2Gain(k = K2);
  Continuous.TransferFunction leadLag1(a = {t2, 1}, b = {t1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Continuous.TransferFunction leadLag2(a = {t4, 1}, b = {t3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Continuous.TransferFunction leadLag3(a = {t6, 1}, b = {t5, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Continuous.TransferFunction leadLag4(a = {t8, 1}, b = {t7, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add channelSum;
  Continuous.TransferFunction outputShape(a = {t10, 1}, b = {t9, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = LsMaxPu, uMin = LsMinPu);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, k1Gain.u);
  connect(k1Gain.y, leadLag1.u);
  connect(leadLag1.y, leadLag2.u);
  connect(PGenPu, gainPGen.u);
  connect(gainPGen.y, dP.u1);
  connect(refPGen.y, dP.u2);
  connect(dP.y, k2Gain.u);
  connect(k2Gain.y, leadLag3.u);
  connect(leadLag3.y, leadLag4.u);
  connect(leadLag2.y, channelSum.u1);
  connect(leadLag4.y, channelSum.u2);
  connect(channelSum.y, outputShape.u);
  connect(outputShape.y, outLim.u);
  connect(outLim.y, UPssPu);

  annotation(preferredView = "text");
end Pss2ST;
