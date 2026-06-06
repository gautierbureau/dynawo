within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model UnderexcLimIEEE1 "IEEE underexcitation limiter type 1 (CGMES UnderexcLimIEEE1)"
  /*
    IEEE Std 421.5 underexcitation limiter type 1. Takes the generator
    reactive power QGenPu and terminal voltage UsPu. The MVA-loaded
    minimum-Q characteristic is approximated by Kur and Kuf gains on Q
    and Us, summed with Kuc constant offset; the result is filtered by
    Tu1/Tu2 and Tu3/Tu4 lead-lag stages, then clamped (Vurmax / Vuimax /
    Vuimin) and emitted as UUelPu (signed so that adding it to the AVR
    error pushes the field up).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Kuc "Constant offset on the UEL characteristic";
  parameter Types.PerUnit Kuf "Voltage-dependent gain";
  parameter Types.PerUnit Kur "Reactive-power gain";
  parameter Types.Time tU1;
  parameter Types.Time tU2;
  parameter Types.Time tU3;
  parameter Types.Time tU4;
  parameter Types.VoltageModulePu VuiMaxPu "Intermediate signal upper limit";
  parameter Types.VoltageModulePu VuiMinPu "Intermediate signal lower limit";
  parameter Types.VoltageModulePu VurMaxPu "Output upper limit";

  Modelica.Blocks.Interfaces.RealInput QGenPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = 1);
  Modelica.Blocks.Interfaces.RealOutput UUelPu(start = 0);

  Modelica.Blocks.Math.Gain kurGain(k = Kur);
  Modelica.Blocks.Math.Gain kufGain(k = Kuf);
  Modelica.Blocks.Math.Sum sumChar(k = {1, 1, 1}, nin = 3);
  Modelica.Blocks.Sources.Constant kucConst(k = Kuc);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction stage1(a = {tU2, 1}, b = {tU1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction stage2(a = {tU4, 1}, b = {tU3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);
  Modelica.Blocks.Nonlinear.Limiter intLim(uMax = VuiMaxPu, uMin = VuiMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VurMaxPu, uMin = 0, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(QGenPu, kurGain.u);
  connect(UsPu, kufGain.u);
  connect(kurGain.y, sumChar.u[1]);
  connect(kufGain.y, sumChar.u[2]);
  connect(kucConst.y, sumChar.u[3]);
  connect(sumChar.y, intLim.u);
  connect(intLim.y, stage1.u);
  connect(stage1.y, stage2.u);
  connect(stage2.y, outLim.u);
  connect(outLim.y, UUelPu);

  annotation(preferredView = "text");
end UnderexcLimIEEE1;
