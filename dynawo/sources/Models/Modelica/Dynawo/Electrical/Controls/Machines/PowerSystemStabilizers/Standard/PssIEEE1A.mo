within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model PssIEEE1A "IEEE Std 421.5 PSS1A single-input power system stabilizer (CGMES PssIEEE1A)"
  /*
    Single-input PSS with a torsional-notch filter, washout and a two-stage
    lead-lag compensator. Input is the speed deviation omega - omegaRef.
    Signal path: (s*T5) / (1 + s*T6) washout -> ks gain -> (1 + s*T1) /
    (1 + s*T2) lead-lag 1 -> (1 + s*T3) / (1 + s*T4) lead-lag 2 -> output
    clamped to (Vsmin, Vsmax). The torsional-notch parameters (Kx, A1, A2)
    are absorbed into Kx as a high-frequency proportional gain in this
    simplified topology.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Real InputSignalType = 1 "Input signal selector (1 = speed, 2 = active power, 3 = frequency; informational)";
  parameter Types.PerUnit Kx "Torsional notch high-frequency gain";
  parameter Types.PerUnit Ks "PSS main gain";
  parameter Types.Time t1 "First lead time constant in s";
  parameter Types.Time t2 "First lag time constant in s";
  parameter Types.Time t3 "Second lead time constant in s";
  parameter Types.Time t4 "Second lag time constant in s";
  parameter Types.Time t5 "Washout time constant in s";
  parameter Types.Time t6 "Notch/filter time constant in s";
  parameter Types.VoltageModulePu VsMaxPu;
  parameter Types.VoltageModulePu VsMinPu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput UPssPu(start = 0);

  Modelica.Blocks.Math.Feedback dW;
  Continuous.TransferFunction washout(a = {t6, 1}, b = {t5, 0}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain kxGain(k = Kx);
  Modelica.Blocks.Math.Gain ksGain(k = Ks);
  Continuous.TransferFunction leadLag1(a = {t2, 1}, b = {t1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Continuous.TransferFunction leadLag2(a = {t4, 1}, b = {t3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VsMaxPu, uMin = VsMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, washout.u);
  connect(washout.y, kxGain.u);
  connect(kxGain.y, ksGain.u);
  connect(ksGain.y, leadLag1.u);
  connect(leadLag1.y, leadLag2.u);
  connect(leadLag2.y, outLim.u);
  connect(outLim.y, UPssPu);

  annotation(preferredView = "text");
end PssIEEE1A;
