within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcOEX3T "Brown Boveri OEX3T AVR (CGMES ExcOEX3T)"
  /*
    Brown Boveri OEX3T multi-stage AVR. Voltage sensing through 1/(1+sTr).
    Error (UsRef + UPss - UsFilt - rate feedback) is shaped by a chain of
    six lead-lag stages with time constants T1 / T2 / T3 / T4 / T5 / T6
    and an outer Ka/(1+sTa) regulator with output (Vrmin, Vrmax) limits.
    Tb / Tc form the transient gain reduction. Kc / Kd / Kf / Tf form the
    rectifier reg + rate feedback path. Saturation Se(Efd) = AEx*exp
    (BEx*Efd) is fitted through (E1, Se1) and (E2, Se2).
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;

  parameter Types.PerUnit AEx; parameter Types.PerUnit BEx;
  parameter Types.PerUnit Ka;
  parameter Types.PerUnit Kc; parameter Types.PerUnit Kd; parameter Types.PerUnit Kf;
  parameter Types.Time tA; parameter Types.Time tB; parameter Types.Time tC;
  parameter Types.Time t1; parameter Types.Time t2; parameter Types.Time t3;
  parameter Types.Time t4; parameter Types.Time t5; parameter Types.Time t6;
  parameter Types.Time tF;
  parameter Types.VoltageModulePu VrMaxPu;
  parameter Types.VoltageModulePu VrMinPu;

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu Vr0Pu = Efd0Pu * (1 + AEx * exp(BEx * Efd0Pu));
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / Ka + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Math.Feedback rateFbk;
  Continuous.TransferFunction tbtc(a = {tB, 1}, b = {tC, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Vr0Pu / Ka);
  Continuous.TransferFunction ll1(a = {t2, 1}, b = {t1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Vr0Pu / Ka);
  Continuous.TransferFunction ll2(a = {t4, 1}, b = {t3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Vr0Pu / Ka);
  Continuous.TransferFunction ll3(a = {t6, 1}, b = {t5, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Vr0Pu / Ka);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VrMaxPu, YMin = VrMinPu);
  Modelica.Blocks.Continuous.Derivative derivative(k = Kf, T = tF, x_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);

equation
  connect(UsPu, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rateFbk.u1);
  connect(rateFbk.y, tbtc.u);
  connect(tbtc.y, ll1.u);
  connect(ll1.y, ll2.u);
  connect(ll2.y, ll3.u);
  connect(ll3.y, regulator.u);
  connect(regulator.y, EfdPu);
  connect(regulator.y, derivative.u);
  connect(derivative.y, rateFbk.u2);

  annotation(preferredView = "text");
end ExcOEX3T;
