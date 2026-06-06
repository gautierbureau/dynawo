within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcREXS "REXS AVR (CGMES ExcREXS)"
  /*
    REXS rotating AC excitation system. Tr filter, error UsRef + UPss -
    UsFilt - rate fb -> two lead-lag stages ((1+sTc1)/(1+sTb1), (1+sTc2)/
    (1+sTb2)) -> Ka/(1+sTa) regulator with (Vamin, Vamax) limits -> field
    integrator (Ke + Se(Efd))*Efd subtraction -> (1/sTe) -> Efd. Rate
    feedback Kf*s/(1+sTf) from Efd. Rectifier reg Kc / Kd / Ki are
    exposed. feedbackSignal and exclfb flags are informational.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;

  parameter Types.PerUnit AEx; parameter Types.PerUnit BEx;
  parameter Types.PerUnit Ka;
  parameter Types.PerUnit Kc; parameter Types.PerUnit Kd; parameter Types.PerUnit Ke;
  parameter Types.PerUnit Kf; parameter Types.PerUnit Ki;
  parameter Types.Time tA; parameter Types.Time tB1; parameter Types.Time tB2;
  parameter Types.Time tC1; parameter Types.Time tC2;
  parameter Types.Time tE; parameter Types.Time tF; parameter Types.Time tR;
  parameter Types.VoltageModulePu VaMaxPu; parameter Types.VoltageModulePu VaMinPu;
  parameter Types.VoltageModulePu VrMaxPu; parameter Types.VoltageModulePu VrMinPu;
  parameter String FeedbackSignal = "field" "Feedback signal selector (informational)";
  parameter Boolean Exclfb = true "Exclusive feedback flag (informational)";

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu Vr0Pu = Efd0Pu * (Ke + AEx * exp(BEx * Efd0Pu));
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / Ka + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Math.Feedback rateFbk;
  Continuous.TransferFunction ll1(a = {tB1, 1}, b = {tC1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Vr0Pu / Ka);
  Continuous.TransferFunction ll2(a = {tB2, 1}, b = {tC2, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Vr0Pu / Ka);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VaMaxPu, YMin = VaMinPu);
  Modelica.Blocks.Math.Feedback excIn;
  Modelica.Blocks.Continuous.Integrator excIntegrator(k = 1 / tE, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add satSum(k1 = Ke);
  Modelica.Blocks.Math.Product satProd;
  Modelica.Blocks.Math.Gain expGain(k = AEx);
  Modelica.Blocks.Math.Power expBex(base = exp(BEx), useExp = true);
  Modelica.Blocks.Continuous.Derivative derivative(k = Kf, T = tF, x_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VrMaxPu, uMin = VrMinPu);

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rateFbk.u1);
  connect(rateFbk.y, ll1.u);
  connect(ll1.y, ll2.u);
  connect(ll2.y, regulator.u);
  connect(regulator.y, excIn.u1);
  connect(satSum.y, excIn.u2);
  connect(excIn.y, excIntegrator.u);
  connect(excIntegrator.y, outLim.u);
  connect(outLim.y, EfdPu);
  connect(excIntegrator.y, satSum.u1);
  connect(excIntegrator.y, satProd.u1);
  connect(excIntegrator.y, expBex.u);
  connect(expBex.y, expGain.u);
  connect(expGain.y, satProd.u2);
  connect(satProd.y, satSum.u2);
  connect(excIntegrator.y, derivative.u);
  connect(derivative.y, rateFbk.u2);

  annotation(preferredView = "text");
end ExcREXS;
