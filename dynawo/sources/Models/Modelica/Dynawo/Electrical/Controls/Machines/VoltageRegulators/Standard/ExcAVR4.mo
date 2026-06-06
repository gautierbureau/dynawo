within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAVR4 "European AVR model 4 (CGMES ExcAVR4)"
  /*
    AVR with separate self-excitation gain Ke and exciter rate feedback
    Kf*s/(1+sTf1). Equivalent to a continuous-acting AC commutator exciter:
    Vr - (Ke + Se(Efd))*Efd drives a 1/sTe integrator producing Efd; Kf
    feeds back the rate of Efd against the error. Lead-lag (1+sTc)/(1+sTb)
    sits before the Ka/(1+sTa) regulator.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit AEx;
  parameter Types.PerUnit BEx;
  parameter Types.PerUnit Ka;
  parameter Types.PerUnit Ke;
  parameter Types.PerUnit Kf;
  parameter Types.Time tA;
  parameter Types.Time tB;
  parameter Types.Time tC;
  parameter Types.Time tE;
  parameter Types.Time tF1;
  parameter Types.Time tR;
  parameter Types.VoltageModulePu VrMaxPu;
  parameter Types.VoltageModulePu VrMinPu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Math.Feedback rateFbk;
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction leadLag(a = {tB, 1}, b = {tC, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Vr0Pu / Ka);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VrMaxPu, YMin = VrMinPu);
  Modelica.Blocks.Math.Feedback excIn;
  Modelica.Blocks.Continuous.Integrator excIntegrator(k = 1 / tE, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add satSum(k1 = Ke);
  Modelica.Blocks.Math.Product satProd;
  Modelica.Blocks.Math.Gain expGain(k = AEx);
  Modelica.Blocks.Math.Power expBex(base = exp(BEx), useExp = true);
  Modelica.Blocks.Continuous.Derivative derivative(k = Kf, T = tF1, x_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;

  final parameter Types.VoltageModulePu Vr0Pu = Efd0Pu * (Ke + AEx * exp(BEx * Efd0Pu));
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / Ka + Us0Pu;

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rateFbk.u1);
  connect(rateFbk.y, leadLag.u);
  connect(leadLag.y, regulator.u);
  connect(regulator.y, excIn.u1);
  connect(satSum.y, excIn.u2);
  connect(excIn.y, excIntegrator.u);
  connect(excIntegrator.y, EfdPu);
  connect(excIntegrator.y, satSum.u1);
  connect(excIntegrator.y, satProd.u1);
  connect(excIntegrator.y, expBex.u);
  connect(expBex.y, expGain.u);
  connect(expGain.y, satProd.u2);
  connect(satProd.y, satSum.u2);
  connect(excIntegrator.y, derivative.u);
  connect(derivative.y, rateFbk.u2);

  annotation(preferredView = "text");
end ExcAVR4;
