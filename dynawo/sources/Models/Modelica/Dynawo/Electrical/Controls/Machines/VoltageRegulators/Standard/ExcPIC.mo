within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcPIC "PIC AVR (CGMES ExcPIC)"
  /*
    PIC multi-stage AVR. Backbone: error UsRef - Us -> Ka/(1+sTa) -> cascade
    of five (1+sTk)/(1+sT?) shaping stages (T1..T5) -> Ka2 secondary
    regulator gain -> output limited (Vrmin, Vrmax) and clamped to (Efdmin,
    Efdmax). Kf*s/((1+sTf1)(1+sTf2)) rate feedback. Vr1, Vr2 are
    intermediate signal references; (E1, Se1) and (E2, Se2) parametrise
    the saturation function.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit AEx; parameter Types.PerUnit BEx;
  parameter Types.VoltageModulePu EfdMaxPu; parameter Types.VoltageModulePu EfdMinPu;
  parameter Types.PerUnit Ka; parameter Types.PerUnit Ka2;
  parameter Types.PerUnit Kf;
  parameter Types.Time tA; parameter Types.Time tF1; parameter Types.Time tF2;
  parameter Types.Time t1; parameter Types.Time t2; parameter Types.Time t3;
  parameter Types.Time t4; parameter Types.Time t5;
  parameter Types.VoltageModulePu Vr1Pu; parameter Types.VoltageModulePu Vr2Pu;
  parameter Types.VoltageModulePu VrMaxPu; parameter Types.VoltageModulePu VrMinPu;

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu / max(Ka * Ka2, 1e-6) + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Math.Feedback rateFbk;
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Efd0Pu / max(Ka2, 1e-6), YMax = VrMaxPu, YMin = VrMinPu);
  Modelica.Blocks.Continuous.FirstOrder s1(T = max(t1, 1e-6), y_start = Efd0Pu / max(Ka2, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s2(T = max(t2, 1e-6), y_start = Efd0Pu / max(Ka2, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s3(T = max(t3, 1e-6), y_start = Efd0Pu / max(Ka2, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s4(T = max(t4, 1e-6), y_start = Efd0Pu / max(Ka2, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s5(T = max(t5, 1e-6), y_start = Efd0Pu / max(Ka2, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain ka2Gain(k = Ka2);
  Modelica.Blocks.Continuous.Derivative rateFbk1(k = Kf, T = max(tF1, 1e-6), x_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder rateFbk2(T = max(tF2, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = EfdMaxPu, uMin = EfdMinPu);

equation
  connect(UsRefPu, errIn.u1);
  connect(UsPu, errIn.u2);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rateFbk.u1);
  connect(rateFbk.y, regulator.u);
  connect(regulator.y, s1.u);
  connect(s1.y, s2.u);
  connect(s2.y, s3.u);
  connect(s3.y, s4.u);
  connect(s4.y, s5.u);
  connect(s5.y, ka2Gain.u);
  connect(ka2Gain.y, outLim.u);
  connect(outLim.y, EfdPu);
  connect(EfdPu, rateFbk1.u);
  connect(rateFbk1.y, rateFbk2.u);
  connect(rateFbk2.y, rateFbk.u2);

  annotation(preferredView = "text");
end ExcPIC;
