within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovSteamFV3 "Extended steam governor with fast valving (CGMES GovSteamFV3)"
  /*
    Extended fast-valving steam governor. Speed error via 1/R droop drives
    a T1 servo lag, then through a Ta/Tb/Tc multi-stage cascade with the K
    overall gain and a (1+sT5)/(1+sT6) compensator. A 6-point gv/pgv
    piecewise-linear table shapes the gate-to-power conversion. Output
    bounded by Pmax/Pmin, with intermediate Prmax limiting the regulator
    output. Valve velocity limited by Uo/Uc.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit K "Overall fast-valving gain";
  parameter Types.PerUnit PMax;
  parameter Types.PerUnit PMin;
  parameter Types.PerUnit PrMax "Regulator output upper limit";
  parameter Types.PerUnit R;
  parameter Types.Time t1;
  parameter Types.Time t2;
  parameter Types.Time t3;
  parameter Types.Time t4;
  parameter Types.Time t5;
  parameter Types.Time t6;
  parameter Types.Time tA;
  parameter Types.Time tB;
  parameter Types.Time tC;
  parameter Types.PerUnit Uc "Maximum valve closing rate in pu/s";
  parameter Types.PerUnit Uo "Maximum valve opening rate in pu/s";
  parameter Types.PerUnit GV1; parameter Types.PerUnit GV2; parameter Types.PerUnit GV3;
  parameter Types.PerUnit GV4; parameter Types.PerUnit GV5; parameter Types.PerUnit GV6;
  parameter Types.PerUnit PGV1; parameter Types.PerUnit PGV2; parameter Types.PerUnit PGV3;
  parameter Types.PerUnit PGV4; parameter Types.PerUnit PGV5; parameter Types.PerUnit PGV6;

  parameter Types.ActivePowerPu Pm0Pu;
  final parameter Real PgvTable[6, 2] = [GV1, PGV1; GV2, PGV2; GV3, PGV3; GV4, PGV4; GV5, PGV5; GV6, PGV6];

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k2 = -1);
  Modelica.Blocks.Math.Gain droop(k = 1 / R);
  Modelica.Blocks.Math.Feedback fb1;
  Modelica.Blocks.Continuous.FirstOrder servoLag(T = max(t1, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain kGain(k = K);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction compensator(a = {t6, 1}, b = {t5, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Nonlinear.Limiter prLim(uMax = PrMax, uMin = 0, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.FirstOrder stageA(T = max(tA, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder stageB(T = max(tB, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder stageC(T = max(tC, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Tables.CombiTable1Ds pgvCurve(table = PgvTable, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments);
  Modelica.Blocks.Nonlinear.Limiter rateLim(uMax = Uo, uMin = -Uc, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder gateInteg(K = 1, tFilter = t4, Y0 = Pm0Pu, YMax = 1, YMin = 0) "Gate servo as lag with position limits (an integrator topology would require the upstream chain to give 0 at init since there is no output feedback)";
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(fb1.y, servoLag.u);
  connect(servoLag.y, kGain.u);
  connect(kGain.y, compensator.u);
  connect(compensator.y, prLim.u);
  connect(prLim.y, rateLim.u);
  connect(rateLim.y, gateInteg.u);
  connect(gateInteg.y, pgvCurve.u);
  connect(pgvCurve.y[1], stageA.u);
  connect(stageA.y, stageB.u);
  connect(stageB.y, stageC.u);
  connect(stageC.y, pmLim.u);
  connect(pmLim.y, PmPu);

  annotation(preferredView = "text");
end GovSteamFV3;
