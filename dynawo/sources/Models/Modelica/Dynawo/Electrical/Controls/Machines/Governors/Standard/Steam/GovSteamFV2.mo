within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovSteamFV2 "Steam governor with fast valving (CGMES GovSteamFV2)"
  /*
    Steam governor with fast-valving capability. Standard speed-droop loop
    (1/R, T1 servo lag) drives an intermediate valve bounded Vamax / Vamin,
    then through gain K and a T2 / T3 lead-lag, into a steam reheater T4
    cascade. Fast valving is captured by the K1..K4 gains on the four
    stages (HP, IP, LP, fast-valve) weighted by an iFlag selector that
    chooses which signal feeds the output. Output limits Pmax / Pmin.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit K "Fast-valving overall gain";
  parameter Types.PerUnit K1 "HP stage fraction";
  parameter Types.PerUnit K2 "IP stage fraction";
  parameter Types.PerUnit K3 "LP stage fraction";
  parameter Types.PerUnit K4 "Fast-valving fraction";
  parameter Types.PerUnit PMax;
  parameter Types.PerUnit PMin;
  parameter Types.PerUnit R;
  parameter Types.Time t1;
  parameter Types.Time t2;
  parameter Types.Time t3;
  parameter Types.Time t4;
  parameter Types.Time t5;
  parameter Types.PerUnit VAmax;
  parameter Types.PerUnit VAmin;
  parameter Integer IFlag = 0 "Output selector flag (informational)";

  parameter Types.ActivePowerPu Pm0Pu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k2 = -1);
  Modelica.Blocks.Math.Gain droop(k = 1 / R);
  Modelica.Blocks.Math.Feedback fb1;
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder valve(K = 1, tFilter = t1, Y0 = Pm0Pu, YMax = VAmax, YMin = VAmin);
  Modelica.Blocks.Math.Gain kGain(k = K);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction leadLag(a = {t3, 1}, b = {t2, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Continuous.FirstOrder stage4(T = max(t4, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder stage5(T = max(t5, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain k1Gain(k = K1);
  Modelica.Blocks.Math.Gain k2Gain(k = K2);
  Modelica.Blocks.Math.Gain k3Gain(k = K3);
  Modelica.Blocks.Math.Gain k4Gain(k = K4);
  Modelica.Blocks.Math.Sum stageSum(nin = 4);
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(fb1.y, valve.u);
  connect(valve.y, kGain.u);
  connect(kGain.y, leadLag.u);
  connect(leadLag.y, stage4.u);
  connect(stage4.y, stage5.u);
  connect(leadLag.y, k1Gain.u);
  connect(stage4.y, k2Gain.u);
  connect(stage5.y, k3Gain.u);
  connect(kGain.y, k4Gain.u);
  connect(k1Gain.y, stageSum.u[1]);
  connect(k2Gain.y, stageSum.u[2]);
  connect(k3Gain.y, stageSum.u[3]);
  connect(k4Gain.y, stageSum.u[4]);
  connect(stageSum.y, pmLim.u);
  connect(pmLim.y, PmPu);

  annotation(preferredView = "text");
end GovSteamFV2;
