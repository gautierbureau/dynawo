within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovSteamCC "Combined-cycle steam governor (CGMES GovSteamCC)"
  /*
    Combined-cycle steam turbine governor with multi-stage steam expansion.
    Speed error with 1/R droop drives a T1 valve servo with Uo/Uc rate
    limits, then through T2 (steam chest), T3 (HP turbine), T4 (IP/LP
    crossover) and Tip / Tlp time constants. HP, IP, LP power fractions
    (Fhp, Fip, Flp) sum the contributions, with Dhp / Dlp turbine damping
    terms. Pm output is clamped Pmin..Pmax.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Dhp "HP turbine damping";
  parameter Types.PerUnit Dlp "LP turbine damping";
  parameter Types.PerUnit Fhp "HP turbine power fraction";
  parameter Types.PerUnit Fip "IP turbine power fraction";
  parameter Types.PerUnit Flp "LP turbine power fraction";
  parameter Types.PerUnit PMax;
  parameter Types.PerUnit PMin;
  parameter Types.PerUnit R;
  parameter Types.Time t1 "Valve servo time constant in s";
  parameter Types.Time t2 "Steam chest time constant in s";
  parameter Types.Time t3 "HP reheater time constant in s";
  parameter Types.Time t4 "IP / LP crossover time constant in s";
  parameter Types.Time tIp "IP stage time constant in s";
  parameter Types.Time tLp "LP stage time constant in s";
  parameter Types.PerUnit Uc "Maximum valve closing rate in pu/s";
  parameter Types.PerUnit Uo "Maximum valve opening rate in pu/s";

  parameter Types.ActivePowerPu Pm0Pu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k2 = -1);
  Modelica.Blocks.Math.Gain droop(k = 1 / R);
  Modelica.Blocks.Math.Feedback fb1;
  Modelica.Blocks.Nonlinear.Limiter rateLim(uMax = Uo, uMin = -Uc, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder valveInteg(K = 1, tFilter = t1, Y0 = Pm0Pu, YMax = PMax, YMin = PMin) "Valve servo as lag with output limits (an integrator topology would require PmRef = 0 at init to satisfy the SteadyState constraint k*u = 0 since there is no output feedback to close the loop)";
  Modelica.Blocks.Continuous.FirstOrder chest(T = max(t2, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder hpStage(T = max(t3, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder crossover(T = max(t4, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder ipStage(T = max(tIp, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder lpStage(T = max(tLp, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain fhpGain(k = Fhp);
  Modelica.Blocks.Math.Gain fipGain(k = Fip);
  Modelica.Blocks.Math.Gain flpGain(k = Flp);
  Modelica.Blocks.Math.Add3 stageSum;
  Modelica.Blocks.Math.Gain dhpGain(k = Dhp);
  Modelica.Blocks.Math.Gain dlpGain(k = Dlp);
  Modelica.Blocks.Math.Add3 dampSum(k2 = 1, k3 = 1);
  Modelica.Blocks.Math.Feedback fbDamp;
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(fb1.y, rateLim.u);
  connect(rateLim.y, valveInteg.u);
  connect(valveInteg.y, chest.u);
  connect(chest.y, hpStage.u);
  connect(hpStage.y, crossover.u);
  connect(crossover.y, ipStage.u);
  connect(ipStage.y, lpStage.u);
  connect(hpStage.y, fhpGain.u);
  connect(ipStage.y, fipGain.u);
  connect(lpStage.y, flpGain.u);
  connect(fhpGain.y, stageSum.u1);
  connect(fipGain.y, stageSum.u2);
  connect(flpGain.y, stageSum.u3);
  connect(dW.y, dhpGain.u);
  connect(dW.y, dlpGain.u);
  connect(dhpGain.y, dampSum.u2);
  connect(dlpGain.y, dampSum.u3);
  connect(stageSum.y, dampSum.u1);
  connect(dampSum.y, pmLim.u);
  connect(pmLim.y, PmPu);
  fbDamp.u1 = 0; fbDamp.u2 = 0;

  annotation(preferredView = "text");
end GovSteamCC;
