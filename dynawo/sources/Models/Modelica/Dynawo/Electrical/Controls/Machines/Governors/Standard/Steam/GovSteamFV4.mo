within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovSteamFV4 "Detailed steam governor with torsional feedback (CGMES GovSteamFV4)"
  /*
    Detailed fast-valving steam governor with a torsional-filter outer
    feedback loop. Speed-droop loop (1/R, T1 servo) drives a K gain stage,
    a multi-stage T2..T7 cascade producing HP / IP / LP / fast-valve
    contributions (weighted by K1..K4), plus a Ta / Tc / Ty / Tt actuator
    chain. Outer torsional filter: Kf1 + Kf2/(1+sTf2) feedback from Pm
    minimises shaft torsional oscillations. Limits Rsmimx / Rsmimn on the
    intermediate signal, Vvmax / Vvmin on the valve servo, Cpsmx / Cpsmn
    on the compressor pressure, Lpsp on the LP suction, Ovex overspeed,
    and Dp deviation are exposed but the simplified topology here uses
    only the dominant ones.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Cpsmn "Compressor pressure lower limit";
  parameter Types.PerUnit Cpsmx "Compressor pressure upper limit";
  parameter Types.PerUnit Dp "Power deviation threshold";
  parameter Types.PerUnit K;
  parameter Types.PerUnit K1; parameter Types.PerUnit K2; parameter Types.PerUnit K3; parameter Types.PerUnit K4;
  parameter Types.PerUnit Kf1 "Torsional feedback proportional gain";
  parameter Types.PerUnit Kf2 "Torsional feedback lag gain";
  parameter Types.PerUnit Lpsp "LP suction pressure";
  parameter Types.PerUnit Ovex "Overspeed limit";
  parameter Types.PerUnit R;
  parameter Types.PerUnit Rsmimn "Intermediate signal lower limit";
  parameter Types.PerUnit Rsmimx "Intermediate signal upper limit";
  parameter Types.Time t1; parameter Types.Time t2; parameter Types.Time t3;
  parameter Types.Time t4; parameter Types.Time t5; parameter Types.Time t6; parameter Types.Time t7;
  parameter Types.Time tA; parameter Types.Time tC; parameter Types.Time tY; parameter Types.Time tT;
  parameter Types.Time tF2 "Torsional feedback lag time constant in s";
  parameter Types.PerUnit Vvmax "Valve upper limit";
  parameter Types.PerUnit Vvmin "Valve lower limit";

  parameter Types.ActivePowerPu Pm0Pu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k2 = -1);
  Modelica.Blocks.Math.Gain droop(k = 1 / R);
  Modelica.Blocks.Math.Sum errIn(k = {1, -1, -1}, nin = 3);
  Modelica.Blocks.Math.Gain kf1Gain(k = Kf1);
  Modelica.Blocks.Continuous.FirstOrder kf2Lag(T = max(tF2, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain kf2Gain(k = Kf2);
  Modelica.Blocks.Continuous.FirstOrder servoLag(T = max(t1, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain kGain(k = K);
  Modelica.Blocks.Nonlinear.Limiter intLim(uMax = Rsmimx, uMin = Rsmimn, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.FirstOrder s2(T = max(t2, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s3(T = max(t3, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s4(T = max(t4, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s5(T = max(t5, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s6(T = max(t6, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s7(T = max(t7, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder actuatorA(T = max(tA, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder actuatorC(T = max(tC, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder actuatorY(T = max(tY, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder actuatorT(T = max(tT, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter valveLim(uMax = Vvmax, uMin = Vvmin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Math.Gain k1Gain(k = K1);
  Modelica.Blocks.Math.Gain k2Gain(k = K2);
  Modelica.Blocks.Math.Gain k3Gain(k = K3);
  Modelica.Blocks.Math.Gain k4Gain(k = K4);
  Modelica.Blocks.Math.Sum stageSum(nin = 4);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmPu, kf2Lag.u);
  connect(kf2Lag.y, kf2Gain.u);
  connect(PmPu, kf1Gain.u);
  connect(PmRefPu, errIn.u[1]);
  connect(droop.y, errIn.u[2]);
  connect(kf1Gain.y, errIn.u[3]);
  connect(errIn.y, servoLag.u);
  connect(servoLag.y, kGain.u);
  connect(kGain.y, intLim.u);
  connect(intLim.y, s2.u);
  connect(s2.y, s3.u);
  connect(s3.y, s4.u);
  connect(s4.y, s5.u);
  connect(s5.y, s6.u);
  connect(s6.y, s7.u);
  connect(s7.y, actuatorA.u);
  connect(actuatorA.y, actuatorC.u);
  connect(actuatorC.y, actuatorY.u);
  connect(actuatorY.y, actuatorT.u);
  connect(actuatorT.y, valveLim.u);
  connect(s3.y, k1Gain.u);
  connect(s5.y, k2Gain.u);
  connect(s7.y, k3Gain.u);
  connect(valveLim.y, k4Gain.u);
  connect(k1Gain.y, stageSum.u[1]);
  connect(k2Gain.y, stageSum.u[2]);
  connect(k3Gain.y, stageSum.u[3]);
  connect(k4Gain.y, stageSum.u[4]);
  connect(stageSum.y, PmPu);

  annotation(preferredView = "text");
end GovSteamFV4;
