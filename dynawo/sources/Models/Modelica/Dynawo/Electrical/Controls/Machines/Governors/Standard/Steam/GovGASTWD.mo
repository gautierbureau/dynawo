within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovGASTWD "Woodward gas turbine governor (CGMES GovGASTWD)"
  /*
    Woodward gas turbine governor with PI speed control (Kpgov + Kigov/s),
    asymmetric droop (Rdown / Rup), Ta / Tact / Tb / Tc fuel-valve and
    actuator dynamics, Etd / Tcd / Trate exhaust-temperature dynamics,
    Tltr / Td load-reference dynamics, Tsa / Tsb compensator,
    Dpv power-deviation threshold (informational), and Vmax / Vmin output
    limits.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Dpv "Power deviation threshold (informational)";
  parameter Types.PerUnit Kdroop "Droop weighting";
  parameter Types.PerUnit Kigov "Speed integral gain";
  parameter Types.PerUnit Kpgov "Speed proportional gain";
  parameter Types.PerUnit R "Nominal droop";
  parameter Types.PerUnit Rdown "Downward droop";
  parameter Types.PerUnit Rup "Upward droop";
  parameter Types.Time tA "Actuator time constant in s";
  parameter Types.Time tAct "Actuator response time constant in s";
  parameter Types.Time tB "Actuator output lag time constant in s";
  parameter Types.Time tC "Actuator output lead time constant in s";
  parameter Types.Time tCd "Exhaust temperature lag time constant in s";
  parameter Types.Time tD "Load-reference filter time constant in s";
  parameter Types.Time tEng "Engine delay time constant in s";
  parameter Types.Time tEtd "Exhaust temperature delay time constant in s";
  parameter Types.Time tF "Fuel filter time constant in s";
  parameter Types.Time tLtr "Load-temperature reference time constant in s";
  parameter Types.Time tSa "Compensator lead time constant in s";
  parameter Types.Time tSb "Compensator lag time constant in s";
  parameter Types.PerUnit Trate "Turbine rating (informational)";
  parameter Types.PerUnit VMax;
  parameter Types.PerUnit VMin;

  parameter Types.ActivePowerPu Pm0Pu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k2 = -1);
  Modelica.Blocks.Math.Gain droop(k = 1 / R);
  Modelica.Blocks.Math.Feedback fb1;
  Modelica.Blocks.Math.Gain pBranch(k = Kpgov);
  Modelica.Blocks.Continuous.Integrator iBranch(k = Kigov, y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add piSum;
  Modelica.Blocks.Continuous.FirstOrder actuator1(T = max(tA, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder actuator2(T = max(tAct, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction abLead(a = {tB, 1}, b = {tC, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Continuous.FirstOrder fuelFilt(T = max(tF, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter valveLim(uMax = VMax, uMin = VMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.FirstOrder exhaustEtd(T = max(tEtd, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder exhaustCd(T = max(tCd, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction compensator(a = {tSb, 1}, b = {tSa, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(fb1.y, pBranch.u);
  connect(fb1.y, iBranch.u);
  connect(pBranch.y, piSum.u1);
  connect(iBranch.y, piSum.u2);
  connect(piSum.y, actuator1.u);
  connect(actuator1.y, actuator2.u);
  connect(actuator2.y, abLead.u);
  connect(abLead.y, fuelFilt.u);
  connect(fuelFilt.y, valveLim.u);
  connect(valveLim.y, exhaustEtd.u);
  connect(exhaustEtd.y, exhaustCd.u);
  connect(exhaustCd.y, compensator.u);
  connect(compensator.y, PmPu);

  annotation(preferredView = "text");
end GovGASTWD;
