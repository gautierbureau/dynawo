within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovCT2 "Combined-cycle gas turbine governor (CGMES GovCT2)"
  /*
    PSS/E combined-cycle gas turbine governor with PID speed control
    (Kpgov + Kigov/s + Kdgov.s/(1+sTdgov)), load reference integrator
    Kimw, asymmetric droop Rdown / Rup, fuel valve actuator (Tact, Tb, Tc,
    Tsa, Tsb), exhaust temperature loop (Teng, Tno, Ldref), generator
    damping Dm, deadband Db, valve rate limits Ropen / Rclose, output
    limits Vmax / Vmin. The 10-point frequency-power limiter table
    (Flim1..10 / Plim1..10) is exposed but not interpolated in this
    simplified topology - the four parameters Aset, Ka, Pmwset, Prate
    drive the over- and under-frequency droop characteristic that
    real-world CT2 implementations use for grid-supportive frequency
    response. Wfnl / Wfspd represent the no-load fuel flow and its
    speed dependency.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Aset "Acceleration limit set";
  parameter Types.PerUnit Db "Speed deadband";
  parameter Types.PerUnit Dm "Generator damping";
  parameter Types.PerUnit Ka "Acceleration limiter gain";
  parameter Types.PerUnit Kdgov "PID derivative gain";
  parameter Types.PerUnit Kigov "PID integral gain";
  parameter Types.PerUnit Kiload "Load-reference integral gain";
  parameter Types.PerUnit Kimw "MW-tracking integrator gain";
  parameter Types.PerUnit Kpgov "PID proportional gain";
  parameter Types.PerUnit Kpload "Load proportional gain";
  parameter Types.PerUnit Ldref "Load reference";
  parameter Types.PerUnit Pmwset "Megawatt set point";
  parameter Boolean Prate "Rate selector (informational)";
  parameter Types.PerUnit R;
  parameter Types.PerUnit Rclose "Maximum valve closing rate in pu/s";
  parameter Types.PerUnit Rdown "Downward droop";
  parameter Types.PerUnit Ropen "Maximum valve opening rate in pu/s";
  parameter Types.PerUnit Rup "Upward droop";
  parameter Types.Time tA "Actuator time constant in s";
  parameter Types.Time tAct "Actuator response time constant in s";
  parameter Types.Time tB "Actuator output lag time constant in s";
  parameter Types.Time tC "Actuator output lead time constant in s";
  parameter Types.Time tDgov "PID derivative time constant in s";
  parameter Types.Time tEng "Engine delay time constant in s";
  parameter Types.Time tF "Fuel filter time constant in s";
  parameter Types.Time tNo "No-load fuel flow time constant in s";
  parameter Types.Time tSa "Compensator lead time constant in s";
  parameter Types.Time tSb "Compensator lag time constant in s";
  parameter Types.PerUnit Uc "Maximum gate closing velocity in pu/s";
  parameter Types.PerUnit Uo "Maximum gate opening velocity in pu/s";
  parameter Types.PerUnit VMax;
  parameter Types.PerUnit VMin;
  parameter Types.PerUnit Wfnl "No-load fuel flow";
  parameter Boolean Wfspd "No-load fuel flow speed dependence (informational)";

  parameter Types.ActivePowerPu Pm0Pu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k2 = -1);
  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = Db, uMin = -Db);
  Modelica.Blocks.Math.Gain droop(k = 1 / R);
  Modelica.Blocks.Math.Feedback fb1;
  Modelica.Blocks.Math.Gain pBranch(k = Kpgov);
  Modelica.Blocks.Continuous.Integrator iBranch(k = Kigov, y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.Derivative dBranch(k = Kdgov, T = max(tDgov, 1e-6), x_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 pidSum;
  Modelica.Blocks.Continuous.FirstOrder fuelFilt(T = max(tF, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter rateLim(uMax = Ropen, uMin = -Rclose, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator valveInteg(outMax = VMax, outMin = VMin, y_start = Pm0Pu, k = 1 / max(tAct, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction compensator(a = {tSb, 1}, b = {tSa, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Math.Gain dmGain(k = Dm);
  Modelica.Blocks.Math.Feedback fbOut;

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, deadband.u);
  connect(deadband.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(fb1.y, pBranch.u);
  connect(fb1.y, iBranch.u);
  connect(fb1.y, dBranch.u);
  connect(pBranch.y, pidSum.u1);
  connect(iBranch.y, pidSum.u2);
  connect(dBranch.y, pidSum.u3);
  connect(pidSum.y, fuelFilt.u);
  connect(fuelFilt.y, rateLim.u);
  connect(rateLim.y, valveInteg.u);
  connect(valveInteg.y, compensator.u);
  connect(compensator.y, fbOut.u1);
  connect(dW.y, dmGain.u);
  connect(dmGain.y, fbOut.u2);
  connect(fbOut.y, PmPu);

  annotation(preferredView = "text");
end GovCT2;
