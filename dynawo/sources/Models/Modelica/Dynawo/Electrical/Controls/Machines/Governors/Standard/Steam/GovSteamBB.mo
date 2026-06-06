within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovSteamBB "Generic boiler/turbine steam governor (CGMES GovSteamBB)"
  /*
    Generic boiler / turbine steam governor (PES TR1 baseline). Speed
    error (omegaRef - omega), with permanent droop R taken from the
    mechanical-power output, drives a Ka / (1 + sTa) regulator into a
    1 / (sTg) valve-positioner integrator that is rate-limited
    (Uo, Uc) and position-limited (Pmin, Pmax). The valve position
    multiplies the boiler steam pressure (a 1 / (1 + sTb) pressure
    storage state) to give the steam flow into a 1 / (1 + sTt)
    turbine stage. The mechanical power output Pm is then clamped to
    Pmin..Pmax.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit R "Permanent speed droop in pu";
  parameter Types.PerUnit Ka "Regulator gain";
  parameter Types.Time tA "Regulator time constant in s";
  parameter Types.Time tG "Valve-positioner time constant in s";
  parameter Types.Time tB "Boiler pressure time constant in s";
  parameter Types.Time tT "Turbine stage time constant in s";
  parameter Types.PerUnit Uo "Maximum valve opening velocity in pu/s";
  parameter Types.PerUnit Uc "Maximum valve closing velocity in pu/s (positive)";
  parameter Types.PerUnit PMax "Maximum valve position / mechanical power";
  parameter Types.PerUnit PMin "Minimum valve position / mechanical power";

  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  final parameter Types.PerUnit Valve0 = Pm0Pu "Initial valve position (with normalised boiler pressure = 1 at init, steam flow = valve = Pm)";
  final parameter Types.PerUnit PRef0Pu = R * Valve0 "Initial power reference";

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1) "omegaPu - omegaRefPu";
  Modelica.Blocks.Math.Gain droopGain(k = R) "Permanent droop on the Pm feedback";
  Modelica.Blocks.Math.Add3 sumErr(k1 = 1, k2 = -1, k3 = -1) "PRef - R*Pm - speed_err";
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = max(tA, 1e-6), Y0 = 0, YMax = max(PMax, 1e3), YMin = min(PMin, -1e3)) "Ka / (1 + sTa) regulator (Y0 = 0 because the regulator only sees the offset from setpoint at steady state)";
  Modelica.Blocks.Nonlinear.Limiter velLim(uMax = Uo, uMin = -Uc, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator valveInteg(outMax = PMax, outMin = PMin, y_start = Valve0, k = 1 / max(tG, 1e-6)) "1 / (sTg) valve-positioner integrator";
  Modelica.Blocks.Continuous.FirstOrder boilerLag(T = max(tB, 1e-6), y_start = 1, initType = Modelica.Blocks.Types.Init.SteadyState) "Boiler steam pressure (normalised, 1 at init)";
  Modelica.Blocks.Sources.Constant pressureRef(k = 1) "Fuel/firing reference - normalised so boiler pressure is 1 at init";
  Modelica.Blocks.Math.Product steamFlow "Flow = valve_position * boiler_pressure";
  Modelica.Blocks.Continuous.FirstOrder turbineLag(T = max(tT, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState) "Turbine stage 1 / (1 + sTt)";
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(PRefPu, sumErr.u1);
  connect(turbineLag.y, droopGain.u);
  connect(droopGain.y, sumErr.u2);
  connect(addErrSpeed.y, sumErr.u3);
  connect(sumErr.y, regulator.u);
  connect(regulator.y, velLim.u);
  connect(velLim.y, valveInteg.u);
  connect(pressureRef.y, boilerLag.u);
  connect(valveInteg.y, steamFlow.u1);
  connect(boilerLag.y, steamFlow.u2);
  connect(steamFlow.y, turbineLag.u);
  connect(turbineLag.y, pmLim.u);
  connect(pmLim.y, PmPu);

  annotation(preferredView = "text");
end GovSteamBB;
