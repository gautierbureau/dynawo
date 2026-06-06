within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovHydroPID2 "Extended PID hydro governor (CGMES GovHydroPID2)"
  /*
    Simple PID hydro governor: parallel Kp + Ki/s + Kd*s/(1+sTa) controller
    on the speed error (with the optional reset / lag time constant Tb and
    the regulator time constant Treg shaping the input), rate-limited gate
    servo with limits Velmin / Velmax, gate position limited to Gmin..Gmax
    starting at G0, water column 1/(1 + sTw) approximation feeding the gate
    flow to the mechanical power output bounded Pmin..Pmax.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit R "Permanent speed droop in pu";
  parameter Types.PerUnit GMax "Gate position upper limit";
  parameter Types.PerUnit GMin "Gate position lower limit";
  parameter Types.PerUnit G0 "Initial gate position (used if Pm0Pu unspecified)";
  parameter Types.PerUnit Kd "PID derivative gain";
  parameter Types.PerUnit Ki "PID integral gain";
  parameter Types.PerUnit Kp "PID proportional gain";
  parameter Types.PerUnit PMax "Maximum mechanical power";
  parameter Types.PerUnit PMin "Minimum mechanical power";
  parameter Types.Time tA "PID derivative filter time constant in s";
  parameter Types.Time tB "Regulator output lag time constant in s";
  parameter Types.Time tReg "Speed error filter time constant in s";
  parameter Types.Time tW "Water inertia (starting) time constant in s";
  parameter Types.PerUnit VelMax "Maximum gate velocity in pu/s";
  parameter Types.PerUnit VelMin "Minimum gate velocity in pu/s";

  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  final parameter Types.PerUnit GV0 = Pm0Pu "Initial gate position (equals Pm0Pu in this simple model)";
  final parameter Types.PerUnit PRef0Pu = R * GV0 "Initial power reference";

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1);
  Modelica.Blocks.Math.Gain droopGain(k = R);
  Modelica.Blocks.Math.Add3 sumErr(k1 = 1, k2 = -1, k3 = -1) "PRef - droop - speed error";
  Modelica.Blocks.Continuous.FirstOrder errFilter(T = max(tReg, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain pBranch(k = Kp);
  Modelica.Blocks.Continuous.Integrator iBranch(k = Ki, y_start = 0, initType = Modelica.Blocks.Types.Init.NoInit) "PID integral branch (NoInit: SteadyState init diverges when Ki = 0 because k*u = 0 leaves u under-constrained)";
  Modelica.Blocks.Continuous.Derivative dBranch(k = Kd, T = max(tA, 1e-6), x_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 pidSum;
  Modelica.Blocks.Continuous.FirstOrder regLag(T = max(tB, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter velLim(uMax = VelMax, uMin = VelMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator gateInteg(outMax = GMax, outMin = GMin, y_start = GV0, k = 1) "Gate position";
  Modelica.Blocks.Continuous.FirstOrder waterCol(T = max(tW, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState) "Water column approximated as a single lag";
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(addErrSpeed.y, errFilter.u);
  connect(gateInteg.y, droopGain.u);
  connect(PRefPu, sumErr.u1);
  connect(droopGain.y, sumErr.u2);
  connect(errFilter.y, sumErr.u3);
  connect(sumErr.y, pBranch.u);
  connect(sumErr.y, iBranch.u);
  connect(sumErr.y, dBranch.u);
  connect(pBranch.y, pidSum.u1);
  connect(iBranch.y, pidSum.u2);
  connect(dBranch.y, pidSum.u3);
  connect(pidSum.y, regLag.u);
  connect(regLag.y, velLim.u);
  connect(velLim.y, gateInteg.u);
  connect(gateInteg.y, waterCol.u);
  connect(waterCol.y, pmLim.u);
  connect(pmLim.y, PmPu);

  annotation(preferredView = "text");
end GovHydroPID2;
