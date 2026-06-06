within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovHydroWPID "Woodward PID hydro governor (CGMES GovHydroWPID)"
  /*
    Woodward PID hydro governor. Speed error (omega - omegaRef) is filtered
    by (1 + sTb) / (1 + sTc), then fed to a parallel Kp + Ki/s + Kd*s/(1+sD)
    PID. Permanent droop R closes the loop from the gate position. The
    rate-limited gate servo (Velmax / Velmin) integrates into a position
    bounded by Gmin..Gmax. The classical rigid-water-column lead-lag
    (1 - Tw.s) / (1 + 0.5.Tw.s) produces the mechanical power, clamped to
    Pmin..Pmax. The mastp parameter (master / slave power) is exposed for
    documentation completeness and used as the steady-state initial gate.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit D "PID derivative filter time constant in s";
  parameter Types.PerUnit GMax "Gate position upper limit";
  parameter Types.PerUnit GMin "Gate position lower limit";
  parameter Types.PerUnit Kd "PID derivative gain";
  parameter Types.PerUnit Ki "PID integral gain";
  parameter Types.PerUnit Kp "PID proportional gain";
  parameter Types.PerUnit Mastp "Master / slave power offset (added to the gate);
                                  used here as the initial gate position";
  parameter Types.PerUnit PMax "Maximum mechanical power";
  parameter Types.PerUnit PMin "Minimum mechanical power";
  parameter Types.PerUnit R "Permanent speed droop in pu";
  parameter Types.Time tA "Speed error input filter time constant in s";
  parameter Types.Time tB "Speed error lead time constant in s";
  parameter Types.Time tC "Speed error lag time constant in s";
  parameter Types.Time tReg "Regulator output time constant in s";
  parameter Types.Time tW "Water inertia time constant in s";
  parameter Types.PerUnit VelMax "Maximum gate velocity in pu/s";
  parameter Types.PerUnit VelMin "Minimum gate velocity in pu/s";

  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  final parameter Types.PerUnit GV0 = Pm0Pu "Initial gate position";
  final parameter Types.PerUnit PRef0Pu = R * GV0 "Initial power reference";

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1);
  Modelica.Blocks.Continuous.FirstOrder errFilter(T = max(tA, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction leadLag(a = {tC, 1}, b = {tB, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);
  Modelica.Blocks.Math.Gain droopGain(k = R);
  Modelica.Blocks.Math.Add3 sumErr(k1 = 1, k2 = -1, k3 = -1);
  Modelica.Blocks.Math.Gain pBranch(k = Kp);
  Modelica.Blocks.Continuous.Integrator iBranch(k = Ki, y_start = 0, initType = Modelica.Blocks.Types.Init.NoInit) "PID integral branch (NoInit: SteadyState init diverges when Ki = 0 because k*u = 0 leaves u under-constrained)";
  Modelica.Blocks.Continuous.Derivative dBranch(k = Kd, T = max(D, 1e-6), x_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 pidSum;
  Modelica.Blocks.Continuous.FirstOrder regLag(T = max(tReg, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter velLim(uMax = VelMax, uMin = VelMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator gateInteg(outMax = GMax, outMin = GMin, y_start = GV0, k = 1);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction waterColumn(a = {0.5 * tW, 1}, b = {-tW, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(addErrSpeed.y, errFilter.u);
  connect(errFilter.y, leadLag.u);
  connect(gateInteg.y, droopGain.u);
  connect(PRefPu, sumErr.u1);
  connect(droopGain.y, sumErr.u2);
  connect(leadLag.y, sumErr.u3);
  connect(sumErr.y, pBranch.u);
  connect(sumErr.y, iBranch.u);
  connect(sumErr.y, dBranch.u);
  connect(pBranch.y, pidSum.u1);
  connect(iBranch.y, pidSum.u2);
  connect(dBranch.y, pidSum.u3);
  connect(pidSum.y, regLag.u);
  connect(regLag.y, velLim.u);
  connect(velLim.y, gateInteg.u);
  connect(gateInteg.y, waterColumn.u);
  connect(waterColumn.y, pmLim.u);
  connect(pmLim.y, PmPu);

  annotation(preferredView = "text");
end GovHydroWPID;
