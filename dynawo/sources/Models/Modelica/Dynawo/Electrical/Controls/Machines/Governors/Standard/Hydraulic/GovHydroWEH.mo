within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovHydroWEH "Woodward Electronic hydro governor (CGMES GovHydroWEH)"
  /*
    Woodward Electronic hydro governor. Permanent droop Rpp (gate position
    feedback) plus speed-droop Rpg act on the speed error which goes through
    a PI compensator (Tp pole, Td1 derivative-filter, Tf integrator); the
    output drives a rate-limited gate servo (gtmxop / gtmxcl) bounded
    Gmin..Gpmax. The 10-point PMSS curve (Pmss1..10) is a pumped-storage
    start-stop schedule exposed as a lookup table (informational here -
    output is used unmodified for steady-state). The water column is the
    classical (1 - Tw.s) / (1 + 0.5.Tw.s) lead-lag with Pmin..Pmax limits.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Reg "Regulator gain";
  parameter Types.PerUnit Rpg "Permanent speed droop";
  parameter Types.PerUnit Rpp "Permanent gate droop";
  parameter Types.Time tD "Compensator derivative time constant in s";
  parameter Types.Time tD2 "Secondary derivative time constant in s";
  parameter Types.Time tF "Integrator time constant in s";
  parameter Types.Time tG "Gate servo time constant in s";
  parameter Types.Time tP "Compensator pole time constant in s";
  parameter Types.Time tW "Water inertia time constant in s";
  parameter Types.PerUnit Gtmxop "Gate maximum opening rate in pu/s";
  parameter Types.PerUnit Gtmxcl "Gate maximum closing rate in pu/s";
  parameter Types.PerUnit Gpmax "Gate position upper limit";
  parameter Types.PerUnit GMax "Output power upper limit";
  parameter Types.PerUnit GMin "Output power lower limit";
  parameter Types.PerUnit PMax "Mechanical power upper limit";
  parameter Types.PerUnit PMin "Mechanical power lower limit";
  parameter Types.PerUnit Pmss1; parameter Types.PerUnit Pmss2; parameter Types.PerUnit Pmss3; parameter Types.PerUnit Pmss4; parameter Types.PerUnit Pmss5;
  parameter Types.PerUnit Pmss6; parameter Types.PerUnit Pmss7; parameter Types.PerUnit Pmss8; parameter Types.PerUnit Pmss9; parameter Types.PerUnit Pmss10;
  parameter Types.PerUnit Dpv "Power deviation threshold";
  parameter Types.PerUnit Dicn "Inertia compensation";
  parameter Types.PerUnit Dspv "Speed deviation threshold";
  parameter Real FeedbackSignal "Feedback signal selector (informational)";

  parameter Types.ActivePowerPu Pm0Pu;
  final parameter Types.PerUnit GV0 = Pm0Pu;
  final parameter Types.PerUnit PRef0Pu = Rpp * GV0 "Steady-state: PRef balances permanent gate droop Rpp*gate (omega_err = 0 at init)";

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1);
  Modelica.Blocks.Math.Gain rpgDroop(k = Rpg);
  Modelica.Blocks.Math.Gain rppDroop(k = Rpp);
  Modelica.Blocks.Math.Sum sumErr(k = {1, -1, -1, -1}, nin = 4);
  Modelica.Blocks.Continuous.FirstOrder pPole(T = max(tP, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.Derivative dStage(k = 1, T = max(tD, 1e-6), x_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.Integrator iStage(k = 1 / max(tF, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.NoInit) "Compensator integrator: holds 0 at steady state (gate position is held by gateInteg, not here); NoInit avoids SteadyState constraint k*u=0 when tF is large";
  Modelica.Blocks.Math.Add piSum;
  Modelica.Blocks.Math.Gain regGain(k = Reg);
  Modelica.Blocks.Nonlinear.Limiter velLim(uMax = Gtmxop, uMin = -Gtmxcl, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator gateInteg(outMax = Gpmax, outMin = GMin, y_start = GV0, k = 1 / max(tG, 1e-6));
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction waterColumn(a = {0.5 * tW, 1}, b = {-tW, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(addErrSpeed.y, rpgDroop.u);
  connect(gateInteg.y, rppDroop.u);
  connect(PRefPu, sumErr.u[1]);
  connect(rppDroop.y, sumErr.u[2]);
  connect(rpgDroop.y, sumErr.u[3]);
  sumErr.u[4] = 0;
  connect(sumErr.y, pPole.u);
  connect(pPole.y, dStage.u);
  connect(pPole.y, iStage.u);
  connect(dStage.y, piSum.u1);
  connect(iStage.y, piSum.u2);
  connect(piSum.y, regGain.u);
  connect(regGain.y, velLim.u);
  connect(velLim.y, gateInteg.u);
  connect(gateInteg.y, waterColumn.u);
  connect(waterColumn.y, pmLim.u);
  connect(pmLim.y, PmPu);

  annotation(preferredView = "text");
end GovHydroWEH;
