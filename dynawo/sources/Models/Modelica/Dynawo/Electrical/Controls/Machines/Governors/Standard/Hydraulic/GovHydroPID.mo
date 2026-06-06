within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovHydroPID "PID-based hydro turbine-governor (CGMES GovHydroPID)"
  /*
    Full PID hydro turbine-governor. Speed error after deadband (Db1 / Db2,
    Eps) feeds a Kp + Ki/s + Kd*s/(1+sTd) parallel PID; output is filtered
    by 1/(1+sTf), rate-limited by Velm, and integrated into the gate
    position (Pmin..Pmax). A 5-point piecewise-linear gate-power table
    (GV1..5 / PGV1..5) shapes the gate-to-power conversion. The lead-lag
    turbine block (Aturb.s + 1) / (Bturb.Tturb.s + 1) approximates the
    elastic water column. Tg / Tp / Tt / Tr are exposed for the dashpot
    droop subsystem; this simplified topology uses only the dominant time
    constants Tf and Tt.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit R "Permanent speed droop";
  parameter Types.PerUnit Kp;
  parameter Types.PerUnit Ki;
  parameter Types.PerUnit Kd;
  parameter Types.Time tD "PID derivative filter time constant in s";
  parameter Types.Time tF "Regulator output filter time constant in s";
  parameter Types.Time tG "Gate servo time constant (exposed; not used in this simplified model)";
  parameter Types.Time tP "Pilot servo time constant (exposed; not used in this simplified model)";
  parameter Types.Time tT "Turbine time constant in s";
  parameter Types.Time tR "Dashpot reset time constant (exposed; not used in this simplified model)";
  parameter Types.Time tTurb "Turbine block denominator time constant in s";
  parameter Types.PerUnit Velm "Gate velocity limit in pu/s (symmetric)";
  parameter Types.PerUnit PMax;
  parameter Types.PerUnit PMin;
  parameter Types.PerUnit ATurb "Turbine block numerator coefficient (typical: -1)";
  parameter Types.PerUnit BTurb "Turbine block denominator coefficient (typical: 0.5)";
  parameter Types.AngularVelocityPu Db1 "Intentional speed deadband half-width (low)";
  parameter Types.AngularVelocityPu Db2 "Intentional speed deadband half-width (high)";
  parameter Types.PerUnit Eps "Speed deadband transition width";
  parameter Types.PerUnit GV1; parameter Types.PerUnit GV2; parameter Types.PerUnit GV3; parameter Types.PerUnit GV4; parameter Types.PerUnit GV5;
  parameter Types.PerUnit PGV1; parameter Types.PerUnit PGV2; parameter Types.PerUnit PGV3; parameter Types.PerUnit PGV4; parameter Types.PerUnit PGV5;

  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";
  final parameter Types.PerUnit GV0 = Pm0Pu "Initial gate position";
  final parameter Types.PerUnit PRef0Pu = R * GV0 "Initial power reference";
  final parameter Real PgvTable[5, 2] = [GV1, PGV1; GV2, PGV2; GV3, PGV3; GV4, PGV4; GV5, PGV5];

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1);
  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = max(Db1, Db2), uMin = -max(Db1, Db2));
  Modelica.Blocks.Math.Gain droopGain(k = R);
  Modelica.Blocks.Math.Add3 sumErr(k1 = 1, k2 = -1, k3 = -1);
  Modelica.Blocks.Math.Gain pBranch(k = Kp);
  Modelica.Blocks.Continuous.Integrator iBranch(k = Ki, y_start = 0, initType = Modelica.Blocks.Types.Init.NoInit) "PID integral branch (NoInit: SteadyState init diverges when Ki = 0 because k*u = 0 leaves u under-constrained)";
  Modelica.Blocks.Continuous.Derivative dBranch(k = Kd, T = max(tD, 1e-6), x_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 pidSum;
  Modelica.Blocks.Continuous.FirstOrder regFilter(T = max(tF, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter velLim(uMax = Velm, uMin = -Velm, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator gateInteg(outMax = PMax, outMin = PMin, y_start = GV0, k = 1);
  Modelica.Blocks.Tables.CombiTable1Ds pgvCurve(table = PgvTable, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction turbineLL(a = {BTurb * tTurb, 1}, b = {ATurb * tT, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(addErrSpeed.y, deadband.u);
  connect(gateInteg.y, droopGain.u);
  connect(PRefPu, sumErr.u1);
  connect(droopGain.y, sumErr.u2);
  connect(deadband.y, sumErr.u3);
  connect(sumErr.y, pBranch.u);
  connect(sumErr.y, iBranch.u);
  connect(sumErr.y, dBranch.u);
  connect(pBranch.y, pidSum.u1);
  connect(iBranch.y, pidSum.u2);
  connect(dBranch.y, pidSum.u3);
  connect(pidSum.y, regFilter.u);
  connect(regFilter.y, velLim.u);
  connect(velLim.y, gateInteg.u);
  connect(gateInteg.y, pgvCurve.u);
  connect(pgvCurve.y[1], turbineLL.u);
  connect(turbineLL.y, pmLim.u);
  connect(pmLim.y, PmPu);

  annotation(preferredView = "text");
end GovHydroPID;
