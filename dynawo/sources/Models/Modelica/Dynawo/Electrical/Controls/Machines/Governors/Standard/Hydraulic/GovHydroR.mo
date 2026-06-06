within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovHydroR "Fourth-order lead-lag hydro governor HYDROGOVR (CGMES GovHydroR)"
  /*
    PSS/E HYDROGOVR hydro governor. Speed error with intentional deadband
    (Db1, Db2, Eps) feeds two lead-lag stages: (1+sTf)/(1+sTr) for the
    transient droop and (1+sTp)/(1+sTd) for further gain reduction, then
    1/(sTg) gate servo with rate limit Velm and position limits
    (Pmin, Pmax). A 5-point gate-power table (GV1..5 / PGV1..5) shapes the
    gate-to-power conversion. The turbine block (1 + sAturb.Tturb) /
    (1 + sBturb.Tturb) acts on the table output. Tt is the secondary
    turbine time constant (informational here; the simplified topology
    folds it into Tturb).
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit ATurb "Turbine lead time-constant coefficient";
  parameter Types.PerUnit BTurb "Turbine lag time-constant coefficient";
  parameter Types.AngularVelocityPu Db1 "Speed deadband half-width (low)";
  parameter Types.AngularVelocityPu Db2 "Speed deadband half-width (high)";
  parameter Types.PerUnit Eps "Speed deadband transition width";
  parameter Types.PerUnit PMax;
  parameter Types.PerUnit PMin;
  parameter Types.PerUnit R "Permanent speed droop";
  parameter Types.Time tD "First lead-lag lag time constant in s";
  parameter Types.Time tF "First lead-lag lead time constant in s";
  parameter Types.Time tG "Gate servo time constant in s";
  parameter Types.Time tP "Second lead-lag lead time constant in s";
  parameter Types.Time tR "Reset (transient droop washout) time constant in s";
  parameter Types.Time tT "Secondary turbine time constant in s (informational)";
  parameter Types.Time tTurb "Turbine block denominator time constant in s";
  parameter Types.PerUnit Velm "Gate velocity limit in pu/s";
  parameter Types.PerUnit GV1; parameter Types.PerUnit GV2; parameter Types.PerUnit GV3;
  parameter Types.PerUnit GV4; parameter Types.PerUnit GV5;
  parameter Types.PerUnit PGV1; parameter Types.PerUnit PGV2; parameter Types.PerUnit PGV3;
  parameter Types.PerUnit PGV4; parameter Types.PerUnit PGV5;

  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  final parameter Types.PerUnit GV0 = Pm0Pu "Initial gate position";
  final parameter Types.PerUnit PRef0Pu = R * GV0;
  final parameter Real PgvTable[5, 2] = [GV1, PGV1; GV2, PGV2; GV3, PGV3; GV4, PGV4; GV5, PGV5];

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1);
  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = max(Db1, Db2), uMin = -max(Db1, Db2));
  Modelica.Blocks.Math.Gain droopGain(k = R);
  Modelica.Blocks.Math.Add3 sumErr(k1 = 1, k2 = -1, k3 = -1);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction leadLag1(a = {tR, 1}, b = {tF, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction leadLag2(a = {tD, 1}, b = {tP, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);
  Modelica.Blocks.Math.Gain gain1Tg(k = 1 / tG);
  Modelica.Blocks.Nonlinear.Limiter velLim(uMax = Velm, uMin = -Velm, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator gateInteg(outMax = PMax, outMin = PMin, y_start = GV0, k = 1);
  Modelica.Blocks.Tables.CombiTable1Ds pgvCurve(table = PgvTable, smoothness = Modelica.Blocks.Types.Smoothness.LinearSegments);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction turbineLL(a = {BTurb * tTurb, 1}, b = {ATurb * tTurb, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Nonlinear.Limiter pmLim(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(addErrSpeed.y, deadband.u);
  connect(gateInteg.y, droopGain.u);
  connect(PRefPu, sumErr.u1);
  connect(droopGain.y, sumErr.u2);
  connect(deadband.y, sumErr.u3);
  connect(sumErr.y, leadLag1.u);
  connect(leadLag1.y, leadLag2.u);
  connect(leadLag2.y, gain1Tg.u);
  connect(gain1Tg.y, velLim.u);
  connect(velLim.y, gateInteg.u);
  connect(gateInteg.y, pgvCurve.u);
  connect(pgvCurve.y[1], turbineLL.u);
  connect(turbineLL.y, pmLim.u);
  connect(pmLim.y, PmPu);

  annotation(preferredView = "text");
end GovHydroR;
