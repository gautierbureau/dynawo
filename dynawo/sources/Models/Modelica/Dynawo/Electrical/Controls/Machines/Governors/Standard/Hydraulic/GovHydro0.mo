within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovHydro0 "Simplified linearised hydro governor (CGMES GovHydro0)"
  /*
    Simplest linearised hydro governor (PES TR1 baseline model). The
    error PRef - R*gate - (omega - omegaRef) drives a 1/(sTg) gate
    servo (LimIntegrator) bounded Pmin..Pmax; the gate position then
    feeds a linear 1/(1+sTw) water column to produce the mechanical
    power Pm. This minimal model omits the rigid-water-column non-
    minimum-phase term (1 - Tw.s) so it captures only the dominant
    first-order dynamics and is intended for stability studies where
    a simplified linear response is sufficient.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit R "Permanent speed droop in pu";
  parameter Types.Time tG "Gate servo time constant in s";
  parameter Types.Time tW "Water inertia time constant in s";
  parameter Types.PerUnit PMax "Maximum gate position / mechanical power";
  parameter Types.PerUnit PMin "Minimum gate position / mechanical power";

  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  final parameter Types.PerUnit GV0 = Pm0Pu "Initial gate position";
  final parameter Types.PerUnit PRef0Pu = R * GV0 "Initial power reference";

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1) "omegaPu - omegaRefPu";
  Modelica.Blocks.Math.Gain droopGain(k = R);
  Modelica.Blocks.Math.Add3 sumErr(k1 = 1, k2 = -1, k3 = -1) "PRef - R*gate - speed_err";
  Modelica.Blocks.Continuous.LimIntegrator gateInteg(outMax = PMax, outMin = PMin, y_start = GV0, k = 1 / max(tG, 1e-6)) "Gate servo as 1/(sTg) integrator with position limits";
  Modelica.Blocks.Continuous.FirstOrder waterCol(T = max(tW, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState) "Linearised water column 1/(1 + sTw)";

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(gateInteg.y, droopGain.u);
  connect(PRefPu, sumErr.u1);
  connect(droopGain.y, sumErr.u2);
  connect(addErrSpeed.y, sumErr.u3);
  connect(sumErr.y, gateInteg.u);
  connect(gateInteg.y, waterCol.u);
  connect(waterCol.y, PmPu);

  annotation(preferredView = "text");
end GovHydro0;
