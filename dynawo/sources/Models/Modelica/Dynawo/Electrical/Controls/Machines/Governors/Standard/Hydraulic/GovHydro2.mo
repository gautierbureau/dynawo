within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2024, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model GovHydro2 "Modified IEEE hydro turbine-governor model (CGMES GovHydro2 / IEEEG3)"
  /*
    Modified IEEE hydro turbine-governor: the GovHydro1 mechanical-hydraulic
    dashpot governor extended with an intentional speed deadband, asymmetric
    gate rate limits and an elastic water column turbine modelled as a
    lead-lag transfer function. Corresponds to the CGMES IEC 61970-302
    GovHydro2 model (PSS/E IEEEG3). Block-based: the equation section
    contains only connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  // Governor parameters
  parameter Types.PerUnit RPerm "Permanent speed droop in pu, typical: 0.05";
  parameter Types.PerUnit RTemp "Temporary (transient) speed droop in pu, typical: 0.5";
  parameter Types.Time tP "Pilot valve servo time constant in s, typical: 0.03";
  parameter Types.Time tG "Gate servo time constant in s, typical: 0.5";
  parameter Types.Time tR "Dashpot (washout) time constant in s, typical: 12";
  parameter Types.PerUnit PMax "Maximum gate opening in pu, typical: 1";
  parameter Types.PerUnit PMin "Minimum gate opening in pu, typical: 0";
  parameter Types.PerUnit UO "Maximum gate opening velocity in pu/s, typical: 0.1";
  parameter Types.PerUnit UC "Maximum gate closing velocity in pu/s, typical: -0.1";
  parameter Types.AngularVelocityPu DeltaOmegaDbPu "Intentional speed deadband half-width in pu, typical: 0";

  // Turbine parameters
  parameter Types.PerUnit ATurb "Turbine transfer function numerator coefficient, typical: -1";
  parameter Types.PerUnit BTurb "Turbine transfer function denominator coefficient, typical: 0.5";
  parameter Types.PerUnit KTurb "Turbine gain in pu, typical: 1";
  parameter Types.Time tW "Water starting time (water inertia) in s, typical: 2";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  // Derived initial values
  final parameter Types.PerUnit GV0 = Pm0Pu / KTurb "Initial gate opening in pu";
  final parameter Types.PerUnit PRef0Pu = RPerm * GV0 "Initial power reference in pu";

  // Inputs / outputs
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu) "Power reference in pu (base PNomTurb)";
  Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNomTurb)";

  // Governor blocks
  Blocks.Math.Add addErrSpeed(k2 = -1) "Speed error omegaRef - omega";
  Blocks.Nonlinear.DeadZone deadband(uMax = DeltaOmegaDbPu, uMin = -DeltaOmegaDbPu) "Intentional speed deadband";
  Blocks.Math.Add addGovRef "Governor reference PRef + speed error";
  Blocks.Math.Gain gainRPerm(k = RPerm) "Permanent droop feedback";
  Blocks.Continuous.TransferFunction dashpot(a = {tR, 1}, b = {RTemp * tR, 0}, initType = Modelica.Blocks.Types.Init.SteadyState) "Dashpot transient droop feedback";
  Blocks.Math.Add3 addGovErr(k2 = -1, k3 = -1) "Governor error govRef - permanent droop - dashpot";
  Blocks.Continuous.FirstOrder pilotLag(T = tP, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = 0) "Pilot valve filter 1 / (1 + tP.s)";
  Blocks.Math.Gain gain1Tg(k = 1 / tG) "Gate servo velocity gain 1 / tG";
  Blocks.Nonlinear.Limiter limGateRate(uMax = UO, uMin = UC, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Asymmetric gate velocity limiter";
  Blocks.Continuous.LimIntegrator gateInteg(outMax = PMax, outMin = PMin, y_start = GV0) "Gate position integrator";

  // Turbine blocks
  Blocks.Continuous.TransferFunction turbineLL(a = {BTurb * tW, 1}, b = {KTurb * ATurb * tW, KTurb}, initType = Modelica.Blocks.Types.Init.SteadyState) "Elastic water column turbine lead-lag";
  Blocks.Nonlinear.Limiter limPm(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Mechanical power output limiter";

equation
  connect(omegaRefPu, addErrSpeed.u1);
  connect(omegaPu, addErrSpeed.u2);
  connect(addErrSpeed.y, deadband.u);
  connect(PRefPu, addGovRef.u1);
  connect(deadband.y, addGovRef.u2);
  connect(gateInteg.y, gainRPerm.u);
  connect(gateInteg.y, dashpot.u);
  connect(addGovRef.y, addGovErr.u1);
  connect(gainRPerm.y, addGovErr.u2);
  connect(dashpot.y, addGovErr.u3);
  connect(addGovErr.y, pilotLag.u);
  connect(pilotLag.y, gain1Tg.u);
  connect(gain1Tg.y, limGateRate.u);
  connect(limGateRate.y, gateInteg.u);
  connect(gateInteg.y, turbineLL.u);
  connect(turbineLL.y, limPm.u);
  connect(limPm.y, PmPu);

  annotation(preferredView = "text");
end GovHydro2;
