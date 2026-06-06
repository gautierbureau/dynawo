within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

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

model GovSteam2 "Simplified steam turbine-governor model (CGMES GovSteam2)"
  /*
    Simplified steam turbine-governor: a proportional governor (gain equal to
    the reciprocal of the droop) with a frequency deadband, an error limiter
    and a single lead-lag aggregating the governor and turbine dynamics.
    Corresponds to the CGMES IEC 61970-302 GovSteam2 model (PSS/E GOV21GEQ).
    Block-based: the equation section contains only connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit K "Governor gain (reciprocal of droop), typical: 20";
  parameter Types.AngularVelocityPu DeltaOmegaDbPu "Speed deadband half-width in pu, typical: 0";
  parameter Types.PerUnit PMax "Maximum steam flow in pu, typical: 1";
  parameter Types.PerUnit PMin "Minimum steam flow in pu, typical: 0";
  parameter Types.Time t1 "Governor lag time constant in s, typical: 0.45";
  parameter Types.Time t2 "Governor lead time constant in s, typical: 0";
  parameter Types.PerUnit MnEfPu "Maximum negative error value in pu, typical: -1";
  parameter Types.PerUnit MxEfPu "Maximum positive error value in pu, typical: 1";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  // Inputs / outputs
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput PRefPu(start = Pm0Pu) "Power reference in pu (base PNomTurb)";
  Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNomTurb)";

  // Blocks
  Blocks.Math.Add addErrSpeed(k2 = -1) "Speed deviation omega - omegaRef";
  Blocks.Nonlinear.DeadZone deadband(uMax = DeltaOmegaDbPu, uMin = -DeltaOmegaDbPu) "Speed deadband";
  Blocks.Math.Gain gainK(k = K) "Governor gain";
  Blocks.Math.Add addErr(k2 = -1) "Governor error PRef - K.deltaOmega";
  Blocks.Nonlinear.Limiter limEf(uMax = MxEfPu, uMin = MnEfPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Error limiter";
  Continuous.TransferFunction leadLag(a = {t1, 1}, b = {t2, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu) "Governor and turbine lead-lag";
  Blocks.Nonlinear.Limiter limPm(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Mechanical power output limiter";

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(addErrSpeed.y, deadband.u);
  connect(deadband.y, gainK.u);
  connect(PRefPu, addErr.u1);
  connect(gainK.y, addErr.u2);
  connect(addErr.y, limEf.u);
  connect(limEf.y, leadLag.u);
  connect(leadLag.y, limPm.u);
  connect(limPm.y, PmPu);

  annotation(preferredView = "text");
end GovSteam2;
