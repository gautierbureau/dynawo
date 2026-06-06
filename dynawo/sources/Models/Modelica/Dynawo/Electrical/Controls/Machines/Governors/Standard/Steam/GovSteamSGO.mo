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

model GovSteamSGO "IEEE legacy steam turbine-governor model (CGMES GovSteamSGO)"
  /*
    IEEE 1973 legacy steam turbine-governor: a proportional governor with a
    lead-lag and a servomotor lag feeding a three-stage cascade flow-split
    turbine (high-pressure tap, reheater, low-pressure section). Corresponds
    to the CGMES IEC 61970-302 GovSteamSGO model (PSS/E IEESGO). Block-based:
    the equation section contains only connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  // Governor parameters
  parameter Types.PerUnit K1 "Governor gain (reciprocal of droop), typical: 20";
  parameter Types.Time t1 "Governor lag time constant in s, typical: 0.15";
  parameter Types.Time t2 "Governor lead time constant in s, typical: 0";
  parameter Types.Time t3 "Governor servomotor lag time constant in s, typical: 0.13";
  parameter Types.PerUnit PMax "Maximum valve position in pu, typical: 1";
  parameter Types.PerUnit PMin "Minimum valve position in pu, typical: 0";

  // Turbine parameters
  parameter Types.Time t4 "Steam chest time constant in s, typical: 0.22";
  parameter Types.PerUnit K2 "High-pressure turbine power fraction, typical: 0.2";
  parameter Types.Time t5 "Reheater time constant in s, typical: 5";
  parameter Types.PerUnit K3 "Intermediate-pressure turbine power fraction, typical: 0.3";
  parameter Types.Time t6 "Low-pressure section time constant in s, typical: 0.5";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  // Derived initial values
  final parameter Types.PerUnit Freh0 = (1 - K2) * Pm0Pu "Initial reheater flow in pu";
  final parameter Types.PerUnit Flp0 = (1 - K3) * Freh0 "Initial low-pressure section flow in pu";

  // Inputs / outputs
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput PRefPu(start = Pm0Pu) "Power reference in pu (base PNomTurb)";
  Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNomTurb)";

  // Governor blocks
  Blocks.Math.Add addErrSpeed(k2 = -1) "Speed deviation omega - omegaRef";
  Blocks.Math.Gain gainK1(k = K1) "Governor gain";
  Blocks.Math.Add addErr(k2 = -1) "Governor error PRef - K1.deltaOmega";
  Blocks.Continuous.TransferFunction leadLag(a = {t1, 1}, b = {t2, 1}, initType = Modelica.Blocks.Types.Init.SteadyState) "Governor lead-lag";
  Blocks.Continuous.FirstOrder govLag(T = t3, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = Pm0Pu) "Governor servomotor lag";
  Blocks.Nonlinear.Limiter limValve(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Valve position limiter";

  // Turbine blocks
  Blocks.Continuous.FirstOrder steamBowl(T = t4, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = Pm0Pu) "Steam chest lag";
  Blocks.Math.Gain gainK2(k = K2) "High-pressure power tap";
  Blocks.Math.Gain gainToReh(k = 1 - K2) "Residual flow to reheater";
  Blocks.Continuous.FirstOrder reheater(T = t5, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = Freh0) "Reheater lag";
  Blocks.Math.Gain gainK3(k = K3) "Intermediate-pressure power tap";
  Blocks.Math.Gain gainToLp(k = 1 - K3) "Residual flow to low-pressure section";
  Blocks.Continuous.FirstOrder lpSection(T = t6, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = Flp0) "Low-pressure section lag";
  Blocks.Math.Add3 add3Pm "Mechanical power = high + intermediate + low pressure contributions";

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(addErrSpeed.y, gainK1.u);
  connect(PRefPu, addErr.u1);
  connect(gainK1.y, addErr.u2);
  connect(addErr.y, leadLag.u);
  connect(leadLag.y, govLag.u);
  connect(govLag.y, limValve.u);
  connect(limValve.y, steamBowl.u);
  connect(steamBowl.y, gainK2.u);
  connect(steamBowl.y, gainToReh.u);
  connect(gainToReh.y, reheater.u);
  connect(reheater.y, gainK3.u);
  connect(reheater.y, gainToLp.u);
  connect(gainToLp.y, lpSection.u);
  connect(gainK2.y, add3Pm.u1);
  connect(gainK3.y, add3Pm.u2);
  connect(lpSection.y, add3Pm.u3);
  connect(add3Pm.y, PmPu);

  annotation(preferredView = "text");
end GovSteamSGO;
