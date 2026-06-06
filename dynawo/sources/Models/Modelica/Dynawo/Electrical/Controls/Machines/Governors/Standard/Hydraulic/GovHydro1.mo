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

model GovHydro1 "IEEE simplified hydro turbine-governor model (CGMES GovHydro1)"
  /*
    Simplified hydro turbine-governor: a mechanical-hydraulic governor with
    permanent droop and a dashpot (transient droop) feeding a rigid water
    column turbine model. Corresponds to the CGMES IEC 61970-302 GovHydro1
    model (PSS/E GOVHYDRO1). Block-based: the equation section contains only
    connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  // Governor parameters
  parameter Types.PerUnit RPerm "Permanent speed droop in pu, typical: 0.05";
  parameter Types.PerUnit RTemp "Temporary (transient) speed droop in pu, typical: 0.3";
  parameter Types.Time tFilter "Pilot valve filter time constant in s, typical: 0.05";
  parameter Types.Time tG "Gate servo time constant in s, typical: 0.5";
  parameter Types.Time tR "Dashpot (washout) time constant in s, typical: 5";
  parameter Types.PerUnit GMax "Maximum gate opening in pu, typical: 1";
  parameter Types.PerUnit GMin "Minimum gate opening in pu, typical: 0";
  parameter Types.PerUnit VelMax "Maximum gate velocity in pu/s, typical: 0.2";

  // Turbine parameters
  parameter Types.PerUnit ATurb "Turbine gain in pu, typical: 1.2";
  parameter Types.PerUnit HDam "Turbine nominal head of water in pu, typical: 1";
  parameter Types.PerUnit QNl "No-load turbine flow in pu, typical: 0.08";
  parameter Types.PerUnit DTurb "Turbine speed damping coefficient in pu, typical: 0.5";
  parameter Types.Time tW "Water starting time (water inertia) in s, typical: 1";
  parameter Types.Time tDeriv "Filter time constant of the penstock derivative block in s, typical: 0.02";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  // Derived initial values
  final parameter Types.PerUnit GV0 = Pm0Pu / (ATurb * HDam) + QNl "Initial gate opening in pu";
  final parameter Types.PerUnit PRef0Pu = RPerm * GV0 "Initial power reference in pu";

  // Inputs / outputs
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu) "Power reference in pu (base PNomTurb)";
  Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNomTurb)";

  // Governor blocks
  Blocks.Math.Add addErrSpeed(k2 = -1) "Speed error omegaRef - omega";
  Blocks.Math.Add addGovRef "Governor reference PRef + speed error";
  Blocks.Math.Gain gainRPerm(k = RPerm) "Permanent droop feedback";
  Blocks.Continuous.TransferFunction dashpot(a = {tR, 1}, b = {RTemp * tR, 0}, initType = Modelica.Blocks.Types.Init.SteadyState) "Dashpot transient droop feedback";
  Blocks.Math.Add3 addGovErr(k2 = -1, k3 = -1) "Governor error govRef - permanent droop - dashpot";
  Blocks.Continuous.FirstOrder pilotLag(T = tFilter, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = 0) "Pilot valve filter 1 / (1 + tFilter.s)";
  Blocks.Math.Gain gain1Tg(k = 1 / tG) "Gate servo velocity gain 1 / tG";
  Blocks.Nonlinear.Limiter limGateVel(uMax = VelMax, uMin = -VelMax, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Gate velocity limiter";
  Blocks.Continuous.LimIntegrator gateInteg(outMax = GMax, outMin = GMin, y_start = GV0) "Gate position integrator";

  // Turbine blocks
  Blocks.Continuous.Derivative derivPenstock(k = tW, T = tDeriv, x_start = GV0, initType = Modelica.Blocks.Types.Init.InitialState) "Penstock water inertia derivative tW.s";
  Blocks.Math.Add addPenstock(k2 = -1) "Penstock flow q = gate - tW.gate'";
  Blocks.Sources.Constant srcQNl(k = QNl) "No-load flow constant";
  Blocks.Math.Add subQNl(k2 = -1) "Net turbine flow q - QNl";
  Blocks.Math.Gain gainAtHdam(k = ATurb * HDam) "Turbine power gain ATurb.HDam";
  Blocks.Math.Gain gainDTurb(k = DTurb) "Turbine speed damping gain";
  Blocks.Math.Add addPm "Mechanical power = turbine power + speed damping";

equation
  connect(omegaRefPu, addErrSpeed.u1);
  connect(omegaPu, addErrSpeed.u2);
  connect(PRefPu, addGovRef.u1);
  connect(addErrSpeed.y, addGovRef.u2);
  connect(gateInteg.y, gainRPerm.u);
  connect(gateInteg.y, dashpot.u);
  connect(addGovRef.y, addGovErr.u1);
  connect(gainRPerm.y, addGovErr.u2);
  connect(dashpot.y, addGovErr.u3);
  connect(addGovErr.y, pilotLag.u);
  connect(pilotLag.y, gain1Tg.u);
  connect(gain1Tg.y, limGateVel.u);
  connect(limGateVel.y, gateInteg.u);

  connect(gateInteg.y, derivPenstock.u);
  connect(gateInteg.y, addPenstock.u1);
  connect(derivPenstock.y, addPenstock.u2);
  connect(addPenstock.y, subQNl.u1);
  connect(srcQNl.y, subQNl.u2);
  connect(subQNl.y, gainAtHdam.u);
  connect(addErrSpeed.y, gainDTurb.u);
  connect(gainAtHdam.y, addPm.u1);
  connect(gainDTurb.y, addPm.u2);
  connect(addPm.y, PmPu);

  annotation(preferredView = "text");
end GovHydro1;
