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

model GovHydroDD "Double-derivative hydro governor-turbine model (CGMES GovHydroDD)"
  /*
    Double-derivative hydro governor-turbine: the governor runs three parallel
    control branches at all times (a single derivative, a double derivative
    and an integrator), a unified droop selectable between gate position and
    electrical power, a washout on the governor output, and an elastic water
    column turbine. Corresponds to the CGMES IEC 61970-302 GovHydroDD model
    (PSS/E WSHYDD). Block-based: the equation section contains only connect
    statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  // Governor gains
  parameter Types.PerUnit K1 "Single-derivative gain, typical: 3.6";
  parameter Types.PerUnit K2 "Double-derivative gain, typical: 0.2";
  parameter Types.PerUnit Kg "Gate servo proportional gain, typical: 3";
  parameter Types.PerUnit Ki "Integral gain, typical: 1";
  parameter Types.PerUnit R "Steady-state droop in pu, typical: 0.05";

  // Governor time constants
  parameter Types.Time tD "Derivative filter time constant in s, typical: 0.02";
  parameter Types.Time tF "Washout time constant in s, typical: 0.1";
  parameter Types.Time tP "Gate servo lag time constant in s, typical: 0.35";
  parameter Types.Time tT "Electrical power feedback filter time constant in s, typical: 0.02";

  // Droop source flag
  parameter Boolean InputSignal = true "true: droop from electrical power, false: droop from gate position";

  // Governor limits
  parameter Types.PerUnit GMax "Maximum gate opening in pu, typical: 1";
  parameter Types.PerUnit GMin "Minimum gate opening in pu, typical: 0";
  parameter Types.PerUnit PMax "Maximum mechanical power in pu, typical: 1";
  parameter Types.PerUnit PMin "Minimum mechanical power in pu, typical: 0";
  parameter Types.PerUnit VelOp "Maximum gate opening velocity in pu/s, typical: 0.09";
  parameter Types.PerUnit VelCl "Maximum gate closing velocity in pu/s, typical: -0.14";
  parameter Types.AngularVelocityPu DeltaOmegaDbPu "Intentional speed deadband half-width in pu, typical: 0";

  // Turbine parameters
  parameter Types.PerUnit ATurb "Turbine transfer function numerator coefficient, typical: -1";
  parameter Types.PerUnit BTurb "Turbine transfer function denominator coefficient, typical: 0.5";
  parameter Types.Time tTurb "Turbine time constant in s, typical: 0.8";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  // Derived initial values
  final parameter Types.PerUnit GV0 = Pm0Pu "Initial gate opening in pu";
  final parameter Types.PerUnit PRef0Pu = R * Pm0Pu "Initial power reference in pu";

  // Inputs / outputs
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu) "Power reference in pu (base PNomTurb)";
  Blocks.Interfaces.RealInput PePu(start = Pm0Pu) "Electrical active power in pu (base PNomTurb)";
  Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNomTurb)";

  // Governor blocks
  Blocks.Math.Add addErrSpeed(k2 = -1) "Speed error omegaRef - omega";
  Blocks.Nonlinear.DeadZone deadband(uMax = DeltaOmegaDbPu, uMin = -DeltaOmegaDbPu) "Intentional speed deadband";
  Blocks.Continuous.FirstOrder filtD(T = tD, y_start = 0) "Speed error input filter 1 / (1 + tD.s)";
  Blocks.Math.Add addGovRef "Governor reference PRef + speed error";
  Blocks.Continuous.FirstOrder filtPe(T = tT, y_start = Pm0Pu) "Electrical power feedback filter";
  Blocks.Sources.RealExpression droopSelect(y = if InputSignal then filtPe.y else gateInteg.y) "Droop source selector";
  Blocks.Math.Gain gainR(k = R) "Droop feedback gain";
  Blocks.Math.Add addGovErr(k2 = -1) "Governor error govRef - droop";
  Blocks.Continuous.Derivative branchK1D(k = K1, T = tD, x_start = 0) "Single-derivative branch";
  Blocks.Continuous.Derivative branchK2D1(k = 1, T = tD, x_start = 0) "Double-derivative stage 1";
  Blocks.Continuous.Derivative branchK2D2(k = K2, T = tD, x_start = 0) "Double-derivative stage 2";
  Blocks.Continuous.Integrator branchKi(k = Ki, y_start = 0) "Integral branch";
  Blocks.Math.Add3 ctrlSum "Controller output = single + double derivative + integral";
  Blocks.Continuous.TransferFunction washout(a = {tF, 1}, b = {tF, 0}, initType = Modelica.Blocks.Types.Init.SteadyState) "Governor output washout";
  Blocks.Math.Gain gainKg(k = Kg) "Gate servo gain";
  Blocks.Continuous.FirstOrder pilotLag(T = tP, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = 0) "Gate servo lag 1 / (1 + tP.s)";
  Blocks.Nonlinear.Limiter limGateRate(uMax = VelOp, uMin = VelCl, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Gate velocity limiter";
  Blocks.Continuous.LimIntegrator gateInteg(outMax = GMax, outMin = GMin, y_start = GV0) "Gate position integrator";

  // Turbine blocks
  Blocks.Continuous.TransferFunction turbineLL(a = {BTurb * tTurb, 1}, b = {ATurb * tTurb, 1}, initType = Modelica.Blocks.Types.Init.SteadyState) "Elastic water column turbine lead-lag";
  Blocks.Nonlinear.Limiter limPm(uMax = PMax, uMin = PMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Mechanical power output limiter";

equation
  connect(omegaRefPu, addErrSpeed.u1);
  connect(omegaPu, addErrSpeed.u2);
  connect(addErrSpeed.y, deadband.u);
  connect(deadband.y, filtD.u);
  connect(PRefPu, addGovRef.u1);
  connect(filtD.y, addGovRef.u2);
  connect(PePu, filtPe.u);
  connect(droopSelect.y, gainR.u);
  connect(addGovRef.y, addGovErr.u1);
  connect(gainR.y, addGovErr.u2);
  connect(addGovErr.y, branchK1D.u);
  connect(addGovErr.y, branchK2D1.u);
  connect(branchK2D1.y, branchK2D2.u);
  connect(addGovErr.y, branchKi.u);
  connect(branchK1D.y, ctrlSum.u1);
  connect(branchK2D2.y, ctrlSum.u2);
  connect(branchKi.y, ctrlSum.u3);
  connect(ctrlSum.y, washout.u);
  connect(washout.y, gainKg.u);
  connect(gainKg.y, pilotLag.u);
  connect(pilotLag.y, limGateRate.u);
  connect(limGateRate.y, gateInteg.u);
  connect(gateInteg.y, turbineLL.u);
  connect(turbineLL.y, limPm.u);
  connect(limPm.y, PmPu);

  annotation(preferredView = "text");
end GovHydroDD;
