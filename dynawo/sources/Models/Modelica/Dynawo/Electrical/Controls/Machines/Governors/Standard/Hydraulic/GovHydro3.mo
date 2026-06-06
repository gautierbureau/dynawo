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

model GovHydro3 "Modified IEEE hydro governor-turbine model (CGMES GovHydro3)"
  /*
    Modified IEEE hydro governor-turbine: a selectable PID or double-derivative
    governor with a permanent droop on both gate position and electrical
    power, a washout on the governor output, and a rigid water column
    turbine. Corresponds to the CGMES IEC 61970-302 GovHydro3 model
    (PSS/E GOVHYDRO3). Block-based: the equation section contains only
    connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  // Governor mode
  parameter Boolean GovernorControl = true "true: PID control, false: double-derivative control";

  // Governor gains
  parameter Types.PerUnit K1 "Derivative gain (PID mode), typical: 0.01";
  parameter Types.PerUnit K2 "Double-derivative gain (DD mode), typical: 2.5";
  parameter Types.PerUnit Kg "Gate servo proportional gain, typical: 2";
  parameter Types.PerUnit Ki "Integral gain (PID mode), typical: 0.5";

  // Governor time constants
  parameter Types.Time tD "Speed error / derivative filter time constant in s, typical: 0.05";
  parameter Types.Time tF "Washout time constant in s, typical: 0.1";
  parameter Types.Time tP "Gate servo lag time constant in s, typical: 0.05";
  parameter Types.Time tT "Electrical power feedback filter time constant in s, typical: 0.2";

  // Governor limits
  parameter Types.PerUnit PMax "Maximum gate opening in pu, typical: 1";
  parameter Types.PerUnit PMin "Minimum gate opening in pu, typical: 0";
  parameter Types.PerUnit VelOp "Maximum gate opening velocity in pu/s, typical: 0.2";
  parameter Types.PerUnit VelCl "Maximum gate closing velocity in pu/s, typical: -0.2";

  // Droop coefficients
  parameter Types.PerUnit RElec "Steady-state droop on electrical power in pu, typical: 0.05";
  parameter Types.PerUnit RGate "Steady-state droop on gate position in pu, typical: 0";
  parameter Types.AngularVelocityPu DeltaOmegaDbPu "Intentional speed deadband half-width in pu, typical: 0";

  // Turbine parameters
  parameter Types.PerUnit ATurb "Turbine gain in pu, typical: 1.2";
  parameter Types.PerUnit H0 "Turbine nominal head of water in pu, typical: 1";
  parameter Types.PerUnit QNl "No-load turbine flow in pu, typical: 0.08";
  parameter Types.PerUnit DTurb "Turbine speed damping coefficient in pu, typical: 0.2";
  parameter Types.Time tW "Water starting time (water inertia) in s, typical: 1";
  parameter Types.Time tDeriv "Filter time constant of the penstock derivative block in s, typical: 0.01";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  // Derived initial value
  final parameter Types.PerUnit GV0 = Pm0Pu / (ATurb * H0) + QNl "Initial gate opening in pu";

  // Inputs / outputs
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput PRefPu(start = 0) "Power reference in pu (base PNomTurb)";
  Blocks.Interfaces.RealInput PePu(start = Pm0Pu) "Electrical active power in pu (base PNomTurb)";
  Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNomTurb)";

  // Governor blocks
  Blocks.Math.Add addErrSpeed(k2 = -1) "Speed error omegaRef - omega";
  Blocks.Nonlinear.DeadZone deadband(uMax = DeltaOmegaDbPu, uMin = -DeltaOmegaDbPu) "Intentional speed deadband";
  Blocks.Continuous.FirstOrder filtD(T = tD, y_start = 0) "Speed error input filter 1 / (1 + tD.s)";
  Blocks.Continuous.Derivative pidD(k = K1, T = tD, x_start = 0) "PID derivative branch";
  Blocks.Continuous.Derivative ddD1(k = 1, T = tD, x_start = 0) "Double-derivative stage 1";
  Blocks.Continuous.Derivative ddD2(k = K2, T = tD, x_start = 0) "Double-derivative stage 2";
  Blocks.Continuous.Integrator pidI(k = Ki, y_start = 0) "PID integral branch";
  Blocks.Math.Add pidSum "PID controller output = derivative + integral";
  Blocks.Sources.RealExpression ctrlSelect(y = if GovernorControl then pidSum.y else ddD2.y) "PID or double-derivative controller selector";
  Blocks.Continuous.FirstOrder filtPe(T = tT, y_start = Pm0Pu) "Electrical power feedback filter";
  Blocks.Math.Gain gainRElec(k = RElec) "Electrical power droop feedback";
  Blocks.Math.Gain gainRGate(k = RGate) "Gate position droop feedback";
  Blocks.Math.Add addPmCtrl "Reference plus controller output";
  Blocks.Math.Add3 addGovErr(k2 = -1, k3 = -1) "Governor error";
  Blocks.Continuous.TransferFunction washout(a = {tF, 1}, b = {tF, 0}, initType = Modelica.Blocks.Types.Init.SteadyState) "Governor output washout";
  Blocks.Math.Gain gainKg(k = Kg) "Gate servo gain";
  Blocks.Continuous.FirstOrder pilotLag(T = tP, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = 0) "Gate servo lag 1 / (1 + tP.s)";
  Blocks.Nonlinear.Limiter limGateRate(uMax = VelOp, uMin = VelCl, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Gate velocity limiter";
  Blocks.Continuous.LimIntegrator gateInteg(outMax = PMax, outMin = PMin, y_start = GV0) "Gate position integrator";

  // Turbine blocks
  Blocks.Continuous.Derivative derivPenstock(k = tW, T = tDeriv, x_start = GV0, initType = Modelica.Blocks.Types.Init.InitialState) "Penstock water inertia derivative tW.s";
  Blocks.Math.Add addPenstock(k2 = -1) "Penstock flow q = gate - tW.gate'";
  Blocks.Sources.Constant srcQNl(k = QNl) "No-load flow constant";
  Blocks.Math.Add subQNl(k2 = -1) "Net turbine flow q - QNl";
  Blocks.Math.Gain gainAtH0(k = ATurb * H0) "Turbine power gain ATurb.H0";
  Blocks.Math.Gain gainDTurb(k = DTurb) "Turbine speed damping gain";
  Blocks.Math.Add addPm "Mechanical power = turbine power + speed damping";

equation
  connect(omegaRefPu, addErrSpeed.u1);
  connect(omegaPu, addErrSpeed.u2);
  connect(addErrSpeed.y, deadband.u);
  connect(deadband.y, filtD.u);
  connect(filtD.y, pidD.u);
  connect(filtD.y, ddD1.u);
  connect(ddD1.y, ddD2.u);
  connect(filtD.y, pidI.u);
  connect(pidD.y, pidSum.u1);
  connect(pidI.y, pidSum.u2);
  connect(PePu, filtPe.u);
  connect(filtPe.y, gainRElec.u);
  connect(gateInteg.y, gainRGate.u);
  connect(PRefPu, addPmCtrl.u1);
  connect(ctrlSelect.y, addPmCtrl.u2);
  connect(addPmCtrl.y, addGovErr.u1);
  connect(gainRElec.y, addGovErr.u2);
  connect(gainRGate.y, addGovErr.u3);
  connect(addGovErr.y, washout.u);
  connect(washout.y, gainKg.u);
  connect(gainKg.y, pilotLag.u);
  connect(pilotLag.y, limGateRate.u);
  connect(limGateRate.y, gateInteg.u);

  connect(gateInteg.y, derivPenstock.u);
  connect(gateInteg.y, addPenstock.u1);
  connect(derivPenstock.y, addPenstock.u2);
  connect(addPenstock.y, subQNl.u1);
  connect(srcQNl.y, subQNl.u2);
  connect(subQNl.y, gainAtH0.u);
  connect(addErrSpeed.y, gainDTurb.u);
  connect(gainAtH0.y, addPm.u1);
  connect(gainDTurb.y, addPm.u2);
  connect(addPm.y, PmPu);

  annotation(preferredView = "text");
end GovHydro3;
