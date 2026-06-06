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

model GovCT1 "General purpose turbine-governor model (CGMES GovCT1)"
  /*
    General purpose turbine-governor: a speed/power PID governor, a load
    limiter PI, an acceleration limiter and a rate- and position-limited
    valve actuator combined through a low-value select, feeding a lead-lag
    turbine. Corresponds to the CGMES IEC 61970-302 GovCT1 model
    (PSS/E GGOV1). The temperature/exhaust limiter is not modelled (a
    standard option, disabled here). The droop feedback source is selected
    by Rselect (0: electrical power, 1: isochronous, 2: valve stroke,
    3: governor output). Block-based: the equation section contains only
    connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  // Speed/power governor parameters
  parameter Types.PerUnit R "Permanent droop in pu, typical: 0.05";
  parameter Types.Time Tpelec "Electrical power transducer time constant in s, typical: 1";
  parameter Types.PerUnit MaxErr "Maximum speed error signal in pu, typical: 0.05";
  parameter Types.PerUnit MinErr "Minimum speed error signal in pu, typical: -0.05";
  parameter Types.PerUnit Kpgov "Governor PID proportional gain, typical: 10";
  parameter Types.PerUnit Kigov "Governor PID integral gain in 1/s, typical: 2";
  parameter Types.PerUnit Kdgov "Governor PID derivative gain, typical: 0";
  parameter Types.Time Tdgov "Governor PID derivative filter time constant in s, typical: 1";
  parameter Types.PerUnit VMax "Maximum valve position in pu, typical: 1";
  parameter Types.PerUnit VMin "Minimum valve position in pu, typical: 0.15";
  parameter Types.PerUnit DeltaOmegaDbPu "Speed deadband half-width in pu, typical: 0";

  // Valve actuator parameters
  parameter Types.PerUnit Ropen "Maximum valve opening rate in pu/s, typical: 0.1";
  parameter Types.PerUnit Rclose "Maximum valve closing rate in pu/s, typical: -0.1";
  parameter Types.Time Tact "Actuator time constant in s, typical: 0.5";

  // Turbine parameters
  parameter Types.PerUnit Kturb "Turbine gain, typical: 1.5";
  parameter Types.PerUnit Wfnl "No-load fuel flow in pu, typical: 0.2";
  parameter Types.Time Tb "Turbine lag time constant in s, typical: 0.5";
  parameter Types.Time Tc "Turbine lead time constant in s, typical: 0";
  parameter Types.PerUnit Dm "Speed sensitivity of mechanical power in pu, typical: 0";

  // Load limiter parameters
  parameter Types.Time Tfload "Load limiter transducer time constant in s, typical: 3";
  parameter Types.PerUnit Kpload "Load limiter PI proportional gain, typical: 2";
  parameter Types.PerUnit Kiload "Load limiter PI integral gain in 1/s, typical: 0.67";
  parameter Types.PerUnit Ldref "Load limiter reference in pu, typical: 1";
  parameter Types.PerUnit Rup "Maximum load-limit increase rate in pu/s, typical: 99";
  parameter Types.PerUnit Rdown "Maximum load-limit decrease rate in pu/s, typical: -99";

  // Acceleration limiter parameters
  parameter Types.PerUnit Aset "Acceleration limiter setpoint in pu/s, typical: 0.01";
  parameter Types.PerUnit Ka "Acceleration limiter gain, typical: 10";
  parameter Types.Time Ta "Acceleration limiter time constant in s, typical: 0.1";

  // Power controller reset parameter
  parameter Types.PerUnit Kimw "Power controller reset gain in 1/s, typical: 0";

  // Droop feedback selector (0: Pe, 1: isochronous, 2: valve stroke, 3: governor output)
  parameter Integer Rselect "Droop feedback signal selector, typical: 1";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";

  // Derived initial values
  final parameter Types.PerUnit Pturb0 = (Pm0Pu - Dm * SystemBase.omega0Pu) / Kturb "Initial turbine lead-lag input in pu";
  final parameter Types.PerUnit Fsr0 = Pturb0 + Wfnl "Initial fuel stroke / valve position in pu";

  // Inputs / outputs
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput PePu(start = Pm0Pu) "Electrical active power in pu (base PNomTurb)";
  Blocks.Interfaces.RealInput PRefPu(start = 0) "Power reference offset in pu (base PNomTurb)";
  Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power in pu (base PNomTurb)";

  // Speed/power governor blocks
  Blocks.Continuous.FirstOrder transPe(T = Tpelec, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = Pm0Pu) "Electrical power transducer";
  Blocks.Math.Add addSpeedErr(k1 = 1, k2 = -1) "Speed deviation omegaRef - omega";
  Blocks.Nonlinear.DeadZone deadband(uMax = DeltaOmegaDbPu, uMin = -DeltaOmegaDbPu) "Speed deadband";
  Blocks.Math.Add addMwErr(k2 = -1) "Power controller error PRef - Pe";
  Blocks.Continuous.Integrator integMw(k = Kimw, initType = Modelica.Blocks.Types.Init.InitialState, y_start = 0) "Power controller reset integrator";
  Blocks.Math.Add addRefReset "Effective power reference PRef + reset";
  Blocks.Math.Gain gainDroop(k = R) "Droop feedback scaling";
  Blocks.Math.Add3 addErr(k3 = -1) "Governor error PRef + deltaOmega - R.droop";
  Blocks.Nonlinear.Limiter limErr(uMax = MaxErr, uMin = MinErr, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Speed error limiter";
  Blocks.Math.Gain pidP(k = Kpgov) "Governor PID proportional branch";
  Blocks.Continuous.Integrator pidI(k = Kigov, initType = Modelica.Blocks.Types.Init.InitialState, y_start = Fsr0) "Governor PID integral branch";
  Blocks.Continuous.Derivative pidD(k = Kdgov, T = Tdgov, initType = Modelica.Blocks.Types.Init.SteadyState) "Governor PID derivative branch";
  Blocks.Math.Add3 pidSum "Governor PID sum";
  Blocks.Nonlinear.Limiter limGov(uMax = VMax, uMin = VMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Governor output limiter";

  // Load limiter blocks
  Blocks.Continuous.FirstOrder filtLoad(T = Tfload, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = Pm0Pu) "Load limiter transducer";
  Blocks.Sources.Constant srcLdref(k = Ldref) "Load limiter reference";
  Blocks.Math.Add addLdErr(k2 = -1) "Load limiter error Ldref - Pe";
  Blocks.Math.Gain ldP(k = Kpload) "Load limiter PI proportional branch";
  Blocks.Continuous.LimIntegrator ldI(k = Kiload, outMax = VMax, outMin = VMin, initType = Modelica.Blocks.Types.Init.InitialState, y_start = VMax) "Load limiter PI integral branch";
  Blocks.Math.Add ldSum "Load limiter PI sum";
  Blocks.Nonlinear.Limiter limLdRate(uMax = Rup, uMin = Rdown, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Load limit rate clamp";

  // Acceleration limiter blocks
  Blocks.Continuous.Derivative derivAcc(k = 1, T = Ta, initType = Modelica.Blocks.Types.Init.SteadyState) "Filtered acceleration";
  Blocks.Sources.Constant srcAset(k = Aset) "Acceleration limiter setpoint";
  Blocks.Math.Add addAccErr(k2 = -1) "Acceleration error Aset - acceleration";
  Blocks.Math.Gain gainAcc(k = Ka) "Acceleration limiter gain";
  Blocks.Math.Add addFsra "Acceleration limiter output (offset by valve position)";

  // Low-value select and valve actuator blocks
  Blocks.Math.Min lvsAB "Low-value select: min(governor, load limit)";
  Blocks.Math.Min lvsOut "Low-value select: min(previous, acceleration limit)";
  Blocks.Math.Add subActErr(k2 = -1) "Actuator error: fuel stroke request - valve position";
  Blocks.Math.Gain gainAct(k = 1 / Tact) "Actuator rate gain";
  Blocks.Nonlinear.Limiter limRate(uMax = Ropen, uMin = Rclose, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Valve rate limiter";
  Blocks.Continuous.LimIntegrator intValve(k = 1, outMax = VMax, outMin = VMin, initType = Modelica.Blocks.Types.Init.InitialState, y_start = Fsr0) "Valve position integrator";

  // Turbine blocks
  Blocks.Sources.Constant srcWfnl(k = Wfnl) "No-load fuel flow";
  Blocks.Math.Add subWfnl(k2 = -1) "Net fuel flow: valve position - Wfnl";
  Continuous.TransferFunction turbLl(a = {Tb, 1}, b = {Tc, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pturb0) "Turbine lead-lag";
  Blocks.Math.Gain turbGain(k = Kturb) "Turbine gain";
  Blocks.Math.Gain turbDm(k = Dm) "Speed sensitivity term";
  Blocks.Math.Add addPm "Mechanical power = turbine power + speed sensitivity";

equation
  // Droop feedback source selector (Rselect: 0 power, 1 isochronous, 2 valve stroke, 3 governor output)
  gainDroop.u = if Rselect == 0 then transPe.y elseif Rselect == 2 then intValve.y elseif Rselect == 3 then limGov.y else 0;

  // Power transducer and power controller reset
  connect(PePu, transPe.u);
  connect(PRefPu, addMwErr.u1);
  connect(transPe.y, addMwErr.u2);
  connect(addMwErr.y, integMw.u);
  connect(PRefPu, addRefReset.u1);
  connect(integMw.y, addRefReset.u2);

  // Speed/power governor
  connect(omegaRefPu, addSpeedErr.u1);
  connect(omegaPu, addSpeedErr.u2);
  connect(addSpeedErr.y, deadband.u);
  connect(addRefReset.y, addErr.u1);
  connect(deadband.y, addErr.u2);
  connect(gainDroop.y, addErr.u3);
  connect(addErr.y, limErr.u);
  connect(limErr.y, pidP.u);
  connect(limErr.y, pidI.u);
  connect(limErr.y, pidD.u);
  connect(pidP.y, pidSum.u1);
  connect(pidI.y, pidSum.u2);
  connect(pidD.y, pidSum.u3);
  connect(pidSum.y, limGov.u);

  // Load limiter
  connect(transPe.y, filtLoad.u);
  connect(srcLdref.y, addLdErr.u1);
  connect(filtLoad.y, addLdErr.u2);
  connect(addLdErr.y, ldP.u);
  connect(addLdErr.y, ldI.u);
  connect(ldP.y, ldSum.u1);
  connect(ldI.y, ldSum.u2);
  connect(ldSum.y, limLdRate.u);

  // Acceleration limiter
  connect(omegaPu, derivAcc.u);
  connect(srcAset.y, addAccErr.u1);
  connect(derivAcc.y, addAccErr.u2);
  connect(addAccErr.y, gainAcc.u);
  connect(gainAcc.y, addFsra.u1);
  connect(intValve.y, addFsra.u2);

  // Low-value select
  connect(limGov.y, lvsAB.u1);
  connect(limLdRate.y, lvsAB.u2);
  connect(lvsAB.y, lvsOut.u1);
  connect(addFsra.y, lvsOut.u2);

  // Valve actuator (rate- and position-limited first-order lag)
  connect(lvsOut.y, subActErr.u1);
  connect(intValve.y, subActErr.u2);
  connect(subActErr.y, gainAct.u);
  connect(gainAct.y, limRate.u);
  connect(limRate.y, intValve.u);

  // Turbine
  connect(intValve.y, subWfnl.u1);
  connect(srcWfnl.y, subWfnl.u2);
  connect(subWfnl.y, turbLl.u);
  connect(turbLl.y, turbGain.u);
  connect(omegaPu, turbDm.u);
  connect(turbGain.y, addPm.u1);
  connect(turbDm.y, addPm.u2);
  connect(addPm.y, PmPu);

  annotation(preferredView = "text");
end GovCT1;
