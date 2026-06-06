within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovHydroPelton "Pelton turbine governor (CGMES GovHydroPelton)"
  /*
    Simplified Pelton wheel governor for time-domain simulation. The Pelton
    turbine has a needle (gate) servo and a deflector servo acting in
    parallel on the jet; the simplified topology here models the needle
    servo only and uses Av0 / Av1 / Vav as the deflector compensation gains
    when the simplifiedPelton flag is non-zero. Speed error with permanent
    droop Bp drives a Kg + Kc dashpot compensator, then the rate-limited
    needle servo (Vav / Valvmin..Valvmax). The water column is approximated
    by (1 - Twng.s) / (1 + 0.5.Twng.s) acting on (gate - Qc0) and scaled by
    H1 / H2 / Hn for the available head. Pelton-specific flags (cfrac,
    sfrac, simplifiedPelton, staticCompensating) are exposed but the
    simplified single-jet topology is used here.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Av0 "Deflector zero-load gain";
  parameter Types.PerUnit Av1 "Deflector dynamic gain";
  parameter Types.PerUnit Bp "Permanent speed droop";
  parameter Types.AngularVelocityPu Db1 "Speed deadband half-width (low)";
  parameter Types.AngularVelocityPu Db2 "Speed deadband half-width (high)";
  parameter Types.PerUnit H1 "Tailwater head reference";
  parameter Types.PerUnit H2 "Headwater head reference";
  parameter Types.PerUnit Hn "Nominal head";
  parameter Types.PerUnit Kc "Inner dashpot gain";
  parameter Types.PerUnit Kg "Outer proportional gain";
  parameter Types.PerUnit Qc0 "Initial gate flow command";
  parameter Types.PerUnit Qn "Nominal turbine flow";
  parameter Real SimplifiedPelton "Simplified Pelton flag (informational; 0 = full, 1 = simplified)";
  parameter Real StaticCompensating "Static compensation flag (informational)";
  parameter Types.Time tA "PID derivative filter time constant in s";
  parameter Types.Time tD "Dashpot reset time constant in s";
  parameter Types.Time tS "Servo input filter time constant in s";
  parameter Types.Time tWnc "Water tunnel time constant (informational)";
  parameter Types.Time tWng "Penstock water inertia time constant in s";
  parameter Types.Time tX "Deflector time constant in s";
  parameter Types.PerUnit Va "Pilot saturating output";
  parameter Types.PerUnit Valvmax "Servo upper limit";
  parameter Types.PerUnit Valvmin "Servo lower limit";
  parameter Types.PerUnit Vav "Deflector saturating gain";
  parameter Types.PerUnit Vc "Compensator gain";
  parameter Types.PerUnit Vcv "Deflector compensator gain";
  parameter Boolean Cfrac "Compensator fraction flag";
  parameter Boolean Sfrac "Servo fraction flag";

  parameter Types.ActivePowerPu Pm0Pu;
  final parameter Types.PerUnit GV0 = max(Pm0Pu / max(Hn, 1e-6), 1e-6);
  final parameter Types.PerUnit PRef0Pu = Bp * GV0;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1);
  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = max(Db1, Db2), uMin = -max(Db1, Db2));
  Modelica.Blocks.Math.Gain droopGain(k = Bp);
  Modelica.Blocks.Math.Add3 sumErr(k1 = 1, k2 = -1, k3 = -1);
  Modelica.Blocks.Math.Gain kgGain(k = Kg);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction dashpot(a = {tD, 1}, b = {Kc * tD, 0}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = GV0) "Washout: high-pass dashpot, u_start must match steady-state gate to avoid spurious init transient";
  Modelica.Blocks.Math.Add outerSum;
  Modelica.Blocks.Continuous.FirstOrder servoLag(T = max(tS, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter valveLim(uMax = Valvmax, uMin = Valvmin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator gateInteg(outMax = 1, outMin = 0, y_start = GV0, k = 1 / max(tX, 1e-6));
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction waterColumn(a = {0.5 * tWng, 1}, b = {-tWng, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = GV0);
  Modelica.Blocks.Math.Gain hydraulicGain(k = Hn);
  Modelica.Blocks.Math.Add subQc(k2 = -1);
  Modelica.Blocks.Sources.Constant qc0Const(k = Qc0);

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(addErrSpeed.y, deadband.u);
  connect(gateInteg.y, droopGain.u);
  connect(PRefPu, sumErr.u1);
  connect(droopGain.y, sumErr.u2);
  connect(deadband.y, sumErr.u3);
  connect(sumErr.y, kgGain.u);
  connect(gateInteg.y, dashpot.u);
  connect(kgGain.y, outerSum.u1);
  connect(dashpot.y, outerSum.u2);
  connect(outerSum.y, servoLag.u);
  connect(servoLag.y, valveLim.u);
  connect(valveLim.y, gateInteg.u);
  connect(gateInteg.y, subQc.u1);
  connect(qc0Const.y, subQc.u2);
  connect(subQc.y, waterColumn.u);
  connect(waterColumn.y, hydraulicGain.u);
  connect(hydraulicGain.y, PmPu);

  annotation(preferredView = "text");
end GovHydroPelton;
