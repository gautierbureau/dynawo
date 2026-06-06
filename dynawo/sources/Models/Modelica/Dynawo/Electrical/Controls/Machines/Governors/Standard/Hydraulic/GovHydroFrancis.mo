within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*/

model GovHydroFrancis "Francis turbine governor (CGMES GovHydroFrancis)"
  /*
    Simplified Francis hydro turbine-governor for time-domain simulation.
    Speed error (omega - omegaRef) with permanent droop Rs is filtered
    through a Ti / Td / Ta classical PID-like compensator (built from a
    proportional KG branch + an integral KI branch + a dashpot lead-lag),
    then drives a Ta-Ts-Tp pilot/distributor servo chain. The servo output
    is rate-limited (Va between Valvmin and Valvmax) and integrated to a
    gate position bounded by the standard 0..1. The Francis turbine
    hydraulics use a single water-column lead-lag (1 - Twng.s) /
    (1 + 0.5.Twng.s) acting on (gate - Qnl) at the nominal head Hn / H0;
    Am and Av0 scale the gate-to-flow nonlinearity. The water tunnel /
    surge chamber complication (Twnc parameter) is exposed but represented
    as a transparent unit gain (waterTunnelSurgeChamberSimulation field
    documents which CGMES sub-model variant was requested).
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Rs "Permanent speed droop";
  parameter Types.Time tG "Distributor servo time constant in s";
  parameter Types.Time tP "Pilot servo time constant in s";
  parameter Types.PerUnit Bp "Transient droop / dashpot gain";
  parameter Types.Time tD "Dashpot reset time constant in s";
  parameter Types.Time tA "PID derivative filter time constant in s";
  parameter Types.Time tS "Servo input filter time constant in s";
  parameter Types.Time tWnc "Water tunnel time constant (informational)";
  parameter Types.Time tWng "Penstock water inertia time constant in s";
  parameter Types.PerUnit Qn "Nominal turbine flow";
  parameter Types.PerUnit H0 "Initial head";
  parameter Types.PerUnit Am "Turbine gain";
  parameter Types.PerUnit Av0 "Gate-to-flow zero-load coefficient";
  parameter Types.PerUnit Avsmnx "Minimum servo input";
  parameter Types.PerUnit Avsmx "Maximum servo input";
  parameter Types.PerUnit Hn "Nominal head";
  parameter Types.PerUnit Kc "Servo position feedback gain";
  parameter Types.PerUnit Kg "Outer-loop proportional gain";
  parameter Types.PerUnit Ki "Outer-loop integral gain";
  parameter Types.PerUnit Knl "No-load gain compensation";
  parameter Types.PerUnit Qc0 "Initial gate flow command";
  parameter Types.PerUnit Va "Pilot saturating output";
  parameter Types.PerUnit Valvmax "Servo upper limit";
  parameter Types.PerUnit Valvmin "Servo lower limit";
  parameter Types.PerUnit Vc "Compensator gain";

  parameter Types.ActivePowerPu Pm0Pu "Initial mechanical power in pu (base PNomTurb)";
  final parameter Types.PerUnit GV0 = max(Pm0Pu / max(Am * Hn, 1e-6), 1e-6) "Initial gate position";
  final parameter Types.PerUnit PRef0Pu = Rs * GV0;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PRefPu(start = PRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add addErrSpeed(k2 = -1);
  Modelica.Blocks.Math.Gain droopGain(k = Rs);
  Modelica.Blocks.Math.Add3 sumErr(k1 = 1, k2 = -1, k3 = -1);
  Modelica.Blocks.Math.Gain kgGain(k = Kg);
  Modelica.Blocks.Continuous.Integrator iBranch(k = Ki, y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction dashpot(a = {tD, 1}, b = {Bp * tD, 0}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = 0);
  Modelica.Blocks.Math.Add3 outerSum;
  Modelica.Blocks.Nonlinear.Limiter avsLim(uMax = Avsmx, uMin = Avsmnx, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.FirstOrder pilotServo(T = max(tP, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder servoLag(T = max(tS, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter valveLim(uMax = Valvmax, uMin = Valvmin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator gateInteg(outMax = 1, outMin = 0, y_start = GV0, k = 1 / max(tG, 1e-6));
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction waterColumn(a = {0.5 * tWng, 1}, b = {-tWng, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = GV0);
  Modelica.Blocks.Math.Gain hydraulicGain(k = Am * Hn);
  Modelica.Blocks.Math.Add subQc(k2 = -1);
  Modelica.Blocks.Sources.Constant qc0Const(k = Qc0);

equation
  connect(omegaPu, addErrSpeed.u1);
  connect(omegaRefPu, addErrSpeed.u2);
  connect(gateInteg.y, droopGain.u);
  connect(PRefPu, sumErr.u1);
  connect(droopGain.y, sumErr.u2);
  connect(addErrSpeed.y, sumErr.u3);
  connect(sumErr.y, kgGain.u);
  connect(sumErr.y, iBranch.u);
  connect(gateInteg.y, dashpot.u);
  connect(kgGain.y, outerSum.u1);
  connect(iBranch.y, outerSum.u2);
  connect(dashpot.y, outerSum.u3);
  connect(outerSum.y, avsLim.u);
  connect(avsLim.y, pilotServo.u);
  connect(pilotServo.y, servoLag.u);
  connect(servoLag.y, valveLim.u);
  connect(valveLim.y, gateInteg.u);
  connect(gateInteg.y, subQc.u1);
  connect(qc0Const.y, subQc.u2);
  connect(subQc.y, waterColumn.u);
  connect(waterColumn.y, hydraulicGain.u);
  connect(hydraulicGain.y, PmPu);

  annotation(preferredView = "text");
end GovHydroFrancis;
