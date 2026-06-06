within Dynawo.Electrical.Controls.Machines.Governors.Standard.Hydraulic;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools for power systems.
*/

model GovHydroIEEE2 "IEEE Std 421 / IEEE Type 2 detailed hydro turbine-governor (CGMES GovHydroIEEE2)"
  /*
    Detailed IEEE Type 2 hydro turbine-governor. Backbone is the same as
    GovHydro1 (mechanical-hydraulic governor with permanent droop, dashpot
    transient droop, pilot valve filter, gate servo with velocity and
    position limits, rigid water-column turbine modelled by a derivative
    block on the gate position, plus a Dturb speed-damping term) extended
    with an optional integral feedback Ki on the speed error (set Ki = 0 to
    reduce to GovHydro1). Tp is the pilot servo time constant. Tf (valve
    filter time constant) and the six-point Gv/Pgv gate-power table
    (GV1..6 / PGV1..6) are exposed for parameter-set completeness against
    the CGMES IEC 61970-302 GovHydroIEEE2 schema but unused in this
    simplified topology - the gate-to-power conversion is identity and
    there is no additional valve-filter stage beyond the Tp pilot lag.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  // Governor parameters
  parameter Types.PerUnit RPerm "Permanent speed droop in pu";
  parameter Types.PerUnit RTemp "Temporary (transient) speed droop in pu";
  parameter Types.Time tF "Valve filter time constant in s";
  parameter Types.Time tG "Gate servo time constant in s";
  parameter Types.Time tP "Pilot servo time constant in s";
  parameter Types.Time tR "Dashpot (washout) time constant in s";
  parameter Types.PerUnit PMax "Maximum gate opening in pu";
  parameter Types.PerUnit PMin "Minimum gate opening in pu";
  parameter Types.PerUnit Velm "Symmetric gate velocity limit in pu/s";
  parameter Types.PerUnit Ki "Integral feedback gain on the speed error (0 disables the integrator branch)";

  // Turbine parameters
  parameter Types.PerUnit ATurb "Turbine gain in pu";
  parameter Types.PerUnit DTurb "Turbine speed damping coefficient in pu";
  parameter Types.PerUnit HDam "Turbine nominal head of water in pu";
  parameter Types.PerUnit QNl "No-load turbine flow in pu";
  parameter Types.Time tW "Water starting time (water inertia) in s";

  // Gate-power table (informational; identity in the simplified topology)
  parameter Types.PerUnit GV1; parameter Types.PerUnit GV2; parameter Types.PerUnit GV3;
  parameter Types.PerUnit GV4; parameter Types.PerUnit GV5; parameter Types.PerUnit GV6;
  parameter Types.PerUnit PGV1; parameter Types.PerUnit PGV2; parameter Types.PerUnit PGV3;
  parameter Types.PerUnit PGV4; parameter Types.PerUnit PGV5; parameter Types.PerUnit PGV6;

  // Penstock derivative filter time constant (fixed small value, no CGMES counterpart)
  parameter Types.Time tDeriv = 0.02 "Filter time constant of the penstock derivative block in s";

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
  Blocks.Continuous.Integrator integKi(k = Ki, y_start = 0, initType = Modelica.Blocks.Types.Init.NoInit) "Ki / s integral feedback on speed error (NoInit: SteadyState init diverges when Ki = 0 because k*u = 0 leaves u under-constrained)";
  Blocks.Math.Add3 addGovRef "PRef + speed error + Ki integral";
  Blocks.Math.Gain gainRPerm(k = RPerm) "Permanent droop feedback";
  Blocks.Continuous.TransferFunction dashpot(a = {tR, 1}, b = {RTemp * tR, 0}, initType = Modelica.Blocks.Types.Init.SteadyState) "Dashpot transient droop feedback";
  Blocks.Math.Add3 addGovErr(k2 = -1, k3 = -1) "Governor error govRef - permanent droop - dashpot";
  Blocks.Continuous.FirstOrder pilotLag(T = tP, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = 0) "Pilot valve filter 1 / (1 + tP.s)";
  Blocks.Math.Gain gain1Tg(k = 1 / tG) "Gate servo velocity gain 1 / tG";
  Blocks.Nonlinear.Limiter limGateVel(uMax = Velm, uMin = -Velm, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Gate velocity limiter";
  Blocks.Continuous.LimIntegrator gateInteg(outMax = PMax, outMin = PMin, y_start = GV0) "Gate position integrator";

  // Turbine blocks (rigid water column via gate-velocity feedback)
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
  connect(addErrSpeed.y, integKi.u);
  connect(PRefPu, addGovRef.u1);
  connect(addErrSpeed.y, addGovRef.u2);
  connect(integKi.y, addGovRef.u3);
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
end GovHydroIEEE2;
