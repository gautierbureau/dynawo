within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

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

model Pss5 "Generic dual-input power system stabilizer type 5 (CGMES Pss5)"
  /*
    Dual-input (speed and electrical power) power system stabilizer. The
    speed path applies a deadband, a gain and a washout; the power path
    applies a low-pass filter and a gain. The two paths are summed,
    amplified by an overall gain and, when CTw2 is true, passed through a
    second washout before the output limiter. Corresponds to the CGMES
    IEC 61970-302 Pss5 model. Block-based: the equation section contains
    only connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Boolean CTw2 "Second washout switch: true enables the Tw2 stage";
  parameter Types.AngularVelocityPu DeadBandPu "Speed deadband half-width in pu";
  parameter Types.PerUnit Kf "Speed path gain";
  parameter Types.PerUnit Kpe "Power path gain";
  parameter Types.PerUnit Kpss "Overall stabilizer gain";
  parameter Types.Time tPe "Power path low-pass filter time constant in s";
  parameter Types.Time tW1 "First washout time constant in s";
  parameter Types.Time tW2 "Second washout time constant in s";
  parameter Types.VoltageModulePu UPssMaxPu "Maximum stabilizer output in pu (base UNom)";
  parameter Types.VoltageModulePu UPssMinPu "Minimum stabilizer output in pu (base UNom)";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power in MVA";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu PGen0Pu "Initial active power in pu (base SnRef) (generator convention)";

  final parameter Types.PerUnit PGen0PuSNom = PGen0Pu * SystemBase.SnRef / SNom "Initial active power in pu (base SNom)";

  // Inputs / output
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput PGenPu(start = PGen0Pu) "Active power in pu (base SnRef) (generator convention)";
  Blocks.Interfaces.RealOutput UPssPu(start = 0) "Stabilizer output voltage in pu (base UNom)";

  // Blocks
  Blocks.Math.Feedback dW "Speed deviation omega - omegaRef";
  Blocks.Nonlinear.DeadZone deadband(uMax = DeadBandPu, uMin = -DeadBandPu) "Speed deadband";
  Blocks.Math.Gain gainKf(k = Kf) "Speed path gain";
  Continuous.Washout washout1(tW = tW1) "First washout";
  Blocks.Math.Gain gainPGen(k = SystemBase.SnRef / SNom) "Conversion to machine base power";
  Blocks.Continuous.FirstOrder peFilter(T = if tPe > 1e-6 then tPe else 1e-5, initType = Modelica.Blocks.Types.Init.SteadyState, y_start = PGen0PuSNom) "Power path low-pass filter";
  Blocks.Math.Gain gainKpe(k = Kpe) "Power path gain";
  Blocks.Math.Add addCombined "Speed and power path sum";
  Blocks.Math.Gain gainKpss(k = Kpss) "Overall stabilizer gain";
  Continuous.Washout washout2(tW = tW2, U0 = Kpss * Kpe * PGen0PuSNom) "Second washout";
  Blocks.Sources.BooleanConstant cTw2Signal(k = CTw2) "Second washout switch signal";
  Blocks.Logical.Switch tw2Switch "Routes through the second washout when CTw2 is true";
  Blocks.Nonlinear.Limiter limiter(uMax = UPssMaxPu, uMin = UPssMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Stabilizer output limiter";

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, deadband.u);
  connect(deadband.y, gainKf.u);
  connect(gainKf.y, washout1.u);
  connect(PGenPu, gainPGen.u);
  connect(gainPGen.y, peFilter.u);
  connect(peFilter.y, gainKpe.u);
  connect(washout1.y, addCombined.u1);
  connect(gainKpe.y, addCombined.u2);
  connect(addCombined.y, gainKpss.u);
  connect(gainKpss.y, washout2.u);
  connect(washout2.y, tw2Switch.u1);
  connect(gainKpss.y, tw2Switch.u3);
  connect(cTw2Signal.y, tw2Switch.u2);
  connect(tw2Switch.y, limiter.u);
  connect(limiter.y, UPssPu);

  annotation(preferredView = "text");
end Pss5;
