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

model PssSK "Slovak three-input power system stabilizer (CGMES PssSK)"
  /*
    Three-input PSS combining the active power, the speed deviation and the
    field current. Each branch applies a first-order lag followed by a
    washout (with its own time constants) and a gain; the three signals are
    summed and limited. Corresponds to the CGMES IEC 61970-302 PssSK model
    (NEPLAN PSSSK). Block-based: the equation section contains only connect
    statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit K1 "Power path gain";
  parameter Types.PerUnit K2 "Rotor speed deviation path gain";
  parameter Types.PerUnit K3 "Field current path gain";
  parameter Types.Time t1 "Power path first-order time constant in s";
  parameter Types.Time t2 "Power path washout time constant in s";
  parameter Types.Time t3 "Rotor speed deviation path first-order time constant in s";
  parameter Types.Time t4 "Rotor speed deviation path washout time constant in s";
  parameter Types.Time t5 "Field current path first-order time constant in s";
  parameter Types.Time t6 "Field current path washout time constant in s";
  parameter Types.VoltageModulePu UPssMaxPu "Maximum stabilizer output in pu (base UNom)";
  parameter Types.VoltageModulePu UPssMinPu "Minimum stabilizer output in pu (base UNom)";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power in MVA";

  // Initial parameters (computed by the initialization model)
  parameter Types.ActivePowerPu PGen0Pu "Initial active power in pu (base SnRef) (generator convention)";
  parameter Types.PerUnit IRotor0Pu "Initial field (rotor) current in pu (base IRotorNom)";

  final parameter Types.PerUnit PGen0PuSNom = PGen0Pu * SystemBase.SnRef / SNom "Initial active power in pu (base SNom)";

  // Inputs / output
  Blocks.Interfaces.RealInput PGenPu(start = PGen0Pu) "Active power in pu (base SnRef) (generator convention)";
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput IRotorPu(start = IRotor0Pu) "Field (rotor) current in pu (base IRotorNom)";
  Blocks.Interfaces.RealOutput UPssPu(start = 0) "Stabilizer output voltage in pu (base UNom)";

  // Power branch: gain to SNom base, low-pass, washout, branch gain
  Blocks.Math.Gain gainPGen(k = SystemBase.SnRef / SNom) "Conversion to machine base power";
  Blocks.Continuous.FirstOrder filterP(T = if t1 > 1e-6 then t1 else 1e-5, y_start = PGen0PuSNom) "Power path low-pass";
  Continuous.Washout washoutP(tW = t2, U0 = PGen0PuSNom) "Power path washout";
  Blocks.Math.Gain gainK1(k = K1) "Power branch gain";

  // Speed branch: deviation, low-pass, washout, branch gain
  Blocks.Math.Feedback dW "Speed deviation omega - omegaRef";
  Blocks.Continuous.FirstOrder filterW(T = if t3 > 1e-6 then t3 else 1e-5) "Speed path low-pass";
  Continuous.Washout washoutW(tW = t4) "Speed path washout";
  Blocks.Math.Gain gainK2(k = K2) "Speed branch gain";

  // Field-current branch: low-pass, washout, branch gain
  Blocks.Continuous.FirstOrder filterIfd(T = if t5 > 1e-6 then t5 else 1e-5, y_start = IRotor0Pu) "Field current path low-pass";
  Continuous.Washout washoutIfd(tW = t6, U0 = IRotor0Pu) "Field current path washout";
  Blocks.Math.Gain gainK3(k = K3) "Field current branch gain";

  // Sum of the three branches + limiter
  Blocks.Math.Add3 sum3 "Sum of the three branches";
  Blocks.Nonlinear.Limiter limiter(uMax = UPssMaxPu, uMin = UPssMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Stabilizer output limiter";

equation
  // Power branch
  connect(PGenPu, gainPGen.u);
  connect(gainPGen.y, filterP.u);
  connect(filterP.y, washoutP.u);
  connect(washoutP.y, gainK1.u);

  // Speed branch
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, filterW.u);
  connect(filterW.y, washoutW.u);
  connect(washoutW.y, gainK2.u);

  // Field-current branch
  connect(IRotorPu, filterIfd.u);
  connect(filterIfd.y, washoutIfd.u);
  connect(washoutIfd.y, gainK3.u);

  // Combine and limit
  connect(gainK1.y, sum3.u1);
  connect(gainK2.y, sum3.u2);
  connect(gainK3.y, sum3.u3);
  connect(sum3.y, limiter.u);
  connect(limiter.y, UPssPu);

  annotation(preferredView = "text");
end PssSK;
