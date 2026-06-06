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

model PssPTIST3 "PTI type 3 dual-input power system stabilizer (CGMES PssPTIST3)"
  /*
    Dual-input (speed and electrical power) power system stabilizer. The
    speed path is filtered by a transducer and the power path by a washout
    and a low-pass filter; the two are mixed, washed out and phase-
    compensated by two lead-lag stages. Corresponds to the CGMES IEC
    61970-302 PssPTIST3 model (PSS/E PTIST3). Block-based: the equation
    section contains only connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit K1 "Speed path gain";
  parameter Types.PerUnit K2 "Power path gain";
  parameter Types.Time t1 "Speed path transducer time constant in s";
  parameter Types.Time t2 "Main washout time constant in s";
  parameter Types.Time t3 "Power path washout time constant in s";
  parameter Types.Time t4 "Power path low-pass filter time constant in s";
  parameter Types.Time t5 "First lead time constant in s";
  parameter Types.Time t6 "First lag time constant in s";
  parameter Types.Time t7 "Second lead time constant in s";
  parameter Types.Time t8 "Second lag time constant in s";
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
  Blocks.Continuous.FirstOrder transducerOmega(T = if t1 > 1e-6 then t1 else 1e-5) "Speed path transducer";
  Blocks.Math.Gain gainK1(k = K1) "Speed path gain";
  Blocks.Math.Gain gainPGen(k = SystemBase.SnRef / SNom) "Conversion to machine base power";
  Blocks.Sources.Constant constPGen0(k = PGen0PuSNom) "Initial active power";
  Blocks.Math.Feedback dPe "Power deviation PGen - PGen0";
  Blocks.Math.Gain gainK2(k = K2) "Power path gain";
  Continuous.Washout washoutPe(tW = t3) "Power path washout";
  Blocks.Continuous.FirstOrder filterPe(T = if t4 > 1e-6 then t4 else 1e-5) "Power path low-pass filter";
  Blocks.Math.Add addMix "Speed and power path mixer";
  Continuous.Washout washoutMain(tW = t2) "Main washout";
  Continuous.TransferFunction leadLag1(a = {t6, 1}, b = {t5, 1}) "First lead-lag stage";
  Continuous.TransferFunction leadLag2(a = {t8, 1}, b = {t7, 1}) "Second lead-lag stage";
  Blocks.Nonlinear.Limiter limiter(uMax = UPssMaxPu, uMin = UPssMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Stabilizer output limiter";

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, transducerOmega.u);
  connect(transducerOmega.y, gainK1.u);
  connect(PGenPu, gainPGen.u);
  connect(gainPGen.y, dPe.u1);
  connect(constPGen0.y, dPe.u2);
  connect(dPe.y, gainK2.u);
  connect(gainK2.y, washoutPe.u);
  connect(washoutPe.y, filterPe.u);
  connect(gainK1.y, addMix.u1);
  connect(filterPe.y, addMix.u2);
  connect(addMix.y, washoutMain.u);
  connect(washoutMain.y, leadLag1.u);
  connect(leadLag1.y, leadLag2.u);
  connect(leadLag2.y, limiter.u);
  connect(limiter.y, UPssPu);

  annotation(preferredView = "text");
end PssPTIST3;
