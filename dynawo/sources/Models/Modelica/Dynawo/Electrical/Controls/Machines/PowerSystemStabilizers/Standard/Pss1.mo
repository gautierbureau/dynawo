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

model Pss1 "Generic power system stabilizer type 1 (CGMES Pss1)"
  /*
    Generic single-input (speed) power system stabilizer: a gain followed by
    two cascaded washout stages and two lead-lag phase-compensation stages.
    The dual washout gives a steeper high-pass rolloff than a single washout.
    Corresponds to the CGMES IEC 61970-302 Pss1 model. Block-based: the
    equation section contains only connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit KStab "Stabilizer gain";
  parameter Types.Time tW1 "First washout time constant in s";
  parameter Types.Time tW2 "Second washout time constant in s";
  parameter Types.Time t1 "First lead time constant in s";
  parameter Types.Time t2 "First lag time constant in s";
  parameter Types.Time t3 "Second lead time constant in s";
  parameter Types.Time t4 "Second lag time constant in s";
  parameter Types.VoltageModulePu UPssMaxPu "Maximum stabilizer output in pu (base UNom)";
  parameter Types.VoltageModulePu UPssMinPu "Minimum stabilizer output in pu (base UNom)";

  // Inputs / output
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealOutput UPssPu(start = 0) "Stabilizer output voltage in pu (base UNom)";

  // Blocks
  Blocks.Math.Feedback dW "Speed deviation omega - omegaRef";
  Blocks.Math.Gain gainPss(k = KStab) "Stabilizer gain";
  Continuous.Washout washout1(tW = tW1) "First washout stage";
  Continuous.Washout washout2(tW = tW2) "Second washout stage";
  Continuous.TransferFunction leadLag1(a = {t2, 1}, b = {t1, 1}) "First lead-lag stage";
  Continuous.TransferFunction leadLag2(a = {t4, 1}, b = {t3, 1}) "Second lead-lag stage";
  Blocks.Nonlinear.Limiter limiter(uMax = UPssMaxPu, uMin = UPssMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Stabilizer output limiter";

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, gainPss.u);
  connect(gainPss.y, washout1.u);
  connect(washout1.y, washout2.u);
  connect(washout2.y, leadLag1.u);
  connect(leadLag1.y, leadLag2.u);
  connect(leadLag2.y, limiter.u);
  connect(limiter.y, UPssPu);

  annotation(preferredView = "text");
end Pss1;
