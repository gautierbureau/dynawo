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

model PssPTIST1 "PTI simplified power system stabilizer (CGMES PssPTIST1)"
  /*
    Single-input (speed) power system stabilizer: an input transducer, a
    gain, a washout high-pass stage and two lead-lag phase-compensation
    stages. Corresponds to the CGMES IEC 61970-302 PssPTIST1 model
    (PSS/E PTIST1). Block-based: the equation section contains only
    connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit KStab "Stabilizer gain";
  parameter Types.Time t1 "First lead time constant in s";
  parameter Types.Time t2 "First lag time constant in s";
  parameter Types.Time t3 "Second lead time constant in s";
  parameter Types.Time t4 "Second lag time constant in s";
  parameter Types.Time tW "Washout time constant in s";
  parameter Types.Time t6 "Input transducer time constant in s";
  parameter Types.VoltageModulePu UPssMaxPu "Maximum stabilizer output in pu (base UNom)";
  parameter Types.VoltageModulePu UPssMinPu "Minimum stabilizer output in pu (base UNom)";

  // Inputs / output
  Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Blocks.Interfaces.RealOutput UPssPu(start = 0) "Stabilizer output voltage in pu (base UNom)";

  // Blocks
  Blocks.Math.Feedback dW "Speed deviation omega - omegaRef";
  Blocks.Continuous.FirstOrder transducer(T = if t6 > 1e-6 then t6 else 1e-5) "Input transducer";
  Blocks.Math.Gain gainPss(k = KStab) "Stabilizer gain";
  Continuous.Washout washout(tW = tW) "Washout high-pass stage";
  Continuous.TransferFunction leadLag1(a = {t2, 1}, b = {t1, 1}) "First lead-lag stage";
  Continuous.TransferFunction leadLag2(a = {t4, 1}, b = {t3, 1}) "Second lead-lag stage";
  Blocks.Nonlinear.Limiter limiter(uMax = UPssMaxPu, uMin = UPssMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Stabilizer output limiter";

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, transducer.u);
  connect(transducer.y, gainPss.u);
  connect(gainPss.y, washout.u);
  connect(washout.y, leadLag1.u);
  connect(leadLag1.y, leadLag2.u);
  connect(leadLag2.y, limiter.u);
  connect(limiter.y, UPssPu);

  annotation(preferredView = "text");
end PssPTIST1;
