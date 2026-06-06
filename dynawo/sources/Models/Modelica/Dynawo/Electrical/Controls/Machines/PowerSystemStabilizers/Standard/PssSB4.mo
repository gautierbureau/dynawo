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

model PssSB4 "Power-sensitive STAB4 power system stabilizer (CGMES PssSB4)"
  /*
    Power-sensitive PSS. The input active power is filtered by a first-order
    lag (TT) and the steady-state value (REF = initial power) is subtracted
    to extract the deviation (V2). The deviation is then differentiated with
    a washout-like block s*KX/(1+s*TX2), two lead-lag stages
    (TA/TX1 and TB/TC), two further first-order lags (TD and TE), and
    finally limited to [L1, L2]. Corresponds to the CGMES IEC 61970-302
    PssSB4 model (NEPLAN PSS-STAB4). Block-based: the equation section
    contains only connect statements.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter Types.Time TT "Input low-pass time constant in s";
  parameter Types.PerUnit KX "Washout gain";
  parameter Types.Time TX2 "Washout time constant in s";
  parameter Types.Time TA "First lead time constant in s";
  parameter Types.Time TX1 "First lag time constant in s (reset)";
  parameter Types.Time TB "Second lead time constant in s";
  parameter Types.Time TC "Second lag time constant in s";
  parameter Types.Time TD "Output first lag time constant in s";
  parameter Types.Time TE "Output second lag time constant in s";
  parameter Types.VoltageModulePu L1 "Lower limit (negative) in pu (base UNom)";
  parameter Types.VoltageModulePu L2 "Upper limit (positive) in pu (base UNom)";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power in MVA";

  // Initial parameter (computed by the initialization model)
  parameter Types.ActivePowerPu PGen0Pu "Initial active power in pu (base SnRef) (generator convention)";

  final parameter Types.PerUnit PGen0PuSNom = PGen0Pu * SystemBase.SnRef / SNom "Initial active power in pu (base SNom)";

  // Inputs / output
  Blocks.Interfaces.RealInput PGenPu(start = PGen0Pu) "Active power in pu (base SnRef) (generator convention)";
  Blocks.Interfaces.RealOutput UPssPu(start = 0) "Stabilizer output voltage in pu (base UNom)";

  // Blocks
  Blocks.Math.Gain gainPGen(k = SystemBase.SnRef / SNom) "Conversion to machine base power";
  Blocks.Continuous.FirstOrder filterTT(T = if TT > 1e-6 then TT else 1e-5, y_start = PGen0PuSNom) "Input low-pass (TT)";
  Blocks.Sources.Constant refPGen(k = PGen0PuSNom) "REF = initial P_LP";
  Blocks.Math.Feedback dP "Deviation V2 = V1 - REF";
  Continuous.TransferFunction washoutKX(a = {TX2, 1}, b = {KX, 0}) "s*KX / (1 + s*TX2)";
  Continuous.TransferFunction leadLag1(a = {TX1, 1}, b = {TA, 1}) "First lead-lag (TA / TX1)";
  Continuous.TransferFunction leadLag2(a = {TC, 1}, b = {TB, 1}) "Second lead-lag (TB / TC)";
  Blocks.Continuous.FirstOrder filterTD(T = if TD > 1e-6 then TD else 1e-5) "Output low-pass (TD)";
  Blocks.Continuous.FirstOrder filterTE(T = if TE > 1e-6 then TE else 1e-5) "Output low-pass (TE)";
  Blocks.Nonlinear.Limiter limiter(uMax = L2, uMin = L1, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy) "Stabilizer output limiter";

equation
  connect(PGenPu, gainPGen.u);
  connect(gainPGen.y, filterTT.u);
  connect(filterTT.y, dP.u1);
  connect(refPGen.y, dP.u2);
  connect(dP.y, washoutKX.u);
  connect(washoutKX.y, leadLag1.u);
  connect(leadLag1.y, leadLag2.u);
  connect(leadLag2.y, filterTD.u);
  connect(filterTD.y, filterTE.u);
  connect(filterTE.y, limiter.u);
  connect(limiter.y, UPssPu);

  annotation(preferredView = "text");
end PssSB4;
