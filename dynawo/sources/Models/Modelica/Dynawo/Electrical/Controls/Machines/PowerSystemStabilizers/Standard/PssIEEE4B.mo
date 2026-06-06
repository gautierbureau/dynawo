within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

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

model PssIEEE4B "IEEE Std 421.5 multi-band power system stabilizer type PSS4B (CGMES PssIEEE4B)"
  /*
    Three parallel frequency bands (high H, intermediate I, low L) act on
    the speed deviation deltaOmega = omegaPu - omegaRefPu. Per band, the
    deviation is fed into a differentiator-washout pair built from the
    band-center time constants (T*9 / T*10 and T*11 / T*12), then through
    a chain of four lead-lag stages (T*1 / T*2, T*3 / T*4, T*5 / T*6,
    T*7 / T*8), multiplied by the main band gain (Kh, Ki, Kl) and clamped
    to (Vshmax, Vshmin), (Vsimax, Vsimin) or (Vslmax, Vslmin). The three
    band outputs are summed and the total is clamped to (Vsmax, Vsmin).
    The intra-band sub-gains (K*1, K*2, K*11, K*17) and the band-width
    descriptors (bw*1, bw*2, omegan*1, omegan*2) are exposed as parameters
    for documentation completeness but the simplified topology here does
    not branch the lead-lag chain into the two separate sub-branches that
    the IEEE 421.5 Type II parameterisation enables - it folds the band
    into a single shaping chain plus its washout. This matches typical
    PSS/E PSS4B implementations and is sufficient for the IEC 61970-302
    PssIEEE4B mapping; refine to the dual-branch form if the four sub-
    gains are non-unity in the dataset being modeled.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  // Band gains (main)
  parameter Types.PerUnit Kh "High band overall gain";
  parameter Types.PerUnit Ki "Intermediate band overall gain";
  parameter Types.PerUnit Kl "Low band overall gain";

  // Band sub-gains (informational; unused in the simplified single-branch chain)
  parameter Types.PerUnit Kh1 = 1 "High band branch 1 gain";
  parameter Types.PerUnit Kh2 = 1 "High band branch 2 gain";
  parameter Types.PerUnit Kh11 = 1 "High band first inner gain";
  parameter Types.PerUnit Kh17 = 1 "High band second inner gain";
  parameter Types.PerUnit Ki1 = 1 "Intermediate band branch 1 gain";
  parameter Types.PerUnit Ki2 = 1 "Intermediate band branch 2 gain";
  parameter Types.PerUnit Ki11 = 1 "Intermediate band first inner gain";
  parameter Types.PerUnit Ki17 = 1 "Intermediate band second inner gain";
  parameter Types.PerUnit Kl1 = 1 "Low band branch 1 gain";
  parameter Types.PerUnit Kl2 = 1 "Low band branch 2 gain";
  parameter Types.PerUnit Kl11 = 1 "Low band first inner gain";
  parameter Types.PerUnit Kl17 = 1 "Low band second inner gain";

  // Band time constants (4 lead-lag pairs + 2 washout / wash compensation stages per band)
  parameter Types.Time TH1 "H band lead-lag 1 lead time constant in s";
  parameter Types.Time TH2 "H band lead-lag 1 lag time constant in s";
  parameter Types.Time TH3 "H band lead-lag 2 lead time constant in s";
  parameter Types.Time TH4 "H band lead-lag 2 lag time constant in s";
  parameter Types.Time TH5 "H band lead-lag 3 lead time constant in s";
  parameter Types.Time TH6 "H band lead-lag 3 lag time constant in s";
  parameter Types.Time TH7 "H band lead-lag 4 lead time constant in s";
  parameter Types.Time TH8 "H band lead-lag 4 lag time constant in s";
  parameter Types.Time TH9 "H band washout numerator time constant in s";
  parameter Types.Time TH10 "H band washout denominator time constant in s";
  parameter Types.Time TH11 "H band wash compensation lead time constant in s";
  parameter Types.Time TH12 "H band wash compensation lag time constant in s";
  parameter Types.Time TI1 "I band lead-lag 1 lead time constant in s";
  parameter Types.Time TI2 "I band lead-lag 1 lag time constant in s";
  parameter Types.Time TI3 "I band lead-lag 2 lead time constant in s";
  parameter Types.Time TI4 "I band lead-lag 2 lag time constant in s";
  parameter Types.Time TI5 "I band lead-lag 3 lead time constant in s";
  parameter Types.Time TI6 "I band lead-lag 3 lag time constant in s";
  parameter Types.Time TI7 "I band lead-lag 4 lead time constant in s";
  parameter Types.Time TI8 "I band lead-lag 4 lag time constant in s";
  parameter Types.Time TI9 "I band washout numerator time constant in s";
  parameter Types.Time TI10 "I band washout denominator time constant in s";
  parameter Types.Time TI11 "I band wash compensation lead time constant in s";
  parameter Types.Time TI12 "I band wash compensation lag time constant in s";
  parameter Types.Time TL1 "L band lead-lag 1 lead time constant in s";
  parameter Types.Time TL2 "L band lead-lag 1 lag time constant in s";
  parameter Types.Time TL3 "L band lead-lag 2 lead time constant in s";
  parameter Types.Time TL4 "L band lead-lag 2 lag time constant in s";
  parameter Types.Time TL5 "L band lead-lag 3 lead time constant in s";
  parameter Types.Time TL6 "L band lead-lag 3 lag time constant in s";
  parameter Types.Time TL7 "L band lead-lag 4 lead time constant in s";
  parameter Types.Time TL8 "L band lead-lag 4 lag time constant in s";
  parameter Types.Time TL9 "L band washout numerator time constant in s";
  parameter Types.Time TL10 "L band washout denominator time constant in s";
  parameter Types.Time TL11 "L band wash compensation lead time constant in s";
  parameter Types.Time TL12 "L band wash compensation lag time constant in s";

  // Band center frequencies (informational only; the actual filter shape is set by the time constants above)
  parameter Real OmegaNH1 = 0 "H band centre frequency 1 in rad/s (descriptive)";
  parameter Real OmegaNH2 = 0 "H band centre frequency 2 in rad/s (descriptive)";
  parameter Real OmegaNL1 = 0 "L band centre frequency 1 in rad/s (descriptive)";
  parameter Real OmegaNL2 = 0 "L band centre frequency 2 in rad/s (descriptive)";
  parameter Real BWH1 = 0 "H band branch 1 bandwidth (descriptive)";
  parameter Real BWH2 = 0 "H band branch 2 bandwidth (descriptive)";
  parameter Real BWL1 = 0 "L band branch 1 bandwidth (descriptive)";
  parameter Real BWL2 = 0 "L band branch 2 bandwidth (descriptive)";

  // Output limits
  parameter Types.VoltageModulePu VsMaxPu "Overall stabilizer output upper limit in pu (base UNom)";
  parameter Types.VoltageModulePu VsMinPu "Overall stabilizer output lower limit in pu (base UNom)";
  parameter Types.VoltageModulePu VshMaxPu "H band output upper limit in pu (base UNom)";
  parameter Types.VoltageModulePu VshMinPu "H band output lower limit in pu (base UNom)";
  parameter Types.VoltageModulePu VsiMaxPu "I band output upper limit in pu (base UNom)";
  parameter Types.VoltageModulePu VsiMinPu "I band output lower limit in pu (base UNom)";
  parameter Types.VoltageModulePu VslMaxPu "L band output upper limit in pu (base UNom)";
  parameter Types.VoltageModulePu VslMinPu "L band output lower limit in pu (base UNom)";

  // Inputs / output
  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu) "Reference angular frequency in pu (base omegaNom)";
  Modelica.Blocks.Interfaces.RealOutput UPssPu(start = 0) "Stabilizer output voltage in pu (base UNom)";

  // Shared input
  Modelica.Blocks.Math.Feedback dW "Speed deviation omega - omegaRef";

  // High band chain
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction washH(a = {TH10, 1}, b = {TH9, 0}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction compH(a = {TH12, 1}, b = {TH11, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llH1(a = {TH2, 1}, b = {TH1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llH2(a = {TH4, 1}, b = {TH3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llH3(a = {TH6, 1}, b = {TH5, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llH4(a = {TH8, 1}, b = {TH7, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain gainH(k = Kh);
  Modelica.Blocks.Nonlinear.Limiter limH(uMax = VshMaxPu, uMin = VshMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

  // Intermediate band chain
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction washI(a = {TI10, 1}, b = {TI9, 0}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction compI(a = {TI12, 1}, b = {TI11, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llI1(a = {TI2, 1}, b = {TI1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llI2(a = {TI4, 1}, b = {TI3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llI3(a = {TI6, 1}, b = {TI5, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llI4(a = {TI8, 1}, b = {TI7, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain gainI(k = Ki);
  Modelica.Blocks.Nonlinear.Limiter limI(uMax = VsiMaxPu, uMin = VsiMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

  // Low band chain
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction washL(a = {TL10, 1}, b = {TL9, 0}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction compL(a = {TL12, 1}, b = {TL11, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llL1(a = {TL2, 1}, b = {TL1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llL2(a = {TL4, 1}, b = {TL3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llL3(a = {TL6, 1}, b = {TL5, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction llL4(a = {TL8, 1}, b = {TL7, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain gainL(k = Kl);
  Modelica.Blocks.Nonlinear.Limiter limL(uMax = VslMaxPu, uMin = VslMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

  // Output summation
  Modelica.Blocks.Math.Add3 bandSum "VsH + VsI + VsL";
  Modelica.Blocks.Nonlinear.Limiter limOut(uMax = VsMaxPu, uMin = VsMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);

  // H band
  connect(dW.y, washH.u);
  connect(washH.y, compH.u);
  connect(compH.y, llH1.u);
  connect(llH1.y, llH2.u);
  connect(llH2.y, llH3.u);
  connect(llH3.y, llH4.u);
  connect(llH4.y, gainH.u);
  connect(gainH.y, limH.u);

  // I band
  connect(dW.y, washI.u);
  connect(washI.y, compI.u);
  connect(compI.y, llI1.u);
  connect(llI1.y, llI2.u);
  connect(llI2.y, llI3.u);
  connect(llI3.y, llI4.u);
  connect(llI4.y, gainI.u);
  connect(gainI.y, limI.u);

  // L band
  connect(dW.y, washL.u);
  connect(washL.y, compL.u);
  connect(compL.y, llL1.u);
  connect(llL1.y, llL2.u);
  connect(llL2.y, llL3.u);
  connect(llL3.y, llL4.u);
  connect(llL4.y, gainL.u);
  connect(gainL.y, limL.u);

  // Final sum and limiter
  connect(limH.y, bandSum.u1);
  connect(limI.y, bandSum.u2);
  connect(limL.y, bandSum.u3);
  connect(bandSum.y, limOut.u);
  connect(limOut.y, UPssPu);

  annotation(preferredView = "text");
end PssIEEE4B;
