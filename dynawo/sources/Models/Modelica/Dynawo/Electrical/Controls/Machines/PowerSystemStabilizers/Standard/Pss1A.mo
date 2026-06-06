within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model Pss1A "Single-input PSS variant (CGMES Pss1A)"
  /*
    Single-input PSS variant of PssIEEE1A. Input is the speed deviation
    omega - omegaRef. Signal path adds a second-order torsional notch
    1 + s*A1 + s^2*A2 (approximated here as a Kx high-frequency gain
    multiplied through), a washout (sT5)/(1+sT6), Ks main gain, and two
    lead-lag stages (T1/T2, T3/T4). Output clamped to (Vrmin, Vrmax).
    Voltage cutoff thresholds (Vcl, Vcu) are exposed for documentation
    (CGMES disables the PSS when terminal voltage falls outside the band;
    the simplified topology does not enforce that here). Tdelay is a
    transport delay before the output, modelled here as a 1/(1+s*Tdelay)
    first-order lag.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.NonElectrical.Blocks.Continuous;
  import Dynawo.Electrical.SystemBase;

  parameter String InputSignalType = "speed" "Input signal selector (informational)";
  parameter Real A1 "Notch first-order coefficient";
  parameter Real A2 "Notch second-order coefficient";
  parameter Types.PerUnit Ks "PSS main gain";
  parameter Types.Time t1 "First lead time constant in s";
  parameter Types.Time t2 "First lag time constant in s";
  parameter Types.Time t3 "Second lead time constant in s";
  parameter Types.Time t4 "Second lag time constant in s";
  parameter Types.Time t5 "Washout numerator time constant in s";
  parameter Types.Time t6 "Washout denominator time constant in s";
  parameter Types.Time TDelay "Output transport delay (approximated as lag) in s";
  parameter Types.VoltageModulePu Vcl "Lower terminal voltage cutoff (informational)";
  parameter Types.VoltageModulePu Vcu "Upper terminal voltage cutoff (informational)";
  parameter Types.VoltageModulePu VrMaxPu;
  parameter Types.VoltageModulePu VrMinPu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput UPssPu(start = 0);

  Modelica.Blocks.Math.Feedback dW;
  Continuous.TransferFunction washout(a = {t6, 1}, b = {t5, 0}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain ksGain(k = Ks);
  Continuous.TransferFunction leadLag1(a = {t2, 1}, b = {t1, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Continuous.TransferFunction leadLag2(a = {t4, 1}, b = {t3, 1}, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder delay(T = max(TDelay, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VrMaxPu, uMin = VrMinPu);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, washout.u);
  connect(washout.y, ksGain.u);
  connect(ksGain.y, leadLag1.u);
  connect(leadLag1.y, leadLag2.u);
  connect(leadLag2.y, delay.u);
  connect(delay.y, outLim.u);
  connect(outLim.y, UPssPu);

  annotation(preferredView = "text");
end Pss1A;
