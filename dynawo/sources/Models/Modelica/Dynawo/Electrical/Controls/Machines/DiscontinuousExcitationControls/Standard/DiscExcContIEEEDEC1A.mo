within Dynawo.Electrical.Controls.Machines.DiscontinuousExcitationControls.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model DiscExcContIEEEDEC1A "IEEE Std 421.5-2016 Clause 14.2 Type DEC1A discontinuous excitation controller (CGMES DiscExcContIEEEDEC1A)"
  /*
    IEEE Std 421.5-2016 Clause 14.2 Type DEC1A. Transient excitation
    boosting with terminal voltage limiter. The block adds a boost VAN
    to the PSS signal VstPu during transients (when the terminal voltage
    drops below VtcPu) and independently limits terminal voltage via
    VOPu during overvoltage.

    Structural simplification for CGMES vendor-namespace coverage
    (standalone preassembled, no AVR coupling):

      - Speed washout: omegaPu - 1 -> sTw5 / (1 + sTw5) -> dwPu
        (state-only - kept available as the calculated variable dwPu
        for users who want to log it, but the standard's S1 latch
        wiring "Vt drop AND dw rise AND Ar at ceiling" cannot be
        reproduced in a standalone block; the simplified trigger is Vt
        drop only).
      - Voltage-drop gate: Limiter(uMin = 0, uMax = ...) on
        (VtcPu - VtPu), feeding the angle-boost path whenever VtPu
        falls below VtcPu (proportional, not latched).
      - Angle-integrator path: gated input -> FirstOrder lag tAn ->
        gain kAn -> clamp at (0, VanMaxPu) -> VAN. The FirstOrder lag
        is the faithful continuous surrogate of the standard's reset
        integrator (input gated to zero outside the active window).
      - Voltage-limiter path: smooth max(VtPu - VtLmtPu, 0) ->
        first-order lag tL2 with gain kEtl -> clamp at (0, VOmaxPu)
        -> VOPu.
      - Output: Vs1Pu = clamp(VstPu + VAN - VOPu, VsMinPu, VsMaxPu).

    Compared with the standard, the discrete relay S1, the AVR-ceiling
    arming condition, the speed-deviation arming condition, the
    voltage-limiter lead numerator (1 + sTL1) and the dual HV-gate are
    not represented; the continuous approximation captures the same
    first-order behaviour (transient field boost on voltage drop, fast
    negative bias on overvoltage).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.Time tW5 "Speed-washout time constant in s";
  parameter Types.VoltageModulePu VtcPu "Terminal-voltage drop threshold that arms the booster";
  parameter Types.PerUnit eSigmaC "Speed-deviation threshold (kept for parameter-set compatibility with the standard - not used in this simplified standalone variant)";
  parameter Types.Time tAn "Angle-integrator reset time constant in s";
  parameter Types.PerUnit kAn "Output gain on the angle-integrator state";
  parameter Types.VoltageModulePu VanMaxPu "Upper limit on the angle-boost term";
  parameter Types.VoltageModulePu VtLmtPu "Terminal-voltage limiter reference (overvoltage threshold)";
  parameter Types.Time tL2 "Voltage-limiter lag time constant in s";
  parameter Types.PerUnit kEtl "Voltage-limiter loop gain";
  parameter Types.VoltageModulePu VOmaxPu "Upper limit on the voltage-limiter term";
  parameter Types.VoltageModulePu VsMaxPu "Upper limit on the final output";
  parameter Types.VoltageModulePu VsMinPu "Lower limit on the final output";

  Modelica.Blocks.Interfaces.RealInput VtPu(start = 1) "Terminal voltage magnitude in pu";
  Modelica.Blocks.Interfaces.RealInput omegaPu(start = 1) "Machine speed in pu";
  Modelica.Blocks.Interfaces.RealInput VstPu(start = 0) "PSS output signal in pu";
  Modelica.Blocks.Interfaces.RealOutput Vs1Pu(start = 0) "DEC1A output (PSS + angle boost - voltage limiter) in pu";

  Modelica.Blocks.Continuous.Derivative speedWashout(T = max(tW5, 1e-6), k = max(tW5, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter vDropGate(uMax = 10, uMin = 0, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.FirstOrder angleInteg(T = max(tAn, 1e-6), k = 1, y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain anGain(k = kAn);
  Modelica.Blocks.Nonlinear.Limiter anClamp(uMax = VanMaxPu, uMin = 0, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Nonlinear.Limiter overVGate(uMax = 10, uMin = 0, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.FirstOrder limLag(T = max(tL2, 1e-6), k = kEtl, y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter limClamp(uMax = VOmaxPu, uMin = 0, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VsMaxPu, uMin = VsMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

  Real dwPu "Washed-out speed deviation (informational output)";

equation
  speedWashout.u = omegaPu - 1;
  dwPu = speedWashout.y;

  vDropGate.u = VtcPu - VtPu;
  connect(vDropGate.y, angleInteg.u);
  connect(angleInteg.y, anGain.u);
  connect(anGain.y, anClamp.u);

  overVGate.u = VtPu - VtLmtPu;
  connect(overVGate.y, limLag.u);
  connect(limLag.y, limClamp.u);

  outLim.u = VstPu + anClamp.y - limClamp.y;
  connect(outLim.y, Vs1Pu);

  annotation(preferredView = "text");
end DiscExcContIEEEDEC1A;
