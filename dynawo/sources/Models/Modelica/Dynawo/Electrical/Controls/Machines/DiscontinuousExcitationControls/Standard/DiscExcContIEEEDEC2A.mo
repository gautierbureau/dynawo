within Dynawo.Electrical.Controls.Machines.DiscontinuousExcitationControls.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model DiscExcContIEEEDEC2A "IEEE Std 421.5-2016 Clause 14.3 Type DEC2A discontinuous excitation controller (CGMES DiscExcContIEEEDEC2A)"
  /*
    IEEE Std 421.5-2016 Clause 14.3 Type DEC2A. Open-loop transient
    excitation boosting. A user-defined step VkPu is selected by switch
    Sw1 (on / off externally via VkTrigger), then conditioned by a small
    lag 1 / (1 + sTd1Pu) and a washout sTd2Pu / (1 + sTd2Pu) which
    produces a decaying pulse VdPu that temporarily raises generator
    terminal voltage. A clamp on VdPu (VdMin, VdMax) caps the pulse; the
    standard's bumpless freeze on overvoltage is approximated here by a
    static clamp on VtPu via the VtMax / VtMin limits already enforced
    by the calling AVR.

    Output Vs1Pu = VdPu + VstPu is the signal sent to the AVR summing
    junction (PSS output VstPu is forwarded through).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit VkPu "Step amplitude for the trigger pulse in pu";
  parameter Types.Time tD1 "Lag time constant in s";
  parameter Types.Time tD2 "Washout time constant in s";
  parameter Types.VoltageModulePu VdMaxPu "Upper limit on the decaying pulse";
  parameter Types.VoltageModulePu VdMinPu "Lower limit on the decaying pulse";

  Modelica.Blocks.Interfaces.RealInput VkTrigger(start = 0) "Trigger signal (1 = pulse, 0 = idle) - selects VkPu via switch Sw1";
  Modelica.Blocks.Interfaces.RealInput VstPu(start = 0) "PSS output signal in pu";
  Modelica.Blocks.Interfaces.RealOutput Vs1Pu(start = 0) "DEC2A output (PSS + decaying pulse) in pu";

  Modelica.Blocks.Continuous.FirstOrder lag(T = max(tD1, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.Derivative washout(T = max(tD2, 1e-6), k = max(tD2, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VdMaxPu, uMin = VdMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  // Switch Sw1: select VkPu when triggered, 0 otherwise. VkTrigger is
  // expected to be a 0 / 1 signal from a remote contingency detector.
  lag.u = VkPu * VkTrigger;
  connect(lag.y, washout.u);
  connect(washout.y, outLim.u);
  Vs1Pu = outLim.y + VstPu;

  annotation(preferredView = "text");
end DiscExcContIEEEDEC2A;
