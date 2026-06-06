within Dynawo.Electrical.Controls.Machines.ReactivePowerControlLoops;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model VoltageAdjusterIEEE "IEEE Std 421.5-2016 Clause 11 voltage-reference slewing adjuster (CGMES VoltageAdjusterIEEE)"
  /*
    IEEE Std 421.5-2016 Clause 11 voltage adjuster. The block tracks an
    external Vref command at a bounded slew rate AdjSlew (pu / s) and
    clamps the result to (VrMin, VrMax). It is the continuous-time
    approximation of the original IEEE adjuster's periodic ±Vstep
    increments: AdjSlew = Vstep / Tlevel reproduces the same average
    slope. Typically driven by a raise / lower signal from a PF or VAr
    controller, but exposed here as a standalone block (CGMES uses it as
    a stand-alone reusable component).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit AdjSlew "Voltage adjustment slew rate in pu/s";
  parameter Types.VoltageModulePu VrMaxPu "Output upper limit";
  parameter Types.VoltageModulePu VrMinPu "Output lower limit";
  parameter Types.VoltageModulePu VRef0 "Initial voltage adjustment";

  Modelica.Blocks.Interfaces.RealInput VRefIn(start = VRef0) "Voltage reference command in pu";
  Modelica.Blocks.Interfaces.RealOutput VRefOut(start = VRef0) "Slewed voltage reference in pu";

  Modelica.Blocks.Nonlinear.SlewRateLimiter slew(Rising = AdjSlew, Falling = -AdjSlew, y_start = VRef0, initType = Modelica.Blocks.Types.Init.InitialState);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VrMaxPu, uMin = VrMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(VRefIn, slew.u);
  connect(slew.y, outLim.u);
  connect(outLim.y, VRefOut);

  annotation(preferredView = "text");
end VoltageAdjusterIEEE;
