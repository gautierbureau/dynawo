within Dynawo.Electrical.Controls.Machines.ReactivePowerControlLoops;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model PFVArType2IEEEVArController "IEEE Std 421.5-2016 Clause 11 Type-2 VAr controller (CGMES PFVArType2IEEEVArController)"
  /*
    IEEE Std 421.5-2016 Clause 11 Type-2 VAr controller. Same topology
    as PFVArType1IEEEVArController but with the Type-2 slower-bias time
    constant Tw (typical 30 s) approximating the periodic-sample
    behaviour of the original Type-2 implementation.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.ReactivePowerPu QRef "Reference reactive power in pu (generator convention)";
  parameter Types.PerUnit Vbw "Reactive-power error half-dead-band";
  parameter Types.Time Tw "Voltage adjuster time constant in s";
  parameter Types.VoltageModulePu VrMaxPu "Output upper limit";
  parameter Types.VoltageModulePu VrMinPu "Output lower limit";

  Modelica.Blocks.Interfaces.RealInput QGenPu(start = QRef) "Generator reactive power in pu (generator convention)";
  Modelica.Blocks.Interfaces.RealOutput VVarPu(start = 0) "AVR voltage bias output in pu";

  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = Vbw, uMin = -Vbw);
  Modelica.Blocks.Continuous.LimIntegrator integ(outMax = VrMaxPu, outMin = VrMinPu, y_start = 0, k = 1 / max(Tw, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);

equation
  deadband.u = QRef - QGenPu;

  connect(deadband.y, integ.u);
  connect(integ.y, VVarPu);

  annotation(preferredView = "text");
end PFVArType2IEEEVArController;
