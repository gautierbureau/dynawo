within Dynawo.Electrical.Controls.Machines.ReactivePowerControlLoops;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model PFVArType2IEEEPFController "IEEE Std 421.5-2016 Clause 11 Type-2 power factor controller (CGMES PFVArType2IEEEPFController)"
  /*
    IEEE Std 421.5-2016 Clause 11 Type-2 power factor controller. The
    block senses the generator's active and reactive power, computes
    cos(phi) = P / sqrt(P^2 + Q^2), feeds (PfRef - cos(phi)) through a
    Vbw dead-band, then a slow 1 / (sTw) integrator clamped to (VrMin,
    VrMax) drives the AVR voltage bias VPfPu. Same topology as
    PFVArType1IEEEPFController but with the Type-2 slower-bias time
    constant Tw (typical 30 s) approximating the periodic-sample
    behaviour of the original Type-2 implementation.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit PfRef "Reference power factor (cos phi)";
  parameter Types.PerUnit Vbw "Power-factor error half-dead-band";
  parameter Types.Time Tw "Voltage adjuster time constant in s";
  parameter Types.VoltageModulePu VrMaxPu "Output upper limit";
  parameter Types.VoltageModulePu VrMinPu "Output lower limit";

  Modelica.Blocks.Interfaces.RealInput PGenPu(start = 0) "Generator active power in pu (generator convention)";
  Modelica.Blocks.Interfaces.RealInput QGenPu(start = 0) "Generator reactive power in pu (generator convention)";
  Modelica.Blocks.Interfaces.RealOutput VPfPu(start = 0) "AVR voltage bias output in pu";

  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = Vbw, uMin = -Vbw);
  Modelica.Blocks.Continuous.LimIntegrator integ(outMax = VrMaxPu, outMin = VrMinPu, y_start = 0, k = 1 / max(Tw, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);

  Real pfActual "Computed cos(phi)";

equation
  pfActual = PGenPu / sqrt(noEvent(max(PGenPu * PGenPu + QGenPu * QGenPu, 1e-12)));
  deadband.u = PfRef - pfActual;

  connect(deadband.y, integ.u);
  connect(integ.y, VPfPu);

  annotation(preferredView = "text");
end PFVArType2IEEEPFController;
