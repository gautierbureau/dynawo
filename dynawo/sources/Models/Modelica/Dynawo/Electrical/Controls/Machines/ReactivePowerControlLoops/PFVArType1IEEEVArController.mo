within Dynawo.Electrical.Controls.Machines.ReactivePowerControlLoops;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model PFVArType1IEEEVArController "IEEE Std 421.5-2016 Clause 11 Type-1 VAr controller (CGMES PFVArType1IEEEVArController)"
  /*
    IEEE Std 421.5-2016 Clause 11 Type-1 VAr controller. Same skeleton as
    the PF controller but targets a reactive-power set-point directly:
    the QRef - QGen error passes through a Vbw dead-band, then a 1/(sTpf)
    integrator clamped to (VrMin, VrMax) drives the AVR voltage bias
    VVarPu so the AVR pulls Q toward QRef.

    Sign convention: QGenPu uses the generator convention (positive when
    the machine delivers reactive power), QRef is in the same pu base,
    VVarPu is the AVR bias.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.ReactivePowerPu QRef "Reference reactive power in pu (generator convention)";
  parameter Types.PerUnit Vbw "Reactive-power error half-dead-band";
  parameter Types.Time tPf "Integrator time constant in s";
  parameter Types.VoltageModulePu VrMaxPu "Output upper limit";
  parameter Types.VoltageModulePu VrMinPu "Output lower limit";

  Modelica.Blocks.Interfaces.RealInput QGenPu(start = QRef) "Generator reactive power in pu (generator convention)";
  Modelica.Blocks.Interfaces.RealOutput VVarPu(start = 0) "AVR voltage bias output in pu";

  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = Vbw, uMin = -Vbw);
  Modelica.Blocks.Continuous.LimIntegrator integ(outMax = VrMaxPu, outMin = VrMinPu, y_start = 0, k = 1 / max(tPf, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);

equation
  deadband.u = QRef - QGenPu;

  connect(deadband.y, integ.u);
  connect(integ.y, VVarPu);

  annotation(preferredView = "text");
end PFVArType1IEEEVArController;
