within Dynawo.Electrical.Controls.Machines.ReactivePowerControlLoops;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model PFVArType1IEEEPFController "IEEE Std 421.5-2016 Clause 11 Type-1 power factor controller (CGMES PFVArType1IEEEPFController)"
  /*
    IEEE Std 421.5-2016 Clause 11 Type-1 power factor controller.

    The block senses the generator's active and reactive power, computes
    the actual power factor cos(phi) = P / sqrt(P^2 + Q^2), compares it
    to PfRef, then feeds the dead-banded error into a 1 / (sTpf)
    integrator clamped to (VrMin, VrMax). The integrator output VPfPu is
    added to the AVR error summation so the AVR drives Q toward the
    value matching the requested power factor.

    Sign convention: PGenPu and QGenPu use the generator convention
    (positive when the machine delivers active / reactive power), PfRef
    is a positive cos(phi) reference in (0, 1], and VPfPu is the AVR
    bias.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit PfRef "Reference power factor (cos phi)";
  parameter Types.PerUnit Vbw "Power-factor error half-dead-band";
  parameter Types.Time tPf "Integrator time constant in s";
  parameter Types.VoltageModulePu VrMaxPu "Output upper limit";
  parameter Types.VoltageModulePu VrMinPu "Output lower limit";

  Modelica.Blocks.Interfaces.RealInput PGenPu(start = 0) "Generator active power in pu (generator convention)";
  Modelica.Blocks.Interfaces.RealInput QGenPu(start = 0) "Generator reactive power in pu (generator convention)";
  Modelica.Blocks.Interfaces.RealOutput VPfPu(start = 0) "AVR voltage bias output in pu";

  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = Vbw, uMin = -Vbw);
  Modelica.Blocks.Continuous.LimIntegrator integ(outMax = VrMaxPu, outMin = VrMinPu, y_start = 0, k = 1 / max(tPf, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);

  Real pfActual "Computed cos(phi)";

equation
  // cos(phi) = P / sqrt(P^2 + Q^2), guarded against P = Q = 0. noEvent on
  // max suppresses the spurious zero-crossing event the guard would
  // otherwise trigger during global initialisation.
  pfActual = PGenPu / sqrt(noEvent(max(PGenPu * PGenPu + QGenPu * QGenPu, 1e-12)));
  deadband.u = PfRef - pfActual;

  connect(deadband.y, integ.u);
  connect(integ.y, VPfPu);

  annotation(preferredView = "text");
end PFVArType1IEEEPFController;
