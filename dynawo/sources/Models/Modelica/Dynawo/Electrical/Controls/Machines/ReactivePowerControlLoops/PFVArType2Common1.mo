within Dynawo.Electrical.Controls.Machines.ReactivePowerControlLoops;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model PFVArType2Common1 "IEEE Std 421.5-2016 Clause 11 Type-2 common voltage adjuster (CGMES PFVArType2Common1)"
  /*
    IEEE Std 421.5-2016 Clause 11 Type-2 common voltage adjuster. The
    Type-2 PF / VAr controllers share this slow voltage-bias block: the
    PF / VAr error feeds an outer dead-band and a slow 1 / (sTw)
    integrator clamped to (VrMin, VrMax). The integrator output VAdjPu is
    added to the AVR error summation so the AVR drives the regulated
    quantity (cos(phi) or Q) toward its set-point.

    Compared to PFVArType1IEEE* (Clause 11.1) which uses a faster
    continuous integrator (typical tPf = 5 s), the Type-2 family uses a
    slower bias adjustment (typical Tw = 30 s) reflecting the discrete
    periodic-sample behaviour of the original implementation.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Vbw "Error half-dead-band";
  parameter Types.Time Tw "Voltage adjuster time constant in s";
  parameter Types.VoltageModulePu VrMaxPu "Output upper limit";
  parameter Types.VoltageModulePu VrMinPu "Output lower limit";

  Modelica.Blocks.Interfaces.RealInput errPu(start = 0) "PF / VAr error input (set-point minus measured)";
  Modelica.Blocks.Interfaces.RealOutput VAdjPu(start = 0) "AVR voltage bias output in pu";

  Modelica.Blocks.Nonlinear.DeadZone deadband(uMax = Vbw, uMin = -Vbw);
  Modelica.Blocks.Continuous.LimIntegrator integ(outMax = VrMaxPu, outMin = VrMinPu, y_start = 0, k = 1 / max(Tw, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);

equation
  connect(errPu, deadband.u);
  connect(deadband.y, integ.u);
  connect(integ.y, VAdjPu);

  annotation(preferredView = "text");
end PFVArType2Common1;
