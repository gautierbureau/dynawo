within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model OverexcLimIEEE "IEEE overexcitation limiter (CGMES OverexcLimIEEE)"
  /*
    IEEE Std 421.5 overexcitation limiter with timed inverse-time pickup.
    Input is the rotor (field) current IfdPu. When Ifd exceeds Ifdlim the
    excess (with Hyst deadband) is integrated at rate Kramp. The integrator
    output (clamped to non-negative) feeds a Kcd gain producing the
    negative UOelPu signal that is OR-ed into the AVR error summation /
    output limit. Ifdmax is the absolute upper field current limit and
    Itfpu is the timed-field-current threshold (informational here; the
    simplified topology uses Ifdlim as the single pickup threshold).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Hyst "Hysteresis around the pickup threshold";
  parameter Types.CurrentModulePu IfdLim "Continuous-field-current limit";
  parameter Types.CurrentModulePu IfdMax "Instantaneous field-current limit (informational)";
  parameter Types.CurrentModulePu Itfpu "Timed field current threshold (informational)";
  parameter Types.PerUnit Kcd "Output gain (signed; CGMES sign convention: negative reduces AVR output)";
  parameter Types.PerUnit Kramp "Integrator ramp gain in pu/s";

  Modelica.Blocks.Interfaces.RealInput IfdPu(start = 0) "Rotor field current in pu";
  Modelica.Blocks.Interfaces.RealOutput UOelPu(start = 0) "OEL output (added to AVR with negative sign in CGMES convention)";

  Modelica.Blocks.Math.Feedback err "Ifd - IfdLim";
  Modelica.Blocks.Sources.Constant ifdLimRef(k = IfdLim);
  Modelica.Blocks.Nonlinear.DeadZone hysteresis(uMax = Hyst, uMin = -Hyst);
  Modelica.Blocks.Nonlinear.Limiter positivePart(uMax = Modelica.Constants.inf, uMin = 0, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.LimIntegrator ramp(outMax = max(1, IfdMax - IfdLim), outMin = 0, y_start = 0, k = Kramp, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain kcdGain(k = Kcd);

equation
  connect(IfdPu, err.u1);
  connect(ifdLimRef.y, err.u2);
  connect(err.y, hysteresis.u);
  connect(hysteresis.y, positivePart.u);
  connect(positivePart.y, ramp.u);
  connect(ramp.y, kcdGain.u);
  connect(kcdGain.y, UOelPu);

  annotation(preferredView = "text");
end OverexcLimIEEE;
