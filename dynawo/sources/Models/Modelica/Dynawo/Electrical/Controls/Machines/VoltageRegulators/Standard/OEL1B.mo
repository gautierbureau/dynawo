within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model OEL1B "IEEE overexcitation limiter type 1B (CGMES OEL1B)"
  /*
    IEEE Std 421.5-2016 / 2024 Annex F overexcitation limiter type 1B.
    Unlike the simpler OverexcLimIEEE (single threshold, linear ramp,
    direct linear output), OEL1B implements:
      - a winding timer on the excess Ifd - IfdH (linear ramp here -
        the inverse-square winding from the standard is approximated by
        the choice of Kramp);
      - a latched 'engage' flag set when the timer reaches 1 and reset
        only when Ifd drops below the lower threshold IfdL (hysteresis);
      - a proportional output that activates only after the latch is
        set:  UOel = clamp(Koel * max(Ifd - IfdRef, 0), VrMin, VrMax).
    Koel is negative in the CGMES sign convention so that adding UOel
    to the AVR error pushes the field down. IfdH is the inverse-time
    pickup threshold, IfdL the lower reset threshold, Kramp the timer
    winding rate (1 / time-to-trip at err = 1 pu), IfdRef the field
    current the OEL drives Ifd back toward, and VrMax / VrMin the
    output clamp.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Hyst "Pickup deadband above IfdH";
  parameter Types.CurrentModulePu IfdH "High (pickup) field-current threshold";
  parameter Types.CurrentModulePu IfdL "Low (reset) field-current threshold";
  parameter Types.CurrentModulePu IfdRef "Field current the OEL drives Ifd toward when engaged";
  parameter Types.PerUnit Koel "Output gain (negative in CGMES convention)";
  parameter Types.PerUnit Kramp "Timer winding rate (1 / time-to-trip at err = 1 pu)";
  parameter Types.VoltageModulePu VrMaxPu "Output upper limit";
  parameter Types.VoltageModulePu VrMinPu "Output lower limit";

  Modelica.Blocks.Interfaces.RealInput IfdPu(start = 0) "Rotor field current in pu";
  Modelica.Blocks.Interfaces.RealOutput UOelPu(start = 0) "OEL output (added to AVR with negative sign in CGMES convention)";

  Modelica.Blocks.Math.Feedback err "IfdPu - IfdH";
  Modelica.Blocks.Sources.Constant ifdHRef(k = IfdH);
  Modelica.Blocks.Nonlinear.DeadZone hysteresis(uMax = Hyst, uMin = -Hyst) "Pickup deadband";
  Modelica.Blocks.Math.Abs absExcess "|err - dead-band|";
  Modelica.Blocks.Math.Add positivePart(k1 = 0.5, k2 = 0.5) "Smooth max(u, 0) = (u + |u|) / 2";
  Modelica.Blocks.Continuous.LimIntegrator timer(outMax = 1, outMin = 0, y_start = 0, k = Kramp, initType = Modelica.Blocks.Types.Init.SteadyState) "Time accumulator (1 = trip)";
  Modelica.Blocks.Logical.GreaterEqualThreshold trip(threshold = 1) "Timer reached trip level";
  Modelica.Blocks.Logical.LessThreshold reset(threshold = IfdL) "Ifd dropped below reset threshold";
  Modelica.Blocks.Logical.RSFlipFlop engage(Qini = false) "Latched engage flag (set on trip, reset when Ifd < IfdL)";
  Modelica.Blocks.Math.BooleanToReal engageReal(realTrue = 1, realFalse = 0);
  Modelica.Blocks.Math.Feedback outErr "IfdPu - IfdRef";
  Modelica.Blocks.Sources.Constant ifdSetRef(k = IfdRef);
  Modelica.Blocks.Math.Abs absOutErr "|Ifd - IfdRef|";
  Modelica.Blocks.Math.Add outPositive(k1 = 0.5, k2 = 0.5) "Smooth max(IfdPu - IfdRef, 0)";
  Modelica.Blocks.Math.Gain koelGain(k = Koel);
  Modelica.Blocks.Math.Product gate "Multiply OEL action by the engage flag";
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VrMaxPu, uMin = VrMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(IfdPu, err.u1);
  connect(ifdHRef.y, err.u2);
  connect(err.y, hysteresis.u);
  connect(hysteresis.y, absExcess.u);
  connect(hysteresis.y, positivePart.u1);
  connect(absExcess.y, positivePart.u2);
  connect(positivePart.y, timer.u);
  connect(timer.y, trip.u);
  connect(IfdPu, reset.u);
  connect(trip.y, engage.S);
  connect(reset.y, engage.R);
  connect(engage.Q, engageReal.u);
  connect(IfdPu, outErr.u1);
  connect(ifdSetRef.y, outErr.u2);
  connect(outErr.y, absOutErr.u);
  connect(outErr.y, outPositive.u1);
  connect(absOutErr.y, outPositive.u2);
  connect(outPositive.y, koelGain.u);
  connect(koelGain.y, gate.u1);
  connect(engageReal.y, gate.u2);
  connect(gate.y, outLim.u);
  connect(outLim.y, UOelPu);

  annotation(preferredView = "text");
end OEL1B;
