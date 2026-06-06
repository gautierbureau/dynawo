within Dynawo.Electrical.Controls.Machines.StatorCurrentLimiters.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model SCIEEEOperationalLimiter "IEEE Std 421.5-2016 Annex H operational stator current limiter (CGMES SCIEEEOperationalLimiter)"
  /*
    IEEE Std 421.5-2005/2016 Annex H 'Operational' stator current
    limiter (CGMES SCIEEEOperationalLimiter). Compared with the full
    Scl1c / Scl2c models (which need the complex stator current, the
    reactive power, the over-/under-excited dispatch and an init model)
    this is the simpler classical SCL exposed as a stand-alone block:

      - input  ISclPu      = stator-current magnitude (pu)
      - output VSclPu      = AVR voltage bias (pu, negative sign added
                             to the AVR error summation so the AVR pulls
                             the field down once the current exceeds the
                             limit)

    Topology:
      excess = max(ISclPu - ISclLimPu, 0)  (smooth via Abs + 0.5/0.5 add,
                                            same workaround as OEL1B to
                                            avoid Limiter(0, inf) init
                                            sign-ambiguity)
      filt   = lowpass(K * excess, tScl)
      VSclPu = clamp(filt, VSclMin, VSclMax)
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.CurrentModulePu ISclLimPu "Stator-current pick-up limit in pu";
  parameter Types.PerUnit K "Loop gain";
  parameter Types.Time tScl "Output low-pass time constant in s";
  parameter Types.VoltageModulePu VSclMaxPu "Output upper limit";
  parameter Types.VoltageModulePu VSclMinPu "Output lower limit";

  Modelica.Blocks.Interfaces.RealInput ISclPu(start = 0) "Stator-current magnitude in pu";
  Modelica.Blocks.Interfaces.RealOutput VSclPu(start = 0) "AVR voltage bias output in pu";

  Modelica.Blocks.Math.Feedback err "ISclPu - ISclLimPu";
  Modelica.Blocks.Sources.Constant iLimRef(k = ISclLimPu);
  Modelica.Blocks.Math.Abs absErr "|err|";
  Modelica.Blocks.Math.Add positivePart(k1 = 0.5, k2 = 0.5) "Smooth max(err, 0) = (err + |err|) / 2";
  Modelica.Blocks.Math.Gain kGain(k = K);
  Modelica.Blocks.Continuous.FirstOrder lowpass(T = max(tScl, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VSclMaxPu, uMin = VSclMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);

equation
  connect(ISclPu, err.u1);
  connect(iLimRef.y, err.u2);
  connect(err.y, absErr.u);
  connect(err.y, positivePart.u1);
  connect(absErr.y, positivePart.u2);
  connect(positivePart.y, kGain.u);
  connect(kGain.y, lowpass.u);
  connect(lowpass.y, outLim.u);
  connect(outLim.y, VSclPu);

  annotation(preferredView = "text");
end SCIEEEOperationalLimiter;
