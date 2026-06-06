within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model UnderexcLimX1 "Extended underexcitation limiter variant 1 (CGMES UnderexcLimX1)"
  /*
    Simplified underexcitation limiter variant 1 (single-pole filter with
    a Km gain on a melting-element signal Mel). Takes the rotor / field
    current IfdPu as a proxy for the underexcitation indicator. Km / Tm
    shape the loop; Kf2 / Tf2 add a feedback lead-lag; Melmax clamps the
    output.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit K "Main gain";
  parameter Types.PerUnit Kf2 "Feedback gain";
  parameter Types.PerUnit Km "Mel-element gain";
  parameter Types.PerUnit MelMax "Output upper limit";
  parameter Types.Time tF2;
  parameter Types.Time tM;

  Modelica.Blocks.Interfaces.RealInput IfdPu(start = 0);
  Modelica.Blocks.Interfaces.RealOutput UUelPu(start = 0);

  Modelica.Blocks.Math.Gain kGain(k = K);
  Modelica.Blocks.Math.Feedback fb;
  Modelica.Blocks.Math.Gain kmGain(k = Km);
  Modelica.Blocks.Continuous.FirstOrder mLag(T = max(tM, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder fbLag(T = max(tF2, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain kf2Gain(k = Kf2);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = MelMax, uMin = 0);

equation
  connect(IfdPu, kGain.u);
  connect(kGain.y, fb.u1);
  connect(fb.y, kmGain.u);
  connect(kmGain.y, mLag.u);
  connect(mLag.y, fbLag.u);
  connect(fbLag.y, kf2Gain.u);
  connect(kf2Gain.y, fb.u2);
  connect(mLag.y, outLim.u);
  connect(outLim.y, UUelPu);

  annotation(preferredView = "text");
end UnderexcLimX1;
