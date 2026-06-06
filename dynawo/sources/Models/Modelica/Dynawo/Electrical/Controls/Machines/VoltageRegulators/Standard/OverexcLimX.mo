within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model OverexcLimX "Extended overexcitation limiter (CGMES OverexcLimX)"
  /*
    Field-voltage based overexcitation limiter. Input is EfdPu (excitation
    voltage). When Efd exceeds EfdDes the excess goes through a three-pole
    cascade (1/(1+sT1)(1+sT2)(1+sT3)), is multiplied by Kmx and rate-
    limited by Srew before producing UOelPu. The breakpoints Efd1 / Efd2 /
    Efd3 describe an inverse-time pickup curve; the simplified topology
    here uses EfdDes as the single pickup threshold. Vlow is a low-
    voltage inhibit threshold on the terminal voltage Us - if Us < Vlow,
    the OEL output is held at zero (logic captured here by a simple gate).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.VoltageModulePu Efd1Pu "First breakpoint";
  parameter Types.VoltageModulePu Efd2Pu "Second breakpoint";
  parameter Types.VoltageModulePu Efd3Pu "Third breakpoint";
  parameter Types.VoltageModulePu EfdDesPu "Desired (target) excitation voltage";
  parameter Types.VoltageModulePu EfdRatedPu "Rated excitation voltage";
  parameter Types.PerUnit Kmx "Output gain";
  parameter Types.PerUnit Srew "Slew-rate limit on the output in pu/s";
  parameter Types.Time t1;
  parameter Types.Time t2;
  parameter Types.Time t3;
  parameter Types.VoltageModulePu VLowPu "Terminal voltage inhibit threshold";

  Modelica.Blocks.Interfaces.RealInput EfdPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = 1);
  Modelica.Blocks.Interfaces.RealOutput UOelPu(start = 0);

  Modelica.Blocks.Math.Feedback err;
  Modelica.Blocks.Sources.Constant efdDesRef(k = EfdDesPu);
  Modelica.Blocks.Nonlinear.Limiter positivePart(uMax = max(EfdRatedPu, 10), uMin = 0, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Modelica.Blocks.Continuous.FirstOrder lag1(T = max(t1, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder lag2(T = max(t2, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder lag3(T = max(t3, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain kmxGain(k = Kmx);
  Modelica.Blocks.Nonlinear.SlewRateLimiter slew(Rising = Srew, Falling = -Srew);
  Modelica.Blocks.Math.Product gate "Low-voltage inhibit gate";
  Modelica.Blocks.Nonlinear.Limiter gateLim(uMax = 1, uMin = 0) "Smooth low-voltage inhibit: 0 below Vlow, 1 above Vlow + 0.01";

equation
  connect(EfdPu, err.u1);
  connect(efdDesRef.y, err.u2);
  connect(err.y, positivePart.u);
  connect(positivePart.y, lag1.u);
  connect(lag1.y, lag2.u);
  connect(lag2.y, lag3.u);
  connect(lag3.y, kmxGain.u);
  connect(kmxGain.y, slew.u);
  // Smooth low-voltage inhibit: gate = clamp((Us - VLowPu) / 0.01, 0, 1)
  gateLim.u = (UsPu - VLowPu) / 0.01;
  connect(slew.y, gate.u1);
  connect(gateLim.y, gate.u2);
  connect(gate.y, UOelPu);

  annotation(preferredView = "text");
end OverexcLimX;
