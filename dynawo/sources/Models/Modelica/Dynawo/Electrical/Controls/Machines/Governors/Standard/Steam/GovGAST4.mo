within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovGAST4 "Alternative gas turbine governor (CGMES GovGAST4)"
  /*
    Same Gast backbone (1/R droop, T1 / T2 / T3 cascade, At + Kt ambient
    temperature limit, Vmax / Vmin valve clamp, Dturb damping) extended
    with a Bp / Tr permanent-droop integrator and rate limits rLimMax /
    rLimMin on the valve velocity.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit At;
  parameter Types.PerUnit Bp "Bias on permanent-droop integrator";
  parameter Types.PerUnit DTurb;
  parameter Types.PerUnit Kt;
  parameter Types.PerUnit R;
  parameter Types.PerUnit RLimMax "Maximum valve rate in pu/s";
  parameter Types.PerUnit RLimMin "Minimum valve rate in pu/s";
  parameter Types.Time t1;
  parameter Types.Time t2;
  parameter Types.Time t3;
  parameter Types.Time tR "Permanent-droop integrator time constant in s";
  parameter Types.PerUnit VMax;
  parameter Types.PerUnit VMin;

  parameter Types.ActivePowerPu Pm0Pu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k2 = -1);
  Modelica.Blocks.Math.Gain droop(k = 1 / R);
  Modelica.Blocks.Math.Feedback fb1;
  Modelica.Blocks.Math.Gain bpDroop(k = Bp);
  Modelica.Blocks.Continuous.Integrator droopInteg(k = 1 / max(tR, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Feedback fb2;
  Dynawo.NonElectrical.Blocks.NonLinear.Min2 minTherm;
  Modelica.Blocks.Nonlinear.Limiter rateLim(uMax = RLimMax, uMin = RLimMin, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder valve(K = 1, tFilter = t1, Y0 = Pm0Pu, YMax = VMax, YMin = VMin);
  Modelica.Blocks.Continuous.FirstOrder bowl(T = t2, y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder reheat(T = t3, y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add add1(k1 = -Kt, k2 = Kt + 1);
  Modelica.Blocks.Sources.Constant atConst(k = At);
  Modelica.Blocks.Math.Gain damp(k = DTurb);
  Modelica.Blocks.Math.Feedback fbOut;

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(bowl.y, bpDroop.u);
  connect(bpDroop.y, droopInteg.u);
  connect(fb1.y, fb2.u1);
  connect(droopInteg.y, fb2.u2);
  connect(fb2.y, rateLim.u);
  connect(atConst.y, add1.u2);
  connect(reheat.y, add1.u1);
  connect(rateLim.y, minTherm.u2);
  connect(add1.y, minTherm.u1);
  connect(minTherm.y, valve.u);
  connect(valve.y, bowl.u);
  connect(bowl.y, reheat.u);
  connect(bowl.y, fbOut.u1);
  connect(dW.y, damp.u);
  connect(damp.y, fbOut.u2);
  connect(fbOut.y, PmPu);

  annotation(preferredView = "text");
end GovGAST4;
