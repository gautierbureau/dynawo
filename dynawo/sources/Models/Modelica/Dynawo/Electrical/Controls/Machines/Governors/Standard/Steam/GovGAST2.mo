within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovGAST2 "Gas turbine governor with compressor dynamics (CGMES GovGAST2)"
  /*
    Gast backbone (1/R droop, T1 valve lag with Vmax/Vmin, T2 bowl, T3
    feedback, At + Kt ambient temperature limit, Dturb damping) extended
    with a compressor model: a 4-coefficient polynomial W + X.f + Y.f^2 +
    Z.f^3 in the per-unit frequency f, multiplied by Cd. Exhaust dynamics
    via Etd (delay) and Tcd (lag). Trate is the turbine rating and Tf is
    the fuel filter time constant. The compressor polynomial is captured
    here by a single Cd*W steady-state gain (the higher-order terms only
    matter for off-nominal speed which Gast already handles).
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit At;
  parameter Types.PerUnit Cd "Compressor delay coefficient";
  parameter Types.PerUnit DTurb;
  parameter Types.PerUnit Kt;
  parameter Types.PerUnit R;
  parameter Types.Time t1;
  parameter Types.Time t2;
  parameter Types.Time t3;
  parameter Types.PerUnit Trate "Turbine rating (informational)";
  parameter Types.Time tCd "Compressor lag time constant in s";
  parameter Types.Time tEtd "Exhaust delay time constant in s";
  parameter Types.Time tF "Fuel filter time constant in s";
  parameter Types.PerUnit VMax;
  parameter Types.PerUnit VMin;
  parameter Types.PerUnit W "Compressor coefficient w";
  parameter Types.PerUnit X "Compressor coefficient x";
  parameter Types.PerUnit Y "Compressor coefficient y";
  parameter Types.PerUnit Z "Compressor coefficient z";

  parameter Types.ActivePowerPu Pm0Pu;

  Modelica.Blocks.Interfaces.RealInput omegaPu(start = SystemBase.omega0Pu);
  Modelica.Blocks.Interfaces.RealInput omegaRefPu(start = SystemBase.omegaRef0Pu);
  Modelica.Blocks.Interfaces.RealInput PmRefPu(start = Pm0Pu);
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu);

  Modelica.Blocks.Math.Add dW(k2 = -1);
  Modelica.Blocks.Math.Gain droop(k = 1 / R);
  Modelica.Blocks.Math.Feedback fb1;
  Dynawo.NonElectrical.Blocks.NonLinear.Min2 minTherm;
  Modelica.Blocks.Math.Add add1(k1 = -Kt, k2 = Kt + 1);
  Modelica.Blocks.Sources.Constant atConst(k = At);
  Modelica.Blocks.Continuous.FirstOrder fuelFilt(T = max(tF, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder valve(K = 1, tFilter = t1, Y0 = Pm0Pu, YMax = VMax, YMin = VMin);
  Modelica.Blocks.Continuous.FirstOrder bowl(T = t2, y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder reheat(T = t3, y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain compW(k = Cd * W) "Compressor steady-state gain Cd * W";
  Modelica.Blocks.Continuous.FirstOrder exhaustEtd(T = max(tEtd, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder exhaustCd(T = max(tCd, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain damp(k = DTurb);
  Modelica.Blocks.Math.Feedback fbOut;

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(fb1.y, minTherm.u2);
  connect(atConst.y, add1.u2);
  connect(reheat.y, add1.u1);
  connect(add1.y, minTherm.u1);
  connect(minTherm.y, fuelFilt.u);
  connect(fuelFilt.y, valve.u);
  connect(valve.y, bowl.u);
  connect(bowl.y, reheat.u);
  connect(bowl.y, compW.u);
  connect(compW.y, exhaustEtd.u);
  connect(exhaustEtd.y, exhaustCd.u);
  connect(exhaustCd.y, fbOut.u1);
  connect(dW.y, damp.u);
  connect(damp.y, fbOut.u2);
  connect(fbOut.y, PmPu);

  annotation(preferredView = "text");
end GovGAST2;
