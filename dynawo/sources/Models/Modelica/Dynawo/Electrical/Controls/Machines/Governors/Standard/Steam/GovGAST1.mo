within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovGAST1 "Detailed gas turbine governor (CGMES GovGAST1)"
  /*
    Same backbone as Gast (speed error -> 1/R droop -> valve limit Vmax/Vmin
    with ambient temperature constraint via At + Kt feedback through T2 / T3
    cascade) extended with a temperature-control loop: Fpv * Ka *
    (T_exhaust_setpoint - T_exhaust) acting through Tltr / Tac / Tv shaping
    blocks, plus a fuel-valve bias B. T4 and T5 are extra fuel valve servo
    lags. Pm output includes -Dturb * (omega - omegaRef) damping.
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit At;
  parameter Types.PerUnit B "Fuel valve bias";
  parameter Types.PerUnit DTurb;
  parameter Types.PerUnit Fpv "Temperature control gain";
  parameter Types.PerUnit Ka "Temperature regulator gain";
  parameter Types.PerUnit Kt;
  parameter Types.PerUnit R;
  parameter Types.Time t1;
  parameter Types.Time t2;
  parameter Types.Time t3;
  parameter Types.Time t4;
  parameter Types.Time t5;
  parameter Types.Time tAc "Air control time constant";
  parameter Types.Time tLtr "Load-temperature reference time constant";
  parameter Types.Time tV "Temperature valve time constant";
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
  Dynawo.NonElectrical.Blocks.NonLinear.Min2 minTherm;
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder valve(K = 1, tFilter = t1, Y0 = Pm0Pu, YMax = VMax, YMin = VMin);
  Modelica.Blocks.Continuous.FirstOrder bowl(T = t2, y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder reheat(T = t3, y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add add1(k1 = -Kt, k2 = Kt + 1);
  Modelica.Blocks.Sources.Constant atConst(k = At);
  Modelica.Blocks.Math.Gain damp(k = DTurb);
  Modelica.Blocks.Math.Feedback fbOut;
  Modelica.Blocks.Continuous.FirstOrder fuelValve4(T = max(t4, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder fuelValve5(T = max(t5, 1e-6), y_start = Pm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Sources.Constant biasB(k = B);
  Modelica.Blocks.Math.Add addBias;
  Modelica.Blocks.Math.Gain tempCtrl(k = Fpv * Ka);
  Modelica.Blocks.Continuous.FirstOrder tempLtr(T = max(tLtr, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder tempAc(T = max(tAc, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder tempV(T = max(tV, 1e-6), y_start = 0, initType = Modelica.Blocks.Types.Init.SteadyState);

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(fb1.y, minTherm.u2);
  connect(atConst.y, add1.u2);
  connect(reheat.y, add1.u1);
  connect(add1.y, tempLtr.u);
  connect(tempLtr.y, tempCtrl.u);
  connect(tempCtrl.y, tempAc.u);
  connect(tempAc.y, tempV.u);
  connect(tempV.y, minTherm.u1);
  connect(minTherm.y, addBias.u1);
  connect(biasB.y, addBias.u2);
  connect(addBias.y, valve.u);
  connect(valve.y, fuelValve4.u);
  connect(fuelValve4.y, fuelValve5.u);
  connect(fuelValve5.y, bowl.u);
  connect(bowl.y, reheat.u);
  connect(bowl.y, fbOut.u1);
  connect(dW.y, damp.u);
  connect(damp.y, fbOut.u2);
  connect(fbOut.y, PmPu);

  annotation(preferredView = "text");
end GovGAST1;
