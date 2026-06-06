within Dynawo.Electrical.Controls.Machines.Governors.Standard.Steam;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model GovSteam0 "Simplified steam governor (CGMES GovSteam0)"
  /*
    Most basic steam governor. Speed error with 1/R droop drives a T1 first-
    order valve lag bounded Vmax/Vmin, followed by a (1 + sT2) / (1 + sT3)
    lead-reheater stage. Pm = output - Dt * (omega - omegaRef).
  */
  import Modelica.Blocks;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit Dt "Turbine damping coefficient";
  parameter Types.PerUnit R;
  parameter Types.Time t1;
  parameter Types.Time t2 "Lead time constant in s";
  parameter Types.Time t3 "Reheater time constant in s";
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
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder valve(K = 1, tFilter = t1, Y0 = Pm0Pu, YMax = VMax, YMin = VMin);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction reheat(a = {t3, 1}, b = {t2, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Pm0Pu);
  Modelica.Blocks.Math.Gain damp(k = Dt);
  Modelica.Blocks.Math.Feedback fbOut;

equation
  connect(omegaPu, dW.u1);
  connect(omegaRefPu, dW.u2);
  connect(dW.y, droop.u);
  connect(PmRefPu, fb1.u1);
  connect(droop.y, fb1.u2);
  connect(fb1.y, valve.u);
  connect(valve.y, reheat.u);
  connect(reheat.y, fbOut.u1);
  connect(dW.y, damp.u);
  connect(damp.y, fbOut.u2);
  connect(fbOut.y, PmPu);

  annotation(preferredView = "text");
end GovSteam0;
