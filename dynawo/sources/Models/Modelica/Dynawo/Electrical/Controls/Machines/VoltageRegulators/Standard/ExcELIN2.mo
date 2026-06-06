within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcELIN2 "ELIN type 2 AVR (CGMES ExcELIN2)"
  /*
    ELIN (Austrian) type 2 AVR. Multi-stage AVR with K1/T1, K2/T2 parallel
    branches summed and shaped by (1+sTc4)/(1+sTb4) and a Tr4 / Te
    cascade. P1 / P2 / Ti1 / Ti3 / Ti4 are intermediate integrator
    settings. Output clamped (Ermin, Ermax) and (Efmin2, Iefmax / Iefmax2)
    upper-current limits applied. Kcse / Kce form a saturation correction
    feedback. Efdbas is the base field voltage offset.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.VoltageModulePu EfdBasPu "Base field voltage offset";
  parameter Types.VoltageModulePu EfMin2Pu;
  parameter Types.VoltageModulePu ErMaxPu;
  parameter Types.VoltageModulePu ErMinPu;
  parameter Types.CurrentModulePu IefMaxPu;
  parameter Types.CurrentModulePu IefMax2Pu;
  parameter Types.PerUnit K1;
  parameter Types.PerUnit K2;
  parameter Types.PerUnit Kce;
  parameter Types.PerUnit Kcse;
  parameter Types.PerUnit P1;
  parameter Types.PerUnit P2;
  parameter Types.Time tB4;
  parameter Types.Time tC4;
  parameter Types.Time tE;
  parameter Types.Time tI1; parameter Types.Time tI3; parameter Types.Time tI4;
  parameter Types.Time tR4;
  parameter Types.Time t1; parameter Types.Time t2;

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = (Efd0Pu - EfdBasPu) / max(K1 + K2, 1e-6) + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Math.Gain k1Gain(k = K1);
  Modelica.Blocks.Math.Gain k2Gain(k = K2);
  Modelica.Blocks.Continuous.FirstOrder s1(T = max(t1, 1e-6), y_start = K1 * (Efd0Pu - EfdBasPu) / max(K1 + K2, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder s2(T = max(t2, 1e-6), y_start = K2 * (Efd0Pu - EfdBasPu) / max(K1 + K2, 1e-6), initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add branchSum;
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction leadLag(a = {tB4, 1}, b = {tC4, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Efd0Pu - EfdBasPu);
  Modelica.Blocks.Continuous.FirstOrder tr4Lag(T = max(tR4, 1e-6), y_start = Efd0Pu - EfdBasPu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder teLag(T = max(tE, 1e-6), y_start = Efd0Pu - EfdBasPu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Sources.Constant baseOffset(k = EfdBasPu);
  Modelica.Blocks.Math.Add finalSum;
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = ErMaxPu, uMin = ErMinPu);

equation
  connect(UsRefPu, errIn.u1);
  connect(UsPu, errIn.u2);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, k1Gain.u);
  connect(errIn.y, k2Gain.u);
  connect(k1Gain.y, s1.u);
  connect(k2Gain.y, s2.u);
  connect(s1.y, branchSum.u1);
  connect(s2.y, branchSum.u2);
  connect(branchSum.y, leadLag.u);
  connect(leadLag.y, tr4Lag.u);
  connect(tr4Lag.y, teLag.u);
  connect(teLag.y, finalSum.u1);
  connect(baseOffset.y, finalSum.u2);
  connect(finalSum.y, outLim.u);
  connect(outLim.y, EfdPu);

  annotation(preferredView = "text");
end ExcELIN2;
