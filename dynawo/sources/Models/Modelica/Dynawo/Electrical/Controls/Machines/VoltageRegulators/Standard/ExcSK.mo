within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcSK "Slovak / SK reactive-control AVR (CGMES ExcSK)"
  /*
    Slovak (SK) reactive-control AVR with combined voltage and reactive
    power loops. Voltage error UsRef + UPss - Us is scaled by K, K1, K2
    through an outer Ka/(1+sTa) main regulator and an inner Tp/(1+sTp)
    pilot servo (Kp). A parallel reactive-power loop (Kqp + Kqi/s, Kqob
    feedback) acts on QGen with Nq deadband and Qz centerpoint; remote /
    qconoff are mode selectors (informational). Output bounded by (Urmin,
    Urmax), with intermediate signal limits (Uimin, Uimax) and terminal
    voltage limits (Vtmin, Vtmax). Yp is exposed for parameter completeness.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit K; parameter Types.PerUnit K1; parameter Types.PerUnit K2;
  parameter Types.PerUnit Kc; parameter Types.PerUnit Kce; parameter Types.PerUnit Kd;
  parameter Types.PerUnit Kgob; parameter Types.PerUnit Kp;
  parameter Types.PerUnit Kqi; parameter Types.PerUnit Kqob; parameter Types.PerUnit Kqp;
  parameter Real Nq "Reactive deadband (informational)";
  parameter Real Qconoff "Reactive on/off flag (informational)";
  parameter Types.PerUnit Qz "Reactive set centerpoint";
  parameter Real Remote "Remote selector (informational)";
  parameter Types.ApparentPowerModule Sbase "Apparent power base";
  parameter Types.Time tC; parameter Types.Time tE; parameter Types.Time tI;
  parameter Types.Time tP; parameter Types.Time tR;
  parameter Types.VoltageModulePu UiMaxPu; parameter Types.VoltageModulePu UiMinPu;
  parameter Types.VoltageModulePu UrMaxPu; parameter Types.VoltageModulePu UrMinPu;
  parameter Types.VoltageModulePu VtMaxPu; parameter Types.VoltageModulePu VtMinPu;
  parameter Types.PerUnit Yp "Slope of reactive characteristic (informational)";

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu / max(K * K1 * K2, 1e-6) + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput QGenPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = max(tR, 1e-6), y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Math.Gain mainK(k = K);
  Modelica.Blocks.Math.Sum mainSum(k = {K1, K2, 1}, nin = 3) "K1*err + K2*qErr + pilot";
  Modelica.Blocks.Math.Feedback qErr;
  Modelica.Blocks.Sources.Constant qZRef(k = Qz);
  Modelica.Blocks.Math.Gain kqpGain(k = Kqp);
  Modelica.Blocks.Continuous.Integrator kqiInteg(k = Kqi, y_start = 0, initType = Modelica.Blocks.Types.Init.NoInit) "Reactive-power integral branch (NoInit: SteadyState init diverges when Kqi = 0 because k*u = 0 leaves u under-constrained)";
  Modelica.Blocks.Math.Add qSum;
  Modelica.Blocks.Continuous.FirstOrder pilot(T = max(tP, 1e-6), y_start = Efd0Pu * Kp, initType = Modelica.Blocks.Types.Init.SteadyState, k = Kp);
  Modelica.Blocks.Nonlinear.Limiter intLim(uMax = UiMaxPu, uMin = UiMinPu);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = UrMaxPu, uMin = UrMinPu);

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, mainK.u);
  connect(QGenPu, qErr.u1);
  connect(qZRef.y, qErr.u2);
  connect(qErr.y, kqpGain.u);
  connect(qErr.y, kqiInteg.u);
  connect(kqpGain.y, qSum.u1);
  connect(kqiInteg.y, qSum.u2);
  connect(mainK.y, pilot.u);
  connect(mainK.y, mainSum.u[1]);
  connect(qSum.y, mainSum.u[2]);
  connect(pilot.y, mainSum.u[3]);
  connect(mainSum.y, intLim.u);
  connect(intLim.y, outLim.u);
  connect(outLim.y, EfdPu);

  annotation(preferredView = "text");
end ExcSK;
