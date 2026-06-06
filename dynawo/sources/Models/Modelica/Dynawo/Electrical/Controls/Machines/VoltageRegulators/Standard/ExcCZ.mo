within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcCZ "Czech AVR (CGMES ExcCZ)"
  /*
    Czech AVR. Signal path: Vt error -> Ka/(1+sTa) regulator -> Tc lead/
    compensator -> output limited to (Vrmin, Vrmax). Self-excited 1/sTe
    integrator with Ke feedback, plus Krb backup proportional feedback
    on Efd. Kp / Tp form an inner pilot stage. Output clamped at Efdmax.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit EfdMaxPu;
  parameter Types.PerUnit Ka;
  parameter Types.PerUnit Ke;
  parameter Types.PerUnit Kp;
  parameter Types.PerUnit Krb;
  parameter Types.Time tA;
  parameter Types.Time tC;
  parameter Types.Time tE;
  parameter Types.Time tP;
  parameter Types.VoltageModulePu VrMaxPu;
  parameter Types.VoltageModulePu VrMinPu;

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu * (Ke + Krb) / Ka + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Efd0Pu * (Ke + Krb), YMax = VrMaxPu, YMin = VrMinPu);
  Modelica.Blocks.Continuous.FirstOrder pilot(T = max(tP, 1e-6), y_start = Efd0Pu * Kp, initType = Modelica.Blocks.Types.Init.SteadyState, k = Kp);
  Modelica.Blocks.Math.Feedback excIn;
  Modelica.Blocks.Continuous.Integrator excIntegrator(k = 1 / tE, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Gain keGain(k = Ke);
  Modelica.Blocks.Math.Gain krbGain(k = Krb);
  Modelica.Blocks.Math.Add satSum;
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = EfdMaxPu, uMin = 0);

equation
  connect(UsRefPu, errIn.u1);
  connect(UsPu, errIn.u2);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, regulator.u);
  connect(regulator.y, pilot.u);
  connect(pilot.y, excIn.u1);
  connect(satSum.y, excIn.u2);
  connect(excIn.y, excIntegrator.u);
  connect(excIntegrator.y, keGain.u);
  connect(excIntegrator.y, krbGain.u);
  connect(keGain.y, satSum.u1);
  connect(krbGain.y, satSum.u2);
  connect(excIntegrator.y, outLim.u);
  connect(outLim.y, EfdPu);

  annotation(preferredView = "text");
end ExcCZ;
