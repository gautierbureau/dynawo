within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAVR3 "European AVR model 3 (CGMES ExcAVR3) with high-field gain reduction"
  /*
    Same backbone as ExcAVR1 (Ka/(1+sTa) regulator with 1/(1+sTb) input lag,
    self-excited exciter integrator 1/sTe with exponential saturation
    feedback). The EfdN parameter caps the steady-state field voltage at the
    rated value - implemented here as a hard upper limit on the exciter
    integrator output rather than a gain reduction, matching typical PSS/E
    simplified handling.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit AEx;
  parameter Types.PerUnit BEx;
  parameter Types.VoltageModulePu EfdNPu "Rated (no-load) field voltage; integrator hard upper limit";
  parameter Types.PerUnit Ka;
  parameter Types.Time tA;
  parameter Types.Time tB;
  parameter Types.Time tE;
  parameter Types.Time tR;
  parameter Types.VoltageModulePu VrMaxPu;
  parameter Types.VoltageModulePu VrMinPu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Continuous.FirstOrder preLag(T = max(tB, 1e-6), y_start = Vr0Pu / Ka, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VrMaxPu, YMin = VrMinPu);
  Modelica.Blocks.Math.Feedback excIn;
  Modelica.Blocks.Continuous.LimIntegrator excIntegrator(k = 1 / tE, outMin = 0, outMax = EfdNPu, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Product satProd;
  Modelica.Blocks.Math.Gain expGain(k = AEx);
  Modelica.Blocks.Math.Power expBex(base = exp(BEx), useExp = true);

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;

  final parameter Types.VoltageModulePu Vr0Pu = Efd0Pu * AEx * exp(BEx * Efd0Pu);
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / Ka + Us0Pu;

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, preLag.u);
  connect(preLag.y, regulator.u);
  connect(regulator.y, excIn.u1);
  connect(satProd.y, excIn.u2);
  connect(excIn.y, excIntegrator.u);
  connect(excIntegrator.y, EfdPu);
  connect(excIntegrator.y, expBex.u);
  connect(excIntegrator.y, satProd.u1);
  connect(expBex.y, expGain.u);
  connect(expGain.y, satProd.u2);

  annotation(preferredView = "text");
end ExcAVR3;
