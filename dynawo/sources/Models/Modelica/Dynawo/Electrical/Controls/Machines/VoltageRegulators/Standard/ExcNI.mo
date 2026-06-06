within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcNI "Simple NI AVR (CGMES ExcNI)"
  /*
    Generic AVR with a bus-fed/independent-fed selector. Signal path:
    UsPu through 1/(1+sTr) gives UsFilt; the error UsRef + UPss - UsFilt
    is shaped by 1/(1+sTb) and Ka/(1+sTa); the output is rate-limited
    via Te servo lag and clamped to (Vrmn, Vrmx).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Real BusFedSelector = 0 "Bus-fed selector (informational)";
  parameter Types.PerUnit Ka;
  parameter Types.Time tA;
  parameter Types.Time tB;
  parameter Types.Time tE;
  parameter Types.Time tR;
  parameter Types.VoltageModulePu VrMaxPu;
  parameter Types.VoltageModulePu VrMinPu;

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu / Ka + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Continuous.FirstOrder preLag(T = max(tB, 1e-6), y_start = Efd0Pu / Ka, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Efd0Pu, YMax = VrMaxPu, YMin = VrMinPu);
  Modelica.Blocks.Continuous.FirstOrder servoLag(T = max(tE, 1e-6), y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, preLag.u);
  connect(preLag.y, regulator.u);
  connect(regulator.y, servoLag.u);
  connect(servoLag.y, EfdPu);

  annotation(preferredView = "text");
end ExcNI;
