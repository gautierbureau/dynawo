within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAVR5 "European simplified rotating exciter AVR (CGMES ExcAVR5)"
  /*
    Single-time-constant rotating exciter AVR. The terminal voltage filter
    1/(1+sTr) feeds the error UsRef + UPss - UsFilt - Rex*Efd into the
    regulator Ka/(1+sTa), whose output is Efd directly. Rex acts as a
    proportional inner feedback from the field output (representing the
    self-excited rotating exciter).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Ka;
  parameter Types.PerUnit Rex;
  parameter Types.Time tA;
  parameter Types.Time tR;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Math.Feedback rexFbk;
  Modelica.Blocks.Math.Gain rexGain(k = Rex);
  Modelica.Blocks.Continuous.FirstOrder regulator(T = tA, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState, k = Ka);

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;

  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu / Ka + Rex * Efd0Pu + Us0Pu "Initial reference stator voltage: at steady state errIn - Rex*Efd0 = Efd0/Ka (Rex feedback is subtracted before the Ka/(1+sTa) lag, so its inverse contribution to UsRef is Rex*Efd0, not Rex*Efd0/Ka)";

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rexFbk.u1);
  connect(EfdPu, rexGain.u);
  connect(rexGain.y, rexFbk.u2);
  connect(rexFbk.y, regulator.u);
  connect(regulator.y, EfdPu);

  annotation(preferredView = "text");
end ExcAVR5;
