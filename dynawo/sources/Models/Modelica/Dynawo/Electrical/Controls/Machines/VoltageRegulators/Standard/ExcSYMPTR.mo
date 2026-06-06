within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcSYMPTR "Simple Italian Symptr AVR (CGMES ExcSYMPTR)"
  /*
    Minimal AVR: Vt sensed through 1/(1+sTr), error UsRef + UPss - UsFilt
    clamped to (Vrmin, Vrmax), driving EfdPu directly. Efmx / Efmn / Vopi /
    Vres are exposed for documentation but the simplified topology here
    just enforces (Vrmin, Vrmax) on the output.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.VoltageModulePu EfMaxPu;
  parameter Types.VoltageModulePu EfMinPu;
  parameter Types.Time tR;
  parameter Types.VoltageModulePu VopiPu "Operational integral set (informational)";
  parameter Types.VoltageModulePu VresPu "Reference voltage set (informational)";
  parameter Types.VoltageModulePu VrMaxPu;
  parameter Types.VoltageModulePu VrMinPu;

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;

  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Nonlinear.Limiter outLim(uMax = VrMaxPu, uMin = VrMinPu);

equation
  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, outLim.u);
  connect(outLim.y, EfdPu);

  annotation(preferredView = "text");
end ExcSYMPTR;
