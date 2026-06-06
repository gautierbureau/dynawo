within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcHU "Hungarian AVR (CGMES ExcHU)"
  /*
    Hungarian AVR with parallel voltage and current limit branches. The
    voltage branch uses Ke (gain) and Te (time constant); the current
    branch uses Ki (gain), Ti (time constant), Imax/Imin limits and Ai
    pickup gain. Ae is the voltage-pickup gain. Outputs of both branches
    are bounded to (Emin, Emax) and (Imin, Imax) respectively, then
    combined; Tr is the input filter; Atr is the alarm-trip threshold
    (informational).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Ae "Voltage-error pickup gain";
  parameter Types.PerUnit Ai "Current-error pickup gain";
  parameter Types.PerUnit Atr "Alarm trip threshold (informational)";
  parameter Types.VoltageModulePu EMaxPu;
  parameter Types.VoltageModulePu EMinPu;
  parameter Types.CurrentModulePu IMaxPu;
  parameter Types.CurrentModulePu IMinPu;
  parameter Types.PerUnit Ke;
  parameter Types.PerUnit Ki;
  parameter Types.Time tE;
  parameter Types.Time tI;
  parameter Types.Time tR;

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu / max(Ke, 1e-6) + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealInput IfdPu(start = 0);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Feedback voltageErr;
  Modelica.Blocks.Math.Gain aeGain(k = Ae);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder voltageBranch(K = Ke, tFilter = tE, Y0 = Efd0Pu, YMax = EMaxPu, YMin = EMinPu);
  Modelica.Blocks.Math.Gain aiGain(k = Ai);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder currentBranch(K = Ki, tFilter = tI, Y0 = 0, YMax = IMaxPu, YMin = IMinPu);
  Modelica.Blocks.Math.Feedback combine;

equation
  connect(UsPu, uFilter.u);
  connect(UsRefPu, voltageErr.u1);
  connect(uFilter.y, voltageErr.u2);
  connect(voltageErr.y, aeGain.u);
  connect(aeGain.y, voltageBranch.u);
  connect(IfdPu, aiGain.u);
  connect(aiGain.y, currentBranch.u);
  connect(voltageBranch.y, combine.u1);
  connect(currentBranch.y, combine.u2);
  connect(combine.y, EfdPu);

  annotation(preferredView = "text");
end ExcHU;
