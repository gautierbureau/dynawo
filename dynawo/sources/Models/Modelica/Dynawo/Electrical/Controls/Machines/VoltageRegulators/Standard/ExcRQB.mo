within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcRQB "RQB AVR (CGMES ExcRQB)"
  /*
    RQB AVR. Stator-voltage sensing through 1/(1+sMesu); error UsRef -
    UsFilt clamped to (Ucmin, Ucmax). Ki integrator with output through
    a Tc lead and a (1+sTout)/(1+sTf) shaping stage. Lsmin / Clmt / T4m
    / Xp are exposed for documentation.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Clmt "Continuous-load monitor threshold";
  parameter Types.PerUnit Ki "Integrator gain";
  parameter Types.VoltageModulePu LsMinPu "Low-signal minimum";
  parameter Types.Time Mesu "Voltage sensing filter in s";
  parameter Types.Time t4m "Auxiliary filter time constant in s";
  parameter Types.Time tC "Output lead time constant in s";
  parameter Types.Time tF "Output filter time constant in s";
  parameter Types.Time tOut "Output stage time constant in s";
  parameter Types.VoltageModulePu UcMaxPu;
  parameter Types.VoltageModulePu UcMinPu;
  parameter Types.PerUnit Xp "Reactive compensation (informational)";

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = max(Mesu, 1e-6), y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Feedback err;
  Modelica.Blocks.Nonlinear.Limiter errLim(uMax = UcMaxPu, uMin = UcMinPu);
  Modelica.Blocks.Continuous.Integrator integ(k = Ki, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.NoInit) "Voltage error integrator (NoInit: SteadyState init diverges when Ki = 0 because k*u = 0 leaves u under-constrained)";
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction outShape(a = {tF, 1}, b = {tOut, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Efd0Pu);
  Modelica.Blocks.Continuous.FirstOrder tcLead(T = max(tC, 1e-6), y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);

equation
  connect(UsPu, uFilter.u);
  connect(UsRefPu, err.u1);
  connect(uFilter.y, err.u2);
  connect(err.y, errLim.u);
  connect(errLim.y, integ.u);
  connect(integ.y, outShape.u);
  connect(outShape.y, tcLead.u);
  connect(tcLead.y, EfdPu);

  annotation(preferredView = "text");
end ExcRQB;
