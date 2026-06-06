within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcAVR7 "European AVR model 7 (CGMES ExcAVR7)"
  /*
    Six-stage cascaded lead-lag AVR with gain and limiter on the odd-indexed
    stages (1, 3, 5). Each stage i implements (1 + s.Ai) / (1 + s.Ti), with
    a multiplicative gain Kx on stage x in {1, 3, 5} and an output limiter
    [VminX, VmaxX]. Error path: UsRef + UPss - Us drives stage 1. Chain
    output is Efd directly. DC gain at steady state is K1 * K3 * K5.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Real A1;
  parameter Real A2;
  parameter Real A3;
  parameter Real A4;
  parameter Real A5;
  parameter Real A6;
  parameter Types.PerUnit K1;
  parameter Types.PerUnit K3;
  parameter Types.PerUnit K5;
  parameter Types.Time T1;
  parameter Types.Time T2;
  parameter Types.Time T3;
  parameter Types.Time T4;
  parameter Types.Time T5;
  parameter Types.Time T6;
  parameter Types.VoltageModulePu VMax1Pu;
  parameter Types.VoltageModulePu VMax3Pu;
  parameter Types.VoltageModulePu VMax5Pu;
  parameter Types.VoltageModulePu VMin1Pu;
  parameter Types.VoltageModulePu VMin3Pu;
  parameter Types.VoltageModulePu VMin5Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction ll1(a = {T1, 1}, b = {K1 * A1, K1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Stage1In0Pu);
  Modelica.Blocks.Nonlinear.Limiter lim1(uMax = VMax1Pu, uMin = VMin1Pu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction ll2(a = {T2, 1}, b = {A2, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Stage2In0Pu);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction ll3(a = {T3, 1}, b = {K3 * A3, K3}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Stage3In0Pu);
  Modelica.Blocks.Nonlinear.Limiter lim3(uMax = VMax3Pu, uMin = VMin3Pu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction ll4(a = {T4, 1}, b = {A4, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Stage4In0Pu);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction ll5(a = {T5, 1}, b = {K5 * A5, K5}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Stage5In0Pu);
  Modelica.Blocks.Nonlinear.Limiter lim5(uMax = VMax5Pu, uMin = VMin5Pu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction ll6(a = {T6, 1}, b = {A6, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Efd0Pu);

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;

  final parameter Types.PerUnit DcGain = max(K1 * K3 * K5, 1e-9) "Chain DC gain";
  final parameter Types.VoltageModulePu Stage1In0Pu = Efd0Pu / DcGain;
  final parameter Types.VoltageModulePu Stage2In0Pu = Efd0Pu / max(K3 * K5, 1e-9);
  final parameter Types.VoltageModulePu Stage3In0Pu = Stage2In0Pu;
  final parameter Types.VoltageModulePu Stage4In0Pu = Efd0Pu / max(K5, 1e-9);
  final parameter Types.VoltageModulePu Stage5In0Pu = Stage4In0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = Stage1In0Pu + Us0Pu;

equation
  connect(UsRefPu, errIn.u1);
  connect(UsPu, errIn.u2);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, ll1.u);
  connect(ll1.y, lim1.u);
  connect(lim1.y, ll2.u);
  connect(ll2.y, ll3.u);
  connect(ll3.y, lim3.u);
  connect(lim3.y, ll4.u);
  connect(ll4.y, ll5.u);
  connect(ll5.y, lim5.u);
  connect(lim5.y, ll6.u);
  connect(ll6.y, EfdPu);

  annotation(preferredView = "text");
end ExcAVR7;
