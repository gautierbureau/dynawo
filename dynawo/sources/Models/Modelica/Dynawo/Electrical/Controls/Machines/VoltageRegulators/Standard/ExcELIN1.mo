within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* SPDX-License-Identifier: MPL-2.0
*/

model ExcELIN1 "ELIN type 1 AVR (CGMES ExcELIN1)"
  /*
    ELIN (Austrian) type 1 AVR. Voltage sensing through 1/(1+sTfi) and
    1/(1+sTnu); error UsRef + UPss - UsFilt - Dpnf gain on the deviation
    -> Ka gain -> two-stage filter (Ts1, Ts2) with Ks1 / Ks2 gains -> Vpu
    proportional output -> clamp to (Efmin, Efmax) -> EfdPu.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit Dpnf "Deviation gain";
  parameter Types.VoltageModulePu EfMaxPu;
  parameter Types.VoltageModulePu EfMinPu;
  parameter Types.PerUnit Ka;
  parameter Types.PerUnit Ks1;
  parameter Types.PerUnit Ks2;
  parameter Types.Time tFi "Input filter time constant in s";
  parameter Types.Time tNu "Voltage filter time constant in s";
  parameter Types.Time tS1;
  parameter Types.Time tS2;
  parameter Types.VoltageModulePu Vpu "Reference voltage";

  parameter Types.VoltageModulePu Efd0Pu;
  parameter Types.VoltageModulePu Us0Pu;
  final parameter Types.VoltageModulePu UsRef0Pu = Efd0Pu / max(Ka * Ks1 * Ks2, 1e-6) + Us0Pu;

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder fi(T = max(tFi, 1e-6), y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Continuous.FirstOrder nu(T = max(tNu, 1e-6), y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1);
  Modelica.Blocks.Math.Gain dpnfGain(k = Dpnf);
  Modelica.Blocks.Math.Gain kaGain(k = Ka);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder s1(K = Ks1, tFilter = tS1, Y0 = Efd0Pu / max(Ks2, 1e-6), YMax = EfMaxPu, YMin = EfMinPu);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder s2(K = Ks2, tFilter = tS2, Y0 = Efd0Pu, YMax = EfMaxPu, YMin = EfMinPu);

equation
  connect(UsPu, fi.u);
  connect(fi.y, nu.u);
  connect(nu.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, dpnfGain.u);
  connect(dpnfGain.y, kaGain.u);
  connect(kaGain.y, s1.u);
  connect(s1.y, s2.u);
  connect(s2.y, EfdPu);

  annotation(preferredView = "text");
end ExcELIN1;
