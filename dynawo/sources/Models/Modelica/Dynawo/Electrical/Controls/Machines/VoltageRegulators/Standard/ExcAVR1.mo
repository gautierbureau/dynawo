within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, You can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite
* of simulation tools for power systems.
*/

model ExcAVR1 "European simplified AVR model 1 (CGMES ExcAVR1)"
  /*
    Two-pole AVR with self-excited exciter integrator and exponential
    saturation. Topology: UsPu sensed through 1/(1+sTr) gives UsFilt; the
    error UsRef + UPss - UsFilt is shaped by 1/(1+sTb) then by the gain-
    time-constant regulator Ka/(1+sTa) and clamped to (Vrmn, Vrmx); the
    output drives a 1/(sTe) integrator with the saturation term
    Se(Efd)*Efd subtracted at the integrator input, where Se(Efd) =
    AEx*exp(BEx*Efd) is the exponential saturation function fitted through
    (E1, Se1) and (E2, Se2) externally (AEx, BEx provided in the par file).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  parameter Types.PerUnit AEx "Gain of saturation function";
  parameter Types.PerUnit BEx "Exponential coefficient of saturation function";
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Types.Time tA "Voltage regulator time constant in s";
  parameter Types.Time tB "Voltage regulator input lag time constant in s";
  parameter Types.Time tE "Exciter integrator time constant in s";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.VoltageModulePu VrMaxPu "Regulator output upper limit";
  parameter Types.VoltageModulePu VrMinPu "Regulator output lower limit";

  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0);
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu);
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu);
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu);

  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1) "UsRef - UsFilt + UPss";
  Modelica.Blocks.Continuous.FirstOrder preLag(T = max(tB, 1e-6), y_start = Vr0Pu / Ka, initType = Modelica.Blocks.Types.Init.SteadyState);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VrMaxPu, YMin = VrMinPu);
  Modelica.Blocks.Math.Feedback excIn "Vr - Se(Efd)*Efd";
  Modelica.Blocks.Continuous.Integrator excIntegrator(k = 1 / tE, y_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Product satProd "Efd * Se(Efd)";
  Modelica.Blocks.Math.Gain expGain(k = AEx);
  Modelica.Blocks.Math.Power expBex(base = exp(BEx), useExp = true);

  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage";

  final parameter Types.VoltageModulePu Vr0Pu = Efd0Pu * AEx * exp(BEx * Efd0Pu) "Initial regulator output (= saturation drop)";
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / Ka + Us0Pu "Initial reference stator voltage";

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
end ExcAVR1;
