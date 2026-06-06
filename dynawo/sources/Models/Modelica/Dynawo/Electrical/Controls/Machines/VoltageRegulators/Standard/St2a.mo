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

model St2a "IEEE excitation system type ST2A model (2005/2016 standard, CGMES ExcIEEEST2A)"
  /*
    IEEE Std 421.5 type ST2A compound-source rectifier excitation system.
    The exciter ceiling Vb is derived from a vector combination of terminal
    voltage (gain Kp) and rotor current (gain Ki). The classical Ka/(1+sTa)
    regulator drives an exciter time-constant integrator (1/sTe). The
    rectifier reg correction uses Kc and the ceiling Vb. Rate feedback
    Kf*s/(1+sTf) is taken from Efd. UEL takeover at the AVR input is
    selectable via the uelin flag in CGMES (modeled here as PositionUel = 2
    when present, 0 otherwise).
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  // Regulation parameters
  parameter Types.PerUnit EfdMaxPu "Maximum field voltage output (rectifier ceiling)";
  parameter Types.PerUnit Ka "Voltage regulator gain";
  parameter Types.PerUnit Kc "Rectifier loading factor";
  parameter Types.PerUnit Kf "Exciter rate feedback gain";
  parameter Types.PerUnit Ki "Current source gain factor";
  parameter Types.PerUnit Kp "Voltage source gain factor";
  parameter Integer PositionUel = 0 "UEL location: 0 = none, 2 = takeover at AVR input";
  parameter Types.Time tA "Voltage regulator time constant in s";
  parameter Types.Time tE "Exciter time constant in s";
  parameter Types.Time tF "Exciter rate feedback time constant in s";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.VoltageModulePu VrMaxPu "Maximum field voltage";
  parameter Types.VoltageModulePu VrMinPu "Minimum field voltage";

  // Inputs
  Modelica.Blocks.Interfaces.RealInput IrPu(start = Ir0Pu) "Rotor current in pu (base SNom)";
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0) "PSS output voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UUelPu(start = 0) "UEL output voltage in pu";

  // Output
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu";

  // Blocks
  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1) "UsRef - UsFilt + UPss";
  Modelica.Blocks.Math.Feedback rateFbk "Error - rate feedback";
  Dynawo.NonElectrical.Blocks.NonLinear.Max3 uelMax "UEL takeover at AVR input (PositionUel = 2)";
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VrMaxPu, YMin = VrMinPu);
  Modelica.Blocks.Math.Feedback excIn "Vr - Efd";
  Modelica.Blocks.Continuous.LimIntegrator excIntegrator(k = 1 / tE, outMin = 0, outMax = 1, y_start = Efd0Pu / Vb0Pu, initType = Modelica.Blocks.Types.Init.SteadyState) "Normalized integrator output [0..1]";
  Modelica.Blocks.Math.Gain compoundV(k = Kp) "Kp * Us (voltage source)";
  Modelica.Blocks.Math.Gain compoundI(k = Ki) "Ki * Ir (current source)";
  Modelica.Blocks.Math.Add compoundSum "Vb (raw)";
  Modelica.Blocks.Nonlinear.Limiter compoundLim(uMax = EfdMaxPu, uMin = 0) "Vb clamped to ceiling";
  Modelica.Blocks.Math.Product efdProd "Efd = normalized * Vb";
  Modelica.Blocks.Continuous.Derivative derivative(k = Kf, T = tF, x_start = Efd0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);

  // Initial parameters
  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage";
  parameter Types.CurrentModulePu Ir0Pu "Initial rotor current";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage";

  // Derived initial values
  final parameter Types.VoltageModulePu Vb0Pu = max(Kp * Us0Pu + Ki * Ir0Pu, 1e-6) "Initial ceiling voltage";
  final parameter Types.VoltageModulePu Vr0Pu = Efd0Pu "Initial regulator output";
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / Ka + Us0Pu "Initial reference stator voltage";

equation
  if PositionUel == 2 then
    uelMax.u[1] = rateFbk.y;
    uelMax.u[2] = UUelPu;
    uelMax.u[3] = uelMax.u[1];
  else
    uelMax.u[1] = rateFbk.y;
    uelMax.u[2] = uelMax.u[1];
    uelMax.u[3] = uelMax.u[1];
  end if;

  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(errIn.y, rateFbk.u1);
  connect(uelMax.y, regulator.u);
  connect(regulator.y, excIn.u1);
  connect(EfdPu, excIn.u2);
  connect(excIn.y, excIntegrator.u);
  connect(UsPu, compoundV.u);
  connect(IrPu, compoundI.u);
  connect(compoundV.y, compoundSum.u1);
  connect(compoundI.y, compoundSum.u2);
  connect(compoundSum.y, compoundLim.u);
  connect(compoundLim.y, efdProd.u1);
  connect(excIntegrator.y, efdProd.u2);
  connect(efdProd.y, EfdPu);
  connect(EfdPu, derivative.u);
  connect(derivative.y, rateFbk.u2);

  annotation(preferredView = "diagram");
end St2a;
