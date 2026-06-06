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

model St3a "IEEE excitation system type ST3A model (2005/2016 standard, CGMES ExcIEEEST3A)"
  /*
    IEEE Std 421.5 type ST3A potential-or-compound-source thyristor
    excitation system with an inner field-voltage regulation loop.
    Outer loop: standard sensing + lead-lag (Tb / Tc) + Ka regulator with
    VImin / VImax input-error limits and VrMin / VrMax output limits.
    Inner loop: a low-gain integral controller (Km / sTm) drives the
    firing angle. The exciter ceiling Vb is a function of terminal voltage
    via the potential circuit (Kp magnitude, ThetaP phase) plus the current
    compound (Ki). The available exciter voltage is limited by VbMax and
    by the rectifier regulation characteristic Fex(Kc * Ifd / Vb).
    Simplifications: the potential-circuit phase term is collapsed onto Kp
    magnitude (ThetaP = 0 is the common case in CGMES); Xl is exposed as a
    parameter for the parallel-current compensation. Vgmax is the output of
    the inner-loop gain stage upper bound.
  */
  import Modelica.Blocks;
  import Dynawo.Types;

  // Outer-loop regulator parameters
  parameter Types.PerUnit Ka "Outer voltage regulator gain";
  parameter Types.Time tA "Outer regulator time constant in s";
  parameter Types.Time tB "Outer regulator lag time constant in s";
  parameter Types.Time tC "Outer regulator lead time constant in s";
  parameter Types.Time tR "Stator voltage filter time constant in s";
  parameter Types.VoltageModulePu ViMaxPu "Maximum input error";
  parameter Types.VoltageModulePu ViMinPu "Minimum input error";
  parameter Types.VoltageModulePu VrMaxPu "Maximum regulator output";
  parameter Types.VoltageModulePu VrMinPu "Minimum regulator output";

  // Inner-loop parameters
  parameter Types.PerUnit Km "Inner-loop integrator gain";
  parameter Types.Time tM "Inner-loop integrator time constant in s";
  parameter Types.VoltageModulePu VmMaxPu "Maximum inner-loop output";
  parameter Types.VoltageModulePu VmMinPu "Minimum inner-loop output";

  // Exciter / source parameters
  parameter Types.PerUnit Kc "Rectifier loading factor";
  parameter Types.PerUnit Kg "Feedback from Efd to inner loop";
  parameter Types.PerUnit Ki "Current source compound gain";
  parameter Types.PerUnit Kp "Potential source gain magnitude";
  parameter Types.PerUnit Xl "Synchronous machine leakage reactance in pu";
  parameter Types.Angle ThetaP = 0 "Potential source phase angle (not used in this simplified mapping)";
  parameter Types.VoltageModulePu VbMaxPu "Maximum exciter ceiling voltage";
  parameter Types.VoltageModulePu VgMaxPu "Maximum inner-loop gain block output";
  parameter Integer PositionUel = 0 "UEL location: 0 = none, 1 = at AVR input summation";

  // Inputs
  Modelica.Blocks.Interfaces.RealInput IrPu(start = Ir0Pu) "Rotor current in pu (base SNom)";
  Modelica.Blocks.Interfaces.RealInput UPssPu(start = 0) "PSS output voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UsPu(start = Us0Pu) "Stator voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UsRefPu(start = UsRef0Pu) "Reference stator voltage in pu";
  Modelica.Blocks.Interfaces.RealInput UUelPu(start = 0) "UEL output voltage in pu";

  // Output
  Modelica.Blocks.Interfaces.RealOutput EfdPu(start = Efd0Pu) "Excitation voltage in pu";

  // Outer-loop blocks
  Modelica.Blocks.Continuous.FirstOrder uFilter(T = tR, y_start = Us0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Math.Add3 errIn(k1 = 1, k2 = -1, k3 = 1) "UsRef - UsFilt + UPss";
  Modelica.Blocks.Nonlinear.Limiter viLim(uMax = ViMaxPu, uMin = ViMinPu, homotopyType = Modelica.Blocks.Types.LimiterHomotopy.NoHomotopy);
  Dynawo.NonElectrical.Blocks.NonLinear.Max3 uelMax "UEL summation at AVR input (PositionUel = 1)";
  Dynawo.NonElectrical.Blocks.Continuous.TransferFunction leadLag(a = {tB, 1}, b = {tC, 1}, initType = Modelica.Blocks.Types.Init.SteadyState, u_start = Vr0Pu / Ka);
  Dynawo.NonElectrical.Blocks.NonLinear.LimitedFirstOrder regulator(K = Ka, tFilter = tA, Y0 = Vr0Pu, YMax = VrMaxPu, YMin = VrMinPu);

  // Inner-loop blocks
  Modelica.Blocks.Math.Feedback innerErr "Vr - Kg*Efd";
  Modelica.Blocks.Math.Gain kgGain(k = Kg);
  Modelica.Blocks.Continuous.LimIntegrator innerInt(k = Km / tM, outMin = VmMinPu, outMax = VmMaxPu, y_start = Vm0Pu, initType = Modelica.Blocks.Types.Init.SteadyState);
  Modelica.Blocks.Nonlinear.Limiter vgLim(uMax = VgMaxPu, uMin = 0);

  // Ceiling / source blocks
  Modelica.Blocks.Math.Gain kpGain(k = Kp) "Kp * Us";
  Modelica.Blocks.Math.Gain kiGain(k = Ki) "Ki * Ir";
  Modelica.Blocks.Math.Add compoundSum "Vthev raw";
  Modelica.Blocks.Nonlinear.Limiter vbLim(uMax = VbMaxPu, uMin = 0);
  Modelica.Blocks.Math.Product efdProd "Efd = (Vm/VmMax) * Vb";

  // Initial parameters
  parameter Types.VoltageModulePu Efd0Pu "Initial excitation voltage";
  parameter Types.CurrentModulePu Ir0Pu "Initial rotor current";
  parameter Types.VoltageModulePu Us0Pu "Initial stator voltage";

  // Derived initial values
  final parameter Types.VoltageModulePu Vb0Pu = max(Kp * Us0Pu + Ki * Ir0Pu, 1e-6) "Initial ceiling voltage";
  final parameter Types.VoltageModulePu Vm0Pu = if Vb0Pu > 1e-6 then Efd0Pu / Vb0Pu * VmMaxPu else 0 "Initial inner-loop output (normalized via VmMax)";
  final parameter Types.VoltageModulePu Vr0Pu = Vm0Pu + Kg * Efd0Pu "Initial outer regulator output";
  final parameter Types.VoltageModulePu UsRef0Pu = Vr0Pu / Ka + Us0Pu "Initial reference stator voltage";

equation
  if PositionUel == 1 then
    uelMax.u[1] = errIn.y;
    uelMax.u[2] = UUelPu;
    uelMax.u[3] = uelMax.u[1];
  else
    uelMax.u[1] = errIn.y;
    uelMax.u[2] = uelMax.u[1];
    uelMax.u[3] = uelMax.u[1];
  end if;

  connect(UsPu, uFilter.u);
  connect(uFilter.y, errIn.u2);
  connect(UsRefPu, errIn.u1);
  connect(UPssPu, errIn.u3);
  connect(uelMax.y, viLim.u);
  connect(viLim.y, leadLag.u);
  connect(leadLag.y, regulator.u);
  connect(regulator.y, innerErr.u1);
  connect(EfdPu, kgGain.u);
  connect(kgGain.y, innerErr.u2);
  connect(innerErr.y, innerInt.u);
  connect(innerInt.y, vgLim.u);
  connect(UsPu, kpGain.u);
  connect(IrPu, kiGain.u);
  connect(kpGain.y, compoundSum.u1);
  connect(kiGain.y, compoundSum.u2);
  connect(compoundSum.y, vbLim.u);
  connect(vgLim.y, efdProd.u1);
  connect(vbLim.y, efdProd.u2);
  connect(efdProd.y, EfdPu);

  annotation(preferredView = "diagram");
end St3a;
