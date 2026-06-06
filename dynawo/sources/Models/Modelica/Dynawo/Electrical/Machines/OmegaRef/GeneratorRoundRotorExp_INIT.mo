within Dynawo.Electrical.Machines.OmegaRef;

/*
* Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.
*/

model GeneratorRoundRotorExp_INIT "Initialization model for the round-rotor synchronous machine with saturation"
  extends Dynawo.Electrical.Machines.BaseClasses_INIT.BaseGeneratorParameters_INIT;
  extends AdditionalIcons.Init;

  import Modelica.ComplexMath;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit RaPu "Armature resistance in pu (base SNom)";
  parameter Types.PerUnit XdPu "d-axis synchronous reactance in pu (base SNom)";
  parameter Types.PerUnit XpdPu "d-axis transient reactance in pu (base SNom)";
  parameter Types.PerUnit XqPu "q-axis synchronous reactance in pu (base SNom)";
  parameter Types.PerUnit XpqPu "q-axis transient reactance in pu (base SNom)";
  parameter Types.PerUnit XlPu "Leakage reactance in pu (base SNom)";
  parameter Types.PerUnit Se1Pu "Saturation factor at 1.0 pu air-gap voltage";
  parameter Types.PerUnit Se12Pu "Saturation factor at 1.2 pu air-gap voltage";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power of the generator in MVA";

  final parameter Real kk = SystemBase.SnRef / SNom "Base change factor for currents from SnRef to SNom";
  final parameter Real satIntermediate = if Se1Pu > 0 then sqrt(Se12Pu * 1.2 / Se1Pu) else 0 "Saturation fit intermediate";
  final parameter Real satA = if Se1Pu > 0 then (satIntermediate - 1.2) / (satIntermediate - 1) else 0 "Saturation knee";
  final parameter Real satB = if Se1Pu > 0 then Se1Pu / (1 - satA) ^ 2 else 0 "Saturation coefficient";

  Types.PerUnit Vd0Pu "d-axis terminal voltage at the initial operating point in pu (base UNom)";
  Types.PerUnit Vq0Pu(start = U0Pu) "q-axis terminal voltage at the initial operating point in pu (base UNom)";
  Types.PerUnit PsiAg0Pu(start = 1) "Air-gap flux magnitude at the initial operating point in pu (base UNom)";
  Types.PerUnit Se0Pu(start = 0) "Saturation factor at the initial operating point";
  // Unsaturated "voltage behind the q-axis reactance" estimate, computed from the load-flow parameters; used only as a start guess for the rotor angle
  final parameter Real u0ReTrick = U0Pu * cos(UPhase0) "Real part of the terminal voltage in pu (base UNom)";
  final parameter Real u0ImTrick = U0Pu * sin(UPhase0) "Imaginary part of the terminal voltage in pu (base UNom)";
  final parameter Real i0ReTrick = (P0Pu * u0ReTrick + Q0Pu * u0ImTrick) / (U0Pu * U0Pu) "Real part of the terminal current in pu (base UNom, SnRef, receptor convention)";
  final parameter Real i0ImTrick = (P0Pu * u0ImTrick - Q0Pu * u0ReTrick) / (U0Pu * U0Pu) "Imaginary part of the terminal current in pu (base UNom, SnRef, receptor convention)";
  final parameter Types.Angle ThetaTrick0 = Modelica.Math.atan2(u0ImTrick - (RaPu * i0ImTrick + XqPu * i0ReTrick) * kk, u0ReTrick - (RaPu * i0ReTrick - XqPu * i0ImTrick) * kk) "Unsaturated estimate of the rotor angle in rad";
  Dynawo.Connectors.AngleConnector Theta0(start = ThetaTrick0) "Start value of the rotor angle in rad";
  Dynawo.Connectors.PerUnitConnector Id0 "Start value of the d-axis stator current in pu (base SNom)";
  Dynawo.Connectors.PerUnitConnector Iq0 "Start value of the q-axis stator current in pu (base SNom)";
  Dynawo.Connectors.PerUnitConnector EqP0Pu(start = 1) "Start value of the q-axis transient internal EMF in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector EdP0Pu "Start value of the d-axis transient internal EMF in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector Psi1d0 "Start value of the d-axis subtransient damper flux in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector Psi2q0 "Start value of the q-axis subtransient damper flux in pu (base UNom)";
  Dynawo.Connectors.VoltageModulePuConnector Efd0Pu "Start value of the field voltage in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector Pm0Pu "Start value of the mechanical power in pu (base SNom)";

equation
  // d-q projection of the terminal voltage and current (machine base, generator convention)
  Vd0Pu = u0Pu.re * sin(Theta0) - u0Pu.im * cos(Theta0);
  Vq0Pu = u0Pu.re * cos(Theta0) + u0Pu.im * sin(Theta0);
  Id0 = (-i0Pu.re * sin(Theta0) + i0Pu.im * cos(Theta0)) * kk;
  Iq0 = (-i0Pu.re * cos(Theta0) - i0Pu.im * sin(Theta0)) * kk;

  // q-axis transient EMF (saturation does not enter this stator equation)
  EqP0Pu = Vq0Pu + RaPu * Iq0 + XpdPu * Id0;

  // q-axis transient EMF, reduced by saturation (written without division to keep the init residual well-defined)
  EdP0Pu * (1 + Se0Pu) = (XqPu - XpqPu) * Iq0;

  // Saturation factor at the initial operating point, from the full air-gap flux (as in the dynamic model);
  // written as Se0Pu * PsiAg0Pu = satB * (PsiAg0Pu - satA)^2 to avoid a division by PsiAg0Pu in the residual
  PsiAg0Pu = sqrt(EqP0Pu * EqP0Pu + EdP0Pu * EdP0Pu);
  Se0Pu * PsiAg0Pu = satB * (PsiAg0Pu - satA) * (PsiAg0Pu - satA);

  // d-axis stator equation at steady state; closes the system and locates the rotor angle Theta0 consistently with saturation
  Vd0Pu = - RaPu * Id0 + XpqPu * Iq0 + EdP0Pu;

  // Field voltage, modified by saturation
  Efd0Pu = (1 + Se0Pu) * EqP0Pu + (XdPu - XpdPu) * Id0;

  // Damper fluxes at steady state
  Psi1d0 = EqP0Pu - (XpdPu - XlPu) * Id0;
  Psi2q0 = - EdP0Pu - (XpqPu - XlPu) * Iq0;

  // Mechanical power balances the electrical power at steady state
  Pm0Pu = PGen0Pu * SystemBase.SnRef / SNom;

  annotation(preferredView = "text");
end GeneratorRoundRotorExp_INIT;
