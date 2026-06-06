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

model GeneratorRoundRotorTypeF_INIT "Initialization model for the GENTPF round-rotor synchronous machine"
  extends Dynawo.Electrical.Machines.BaseClasses_INIT.BaseGeneratorParameters_INIT;
  extends AdditionalIcons.Init;

  import Modelica.ComplexMath;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit RaPu "Armature resistance in pu (base SNom)";
  parameter Types.PerUnit XdPu "d-axis synchronous reactance in pu (base SNom)";
  parameter Types.PerUnit XpdPu "d-axis transient reactance in pu (base SNom)";
  parameter Types.PerUnit XppdPu "d-axis subtransient reactance in pu (base SNom)";
  parameter Types.PerUnit XqPu "q-axis synchronous reactance in pu (base SNom)";
  parameter Types.PerUnit XpqPu "q-axis transient reactance in pu (base SNom)";
  parameter Types.PerUnit XppqPu "q-axis subtransient reactance in pu (base SNom)";
  parameter Types.PerUnit XlPu "Leakage reactance in pu (base SNom)";
  parameter Types.PerUnit Se1Pu "Saturation factor at 1.0 pu air-gap voltage";
  parameter Types.PerUnit Se12Pu "Saturation factor at 1.2 pu air-gap voltage";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power of the generator in MVA";

  final parameter Real kk = SystemBase.SnRef / SNom "Base change factor for currents from SnRef to SNom";
  final parameter Real satIntermediate = if Se1Pu > 0 then sqrt(Se12Pu * 1.2 / Se1Pu) else 0 "Saturation fit intermediate";
  final parameter Real satA = if Se1Pu > 0 then (satIntermediate - 1.2) / (satIntermediate - 1) else 0 "Saturation knee";
  final parameter Real satB = if Se1Pu > 0 then Se1Pu / (1 - satA) ^ 2 else 0 "Saturation coefficient";

  Types.ComplexVoltagePu psiAg0Cpx "Air-gap voltage phasor in the network frame in pu (base UNom)";
  Types.ComplexVoltagePu eqTrick0Pu "Voltage behind the saturated q-axis synchronous reactance, in pu (base UNom)";
  Types.PerUnit psiAg0Pu "Air-gap flux magnitude in pu (base UNom)";
  Types.PerUnit Sat0Pu "Saturation function value S(psiAg) at the initial operating point";
  Types.PerUnit satD0Pu "d-axis saturation factor at the initial operating point";
  Types.PerUnit satQ0Pu "q-axis saturation factor at the initial operating point";
  Types.PerUnit Xqeff0Pu "Saturated q-axis synchronous reactance used to locate the q-axis, in pu (base SNom)";
  Types.PerUnit Xdsat0Pu "Saturated d-axis subtransient reactance at the initial operating point in pu (base SNom)";
  Types.PerUnit Xqsat0Pu "Saturated q-axis subtransient reactance at the initial operating point in pu (base SNom)";
  Types.PerUnit Vd0Pu "d-axis terminal voltage at the initial operating point in pu (base UNom)";
  Types.PerUnit Vq0Pu "q-axis terminal voltage at the initial operating point in pu (base UNom)";

  Dynawo.Connectors.AngleConnector Theta0 "Start value of the rotor angle in rad";
  Dynawo.Connectors.PerUnitConnector Id0 "Start value of the d-axis stator current in pu (base SNom)";
  Dynawo.Connectors.PerUnitConnector Iq0 "Start value of the q-axis stator current in pu (base SNom)";
  Dynawo.Connectors.PerUnitConnector EqP0Pu "Start value of the q-axis transient internal EMF in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector EdP0Pu "Start value of the d-axis transient internal EMF in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector EqPP0Pu "Start value of the q-axis subtransient internal EMF in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector EdPP0Pu "Start value of the d-axis subtransient internal EMF in pu (base UNom)";
  Dynawo.Connectors.VoltageModulePuConnector Efd0Pu "Start value of the field voltage in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector Pm0Pu "Start value of the mechanical power in pu (base SNom)";

equation
  // Air-gap voltage phasor, computed in the network reference frame (frame independent)
  psiAg0Cpx = u0Pu - Complex(RaPu, XlPu) * i0Pu * kk;
  psiAg0Pu = ComplexMath.'abs'(psiAg0Cpx);

  // Saturation function S(psiAg) (scaled-quadratic), and SatD / SatQ factors
  Sat0Pu = satB * (psiAg0Pu - satA) * (psiAg0Pu - satA) / psiAg0Pu;
  satD0Pu = 1 + Sat0Pu;
  satQ0Pu = 1 + XqPu / XdPu * Sat0Pu;

  // Locate the q-axis using the voltage behind the saturated synchronous q-axis reactance
  Xqeff0Pu = (XqPu - XlPu) / satQ0Pu + XlPu;
  eqTrick0Pu = u0Pu - Complex(RaPu, Xqeff0Pu) * i0Pu * kk;
  Theta0 = ComplexMath.arg(eqTrick0Pu);

  // Park transformation of the operating point onto the rotor d-q frame
  Vd0Pu = u0Pu.re * sin(Theta0) - u0Pu.im * cos(Theta0);
  Vq0Pu = u0Pu.re * cos(Theta0) + u0Pu.im * sin(Theta0);
  Id0 = (-i0Pu.re * sin(Theta0) + i0Pu.im * cos(Theta0)) * kk;
  Iq0 = (-i0Pu.re * cos(Theta0) - i0Pu.im * sin(Theta0)) * kk;

  // Saturated subtransient reactances
  Xdsat0Pu = (XppdPu - XlPu) / satD0Pu + XlPu;
  Xqsat0Pu = (XppqPu - XlPu) / satQ0Pu + XlPu;

  // Stator algebraic equations solved for the subtransient EMFs (omega = 1 pu at SS)
  EqPP0Pu = Vq0Pu + RaPu * Iq0 + Xdsat0Pu * Id0;
  EdPP0Pu = Vd0Pu + RaPu * Id0 - Xqsat0Pu * Iq0;

  // Subtransient flux dynamics at SS give the transient EMFs
  EqP0Pu = EqPP0Pu + (XpdPu - XlPu) * Id0 / satD0Pu;
  EdP0Pu = EdPP0Pu - (XpqPu - XlPu) * Iq0 / satQ0Pu;

  // Field voltage that holds the operating point (from dEq'/dt = 0)
  Efd0Pu = satD0Pu * ((XdPu - XppdPu) / (XpdPu - XppdPu) * EqP0Pu - (XdPu - XpdPu) / (XpdPu - XppdPu) * EqPP0Pu);

  // Mechanical power balances the electrical power at steady state
  Pm0Pu = PGen0Pu * SystemBase.SnRef / SNom;

  annotation(preferredView = "text");
end GeneratorRoundRotorTypeF_INIT;
