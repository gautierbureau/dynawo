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

model GeneratorRoundRotorTypeF "Sixth-order round-rotor synchronous machine model with WECC air-gap-flux (GENTPF) saturation"
  /*
    Sixth-order round-rotor synchronous machine using the WECC GENTPF saturation
    treatment: instead of an additive saturation term applied only to the field
    voltage equation (GENROU/GENROE), saturation is applied multiplicatively to
    every flux state through factors SatD and SatQ that depend on the air-gap
    flux psiAgPu. The d- and q-axis subtransient reactances appearing in the
    network interface are themselves saturated. With Se1Pu = 0 the saturation
    vanishes and the dynamic equations reduce to GeneratorRoundRotor (modulo
    the GENTPF state-space parameterisation in Eq', Ed', Eq'', Ed'').
    Corresponds to CGMES IEC 61970-302 SynchronousMachineTimeConstantReactance
    with rotorKind = roundRotor and modelKind = subtransientTypeF (PSS/E GENTPF).
    Receptor convention; coupled to OmegaRef.
  */
  extends Dynawo.Electrical.Machines.BaseClasses.BaseGeneratorSimplified;
  extends AdditionalIcons.Machine;

  import Modelica.ComplexMath;
  import Dynawo.Types;
  import Dynawo.Connectors;
  import Dynawo.Electrical.SystemBase;

  // Parameters
  parameter Types.Time H "Inertia constant of the generator in s";
  parameter Types.PerUnit DPu "Damping coefficient of the generator in pu (base SNom, omegaNom)";
  parameter Types.PerUnit RaPu "Armature resistance in pu (base SNom)";
  parameter Types.PerUnit XdPu "d-axis synchronous reactance in pu (base SNom)";
  parameter Types.PerUnit XpdPu "d-axis transient reactance in pu (base SNom)";
  parameter Types.PerUnit XppdPu "d-axis subtransient reactance in pu (base SNom)";
  parameter Types.PerUnit XqPu "q-axis synchronous reactance in pu (base SNom)";
  parameter Types.PerUnit XpqPu "q-axis transient reactance in pu (base SNom)";
  parameter Types.PerUnit XppqPu "q-axis subtransient reactance in pu (base SNom)";
  parameter Types.PerUnit XlPu "Leakage reactance in pu (base SNom)";
  parameter Types.Time Tpd0 "d-axis open-circuit transient time constant in s";
  parameter Types.Time Tppd0 "d-axis open-circuit subtransient time constant in s";
  parameter Types.Time Tpq0 "q-axis open-circuit transient time constant in s";
  parameter Types.Time Tppq0 "q-axis open-circuit subtransient time constant in s";
  parameter Types.PerUnit Se1Pu "Saturation factor at 1.0 pu air-gap voltage";
  parameter Types.PerUnit Se12Pu "Saturation factor at 1.2 pu air-gap voltage";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power of the generator in MVA";

  // Inputs / outputs for the OmegaRef coupling and the regulations
  input Connectors.AngularVelocityPuConnector omegaRefPu(start = SystemBase.omegaRef0Pu) "Network angular reference frequency in pu (base omegaNom)";
  input Connectors.ActivePowerPuConnector PmPu(start = Pm0Pu) "Mechanical power in pu (base SNom)";
  input Connectors.VoltageModulePuConnector efdPu(start = Efd0Pu) "Field voltage in pu (base UNom)";
  Connectors.AngularVelocityPuConnector omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";

  // State variables
  Types.Angle theta(start = Theta0) "Rotor angle: angle of the q-axis from the network reference frame in rad";
  Types.PerUnit eqPPu(start = EqP0Pu) "q-axis transient internal EMF in pu (base UNom)";
  Types.PerUnit edPPu(start = EdP0Pu) "d-axis transient internal EMF in pu (base UNom)";
  Types.PerUnit eqPPPu(start = EqPP0Pu) "q-axis subtransient internal EMF in pu (base UNom)";
  Types.PerUnit edPPPu(start = EdPP0Pu) "d-axis subtransient internal EMF in pu (base UNom)";

  // Algebraic variables
  Types.PerUnit psiAgPu(start = PsiAg0Pu) "Air-gap flux magnitude in pu (base UNom)";
  Types.PerUnit satDPu(start = SatD0Pu) "d-axis saturation factor SatD = 1 + S(psiAg)";
  Types.PerUnit satQPu(start = SatQ0Pu) "q-axis saturation factor SatQ = 1 + (Xq/Xd)*S(psiAg)";
  Types.PerUnit XdsatPu(start = Xdsat0Pu) "Saturated d-axis subtransient reactance in pu (base SNom)";
  Types.PerUnit XqsatPu(start = Xqsat0Pu) "Saturated q-axis subtransient reactance in pu (base SNom)";
  Types.PerUnit Id(start = Id0) "d-axis stator current in pu (base SNom, generator convention)";
  Types.PerUnit Iq(start = Iq0) "q-axis stator current in pu (base SNom, generator convention)";
  Types.PerUnit Vd(start = Vd0Pu) "d-axis terminal voltage in pu (base UNom)";
  Types.PerUnit Vq(start = Vq0Pu) "q-axis terminal voltage in pu (base UNom)";

  // Start values calculated by the initialization model
  parameter Types.Angle Theta0 "Start value of the rotor angle in rad";
  parameter Types.PerUnit EqP0Pu "Start value of the q-axis transient internal EMF in pu (base UNom)";
  parameter Types.PerUnit EdP0Pu "Start value of the d-axis transient internal EMF in pu (base UNom)";
  parameter Types.PerUnit EqPP0Pu "Start value of the q-axis subtransient internal EMF in pu (base UNom)";
  parameter Types.PerUnit EdPP0Pu "Start value of the d-axis subtransient internal EMF in pu (base UNom)";
  parameter Types.VoltageModulePu Efd0Pu "Start value of the field voltage in pu (base UNom)";
  parameter Types.PerUnit Pm0Pu "Start value of the mechanical power in pu (base SNom)";
  parameter Types.PerUnit Id0 "Start value of the d-axis stator current in pu (base SNom)";
  parameter Types.PerUnit Iq0 "Start value of the q-axis stator current in pu (base SNom)";

  final parameter Real kk = SystemBase.SnRef / SNom "Base change factor for currents from SnRef to SNom";
  final parameter Real satIntermediate = if Se1Pu > 0 then sqrt(Se12Pu * 1.2 / Se1Pu) else 0 "Saturation fit intermediate";
  final parameter Real satA = if Se1Pu > 0 then (satIntermediate - 1.2) / (satIntermediate - 1) else 0 "Saturation knee";
  final parameter Real satB = if Se1Pu > 0 then Se1Pu / (1 - satA) ^ 2 else 0 "Saturation coefficient";
  // Ratios for the d/q-axis flux dynamics, identical to the GENROU block diagram
  final parameter Real coefEq1 = (XdPu - XppdPu) / (XpdPu - XppdPu) "Coefficient on Eq' in dEq'/dt";
  final parameter Real coefEq2 = (XdPu - XpdPu) / (XpdPu - XppdPu) "Coefficient on Eq'' in dEq'/dt";
  final parameter Real coefEd1 = (XqPu - XppqPu) / (XpqPu - XppqPu) "Coefficient on Ed' in dEd'/dt";
  final parameter Real coefEd2 = (XqPu - XpqPu) / (XpqPu - XppqPu) "Coefficient on Ed'' in dEd'/dt";
  // Operating-point values used as start values for the algebraic variables (Vd, Vq, psiAg, saturated reactances)
  final parameter Types.PerUnit Vd0Pu = u0Pu.re * sin(Theta0) - u0Pu.im * cos(Theta0) "Start value of d-axis terminal voltage in pu (base UNom)";
  final parameter Types.PerUnit Vq0Pu = u0Pu.re * cos(Theta0) + u0Pu.im * sin(Theta0) "Start value of q-axis terminal voltage in pu (base UNom)";
  final parameter Types.PerUnit PsiAg0Pu = sqrt((Vq0Pu + Iq0 * RaPu + Id0 * XlPu) ^ 2 + (Vd0Pu + Id0 * RaPu - Iq0 * XlPu) ^ 2) "Start value of air-gap flux magnitude in pu (base UNom)";
  final parameter Types.PerUnit Sat0Pu = if PsiAg0Pu > 0 then satB * (PsiAg0Pu - satA) * (PsiAg0Pu - satA) / PsiAg0Pu else 0 "Start value of saturation function S(psiAg)";
  final parameter Types.PerUnit SatD0Pu = 1 + Sat0Pu "Start value of d-axis saturation factor";
  final parameter Types.PerUnit SatQ0Pu = 1 + XqPu / XdPu * Sat0Pu "Start value of q-axis saturation factor";
  final parameter Types.PerUnit Xdsat0Pu = (XppdPu - XlPu) / SatD0Pu + XlPu "Start value of saturated d-axis subtransient reactance in pu (base SNom)";
  final parameter Types.PerUnit Xqsat0Pu = (XppqPu - XlPu) / SatQ0Pu + XlPu "Start value of saturated q-axis subtransient reactance in pu (base SNom)";

equation
  // Park transformation between the network reference frame and the rotor d-q frame
  Vd = terminal.V.re * sin(theta) - terminal.V.im * cos(theta);
  Vq = terminal.V.re * cos(theta) + terminal.V.im * sin(theta);
  Id = (-terminal.i.re * sin(theta) + terminal.i.im * cos(theta)) * kk;
  Iq = (-terminal.i.re * cos(theta) - terminal.i.im * sin(theta)) * kk;

  // Air-gap flux magnitude: voltage behind (Ra + jXl) seen from the d-q frame
  psiAgPu = sqrt((Vq + Iq * RaPu + Id * XlPu) ^ 2 + (Vd + Id * RaPu - Iq * XlPu) ^ 2);

  // Multiplicative saturation factors (scaled-quadratic), per PowerWorld GENTPF
  // (max() guards the 1/psiAg singularity when the machine is fully de-energised)
  satDPu = 1 + satB * (psiAgPu - satA) * (psiAgPu - satA) / max(psiAgPu, 1e-6);
  satQPu = 1 + XqPu / XdPu * satB * (psiAgPu - satA) * (psiAgPu - satA) / max(psiAgPu, 1e-6);

  // Saturated subtransient reactances entering the network interface
  XdsatPu = (XppdPu - XlPu) / satDPu + XlPu;
  XqsatPu = (XppqPu - XlPu) / satQPu + XlPu;

  if running.value then
    // Swing equations
    der(theta) = SystemBase.omegaNom * (omegaPu - omegaRefPu);
    2 * H * der(omegaPu) = PmPu - PGenPu * SystemBase.SnRef / (SNom * omegaPu) - DPu * (omegaPu - omegaRefPu);
    // Transient d-axis flux dynamics, saturation applied to the EMF terms
    Tpd0 * der(eqPPu) = efdPu + satDPu * (-coefEq1 * eqPPu + coefEq2 * eqPPPu);
    // Transient q-axis flux dynamics, saturation applied to the EMF terms
    Tpq0 * der(edPPu) = satQPu * (-coefEd1 * edPPu + coefEd2 * edPPPu);
    // Subtransient d-axis flux dynamics
    Tppd0 * der(eqPPPu) = satDPu * (eqPPu - eqPPPu) - (XpdPu - XlPu) * Id;
    // Subtransient q-axis flux dynamics
    Tppq0 * der(edPPPu) = satQPu * (edPPu - edPPPu) + (XpqPu - XlPu) * Iq;
    // Stator algebraic equations: speed-multiplied EMF, saturated reactances
    Vd = edPPPu * omegaPu - RaPu * Id + XqsatPu * Iq;
    Vq = eqPPPu * omegaPu - RaPu * Iq - XdsatPu * Id;
  else
    der(theta) = 0;
    der(omegaPu) = 0;
    der(eqPPu) = 0;
    der(edPPu) = 0;
    der(eqPPPu) = 0;
    der(edPPPu) = 0;
    terminal.i = Complex(0, 0);
  end if;

  annotation(preferredView = "text");
end GeneratorRoundRotorTypeF;
