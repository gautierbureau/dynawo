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

model GeneratorRoundRotorExp "Sixth-order round-rotor synchronous machine model with magnetic saturation"
  /*
    Sixth-order round-rotor synchronous machine with magnetic saturation: the
    GeneratorRoundRotor model with a saturation factor sePu, fitted through two
    operating points, scaling the field and q-axis flux states. With Se1Pu = 0
    the saturation vanishes and the model reduces to GeneratorRoundRotor.
    Corresponds to the CGMES IEC 61970-302 GenRoe model (PSS/E GENROE).
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

  // Complex terminal V and I exposed for downstream controls (voltage compensators, current-driven limiters)
  Connectors.ComplexVoltagePuConnector uPu(re(start = u0Pu.re), im(start = u0Pu.im)) "Complex voltage at terminal in pu (base UNom)";
  Connectors.ComplexCurrentPuConnector iPu(re(start = i0Pu.re), im(start = i0Pu.im)) "Complex current at terminal in pu (base SnRef, UNom) (receptor convention)";

  // State variables
  Types.Angle theta(start = Theta0) "Rotor angle: angle of the q-axis from the network reference frame in rad";
  Types.PerUnit eqPPu(start = EqP0Pu) "q-axis transient internal EMF in pu (base UNom)";
  Types.PerUnit edPPu(start = EdP0Pu) "d-axis transient internal EMF in pu (base UNom)";
  Types.PerUnit psi1d(start = Psi1d0) "d-axis subtransient damper flux linkage in pu (base UNom)";
  Types.PerUnit psi2q(start = Psi2q0) "q-axis subtransient damper flux linkage in pu (base UNom)";

  // Algebraic variables
  Types.PerUnit eqPPPu "q-axis subtransient internal EMF in pu (base UNom)";
  Types.PerUnit edPPPu "d-axis subtransient internal EMF in pu (base UNom)";
  Types.PerUnit sePu "Saturation factor";
  Types.PerUnit Id(start = Id0) "d-axis stator current in pu (base SNom, generator convention)";
  Types.PerUnit Iq(start = Iq0) "q-axis stator current in pu (base SNom, generator convention)";
  Types.PerUnit Vd "d-axis terminal voltage in pu (base UNom)";
  Types.PerUnit Vq "q-axis terminal voltage in pu (base UNom)";

  // Start values calculated by the initialization model
  parameter Types.Angle Theta0 "Start value of the rotor angle in rad";
  parameter Types.PerUnit EqP0Pu "Start value of the q-axis transient internal EMF in pu (base UNom)";
  parameter Types.PerUnit EdP0Pu "Start value of the d-axis transient internal EMF in pu (base UNom)";
  parameter Types.PerUnit Psi1d0 "Start value of the d-axis subtransient damper flux in pu (base UNom)";
  parameter Types.PerUnit Psi2q0 "Start value of the q-axis subtransient damper flux in pu (base UNom)";
  parameter Types.VoltageModulePu Efd0Pu "Start value of the field voltage in pu (base UNom)";
  parameter Types.PerUnit Pm0Pu "Start value of the mechanical power in pu (base SNom)";
  parameter Types.PerUnit Id0 "Start value of the d-axis stator current in pu (base SNom)";
  parameter Types.PerUnit Iq0 "Start value of the q-axis stator current in pu (base SNom)";

  final parameter Real kk = SystemBase.SnRef / SNom "Base change factor for currents from SnRef to SNom";
  final parameter Real gammaDPu = (XpdPu - XppdPu) / (XpdPu - XlPu) ^ 2 "d-axis damper screening factor";
  final parameter Real gammaQPu = (XpqPu - XppqPu) / (XpqPu - XlPu) ^ 2 "q-axis damper screening factor";
  final parameter Real satIntermediate = if Se1Pu > 0 then sqrt(Se12Pu * 1.2 / Se1Pu) else 0 "Saturation fit intermediate";
  final parameter Real satA = if Se1Pu > 0 then (satIntermediate - 1.2) / (satIntermediate - 1) else 0 "Saturation knee";
  final parameter Real satB = if Se1Pu > 0 then Se1Pu / (1 - satA) ^ 2 else 0 "Saturation coefficient";

equation
  // Mirror terminal V and I onto the named complex connectors
  uPu = terminal.V;
  iPu = terminal.i;

  // Park transformation between the network reference frame and the rotor d-q frame
  Vd = terminal.V.re * sin(theta) - terminal.V.im * cos(theta);
  Vq = terminal.V.re * cos(theta) + terminal.V.im * sin(theta);
  Id = (-terminal.i.re * sin(theta) + terminal.i.im * cos(theta)) * kk;
  Iq = (-terminal.i.re * cos(theta) - terminal.i.im * sin(theta)) * kk;

  // Subtransient internal EMFs
  eqPPPu = (XppdPu - XlPu) / (XpdPu - XlPu) * eqPPu + (XpdPu - XppdPu) / (XpdPu - XlPu) * psi1d;
  edPPPu = (XppqPu - XlPu) / (XpqPu - XlPu) * edPPu - (XpqPu - XppqPu) / (XpqPu - XlPu) * psi2q;

  // Magnetic saturation factor (air-gap flux from the transient EMFs)
  sePu = satB * (sqrt(eqPPu * eqPPu + edPPu * edPPu) - satA) * (sqrt(eqPPu * eqPPu + edPPu * edPPu) - satA) / sqrt(eqPPu * eqPPu + edPPu * edPPu);

  if running.value then
    // Swing equations
    der(theta) = SystemBase.omegaNom * (omegaPu - omegaRefPu);
    2 * H * der(omegaPu) = PmPu - PGenPu * SystemBase.SnRef / (SNom * omegaPu) - DPu * (omegaPu - omegaRefPu);
    // d-axis field flux dynamics, with saturation
    Tpd0 * der(eqPPu) = efdPu - (1 + sePu) * eqPPu - (XdPu - XpdPu) * (Id - gammaDPu * (psi1d + (XpdPu - XlPu) * Id - eqPPu));
    // q-axis transient flux dynamics, with saturation
    Tpq0 * der(edPPu) = - (1 + sePu) * edPPu + (XqPu - XpqPu) * (Iq + gammaQPu * (psi2q + edPPu + (XpqPu - XlPu) * Iq));
    // d-axis subtransient damper dynamics
    Tppd0 * der(psi1d) = eqPPu - psi1d - (XpdPu - XlPu) * Id;
    // q-axis subtransient damper dynamics
    Tppq0 * der(psi2q) = - edPPu - psi2q - (XpqPu - XlPu) * Iq;
    // Stator algebraic equations
    Vd = - RaPu * Id + XppqPu * Iq + edPPPu;
    Vq = eqPPPu - RaPu * Iq - XppdPu * Id;
  else
    der(theta) = 0;
    der(omegaPu) = 0;
    der(eqPPu) = 0;
    der(edPPu) = 0;
    der(psi1d) = 0;
    der(psi2q) = 0;
    terminal.i = Complex(0, 0);
  end if;

  annotation(preferredView = "text");
end GeneratorRoundRotorExp;
