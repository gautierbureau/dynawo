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

model GeneratorFourthOrder "Fourth-order (two-axis) synchronous machine model"
  /*
    Fourth-order two-axis synchronous machine model: the electromechanical
    model (rotor angle and rotor speed) augmented with the d-axis field flux
    dynamics (q-axis transient EMF eqPPu, driven by the field voltage efdPu)
    and the q-axis damper flux dynamics (d-axis transient EMF edPPu). No
    subtransient winding is represented on either axis. Four states: rotor
    angle and speed, eqPPu, edPPu. The model uses the receptor convention
    (i > 0 when entering the device) and is coupled to the OmegaRef frequency
    handling.
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
  parameter Types.PerUnit XqPu "q-axis synchronous reactance in pu (base SNom)";
  parameter Types.PerUnit XpqPu "q-axis transient reactance in pu (base SNom)";
  parameter Types.Time Tpd0 "d-axis open-circuit transient time constant in s";
  parameter Types.Time Tpq0 "q-axis open-circuit transient time constant in s";
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

  // Algebraic variables
  Types.PerUnit Id(start = Id0) "d-axis stator current in pu (base SNom, generator convention)";
  Types.PerUnit Iq(start = Iq0) "q-axis stator current in pu (base SNom, generator convention)";
  Types.PerUnit Vd "d-axis terminal voltage in pu (base UNom)";
  Types.PerUnit Vq "q-axis terminal voltage in pu (base UNom)";

  // Start values calculated by the initialization model
  parameter Types.Angle Theta0 "Start value of the rotor angle in rad";
  parameter Types.PerUnit EqP0Pu "Start value of the q-axis transient internal EMF in pu (base UNom)";
  parameter Types.PerUnit EdP0Pu "Start value of the d-axis transient internal EMF in pu (base UNom)";
  parameter Types.VoltageModulePu Efd0Pu "Start value of the field voltage in pu (base UNom)";
  parameter Types.PerUnit Pm0Pu "Start value of the mechanical power in pu (base SNom)";
  parameter Types.PerUnit Id0 "Start value of the d-axis stator current in pu (base SNom)";
  parameter Types.PerUnit Iq0 "Start value of the q-axis stator current in pu (base SNom)";

  final parameter Real kk = SystemBase.SnRef / SNom "Base change factor for currents from SnRef to SNom";

equation
  // Park transformation between the network reference frame and the rotor d-q frame
  Vd = terminal.V.re * sin(theta) - terminal.V.im * cos(theta);
  Vq = terminal.V.re * cos(theta) + terminal.V.im * sin(theta);
  Id = (-terminal.i.re * sin(theta) + terminal.i.im * cos(theta)) * kk;
  Iq = (-terminal.i.re * cos(theta) - terminal.i.im * sin(theta)) * kk;

  if running.value then
    // Swing equations
    der(theta) = SystemBase.omegaNom * (omegaPu - omegaRefPu);
    2 * H * der(omegaPu) = PmPu - PGenPu * SystemBase.SnRef / (SNom * omegaPu) - DPu * (omegaPu - omegaRefPu);
    // d-axis field flux dynamics
    Tpd0 * der(eqPPu) = efdPu - eqPPu - (XdPu - XpdPu) * Id;
    // q-axis damper flux dynamics
    Tpq0 * der(edPPu) = - edPPu + (XqPu - XpqPu) * Iq;
    // Stator algebraic equations
    Vd = - RaPu * Id + XpqPu * Iq + edPPu;
    Vq = eqPPu - RaPu * Iq - XpdPu * Id;
  else
    der(theta) = 0;
    der(omegaPu) = 0;
    der(eqPPu) = 0;
    der(edPPu) = 0;
    terminal.i = Complex(0, 0);
  end if;

  annotation(preferredView = "text");
end GeneratorFourthOrder;
