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

model GeneratorThirdOrder_INIT "Initialization model for the third-order synchronous machine"
  extends Dynawo.Electrical.Machines.BaseClasses_INIT.BaseGeneratorParameters_INIT;
  extends AdditionalIcons.Init;

  import Modelica.ComplexMath;
  import Dynawo.Types;
  import Dynawo.Electrical.SystemBase;

  parameter Types.PerUnit RaPu "Armature resistance in pu (base SNom)";
  parameter Types.PerUnit XdPu "d-axis synchronous reactance in pu (base SNom)";
  parameter Types.PerUnit XpdPu "d-axis transient reactance in pu (base SNom)";
  parameter Types.PerUnit XqPu "q-axis synchronous reactance in pu (base SNom)";
  parameter Types.ApparentPowerModule SNom "Nominal apparent power of the generator in MVA";

  final parameter Real kk = SystemBase.SnRef / SNom "Base change factor for currents from SnRef to SNom";

  Types.ComplexVoltagePu eqTrick0Pu "Voltage behind the q-axis reactance, aligned with the q-axis, in pu (base UNom)";
  Dynawo.Connectors.AngleConnector Theta0 "Start value of the rotor angle in rad";
  Dynawo.Connectors.PerUnitConnector Id0 "Start value of the d-axis stator current in pu (base SNom)";
  Dynawo.Connectors.PerUnitConnector Iq0 "Start value of the q-axis stator current in pu (base SNom)";
  Dynawo.Connectors.PerUnitConnector EqP0Pu "Start value of the q-axis transient internal EMF in pu (base UNom)";
  Dynawo.Connectors.VoltageModulePuConnector Efd0Pu "Start value of the field voltage in pu (base UNom)";
  Dynawo.Connectors.PerUnitConnector Pm0Pu "Start value of the mechanical power in pu (base SNom)";

equation
  // The "voltage behind the q-axis reactance" has a zero d-axis component, so it locates the q-axis
  eqTrick0Pu = u0Pu - Complex(RaPu, XqPu) * i0Pu * kk;
  Theta0 = ComplexMath.arg(eqTrick0Pu);

  // d-q stator currents (machine base, generator convention)
  Id0 = (-i0Pu.re * sin(Theta0) + i0Pu.im * cos(Theta0)) * kk;
  Iq0 = (-i0Pu.re * cos(Theta0) - i0Pu.im * sin(Theta0)) * kk;

  // q-axis transient EMF from the q-axis stator equation
  EqP0Pu = (u0Pu.re * cos(Theta0) + u0Pu.im * sin(Theta0)) + RaPu * Iq0 + XpdPu * Id0;

  // Field voltage from the steady-state field flux equation
  Efd0Pu = EqP0Pu + (XdPu - XpdPu) * Id0;

  // Mechanical power balances the electrical power at steady state
  Pm0Pu = PGen0Pu * SystemBase.SnRef / SNom;

  annotation(preferredView = "text");
end GeneratorThirdOrder_INIT;
