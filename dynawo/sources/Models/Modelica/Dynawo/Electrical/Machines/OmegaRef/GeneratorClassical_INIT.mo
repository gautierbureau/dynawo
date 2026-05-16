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

model GeneratorClassical_INIT "Initialization model for the classical second-order synchronous generator"
  extends Dynawo.Electrical.Machines.BaseClasses_INIT.BaseGeneratorParameters_INIT;
  extends AdditionalIcons.Init;

  import Modelica.ComplexMath;
  import Dynawo.Types;

  parameter Types.PerUnit XdPu "Transient reactance behind the constant internal EMF in pu (base UNom, SnRef)";

  Types.ComplexVoltagePu Ep0PuComplex "Start value of the internal EMF phasor in pu (base UNom)";
  Dynawo.Connectors.AngleConnector delta0 "Start value of the rotor angle in rad";
  Dynawo.Connectors.VoltageModulePuConnector Ep0Pu "Start value of the internal EMF magnitude in pu (base UNom)";
  Dynawo.Connectors.ActivePowerPuConnector Tm0Pu "Start value of the mechanical power in pu (base SnRef)";

equation
  // Internal EMF phasor: Ep = U - j.Xd.I, with U and I (receptor convention) at the terminal
  Ep0PuComplex = u0Pu - Complex(0, XdPu) * i0Pu;
  Ep0Pu = ComplexMath.'abs'(Ep0PuComplex);
  delta0 = ComplexMath.arg(Ep0PuComplex);

  // At steady state the mechanical power balances the electrical power
  Tm0Pu = PGen0Pu;

  annotation(preferredView = "text");
end GeneratorClassical_INIT;
