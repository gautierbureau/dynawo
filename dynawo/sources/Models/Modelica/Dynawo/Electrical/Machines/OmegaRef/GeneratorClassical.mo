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

model GeneratorClassical "Classical second-order (electromechanical) synchronous generator model"
  /*
    Classical model of a synchronous machine: a constant internal EMF Ep
    behind a transient reactance XdPu. The electromechanical behavior is
    described by the two-state swing equation (rotor angle and rotor speed).
    There is no voltage regulator and no governor: the internal EMF magnitude
    and the mechanical power are constant and equal to their start values.
    The model respects the receptor convention (terminal.i > 0 when entering
    the device) and is coupled to the OmegaRef frequency handling.
  */
  extends Dynawo.Electrical.Machines.BaseClasses.BaseGeneratorSimplified;
  extends AdditionalIcons.Machine;

  import Modelica.ComplexMath;
  import Dynawo.Types;
  import Dynawo.Connectors;
  import Dynawo.Electrical.SystemBase;

  // Parameters
  parameter Types.PerUnit H "Inertia constant of the generator in s";
  parameter Types.PerUnit DPu "Damping coefficient of the generator in pu (base SnRef, omegaNom)";
  parameter Types.PerUnit RaPu "Armature resistance in pu (base UNom, SnRef)";
  parameter Types.PerUnit XdPu "Transient reactance behind the constant internal EMF in pu (base UNom, SnRef)";

  // Inputs / outputs for the OmegaRef coupling
  input Connectors.AngularVelocityPuConnector omegaRefPu(start = SystemBase.omegaRef0Pu) "Network angular reference frequency in pu (base omegaNom)";
  Connectors.AngularVelocityPuConnector omegaPu(start = SystemBase.omega0Pu) "Rotor angular frequency in pu (base omegaNom)";

  // Internal variables
  Types.Angle delta(start = delta0) "Rotor angle between the internal EMF and the network reference frame in rad";
  Types.ComplexVoltagePu EpPu(re(start = Ep0Pu * cos(delta0)), im(start = Ep0Pu * sin(delta0))) "Constant magnitude internal EMF phasor in pu (base UNom)";

  // Start values calculated by the initialization model
  parameter Types.Angle delta0 "Start value of the rotor angle in rad";
  parameter Types.VoltageModulePu Ep0Pu "Start value of the internal EMF magnitude in pu (base UNom)";
  parameter Types.ActivePowerPu Tm0Pu "Mechanical power of the generator in pu (base SnRef), constant and equal to its start value";

equation
  // Internal EMF: constant magnitude, rotating with the rotor angle
  EpPu = ComplexMath.fromPolar(Ep0Pu, delta);

  if running.value then
    // Stator equation: constant EMF behind the armature resistance and transient reactance (receptor convention)
    terminal.i = (terminal.V - EpPu) / Complex(RaPu, XdPu);
    // Swing equations
    der(delta) = (omegaPu - omegaRefPu) * SystemBase.omegaNom;
    2 * H * der(omegaPu) = Tm0Pu - PGenPu - DPu * (omegaPu - omegaRefPu);
  else
    terminal.i = Complex(0, 0);
    der(delta) = 0;
    der(omegaPu) = 0;
  end if;

  annotation(preferredView = "text");
end GeneratorClassical;
