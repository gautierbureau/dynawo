within Dynawo.Electrical.EMT;

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

model GeneratorSynchronous_INIT "Initialization model for the EMT GeneratorSynchronous"

  /*
    Dedicated EMT init. It reuses the synchronous-machine operating-point computation of the
    phasor internal-parameter init (same steady state: the *0Pu winding/flux start values and the
    complex terminal voltage u0Pu = fromPolar(U0Pu, UPhase0)) and ADDS the EMT-specific
    instantaneous start value the three-phase dynamic model needs but a phasor init does not
    provide: the abc terminal voltage at t = 0. It transfers BY NAME to the matching parameter of
    Dynawo.Electrical.EMT.GeneratorSynchronous, which uses it to seed the abc terminal voltage.
    (The local init has no terminal of its own -- this is computed here as a plain array and handed
    to the dynamic model through the name-matching transfer convention.)
  */

  extends Dynawo.Electrical.Machines.OmegaRef.GeneratorSynchronousInt_INIT;

  import Modelica.Constants.pi;

  // EMT-specific instantaneous abc terminal voltage at t = 0 (transferred by name to the dynamic model)
  Dynawo.Types.PerUnit uTerminal0Pu[3] "Instantaneous abc terminal voltage at t = 0 in pu (base UNom)";

equation
  // v_k(0) = sqrt(2) * Re(u0Pu * e^(j*phi_k)), phi = {0, -2pi/3, +2pi/3}
  uTerminal0Pu[1] = sqrt(2.0) * u0Pu.re;
  uTerminal0Pu[2] = sqrt(2.0) * (u0Pu.re * cos(2 * pi / 3) + u0Pu.im * sin(2 * pi / 3));
  uTerminal0Pu[3] = sqrt(2.0) * (u0Pu.re * cos(2 * pi / 3) - u0Pu.im * sin(2 * pi / 3));

  annotation(preferredView = "text");
end GeneratorSynchronous_INIT;
