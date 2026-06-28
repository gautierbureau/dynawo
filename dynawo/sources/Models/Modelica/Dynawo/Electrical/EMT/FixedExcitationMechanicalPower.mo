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

model FixedExcitationMechanicalPower "Constant field-voltage and mechanical-power source for an uncontrolled SynchronousMachine (the EMT analogue of a constant-excitation / PmConst feed)"

  /*
    Holds the machine field voltage and mechanical power at their steady-state
    values, so the single inputs-driven SynchronousMachine can be used without an
    exciter/governor: bundle it onto the machine and connect efdPu/PmPu. The
    seeds Efd0Pu (= Ifd0) and Pm0Pu come from the machine _INIT via initConnect
    (bumpless), exactly like a phasor governor's Pm0Pu.
  */

  parameter Real Efd0Pu = 0.5882352941176471 "Constant normalised field voltage (= Ifd0; default = open-circuit 1/Lmd)";
  parameter Real Pm0Pu = 0 "Constant mechanical power in pu";

  Modelica.Blocks.Interfaces.RealOutput efdPu(start = Efd0Pu) "Field voltage command";
  Modelica.Blocks.Interfaces.RealOutput PmPu(start = Pm0Pu) "Mechanical power command";

equation
  efdPu = Efd0Pu;
  PmPu = Pm0Pu;

  annotation(preferredView = "text");
end FixedExcitationMechanicalPower;
