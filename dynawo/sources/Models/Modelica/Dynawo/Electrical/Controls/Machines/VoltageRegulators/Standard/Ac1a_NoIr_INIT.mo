within Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard;

/*
* Copyright (c) 2026, RTE (http://www.rte-france.com)
* See AUTHORS.txt
* All rights reserved.
* This Source Code Form is subject to the terms of the Mozilla Public
* License, v. 2.0. If a copy of the MPL was not distributed with this
* file, you can obtain one at http://mozilla.org/MPL/2.0/.
* SPDX-License-Identifier: MPL-2.0
*
* This file is part of Dynawo, an hybrid C++/Modelica open source suite
* of simulation tools for power systems.
*/

model Ac1a_NoIr_INIT "IEEE excitation system type AC1A init when machine does not expose rotor current (Ir0Pu held at 0)"
  extends Dynawo.Electrical.Controls.Machines.VoltageRegulators.Standard.Ac1a_INIT;

equation
  Ir0Pu = 0;

  annotation(preferredView = "text");
end Ac1a_NoIr_INIT;
