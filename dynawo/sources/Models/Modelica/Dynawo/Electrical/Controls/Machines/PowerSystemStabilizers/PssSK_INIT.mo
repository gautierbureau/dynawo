within Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers;

/*
* Copyright (c) 2024, RTE (http://www.rte-france.com)
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

model PssSK_INIT "Initialization model for PssSK (CGMES PssSK) -- requires the field current in addition to PGen"
  extends Dynawo.Electrical.Controls.Machines.PowerSystemStabilizers.Pss_INIT;

  Dynawo.Connectors.PerUnitConnector IRotor0Pu "Initial field (rotor) current in pu (base IRotorNom)";

  annotation(preferredView = "text");
end PssSK_INIT;
