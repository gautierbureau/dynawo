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

connector EmtTerminal "Three-phase instantaneous (EMT) electrical terminal carrying phases a, b, c"
  Real v[3] "Instantaneous phase-to-ground voltages in pu (base UNom), order a, b, c";
  flow Real i[3] "Instantaneous phase currents in pu (base SnRef/UNom), positive when entering the device, order a, b, c";

  annotation(preferredView = "text",
    Icon(graphics = {Rectangle(lineColor = {0, 0, 255}, fillColor = {213, 255, 170}, fillPattern = FillPattern.Solid, extent = {{-100, 98}, {100, -102}})}, coordinateSystem(initialScale = 0.1)));
end EmtTerminal;
