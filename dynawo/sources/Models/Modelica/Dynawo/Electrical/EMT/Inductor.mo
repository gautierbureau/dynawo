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

model Inductor "Three-phase instantaneous (EMT) inductor: v = L * der(i)"
  extends Dynawo.Electrical.EMT.BaseEmtTwoTerminal;
  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffLine;

  parameter Real LPu "Inductance per phase in pu (L = X / omegaNom, with time in seconds)";
  parameter Real I0Pu[3] = {0, 0, 0} "Initial per-phase currents in pu";

initial equation
  p.i = I0Pu;

equation
  if running.value then
    for k in 1:3 loop
      v[k] = LPu * der(p.i[k]);
    end for;
  else
    for k in 1:3 loop
      p.i[k] = 0;
    end for;
  end if;

  annotation(preferredView = "text");
end Inductor;
