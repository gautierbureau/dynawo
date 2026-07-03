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

model Capacitor "Three-phase instantaneous (EMT) capacitor: i = C * der(v)"
  extends Dynawo.Electrical.EMT.BaseEmtTwoTerminal;
  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffShunt;

  parameter Real CPu "Capacitance per phase in pu (C = B / omegaNom, with time in seconds)";
  parameter Real U0Pu[3] = {0, 0, 0} "Initial per-phase voltages in pu";

initial equation
  v = U0Pu;

equation
  if running.value then
    for k in 1:3 loop
      p.i[k] = CPu * der(v[k]);
    end for;
  else
    for k in 1:3 loop
      p.i[k] = 0;
    end for;
  end if;

  annotation(preferredView = "text");
end Capacitor;
