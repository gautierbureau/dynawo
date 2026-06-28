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

model SeriesRL "Three-phase instantaneous (EMT) series R-L branch / simple line: v = R * i + L * der(i)"
  import Dynawo.Electrical.SystemBase;
  extends Dynawo.Electrical.EMT.BaseEmtTwoTerminal;
  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffLine;

  parameter Real RPu "Series resistance per phase in pu (base ZNom)";
  parameter Real LPu = -1 "Series inductance per phase in pu (L = X / omegaNom; legacy, used only if XPu < 0)";
  parameter Real XPu = -1 "Series reactance per phase in pu (IIDM-native, referenceable from x_pu; if >= 0 it sets L = XPu / omegaNom)";
  final parameter Real LPuEff = if XPu >= 0 then XPu / SystemBase.omegaNom else LPu "Effective inductance used in the constitutive law";
  parameter Real I0Pu[3] = {0, 0, 0} "Initial per-phase currents in pu";

initial equation
  p.i = I0Pu;

equation
  if running.value then
    for k in 1:3 loop
      v[k] = RPu * p.i[k] + LPuEff * der(p.i[k]);
    end for;
  else
    for k in 1:3 loop
      p.i[k] = 0;
    end for;
  end if;

  annotation(preferredView = "text");
end SeriesRL;
