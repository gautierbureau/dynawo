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

model TransformerYgYg "Three-phase grounded-wye / grounded-wye transformer (ideal core + primary leakage), no phase shift"

  /*
    An ideal core (v2 = g*v1; i1 = -g*i2) wrapped with a
    primary leakage R-L, composed as a grounded-wye / grounded-wye group. Both
    neutrals are at ground, so each terminal phase voltage equals its winding
    voltage. Per phase, with turns ratio rTfoPu = N1/N2 and leakage referred to
    the primary:

      primary.v  = rTfoPu * secondary.v + (R + L*der) * primary.i
      secondary.i = - rTfoPu * primary.i        (ampere-turn balance, ideal core)

    Dynawo-native, no MSL. Magnetising / saturation branch omitted (linear core).
  */

  import Dynawo.Electrical.SystemBase;

  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffTransformer;

  Dynawo.Electrical.EMT.EmtTerminal primary "Primary (HV) three-phase terminal";
  Dynawo.Electrical.EMT.EmtTerminal secondary "Secondary (LV) three-phase terminal";

  parameter Real rTfoPu = 1.0 "Turns ratio N1/N2 (primary/secondary)";
  parameter Real RPu = 0.005 "Leakage resistance referred to primary in pu";
  parameter Real LPu = 3.2e-4 "Leakage inductance referred to primary in pu (L = X / omegaNom; legacy, used only if XPu < 0)";
  parameter Real XPu = -1 "Leakage reactance referred to primary in pu (IIDM-native, referenceable from x_pu; if >= 0 it sets L = XPu / omegaNom)";
  final parameter Real LPuEff = if XPu >= 0 then XPu / SystemBase.omegaNom else LPu "Effective leakage inductance used in the constitutive law";
  parameter Real I0Pu[3] = {0, 0, 0} "Initial primary phase currents in pu";

initial equation
  primary.i = I0Pu;

equation
  if running.value then
    for k in 1:3 loop
      primary.v[k] = rTfoPu * secondary.v[k] + RPu * primary.i[k] + LPuEff * der(primary.i[k]);
      secondary.i[k] = -rTfoPu * primary.i[k];
    end for;
  else
    for k in 1:3 loop
      primary.i[k] = 0;
      secondary.i[k] = 0;
    end for;
  end if;

  annotation(preferredView = "text");
end TransformerYgYg;
