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

model TransformerYgD "Three-phase grounded-wye / delta transformer (ideal core + primary leakage); introduces the 30 deg shift and blocks zero sequence"

  /*
    An ideal-core transformer composed as a grounded-wye primary /
    delta secondary group (e.g. YNd). Primary winding k is between primary line k
    and ground, so its EMF is primary.v[k]. Secondary winding k is the delta
    winding spanning secondary lines k and k+1, so its EMF is the line-line
    difference. With turns ratio rTfoPu = N1/N2 and primary leakage R-L:

      primary.v[k]   = rTfoPu*(secondary.v[k] - secondary.v[next(k)]) + (R + L*der)*primary.i[k]
      secondary.i[k] = - rTfoPu*(primary.i[k] - primary.i[prev(k)])

    next(k) = mod(k,3)+1 ; prev(k) = mod(k+1,3)+1. The delta makes the secondary
    line currents sum to zero (zero-sequence blocked) and shifts the secondary by
    30 deg. The delta neutral floats: the secondary circuit must provide its own
    reference (e.g. a grounded load). Dynawo-native, no MSL.
  */

  extends Dynawo.Electrical.Controls.Basics.SwitchOff.SwitchOffTransformer;

  Dynawo.Electrical.EMT.EmtTerminal primary "Primary (grounded-wye) three-phase terminal";
  Dynawo.Electrical.EMT.EmtTerminal secondary "Secondary (delta) three-phase terminal";

  parameter Real rTfoPu = 1.0 "Turns ratio N1/N2 (primary winding / secondary winding)";
  parameter Real RPu = 0.005 "Leakage resistance referred to primary in pu";
  parameter Real LPu = 3.2e-4 "Leakage inductance referred to primary in pu (L = X / omegaNom)";
  parameter Real I0Pu[3] = {0, 0, 0} "Initial primary phase currents in pu";

initial equation
  primary.i = I0Pu;

equation
  if running.value then
    for k in 1:3 loop
      primary.v[k] = rTfoPu * (secondary.v[k] - secondary.v[mod(k, 3) + 1]) + RPu * primary.i[k] + LPu * der(primary.i[k]);
      secondary.i[k] = -rTfoPu * (primary.i[k] - primary.i[mod(k + 1, 3) + 1]);
    end for;
  else
    for k in 1:3 loop
      primary.i[k] = 0;
      secondary.i[k] = 0;
    end for;
  end if;

  annotation(preferredView = "text");
end TransformerYgD;
