within Dynawo.Electrical.EMT.Functions;

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

function balancedAbcInit "Instantaneous a,b,c values at t = 0 of a balanced positive-sequence phasor (peak)"
  input Real magPu "Phasor magnitude (peak value) in pu";
  input Real angleRad "Phasor angle in rad";
  output Real abc[3] "Instantaneous {a, b, c} values at t = 0 in pu";
protected
  constant Real PI = 3.141592653589793;
algorithm
  // x_k(t) = magPu * cos(omega*t + angleRad + shift_k); evaluated at t = 0,
  // shift = {0, -2pi/3, +2pi/3} (matching VoltageSource's PhaseShift).
  abc := {magPu * cos(angleRad), magPu * cos(angleRad - 2 * PI / 3), magPu * cos(angleRad + 2 * PI / 3)};
  annotation(preferredView = "text");
end balancedAbcInit;
