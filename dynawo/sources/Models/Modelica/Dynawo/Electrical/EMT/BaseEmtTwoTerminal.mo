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

partial model BaseEmtTwoTerminal "Base class for two-terminal three-phase EMT components (receptor convention: i > 0 entering terminal p)"

  /*
    Connector-direct form: the constitutive law in each derived model is written
    on the connector variables themselves -- the voltage drop v = p.v - n.v and
    the through-current p.i -- and the series KCL p.i + n.i = 0 is declared here.
    There is deliberately NO through-current alias "i = p.i": such an alias makes
    OpenModelica eliminate the flow variable n.i in favour of a calculated
    (non-flow) variable, which then cannot be connected to another flow terminal
    when the component is used as a black-box model. v (a voltage difference, not
    a flow) is safe to expose and is kept for convenience / curves.
  */

  Dynawo.Electrical.EMT.EmtTerminal p "Positive-side terminal";
  Dynawo.Electrical.EMT.EmtTerminal n "Negative-side terminal";

  Real v[3] "Per-phase voltage drop across the component, v = p.v - n.v, in pu";

equation
  for k in 1:3 loop
    v[k] = p.v[k] - n.v[k];
    p.i[k] + n.i[k] = 0;
  end for;

  annotation(preferredView = "text");
end BaseEmtTwoTerminal;
