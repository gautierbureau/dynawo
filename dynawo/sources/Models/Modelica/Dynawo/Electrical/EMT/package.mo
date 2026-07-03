within Dynawo.Electrical;

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

package EMT "Electromagnetic-transient (EMT) instantaneous three-phase (abc) models"
  extends Icons.Package;

  annotation(preferredView = "text",
    Documentation(info = "<html><head></head><body>
<p>Proof-of-concept library of <b>instantaneous three-phase (abc)</b> electrical
components for electromagnetic-transient (EMT) simulation inside Dynawo.</p>
<p>Unlike the rest of the Dynawo library, these models do <b>not</b> use complex
phasors. Each terminal carries three instantaneous per-phase voltages and
currents (real numbers), and the components are written with real time
derivatives (<code>L&middot;der(i)</code>, <code>C&middot;der(v)</code>). The
resulting DAE is integrated by Dynawo's fixed-step solvers (<code>SIM</code> /
<code>TRAP</code>), producing full three-phase waveforms.</p>
<p>This is exploratory \"Path B\" material — see <code>emt-exploration/FINDINGS.md</code>.</p>
</body></html>"));
end EMT;
