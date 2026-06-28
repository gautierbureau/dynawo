# -*- coding: utf-8 -*-

# Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain
# simulation tool for power systems.

import os

test_cases = []
standardReturnCode = [0]
standardReturnCodeType = "ALLOWED"

###############################################################################
#   Loaded EMT single-machine infinite-bus built from black-box models         #
###############################################################################

case_name = "EmtSMIBLoadedBlackBox"
case_description = "Instantaneous three-phase (EMT) loaded single-machine infinite-bus assembled entirely from per-component black-box models (SynchronousMachine with its _INIT companion, TransformerYgYg, Capacitor x2, Resistor x2, SeriesRL, VoltageSource, Fault, Ground x3) wired through dyn:connect on the abc EmtTerminal connectors. The generator delivers 0.60 pu pre-fault, started in exact steady state via the machine _INIT; a single-line-to-ground fault on phase a of the HV bus (0.4-0.5 s) makes the rotor swing. Exercises the _INIT transfer, the transformer, and 4- and 5-element node current summation in a black-box assembly"
job_file = os.path.join(os.path.dirname(__file__), "EmtSMIBLoadedBlackBox.jobs")

test_cases.append((case_name, case_description, job_file, 5, standardReturnCodeType, standardReturnCode))
