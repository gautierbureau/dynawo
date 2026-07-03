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
#   EMT three-phase R-L-C circuit assembled from per-component black boxes     #
###############################################################################

case_name = "EmtThreePhaseRLBlackBox"
case_description = "Instantaneous three-phase (EMT) R-L-C circuit built from individual EMT component black-box models (VoltageSource, SeriesRL, Resistor, Capacitor, Ground) wired through dyn:connect on the abc EmtTerminal connectors; energised from rest, exercising the per-component preassembled models and the multi-element node current summation"
job_file = os.path.join(os.path.dirname(__file__), "EmtThreePhaseRLBlackBox.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))
