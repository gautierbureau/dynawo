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

case_name = "EmtLineToLineFault"
case_description = "EMT example: an unbalanced line-to-line fault, exercising phase-selective fault handling in the abc time domain."
job_file = os.path.join(os.path.dirname(__file__), "LineToLineFaultDemo.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))
