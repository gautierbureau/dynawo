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

case_name = "EmtNetworkIEEE14"
case_description = "IEEE 14-bus system on the abc EMT network model (DYNModelNetworkEMT) with five detailed EMT synchronous machines. Exercises a meshed EMT network (14 buses, lines, transformers, loads) driven by external machines with no infinite bus."
job_file = os.path.join(os.path.dirname(__file__), "ieee14.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))
