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

case_name = "EmtNetworkSMIBNodeFault"
case_description = "EMT network SMIB (DYNModelNetworkEMT) with a three-phase node fault applied at the HV bus and cleared, making the rotor swing. Exercises the network bus-fault shunt-to-ground on top of the external-machine connection."
job_file = os.path.join(os.path.dirname(__file__), "smibnetflt.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))
