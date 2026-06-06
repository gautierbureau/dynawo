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

#############################################################################
#     GeneratorRoundRotorExp balanced-island initialization test            #
#############################################################################

case_name = "GeneratorRoundRotorExpIsland"
case_description = "Two different saturated sixth-order round-rotor synchronous machines feeding a common load through a line, forming a balanced island with no infinite bus; the consistent load flow has no disturbance, so the machines must stay exactly at their steady state for the whole run (initialization regression test)"
job_file = os.path.join(os.path.dirname(__file__), "GeneratorRoundRotorExpIsland.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))
