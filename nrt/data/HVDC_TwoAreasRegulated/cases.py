# -*- coding: utf-8 -*-

# Copyright (c) 2026, RTE (http://www.rte-france.com)
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

####################################################################################
#   HVDC_TwoAreasRegulated - two asymmetric AC areas linked by two parallel HVDC   #
####################################################################################

case_name = "HVDC_TwoAreasRegulated"
case_description = "Two asymmetric AC areas (Area A meshed with 2 gens, Area B radial with 2 gens) connected only by two parallel HVDC links; IEEE14-style synchronous generators with AVR and governor proportional regulations; trip HVDC1 at t = 1 s and watch the governors hold both island frequencies near nominal"
job_file = os.path.join(os.path.dirname(__file__), "HVDC_TwoAreasRegulated.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))
