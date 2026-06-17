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

##################################################################################
#   HVDC_TwoAreasRegulated_PropagatingEvent - Area A load step propagates to     #
#   Area B through HVDC1 (Modelica HvdcPQPropEmulation with AC emulation)        #
##################################################################################

case_name = "HVDC_TwoAreasRegulated_PropagatingEvent"
case_description = "Same two asymmetric AC areas as HVDC_TwoAreasRegulated, but HVDC1 is replaced by a Modelica HvdcPQPropEmulation black-box whose AC emulation makes the active-power transfer react to the angle difference between the two AC sides. A load step on LOAD_A1 at t = 1 s (350 -> 500 MW) causes Area A frequency to drop; the AC emulation shifts power across HVDC1, and Area B frequency dips too - a disturbance in Area A propagates to Area B"
job_file = os.path.join(os.path.dirname(__file__), "HVDC_TwoAreasRegulated_PropagatingEvent.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))
