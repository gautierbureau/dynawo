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

##########################################################
#   TwoZones AC - two classical machines with OmegaRef   #
##########################################################

case_name = "TwoZones - AC"
case_description = "Two zones linked by an AC line, each with a classical second-order generator, sharing a single DYNModelOmegaRef frequency reference; a fault is applied on the GEN2 bus"
job_file = os.path.join(os.path.dirname(__file__), "TwoZones_AC", "TwoZones.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

############################################################
#   TwoZones HVDC - two synchronous zones across an HVDC   #
############################################################

case_name = "TwoZones - HVDC"
case_description = "Two synchronous zones linked by a VSC-HVDC line, each with a classical second-order generator and a DYNModelOmegaRef frequency reference; the keepHvdcForeignNodes network parameter keeps both zones energized"
job_file = os.path.join(os.path.dirname(__file__), "TwoZones_HVDC", "TwoZones.jobs")

test_cases.append((case_name, case_description, job_file, 20, standardReturnCodeType, standardReturnCode))
