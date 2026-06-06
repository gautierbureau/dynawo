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

###################################################################################
#     SimpleGeneratorLoad_LoadStep - load step on PRefPu, run with IDA and SIM    #
###################################################################################

case_name = "SimpleGeneratorLoad_LoadStep"
case_description = "Same minimal GeneratorAlphaBeta + LoadPQ + InfiniteBus as SimpleGeneratorLoad, with a Step driving the load's PRefPu (1.0 -> 1.2 pu at t = 0.5 s). The case is run twice in the same jobs file, once with the variable-step SolverIDA and once with the fixed-step SolverSIM"
job_file = os.path.join(os.path.dirname(__file__), "SimpleGeneratorLoad_LoadStep.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))
