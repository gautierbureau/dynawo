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

###############################################################################
#  SimpleClassicalLoad_LoadStep - load step on a single GeneratorClassical    #
###############################################################################

case_name = "SimpleClassicalLoad_LoadStep"
case_description = "Minimal one-bus IIDM with a GeneratorClassical and a LoadPQ; a Step block raises the load's PRefPu from 1.0 to 1.2 pu at t = 0.5 s. With no governor, the classical machine's rotor decelerates: omega drifts from 1.0 down to ~0.81 pu by t = 10 s. The case is run twice (SolverIDA + SolverSIM)"
job_file = os.path.join(os.path.dirname(__file__), "SimpleClassicalLoad_LoadStep.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))
