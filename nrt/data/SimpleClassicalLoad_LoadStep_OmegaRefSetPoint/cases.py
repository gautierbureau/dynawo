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
#  SimpleClassicalLoad_LoadStep_OmegaRefSetPoint                              #
#  Variant of SimpleClassicalLoad_LoadStep with the DYNModelOmegaRef black    #
#  box swapped for a fixed SetPoint feeding generator_omegaRefPu = 1 pu.      #
###############################################################################

case_name = "SimpleClassicalLoad_LoadStep_OmegaRefSetPoint"
case_description = "Same one-bus GeneratorClassical + LoadPQ + Step load step as SimpleClassicalLoad_LoadStep, but generator_omegaRefPu is fed by a Basics.SetPoint fixed at 1 pu instead of the C++ DYNModelOmegaRef. With a non-zero damping coefficient DPu = 2.0 and a constant 1 pu reference, the machine now sees a real damping term D*(omegaPu - 1) and settles to a new equilibrium below 1 pu instead of decelerating monotonically. Run twice (SolverIDA + SolverSIM)"
job_file = os.path.join(os.path.dirname(__file__), "SimpleClassicalLoad_LoadStep_OmegaRefSetPoint.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))
