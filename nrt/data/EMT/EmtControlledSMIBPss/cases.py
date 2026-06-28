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

###############################################################################
#  EMT SMIB whose generator reuses a PHASOR speed-input power system stabilizer #
###############################################################################

case_name = "EmtControlledSMIBPss"
case_description = "Instantaneous three-phase (EMT) single-machine infinite-bus assembled from black boxes, where the generator is a preassembled bundle of the EMT SynchronousMachine with three PHASOR-domain controls reused unchanged: the VRProportionalIntegral voltage regulator, the GoverProportional governor and the Pss1aOmega speed-input power system stabilizer, wired by initConnect/connect and starting bumplessly. The AVR runs at a deliberately high gain so the local electromechanical mode is poorly damped; a mild balanced three-phase-to-ground fault (0.4-0.45 s) excites the mode, and the tuned PSS (two 0.12/0.02 s lead-lag stages, Ks = 4) feeds the rotor-speed deviation back into the AVR reference to damp the rotor oscillation, cutting the post-fault settling energy and peak swing by roughly 60% versus the same case with the PSS removed"
job_file = os.path.join(os.path.dirname(__file__), "EmtControlledSMIBPss.jobs")

test_cases.append((case_name, case_description, job_file, 5, standardReturnCodeType, standardReturnCode))
