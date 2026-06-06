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
forbiddenReturnCodeType = "FORBIDDEN"

########################################
#           SMIB_1_StepPm              #
########################################

case_name = "SMIB - StepPm"
case_description = "SMIB test case with a step on the mechanical power"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_1_StepPm", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

########################################
#           SMIB_1_StepPm_IIDM         #
########################################

case_name = "SMIB - StepPm IIDM"
case_description = "SMIB test case with a step on the mechanical power and an iidm network"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_1_StepPm_IIDM", "baseCase", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

case_name = "SMIB - StepPm IIDM and references"
case_description = "SMIB test case with a step on the mechanical power, an iidm network and references for parameters"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_1_StepPm_IIDM", "LinesAndTfosReferences", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

case_name = "SMIB - StepPm IIDM model network"
case_description = "SMIB test case with a step on the mechanical power, an iidm network and ModelNetwork used instead of Modelica"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_1_StepPm_IIDM", "baseCaseModelNetwork", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

########################################
#           SMIB_2_StepEfd             #
########################################

case_name = "SMIB - StepEfd"
case_description = "SMIB test case with a step on the excitation voltage"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_2_StepEfd", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

########################################
#           SMIB_3_LoadVarQ            #
########################################

case_name = "SMIB - LoadVarQ"
case_description = "SMIB test case with a step on the load reactive power"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_3_LoadVarQ", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

########################################
#           SMIB_4_DisconnectLine      #
########################################

case_name = "SMIB - DisconnectLine"
case_description = "SMIB test case with a line disconnection"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_4_DisconnectLine", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

########################################
#           SMIB_5_Fault               #
########################################

case_name = "SMIB - Fault"
case_description = "SMIB test case with a fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_5_Fault", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

########################################
#      SMIB_GeneratorClassical         #
########################################

case_name = "SMIB - GeneratorClassical"
case_description = "SMIB test case with the classical second-order generator model and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorClassical", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#  SMIB_GeneratorClassicalLineTrip     #
########################################

case_name = "SMIB - GeneratorClassicalLineTrip"
case_description = "Classical second-order generator feeding an infinite bus through two parallel lines, one of which is tripped"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorClassicalLineTrip", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#     SMIB_GeneratorThirdOrder         #
########################################

case_name = "SMIB - GeneratorThirdOrder"
case_description = "SMIB test case with the third-order (one-axis flux-decay) synchronous machine model and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorThirdOrder", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#     SMIB_GeneratorFourthOrder        #
########################################

case_name = "SMIB - GeneratorFourthOrder"
case_description = "SMIB test case with the fourth-order (two-axis) synchronous machine model and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorFourthOrder", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#     SMIB_GeneratorSalientPole        #
########################################

case_name = "SMIB - GeneratorSalientPole"
case_description = "SMIB test case with the fifth-order salient-pole synchronous machine model and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorSalientPole", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#     SMIB_GeneratorRoundRotor         #
########################################

case_name = "SMIB - GeneratorRoundRotor"
case_description = "SMIB test case with the sixth-order round-rotor synchronous machine model and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorRoundRotor", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#     SMIB_GeneratorRoundRotorExp      #
########################################

case_name = "SMIB - GeneratorRoundRotorExp"
case_description = "SMIB test case with the sixth-order round-rotor synchronous machine model with saturation and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorRoundRotorExp", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#    SMIB_GeneratorSalientPoleExp      #
########################################

case_name = "SMIB - GeneratorSalientPoleExp"
case_description = "SMIB test case with the fifth-order salient-pole synchronous machine model with saturation and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorSalientPoleExp", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#   SMIB_GeneratorRoundRotorTypeF      #
########################################

case_name = "SMIB - GeneratorRoundRotorTypeF"
case_description = "SMIB test case with the sixth-order round-rotor synchronous machine model with GENTPF (air-gap-flux) saturation and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorRoundRotorTypeF", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#   SMIB_GeneratorRoundRotorTypeJ      #
########################################

case_name = "SMIB - GeneratorRoundRotorTypeJ"
case_description = "SMIB test case with the sixth-order round-rotor synchronous machine model with GENTPJ (air-gap-flux + stator-current) saturation and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorRoundRotorTypeJ", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#  SMIB_GeneratorSalientPoleTypeF      #
########################################

case_name = "SMIB - GeneratorSalientPoleTypeF"
case_description = "SMIB test case with the fifth-order salient-pole synchronous machine model with GENTPF (air-gap-flux) saturation and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorSalientPoleTypeF", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

########################################
#  SMIB_GeneratorSalientPoleTypeJ      #
########################################

case_name = "SMIB - GeneratorSalientPoleTypeJ"
case_description = "SMIB test case with the fifth-order salient-pole synchronous machine model with GENTPJ (air-gap-flux + stator-current) saturation and a terminal fault"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_BasicTestCases", "SMIB_GeneratorSalientPoleTypeJ", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 10, standardReturnCodeType, standardReturnCode))

###############################################
#           SMIB Test Case 1 ST4B             #
###############################################

case_name = "SMIB - Test Case 1 ST4B"
case_description = "Voltage reference step on the synchronous machine (and its regulations) connected to a zero current bus"
job_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), "nrt", "data", "SMIB", "Standard", "TestCase1ST4B", "TestCase1ST4B.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

###############################################
#           SMIB Test Case 2 ST4B             #
###############################################

case_name = "SMIB - Test Case 2 ST4B"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), "nrt", "data", "SMIB", "Standard", "TestCase2ST4B", "TestCase2ST4B.jobs")

test_cases.append((case_name, case_description, job_file, 30, standardReturnCodeType, standardReturnCode))

###############################################
#           SMIB Test Case 3 ST4B             #
###############################################

case_name = "SMIB - Test Case 3 ST4B"
case_description = "Bolted three-phase short circuit at the high-level side of the transformer"
job_file = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))), "nrt", "data", "SMIB", "Standard", "TestCase3ST4B", "TestCase3ST4B.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#             SMIB with GoverNordic and VRNordic              #
###############################################################

case_name = "SMIB - Fault - GoverNordic - VRNordic"
case_description = "SMIB test case with a fault using GoverNordic and VRNordic regulations"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_Nordic", "SMIB_GoverNordicVRNordic", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#      SMIB with constant mechanical power and VRNordic       #
###############################################################

case_name = "SMIB - Fault - PmConst - VRNordic"
case_description = "SMIB test case with a fault using VRNordic regulation"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_Nordic", "SMIB_PmConstVRNordic", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#         SMIB with no mechanical power and VRNordic          #
###############################################################

case_name = "SMIB - Fault - PmConst - VRNordic - SynchronousCondenser"
case_description = "SMIB test case with a fault using VRNordic regulation, for a synchronous condenser"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_Nordic", "SMIB_SynchronousCondenser", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, Ac6a and Pss3b     #
###############################################################

case_name = "SMIB - Fault - PmConst - Ac6a - Pss3b"
case_description = "SMIB test case with a fault using Ac6a and Pss3b regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstAc6aPss3b", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, Ac7b and Pss3b     #
###############################################################

case_name = "SMIB - Fault - PmConst - Ac7b - Pss3b"
case_description = "SMIB test case with a fault using Ac7b and Pss3b regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstAc7bPss3b", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, Ac7c and Pss2c     #
###############################################################

case_name = "SMIB - Fault - PmConst - Ac7c - Pss2c"
case_description = "SMIB test case with a fault using Ac7c and Pss2c regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstAc7cPss2c", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#         SMIB with constant mechanical power and Ac8b        #
###############################################################

case_name = "SMIB - Fault - PmConst - Ac8b"
case_description = "SMIB test case with a fault using Ac8b regulation"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstAc8b", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#        SMIB with constant mechanical power and IeeeT2       #
###############################################################

case_name = "SMIB - Fault - PmConst - IeeeT2"
case_description = "SMIB test case with a fault using the IEEE type 2 exciter (IEEET2)"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstIeeeT2", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#        SMIB with constant mechanical power and Ac4a         #
###############################################################

case_name = "SMIB - Fault - PmConst - Ac4a"
case_description = "SMIB test case with a fault using the IEEE type AC4A exciter"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstAc4a", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#        SMIB with constant mechanical power and Ac4c         #
###############################################################

case_name = "SMIB - Fault - PmConst - Ac4c"
case_description = "SMIB test case with a fault using the IEEE type AC4C exciter"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstAc4c", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#        SMIB with constant mechanical power and Dc2a         #
###############################################################

case_name = "SMIB - Fault - PmConst - Dc2a"
case_description = "SMIB test case with a fault using the IEEE type DC2A exciter"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstDc2a", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#        SMIB with constant mechanical power and Dc2c         #
###############################################################

case_name = "SMIB - Fault - PmConst - Dc2c"
case_description = "SMIB test case with a fault using the IEEE type DC2C exciter"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstDc2c", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, Ac8b and Pss3b     #
###############################################################

case_name = "SMIB - Fault - PmConst - Ac8b - Pss3b"
case_description = "SMIB test case with a fault using Ac8b and Pss3b regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstAc8bPss3b", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, St5b and Pss2b     #
###############################################################

case_name = "SMIB - Fault - PmConst - St5b - Pss2b"
case_description = "SMIB test case with a fault using St5b and Pss2b regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstSt5bPss2b", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, St6b and Pss3b     #
###############################################################

case_name = "SMIB - Fault - PmConst - St6b - Pss3b"
case_description = "SMIB test case with a fault using St6b and Pss3b regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstSt6bPss3b", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, St6c and Pss6c     #
###############################################################

case_name = "SMIB - Fault - PmConst - St6c - Pss6c"
case_description = "SMIB test case with a fault using St6c and Pss6c regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstSt6cPss6c", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, St7b and Pss2a     #
###############################################################

case_name = "SMIB - Fault - PmConst - St7b - Pss2a"
case_description = "SMIB test case with a fault using St7b and Pss2a regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstSt7bPss2a", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#     SMIB with constant mechanical power, St9c and Pss2c     #
###############################################################

case_name = "SMIB - Fault - PmConst - St9c - Pss2c"
case_description = "SMIB test case with a fault using St9c and Pss2c regulations"
job_file = os.path.join(os.path.dirname(__file__), "IEEE", "PmConstSt9cPss2c", "SMIB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###############################################################
#             SMIB with GovCt2 and St4b                       #
###############################################################

case_name = "SMIB - TestCase - GovCt2 - St4b"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovCt2St4b", "TestCaseGovCt2St4b.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovHydro4 and St4b                       #
###################################################################

case_name = "SMIB - TestCase - GovHydro4 - St4b"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovHydro4St4b", "TestCaseGovHydro4St4b.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovSteamEu and St4b                       #
###################################################################

case_name = "SMIB - TestCase - GovSteamEu - St4b"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovSteamEuSt4b", "TestCaseGovSteamEuSt4b.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovSteamEu and St7b                       #
###################################################################

case_name = "SMIB - TestCase - GovSteamEu - St7b"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovSteamEuSt7b", "TestCaseGovSteamEuSt7b.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovHydro1                                 #
###################################################################

case_name = "SMIB - TestCase - GovHydro1"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovHydro1", "TestCaseGovHydro1.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovHydro2                                 #
###################################################################

case_name = "SMIB - TestCase - GovHydro2"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovHydro2", "TestCaseGovHydro2.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovHydro3                                 #
###################################################################

case_name = "SMIB - TestCase - GovHydro3"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovHydro3", "TestCaseGovHydro3.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovHydroDD                                #
###################################################################

case_name = "SMIB - TestCase - GovHydroDD"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovHydroDD", "TestCaseGovHydroDD.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovCT1                                    #
###################################################################

case_name = "SMIB - TestCase - GovCT1"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovCT1", "TestCaseGovCT1.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovSteam2                                 #
###################################################################

case_name = "SMIB - TestCase - GovSteam2"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovSteam2", "TestCaseGovSteam2.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GovSteamSGO                               #
###################################################################

case_name = "SMIB - TestCase - GovSteamSGO"
case_description = "Active power variation on the load"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseGovSteamSGO", "TestCaseGovSteamSGO.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the Pss1 power system stabilizer          #
###################################################################

case_name = "SMIB - TestCase - Pss1"
case_description = "Fault and line opening damped by the Pss1 stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePss1", "TestCasePss1.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the PssPTIST1 power system stabilizer     #
###################################################################

case_name = "SMIB - TestCase - PssPTIST1"
case_description = "Fault and line opening damped by the PssPTIST1 stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePssPTIST1", "TestCasePssPTIST1.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the PssPTIST3 power system stabilizer     #
###################################################################

case_name = "SMIB - TestCase - PssPTIST3"
case_description = "Fault and line opening damped by the PssPTIST3 stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePssPTIST3", "TestCasePssPTIST3.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the PssRQB power system stabilizer        #
###################################################################

case_name = "SMIB - TestCase - PssRQB"
case_description = "Fault and line opening damped by the PssRQB stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePssRQB", "TestCasePssRQB.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#  SMIB with the IEEE Type-1 voltage compensator (VCompIEEEType1) #
###################################################################

case_name = "SMIB - TestCase - VCompIEEEType1"
case_description = "Fault and line opening through the IEEE Type-1 voltage compensator (Rc = Xc = 0; reduces to identity)"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCaseVCompIEEEType1", "TestCaseVCompIEEEType1.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the PssELIN2 power system stabilizer      #
###################################################################

case_name = "SMIB - TestCase - PssELIN2"
case_description = "Fault and line opening damped by the PssELIN2 (Austrian ELIN type 2) stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePssELIN2", "TestCasePssELIN2.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the PssSB4 power system stabilizer        #
###################################################################

case_name = "SMIB - TestCase - PssSB4"
case_description = "Fault and line opening damped by the PssSB4 (power-sensitive STAB4) stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePssSB4", "TestCasePssSB4.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the PssSH power system stabilizer         #
###################################################################

case_name = "SMIB - TestCase - PssSH"
case_description = "Fault and line opening damped by the PssSH (Siemens H-infinity) stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePssSH", "TestCasePssSH.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the PssSK power system stabilizer         #
###################################################################

case_name = "SMIB - TestCase - PssSK"
case_description = "Fault and line opening damped by the PssSK (Slovak three-input) stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePssSK", "TestCasePssSK.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the PssWECC power system stabilizer       #
###################################################################

case_name = "SMIB - TestCase - PssWECC"
case_description = "Fault and line opening damped by the PssWECC stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePssWECC", "TestCasePssWECC.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with the Pss5 power system stabilizer          #
###################################################################

case_name = "SMIB - TestCase - Pss5"
case_description = "Fault and line opening damped by the Pss5 stabilizer"
job_file = os.path.join(os.path.dirname(__file__), "Standard", "TestCasePss5", "TestCasePss5.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))

###################################################################
#             SMIB with GeneratorPTanPhi                          #
###################################################################

case_name = "SMIB - GeneratorPTanPhi"
case_description = "GeneratorPTanPhi"
job_file = os.path.join(os.path.dirname(__file__), "SMIB_GeneratorPTanPhi", "GeneratorPTanPhi.jobs")

test_cases.append((case_name, case_description, job_file, 1, standardReturnCodeType, standardReturnCode))
