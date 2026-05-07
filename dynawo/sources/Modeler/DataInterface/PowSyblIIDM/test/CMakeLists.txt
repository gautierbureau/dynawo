# Copyright (c) 2015-2020, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.

set(MODULE_NAME MODELER_DATAINTERFACEIIDM_unittest)

# Temporarily restricted to the tests that exercise iidm::NetworkFactory::load
# (and related XIIDM-loading paths) to bisect a segmentation fault. Re-enable
# the rest once the load path is stable.
set(MODULE_SOURCES
  TestBatteryInterface.cpp
  TestBusInterface.cpp
  TestDanglingLineInterface.cpp
  TestGeneratorInterface.cpp
  TestHvdcLineInterface.cpp
  TestInjectorInterface.cpp
  TestLccConverterInterface.cpp
  TestLineInterface.cpp
  TestLoadInterface.cpp
  TestShuntCompensatorInterface.cpp
  TestStaticVarCompensatorInterface.cpp
  TestSwitchInterface.cpp
  TestVscConverterInterface.cpp
)

add_executable(${MODULE_NAME} ${MODULE_SOURCES})

target_link_libraries(${MODULE_NAME}
  dynawo_API_PAR
  dynawo_API_CRT
  dynawo_API_TL
  dynawo_API_LEQ
  dynawo_ModelerCommon
  dynawo_DataInterfaceIIDM
  dynawo_DataInterface
  IidmBridge::iidmbridge
  dynawo_Test)

add_custom_target(${MODULE_NAME}-pre-tests
  COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/resources ${CMAKE_CURRENT_BINARY_DIR}/resources)

add_custom_target(${MODULE_NAME}-tests
  COMMAND ${CMAKE_COMMAND} -E env "${runtime_tests_PATH}"
  "DYNAWO_LIBIIDM_EXTENSIONS=${LIBIIDM_HOME}/lib" $<TARGET_FILE:${MODULE_NAME}>
  DEPENDS ${MODULE_NAME}-pre-tests ${MODULE_NAME}
  COMMENT "Running ${MODULE_NAME}...")

if(BUILD_TESTS_COVERAGE)
  set(EXTRACT_PATTERNS "'*/sources/Modeler/DataInterface/PowSyblIIDM/DYN*'")
  add_test_coverage(${MODULE_NAME}-tests "${EXTRACT_PATTERNS}")
endif()

if(BUILD_TESTS)
  add_test_run(${MODULE_NAME}-tests)
endif()
