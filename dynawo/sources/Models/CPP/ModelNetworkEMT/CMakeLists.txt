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

set(MODEL_NAME
  DYNModelNetworkEMT
  )

# phasor tap-changer regulation logic, reused verbatim (only the transformer
# physics around it is EMT-specific); see DYNModelTwoWindingsTransformerEMT
set(MODELNETWORK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ModelNetwork)

set(MODEL_SOURCES
  DYNNetworkComponentEMT.cpp
  DYNModelBusEMT.cpp
  DYNModelSwitchEMT.cpp
  DYNModelLineEMT.cpp
  DYNModelTwoWindingsTransformerEMT.cpp
  DYNModelLoadEMT.cpp
  DYNModelGeneratorEMT.cpp
  DYNModelNetworkEMT.cpp
  ${MODELNETWORK_DIR}/DYNModelTapChanger.cpp
  ${MODELNETWORK_DIR}/DYNModelRatioTapChanger.cpp
  ${MODELNETWORK_DIR}/DYNModelPhaseTapChanger.cpp
  ${MODELNETWORK_DIR}/DYNModelCurrentLimits.cpp
  )

compile_cpp_model(${MODEL_NAME} "${MODEL_SOURCES}")

target_include_directories(${MODEL_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<TARGET_PROPERTY:dynawo_CPPModelsCommon,INTERFACE_INCLUDE_DIRECTORIES>
  )
target_include_directories(${MODEL_NAME}
  PRIVATE
  # IIDM data-interface headers (used in initializeFromData to build the network);
  # only pure-virtual interfaces are called, resolved at runtime by the host process
  $<TARGET_PROPERTY:dynawo_DataInterface,INTERFACE_INCLUDE_DIRECTORIES>
  # phasor ModelNetwork dir for the reused tap-changer headers/sources
  ${MODELNETWORK_DIR}
  )
target_include_directories(${MODEL_NAME} SYSTEM
  PRIVATE
  $<TARGET_PROPERTY:Boost::boost,INTERFACE_INCLUDE_DIRECTORIES>
  )

# KLU (SuiteSparse): the sparse linear solver used by the opt-in steady-state init seed
# correction (ModelNetworkEMT::applySnubberSeedCorrection). SuiteSparse is found at the top
# level (find_package(SuiteSparse REQUIRED)); link KLU and its ordering/config dependencies.
target_link_libraries(${MODEL_NAME} PRIVATE
  SuiteSparse::SuiteSparse_KLU
  SuiteSparse::SuiteSparse_AMD
  SuiteSparse::SuiteSparse_COLAMD
  SuiteSparse::SuiteSparse_BTF
  SuiteSparse::SuiteSparse_Config
  )
