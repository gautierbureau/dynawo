# Copyright (c) 2015-2020, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.
#
# Replacement for powsybl/powsybl-iidm4cpp:
# https://github.com/gautierbureau/iidm4cpp (project: iidm-bridge-cpp)
# CMake package: IidmBridge, exported target: IidmBridge::iidmbridge
# The library is a GraalVM/JNI bridge; the consumer still needs
# libpowsybl-iidm-native.so (GraalVM) or a JVM + powsybl-core at runtime.

cmake_minimum_required(VERSION 3.12)

include(${CMAKE_CURRENT_SOURCE_DIR}/../cmake/CPUCount.cmake)
if(NOT DEFINED CPU_COUNT)
  message(FATAL_ERROR "CPUCount.cmake: file not found.")
endif()

set(package_name        "libiidm")
set(package_config_dir  "IidmBridge")
set(package_install_dir "${CMAKE_INSTALL_PREFIX}/${package_name}")

# Pinning the master branch by commit SHA keeps builds reproducible while the
# upstream project is in rapid pre-1.0 iteration. Bump this together with any
# API migration work.
set(iidm_bridge_git_tag "b8590f2")

if(DEFINED ENV{DYNAWO_LIBIIDM_GIT_URL})
  set(iidm_bridge_git_url $ENV{DYNAWO_LIBIIDM_GIT_URL})
else()
  set(iidm_bridge_git_url https://github.com/gautierbureau/iidm4cpp.git)
endif()

set(CMAKE_PREFIX_PATH "${package_install_dir}/lib/cmake/${package_config_dir};${package_install_dir}")

find_package(IidmBridge QUIET CONFIG)

if(IidmBridge_FOUND)
  add_custom_target("${package_name}" DEPENDS libxml2 boost)
  message(STATUS "Found IidmBridge ${IidmBridge_VERSION}")
else()
  include(ExternalProject)
  ExternalProject_Add(
                      "${package_name}"

    DEPENDS           libxml2 boost
    INSTALL_DIR       "${package_install_dir}"

    GIT_REPOSITORY    ${iidm_bridge_git_url}
    GIT_TAG           ${iidm_bridge_git_tag}
    GIT_SHALLOW       0
    UPDATE_DISCONNECTED 1

    DOWNLOAD_DIR      "${CMAKE_CURRENT_SOURCE_DIR}/${package_name}"
    TMP_DIR           "${TMP_DIR}"
    STAMP_DIR         "${DOWNLOAD_DIR}/${package_name}-stamp"
    BINARY_DIR        "${DOWNLOAD_DIR}/${package_name}-build"
    SOURCE_DIR        "${DOWNLOAD_DIR}/${package_name}"

    CMAKE_CACHE_ARGS  -DCMAKE_CXX_COMPILER:STRING=${CMAKE_CXX_COMPILER}
                      -DCMAKE_CXX_FLAGS_INIT:STRING=$<$<CONFIG:Debug>:-O0>

    CMAKE_ARGS        "-DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>"
                      "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
                      "-DBOOST_ROOT:PATH=${BOOST_ROOT}"
                      "$<$<BOOL:${MSVC}>:-DBOOST_LIBRARYDIR=${BOOST_ROOT}/bin>"
                      "$<$<BOOL:${FORCE_CXX11_ABI}>:-DCMAKE_CXX_FLAGS=-D_GLIBCXX_USE_CXX11_ABI=1>"
                      "$<$<BOOL:${MSVC}>:-DINSTALL_LIB_DIR:STRING=bin>"
                      "-DBUILD_SHARED_LIBS=ON"
                      "-DIIDM_BRIDGE_BUILD_TESTS=OFF"
                      "-DIIDM_BRIDGE_BUILD_EXAMPLES=OFF"
                      "-DIIDM_BRIDGE_BUILD_JAVA=OFF"
                      "-DIIDM_BRIDGE_BUILD_NATIVE=OFF"
  )

  unset(iidm_bridge_git_url)
  unset(iidm_bridge_git_tag)
endif()

unset(package_install_dir)
unset(package_config_dir)
unset(package_name)
