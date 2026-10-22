# Copyright (c) 2026, RTE (http://www.rte-france.com)
# See AUTHORS.txt
# All rights reserved.
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, you can obtain one at http://mozilla.org/MPL/2.0/.
# SPDX-License-Identifier: MPL-2.0
#
# This file is part of Dynawo, an hybrid C++/Modelica open source time domain simulation tool for power systems.

# The native methods of com.powsybl.dynawo.jni.DynawoNative: Dynawo run from a JVM, on a network of that JVM.
# It only needs the JNI headers: it runs in the process of a JVM, whose libjvm is already loaded.
find_package(JNI)

if(JAVA_INCLUDE_PATH AND JAVA_INCLUDE_PATH2)
  add_library(dynawo_JNILauncher SHARED DYNJNILauncher.cpp)

  target_include_directories(dynawo_JNILauncher
    PRIVATE
      ${JAVA_INCLUDE_PATH}
      ${JAVA_INCLUDE_PATH2}
      $<TARGET_PROPERTY:dynawo_SimulationCommon,INTERFACE_INCLUDE_DIRECTORIES>
    )

  target_link_libraries(dynawo_JNILauncher
    PRIVATE
      dynawo_Common
      dynawo_Simulation
      dynawo_SimulationCommon
      dynawo_API_PAR
      dynawo_API_CRT
      dynawo_API_TL
      dynawo_API_LEQ
      dynawo_ModelerCommon
      dynawo_DataInterface
      dynawo_DataInterfaceIIDM
      XMLSAXParser${LibXML_LINK_SUFFIX}
      LibXml2::LibXml2
      Boost::filesystem
    )

  set_target_properties(dynawo_JNILauncher PROPERTIES VERSION ${DYNAWO_VERSION_STRING}
                                                      SOVERSION ${DYNAWO_VERSION_MAJOR})

  install(TARGETS dynawo_JNILauncher DESTINATION ${LIBDIR_NAME})
else()
  message(STATUS "JNI headers not found: dynawo_JNILauncher, to run Dynawo from a JVM, is not built")
endif()
