//
// Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
// See AUTHORS.txt
// All rights reserved.
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, you can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
//
// This file is part of Dynawo, an hybrid C++/Modelica open source time domain
// simulation tool for power systems.
//

/**
 * @file  DYNSimulationContext.cpp
 *
 * @brief Simulation context implementation
 *
 */

#include "DYNSimulationContext.h"

namespace DYN {

void
SimulationContext::setWorkingDirectory(const std::string& directory) {
  workingDirectory_ = directory;
}

void
SimulationContext::setInputDirectory(const std::string& directory) {
  inputDirectory_ = directory;
}

void
SimulationContext::setResourcesDirectory(const std::string& directory) {
  resourcesDirectory_ = directory;
}

void
SimulationContext::setLocale(const std::string& locale) {
  locale_ = locale;
}

void
SimulationContext::setInputFiles(const std::shared_ptr<const InputFiles>& inputFiles) {
  inputFiles_ = inputFiles ? inputFiles : std::make_shared<InputFiles>();
}

const std::string&
SimulationContext::getWorkingDirectory() {
  return workingDirectory_;
}

const std::string&
SimulationContext::getInputDirectory() {
  return inputDirectory_;
}

const std::string&
SimulationContext::getResourcesDirectory() {
  return resourcesDirectory_;
}

const std::string&
SimulationContext::getLocale() {
  return locale_;
}

const InputFiles&
SimulationContext::getInputFiles() const {
  return *inputFiles_;
}

const std::shared_ptr<const InputFiles>&
SimulationContext::getSharedInputFiles() const {
  return inputFiles_;
}

}  // end of namespace DYN
