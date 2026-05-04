//
// Copyright (c) 2015-2020, RTE (http://www.rte-france.com)
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
 * @file  DYNRatioTapChangerInterfaceIIDM.cpp
 *
 * @brief Ratio tap changer data interface : implementation file for IIDM implementation
 *
 */

#include "DYNRatioTapChangerInterfaceIIDM.h"

#include "DYNStepInterfaceIIDM.h"

#include "make_unique.hpp"

#include <cmath>

using boost::shared_ptr;

namespace DYN {

RatioTapChangerInterfaceIIDM::RatioTapChangerInterfaceIIDM(iidm::RatioTapChanger tapChanger, const std::string& terminalRefSide) :
  tapChangerIIDM_(tapChanger),
  terminalRefSide_(terminalRefSide) {
  for (const auto& step : tapChanger.getAllSteps()) {
    steps_.push_back(DYN::make_unique<StepInterfaceIIDM>(step));
  }
}

void
RatioTapChangerInterfaceIIDM::addStep(std::unique_ptr<StepInterface> step) {
  steps_.push_back(std::move(step));
}

const std::vector<std::unique_ptr<StepInterface> >&
RatioTapChangerInterfaceIIDM::getSteps() const {
  return steps_;
}

int
RatioTapChangerInterfaceIIDM::getCurrentPosition() const {
  return static_cast<int>(tapChangerIIDM_.getTapPosition());  // getTapPosition() is 'long' in powsybl
}

void
RatioTapChangerInterfaceIIDM::setCurrentPosition(const int& position) {
  tapChangerIIDM_.setTapPosition(position);
}

int
RatioTapChangerInterfaceIIDM::getLowPosition() const {
  return static_cast<int>(tapChangerIIDM_.getLowTapPosition());  // NOLINT renamed from getLowTapPosition
}

unsigned int
RatioTapChangerInterfaceIIDM::getNbTap() const {
  return static_cast<unsigned int>(steps_.size());
}

bool
RatioTapChangerInterfaceIIDM::hasLoadTapChangingCapabilities() const {
  return tapChangerIIDM_.hasLoadTapChangingCapabilities();
}

bool
RatioTapChangerInterfaceIIDM::getRegulating() const {
  return tapChangerIIDM_.isRegulating();
}

double
RatioTapChangerInterfaceIIDM::getTargetV() const {
  if (std::isnan(tapChangerIIDM_.getTargetV())) {
    return 99999.0;
  }
  return tapChangerIIDM_.getTargetV();
}

std::string
RatioTapChangerInterfaceIIDM::getTerminalRefId() const {
  return getRegulating() ? tapChangerIIDM_.getRegulationTerminal().getConnectableId() : std::string();
}

std::string
RatioTapChangerInterfaceIIDM::getTerminalRefSide() const {
  return terminalRefSide_;
}

double
RatioTapChangerInterfaceIIDM::getCurrentR() const {
  auto currentStep = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(currentStep)->getR();
}

double
RatioTapChangerInterfaceIIDM::getCurrentX() const {
  auto currentStep = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(currentStep)->getX();
}

double
RatioTapChangerInterfaceIIDM::getCurrentB() const {
  auto currentStep = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(currentStep)->getB();
}

double
RatioTapChangerInterfaceIIDM::getCurrentG() const {
  auto currentStep = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(currentStep)->getG();
}

double
RatioTapChangerInterfaceIIDM::getCurrentRho() const {
  auto currentStep = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(currentStep)->getRho();
}

double
RatioTapChangerInterfaceIIDM::getTargetDeadBand() const {
  if (!getRegulating()) {
    return 0.;
  }
  const auto d = tapChangerIIDM_.getTargetDeadband();
  return d.has_value() ? d.value() : 0.;
}

}  // namespace DYN
