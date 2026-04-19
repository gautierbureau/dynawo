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
 * @file  DYNPhaseTapChangerInterfaceIIDM.cpp
 *
 * @brief Phase tap changer data interface : implementation file for IIDM implementation
 *
 */

#include "DYNPhaseTapChangerInterfaceIIDM.h"
#include "DYNStepInterfaceIIDM.h"
#include "make_unique.hpp"

#include <iidm/PhaseTapChanger.h>

#include <cmath>


namespace DYN {

PhaseTapChangerInterfaceIIDM::PhaseTapChangerInterfaceIIDM(iidm::PhaseTapChanger tapChanger) : tapChangerIIDM_(tapChanger) {
  // TODO(iidm-bridge): iidm::PhaseTapChangerStep has no multi-arg constructor; iterate through getAllSteps().
  const auto all = tapChanger.getAllSteps();
  for (const auto& s : all) {
    steps_.push_back(DYN::make_unique<StepInterfaceIIDM>(s));
  }
}

void
PhaseTapChangerInterfaceIIDM::addStep(std::unique_ptr<StepInterface> step) {
  steps_.push_back(std::move(step));
}

const std::vector<std::unique_ptr<StepInterface> >&
PhaseTapChangerInterfaceIIDM::getSteps() const {
  return steps_;
}

int
PhaseTapChangerInterfaceIIDM::getCurrentPosition() const {
  return static_cast<int>(tapChangerIIDM_.getTapPosition());
}

void
PhaseTapChangerInterfaceIIDM::setCurrentPosition(const int& position) {
  tapChangerIIDM_.setTapPosition(position);
}

int
PhaseTapChangerInterfaceIIDM::getLowPosition() const {
  return static_cast<int>(tapChangerIIDM_.getLowTapPosition());
}

unsigned int
PhaseTapChangerInterfaceIIDM::getNbTap() const {
  return static_cast<unsigned int>(steps_.size());
}

bool
PhaseTapChangerInterfaceIIDM::isCurrentLimiter() const {
  return tapChangerIIDM_.getRegulationMode() == iidm::PhaseTapChangerRegulationMode::CURRENT_LIMITER;
}

bool
PhaseTapChangerInterfaceIIDM::getRegulating() const {
  return tapChangerIIDM_.isRegulating();
}

double
PhaseTapChangerInterfaceIIDM::getRegulationValue() const {
  if (std::isnan(tapChangerIIDM_.getRegulationValue())) {
    return 99999.0;
  }
  return tapChangerIIDM_.getRegulationValue();
}

double
PhaseTapChangerInterfaceIIDM::getCurrentR() const {
  auto i = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(i)->getR();
}

double
PhaseTapChangerInterfaceIIDM::getCurrentX() const {
  auto i = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(i)->getX();
}

double
PhaseTapChangerInterfaceIIDM::getCurrentB() const {
  auto i = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(i)->getB();
}

double
PhaseTapChangerInterfaceIIDM::getCurrentG() const {
  auto i = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(i)->getG();
}

double
PhaseTapChangerInterfaceIIDM::getCurrentRho() const {
  auto i = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(i)->getRho();
}

double
PhaseTapChangerInterfaceIIDM::getCurrentAlpha() const {
  auto i = tapChangerIIDM_.getTapPosition() - tapChangerIIDM_.getLowTapPosition();
  return steps_.at(i)->getAlpha();
}

double
PhaseTapChangerInterfaceIIDM::getTargetDeadBand() const {
  if (!getRegulating()) {
    return 0.;
  }
  const auto d = tapChangerIIDM_.getTargetDeadband();
  return d.has_value() ? d.value() : 0.;
}

}  // namespace DYN
