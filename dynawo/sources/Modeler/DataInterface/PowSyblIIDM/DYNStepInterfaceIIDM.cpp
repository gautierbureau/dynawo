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
 * @file  DYNStepInterfaceIIDM.cpp
 *
 * @brief Step data interface : implementation file for IIDM implementation
 *
 */

#include "DYNStepInterfaceIIDM.h"

namespace DYN {

StepInterfaceIIDM::StepInterfaceIIDM(const iidm::PhaseTapChangerStep& step) : phaseStep_(step),
                                                                              kind_(Kind::PHASE) {
}

StepInterfaceIIDM::StepInterfaceIIDM(const iidm::RatioTapChangerStep& step) : ratioStep_(step),
                                                                              kind_(Kind::RATIO) {
}

double
StepInterfaceIIDM::getR() const {
  return kind_ == Kind::PHASE ? phaseStep_->getR() : ratioStep_->getR();
}

double
StepInterfaceIIDM::getX() const {
  return kind_ == Kind::PHASE ? phaseStep_->getX() : ratioStep_->getX();
}

double
StepInterfaceIIDM::getG() const {
  return kind_ == Kind::PHASE ? phaseStep_->getG() : ratioStep_->getG();
}

double
StepInterfaceIIDM::getB() const {
  return kind_ == Kind::PHASE ? phaseStep_->getB() : ratioStep_->getB();
}

double
StepInterfaceIIDM::getRho() const {
  return kind_ == Kind::PHASE ? phaseStep_->getRho() : ratioStep_->getRho();
}

double
StepInterfaceIIDM::getAlpha() const {
  return kind_ == Kind::PHASE ? phaseStep_->getAlpha() : 0.0;
}

}  // namespace DYN
