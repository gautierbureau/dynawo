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

StepInterfaceIIDM::StepInterfaceIIDM(const SyntheticStep& step) : syntheticStep_(step),
                                                                  kind_(Kind::SYNTHETIC) {
}

double
StepInterfaceIIDM::getR() const {
  switch (kind_) {
    case Kind::PHASE:     return phaseStep_->getR();
    case Kind::RATIO:     return ratioStep_->getR();
    case Kind::SYNTHETIC: return syntheticStep_.r;
  }
  return 0.0;
}

double
StepInterfaceIIDM::getX() const {
  switch (kind_) {
    case Kind::PHASE:     return phaseStep_->getX();
    case Kind::RATIO:     return ratioStep_->getX();
    case Kind::SYNTHETIC: return syntheticStep_.x;
  }
  return 0.0;
}

double
StepInterfaceIIDM::getG() const {
  switch (kind_) {
    case Kind::PHASE:     return phaseStep_->getG();
    case Kind::RATIO:     return ratioStep_->getG();
    case Kind::SYNTHETIC: return syntheticStep_.g;
  }
  return 0.0;
}

double
StepInterfaceIIDM::getB() const {
  switch (kind_) {
    case Kind::PHASE:     return phaseStep_->getB();
    case Kind::RATIO:     return ratioStep_->getB();
    case Kind::SYNTHETIC: return syntheticStep_.b;
  }
  return 0.0;
}

double
StepInterfaceIIDM::getRho() const {
  switch (kind_) {
    case Kind::PHASE:     return phaseStep_->getRho();
    case Kind::RATIO:     return ratioStep_->getRho();
    case Kind::SYNTHETIC: return syntheticStep_.rho;
  }
  return 0.0;
}

double
StepInterfaceIIDM::getAlpha() const {
  switch (kind_) {
    case Kind::PHASE:     return phaseStep_->getAlpha();
    case Kind::RATIO:     return 0.0;
    case Kind::SYNTHETIC: return 0.0;
  }
  return 0.0;
}

}  // namespace DYN
