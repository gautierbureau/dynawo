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
 * @file  DataInterface/PowSyblIIDM/DYNStepInterfaceIIDM.h
 *
 * @brief Step data interface : header file for IIDM implementation
 *
 */
#ifndef MODELER_DATAINTERFACE_POWSYBLIIDM_DYNSTEPINTERFACEIIDM_H_
#define MODELER_DATAINTERFACE_POWSYBLIIDM_DYNSTEPINTERFACEIIDM_H_

#include "DYNStepInterface.h"

#include <iidm/PhaseTapChangerStep.h>
#include <iidm/RatioTapChangerStep.h>

#include <boost/optional.hpp>

namespace DYN {

/**
 * class StepInterfaceIIDM
 */
class StepInterfaceIIDM : public StepInterface {
 public:
  /**
   * @brief Constructor
   * @param step phase tap changer step's iidm instance
   */
  explicit StepInterfaceIIDM(const iidm::PhaseTapChangerStep& step);

  /**
   * @brief Constructor
   * @param step ratio tap changer step's iidm instance
   */
  explicit StepInterfaceIIDM(const iidm::RatioTapChangerStep& step);

  /**
   * @copydoc StepInterface::getR() const
   */
  double getR() const override;

  /**
   * @copydoc StepInterface::getX() const
   */
  double getX() const override;

  /**
   * @copydoc StepInterface::getB() const
   */
  double getB() const override;

  /**
   * @copydoc StepInterface::getG() const
   */
  double getG() const override;

  /**
   * @copydoc StepInterface::getRho() const
   */
  double getRho() const override;

  /**
   * @copydoc StepInterface::getAlpha() const
   */
  double getAlpha() const override;

 private:
  /// Kind of step stored.
  enum class Kind { PHASE, RATIO };

  boost::optional<iidm::PhaseTapChangerStep> phaseStep_;  ///< reference to the iidm phase tap changer step instance
  boost::optional<iidm::RatioTapChangerStep> ratioStep_;  ///< reference to the iidm ratio tap changer step instance
  Kind kind_;                                             ///< discriminant for the step kind
};                                                        ///< interface class for step of tap changer

}  // namespace DYN

#endif  // MODELER_DATAINTERFACE_POWSYBLIIDM_DYNSTEPINTERFACEIIDM_H_
