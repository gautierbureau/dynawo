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

#include "DYNPhaseTapChangerInterfaceIIDM.h"

#include "DYNStepInterface.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/TwoWindingsTransformer.h>

#include "gtest_dynawo.h"

namespace DYN {

TEST(DataInterfaceTest, PhaseTapChanger_2WT) {
  iidm::Network network = iidm::NetworkFactory::load("resources/PhaseTapChanger.xiidm");
  iidm::TwoWindingsTransformer twoWT = network.getTwoWindingsTransformer("2WT_VL1_VL2").value();
  iidm::PhaseTapChanger ptc = twoWT.getPhaseTapChanger().value();

  PhaseTapChangerInterfaceIIDM Ifce(ptc);

  ASSERT_EQ(Ifce.getNbTap(), 3);
  ASSERT_EQ(Ifce.getSteps().size(), 3);
  ASSERT_EQ(Ifce.getLowPosition(), 0);
  ASSERT_EQ(Ifce.getCurrentPosition(), 1);

  ASSERT_TRUE(Ifce.isCurrentLimiter());
  ASSERT_FALSE(Ifce.getRegulating());
  ASSERT_DOUBLE_EQ(Ifce.getRegulationValue(), 770.0);
  // a non-regulating tap changer has no effective dead band
  ASSERT_DOUBLE_EQ(Ifce.getTargetDeadBand(), 0.0);

  Ifce.setCurrentPosition(2);
  ASSERT_EQ(Ifce.getCurrentPosition(), 2);
  Ifce.setCurrentPosition(1);
  ASSERT_EQ(Ifce.getCurrentPosition(), 1);

  // step values, indexed by (currentPosition - lowPosition)
  Ifce.setCurrentPosition(0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentAlpha(), 3.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentRho(), 13.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentR(), 12.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentX(), 14.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentG(), 11.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentB(), 10.0);
  Ifce.setCurrentPosition(1);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentAlpha(), 2.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentRho(), 18.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentR(), 17.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentX(), 19.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentG(), 16.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentB(), 15.0);
  Ifce.setCurrentPosition(2);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentAlpha(), 1.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentRho(), 23.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentR(), 22.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentX(), 24.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentG(), 21.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentB(), 20.0);
}

}  // namespace DYN
