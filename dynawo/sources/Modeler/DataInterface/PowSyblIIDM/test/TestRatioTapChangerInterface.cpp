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

#include "DYNRatioTapChangerInterfaceIIDM.h"

#include "DYNDataInterfaceIIDM.h"
#include "DYNNetworkInterface.h"
#include "DYNRatioTapChangerInterface.h"
#include "DYNStepInterface.h"
#include "DYNTwoWTransformerInterface.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/ThreeWindingsTransformer.h>
#include <iidm/TwoWindingsTransformer.h>

#include "gtest_dynawo.h"

namespace DYN {

TEST(DataInterfaceTest, RatioTapChanger_2WT) {
  iidm::Network network = iidm::NetworkFactory::load("resources/RatioTapChanger.xiidm");
  iidm::TwoWindingsTransformer twoWT = network.getTwoWindingsTransformer("2WT_VL1_VL2").value();
  iidm::RatioTapChanger rtc = twoWT.getRatioTapChanger().value();

  RatioTapChangerInterfaceIIDM Ifce(rtc, "ONE");

  ASSERT_EQ(Ifce.getNbTap(), 3);
  ASSERT_EQ(Ifce.getSteps().size(), 3);
  ASSERT_EQ(Ifce.getLowPosition(), 1);
  ASSERT_EQ(Ifce.getCurrentPosition(), 2);

  Ifce.setCurrentPosition(3);
  ASSERT_EQ(Ifce.getCurrentPosition(), 3);
  Ifce.setCurrentPosition(2);
  ASSERT_EQ(Ifce.getCurrentPosition(), 2);

  // step values, indexed by (currentPosition - lowPosition)
  Ifce.setCurrentPosition(1);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentRho(), 13.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentR(), 12.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentX(), 14.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentG(), 11.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentB(), 10.0);
  Ifce.setCurrentPosition(2);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentRho(), 18.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentR(), 17.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentX(), 19.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentG(), 16.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentB(), 15.0);
  Ifce.setCurrentPosition(3);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentRho(), 23.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentR(), 22.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentX(), 24.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentG(), 21.0);
  ASSERT_DOUBLE_EQ(Ifce.getCurrentB(), 20.0);

  ASSERT_TRUE(Ifce.hasLoadTapChangingCapabilities());
  ASSERT_TRUE(Ifce.getRegulating());
  ASSERT_DOUBLE_EQ(Ifce.getTargetV(), 225.0);
  ASSERT_DOUBLE_EQ(Ifce.getTargetDeadBand(), 0.1);
  ASSERT_EQ(Ifce.getTerminalRefSide(), "ONE");
  // the ratio tap changer regulates the transformer's own terminal
  ASSERT_EQ(Ifce.getTerminalRefId(), "2WT_VL1_VL2");
}

TEST(DataInterfaceTest, RatioTapChanger_3WT) {
  iidm::Network network = iidm::NetworkFactory::load("resources/RatioTapChanger3WT.xiidm");
  iidm::ThreeWindingsTransformer threeWT = network.getThreeWindingsTransformer("3WT_VL1_VL2_VL3").value();

  RatioTapChangerInterfaceIIDM IfceLeg2(threeWT.getLeg2().getRatioTapChanger(), "TWO");
  ASSERT_EQ(IfceLeg2.getNbTap(), 3);
  ASSERT_EQ(IfceLeg2.getSteps().size(), 3);
  ASSERT_EQ(IfceLeg2.getLowPosition(), 1);
  ASSERT_EQ(IfceLeg2.getCurrentPosition(), 2);
  ASSERT_FALSE(IfceLeg2.hasLoadTapChangingCapabilities());

  RatioTapChangerInterfaceIIDM IfceLeg3(threeWT.getLeg3().getRatioTapChanger(), "THREE");
  ASSERT_EQ(IfceLeg3.getNbTap(), 4);
  ASSERT_EQ(IfceLeg3.getSteps().size(), 4);
  ASSERT_EQ(IfceLeg3.getLowPosition(), -3);
  ASSERT_EQ(IfceLeg3.getCurrentPosition(), -2);
}

// Regression test: the ratio tap changer regulates the transformer's own
// terminal (side TWO). DataInterfaceIIDM must derive that side so the network
// model builds the load tap changer; comparing iidm::Terminal handles is not
// reliable for this, hence the bus-based comparison in addTwoWTransformer().
TEST(DataInterfaceTest, RatioTapChanger_sideDetection) {
  boost::shared_ptr<DataInterface> data = DataInterfaceIIDM::build("resources/RatioTapChanger.xiidm");
  boost::shared_ptr<NetworkInterface> network = data->getNetwork();

  bool found = false;
  for (const auto& twt : network->getTwoWTransformers()) {
    if (twt->getID() == "2WT_VL1_VL2") {
      found = true;
      ASSERT_NE(twt->getRatioTapChanger().get(), nullptr);
      ASSERT_EQ(twt->getRatioTapChanger()->getTerminalRefSide(), "TWO");
    }
  }
  ASSERT_TRUE(found);
}

}  // namespace DYN
