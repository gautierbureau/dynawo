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

#include "DYNHvdcLineInterfaceIIDM.h"

#include "DYNLccConverterInterfaceIIDM.h"
#include "DYNVscConverterInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/HvdcLine.h>
#include <iidm/Enums.h>

#include "gtest_dynawo.h"

namespace DYN {

static iidm::VoltageLevel findVL(iidm::Network& net, const std::string& id) {
  for (auto& vl : net.getVoltageLevels())
    if (vl.getId() == id) return vl;
  throw std::runtime_error("VoltageLevel not found: " + id);
}

TEST(DataInterfaceTest, HvdcLine) {
  auto net = iidm::NetworkFactory::load("resources/hvdc.xiidm");
  auto hvdcLine = net.getHvdcLine("HVDC1").value();
  auto lcc = net.getLccConverterStation("LCC1").value();
  auto vsc = net.getVscConverterStation("VSC2").value();
  auto vl1 = findVL(net, "VL1");

  const std::shared_ptr<LccConverterInterface> LccIfce = std::make_shared<LccConverterInterfaceIIDM>(lcc);
  const std::shared_ptr<VscConverterInterface> VscIfce = std::make_shared<VscConverterInterfaceIIDM>(vsc);

  HvdcLineInterfaceIIDM Ifce(hvdcLine, LccIfce, VscIfce);
  const std::shared_ptr<VoltageLevelInterface> vlItf = std::make_shared<VoltageLevelInterfaceIIDM>(vl1);
  LccIfce->setVoltageLevelInterface(vlItf);
  VscIfce->setVoltageLevelInterface(vlItf);

  ASSERT_EQ(Ifce.getID(), "HVDC1");

  ASSERT_EQ(Ifce.getIdConverter1(), "LCC1");
  ASSERT_EQ(Ifce.getConverter1().get()->getID(), "LCC1");
  ASSERT_EQ(Ifce.getIdConverter2(), "VSC2");
  ASSERT_EQ(Ifce.getConverter2().get()->getID(), "VSC2");

  ASSERT_DOUBLE_EQ(Ifce.getResistanceDC(), 14.0);
  ASSERT_DOUBLE_EQ(Ifce.getVNom(), 13.0);
  ASSERT_DOUBLE_EQ(Ifce.getPmax(), 12.0);
  ASSERT_DOUBLE_EQ(Ifce.getActivePowerSetpoint(), 111.1);

  ASSERT_EQ(Ifce.getComponentVarIndex(std::string("p1")), HvdcLineInterfaceIIDM::VAR_P1);
  ASSERT_EQ(Ifce.getComponentVarIndex(std::string("p2")), HvdcLineInterfaceIIDM::VAR_P2);
  ASSERT_EQ(Ifce.getComponentVarIndex(std::string("q1")), HvdcLineInterfaceIIDM::VAR_Q1);
  ASSERT_EQ(Ifce.getComponentVarIndex(std::string("q2")), HvdcLineInterfaceIIDM::VAR_Q2);
  ASSERT_EQ(Ifce.getComponentVarIndex(std::string("state1")), HvdcLineInterfaceIIDM::VAR_STATE1);
  ASSERT_EQ(Ifce.getComponentVarIndex(std::string("state2")), HvdcLineInterfaceIIDM::VAR_STATE2);
  ASSERT_EQ(Ifce.getComponentVarIndex(std::string("invalid")), -1);

  ASSERT_EQ(Ifce.getConverterMode(), HvdcLineInterface::RECTIFIER_INVERTER);
  hvdcLine.setConvertersMode(iidm::HvdcConverterStationMode::INVERTER);
  ASSERT_EQ(Ifce.getConverterMode(), HvdcLineInterface::INVERTER_RECTIFIER);
  hvdcLine.setConvertersMode(iidm::HvdcConverterStationMode::RECTIFIER);
  ASSERT_EQ(Ifce.getConverterMode(), HvdcLineInterface::RECTIFIER_INVERTER);

  ASSERT_FALSE(Ifce.getDroop());
  ASSERT_FALSE(Ifce.getP0());
  ASSERT_FALSE(Ifce.isActivePowerControlEnabled());
  ASSERT_FALSE(Ifce.getOprFromCS1toCS2());
  ASSERT_FALSE(Ifce.getOprFromCS2toCS1());

  ASSERT_TRUE(Ifce.isConnected());
  ASSERT_TRUE(Ifce.isPartiallyConnected());
  lcc.getTerminal().disconnect();
  ASSERT_FALSE(Ifce.isConnected());
  ASSERT_TRUE(Ifce.isPartiallyConnected());
  vsc.getTerminal().disconnect();
  ASSERT_FALSE(Ifce.isConnected());
  ASSERT_FALSE(Ifce.isPartiallyConnected());
  lcc.getTerminal().connect();
  ASSERT_FALSE(Ifce.isConnected());
  ASSERT_TRUE(Ifce.isPartiallyConnected());

  ASSERT_TRUE(Ifce.hasInitialConditions());
}

}  // namespace DYN
