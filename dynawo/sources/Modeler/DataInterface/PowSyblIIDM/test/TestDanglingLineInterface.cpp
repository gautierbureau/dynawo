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

#include "DYNDanglingLineInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNCurrentLimitInterfaceIIDM.h"
#include "DYNInjectorInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"
#include "make_unique.hpp"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/DanglingLine.h>

#include "gtest_dynawo.h"

namespace DYN {

static iidm::VoltageLevel findVL(iidm::Network& net, const std::string& id) {
  for (auto& vl : net.getVoltageLevels())
    if (vl.getId() == id) return vl;
  throw std::runtime_error("VoltageLevel not found: " + id);
}

static iidm::Bus findBus(iidm::VoltageLevel& vl, const std::string& id) {
  for (auto& b : vl.getBusBreakerView().getBuses())
    if (b.getId() == id) return b;
  throw std::runtime_error("Bus not found: " + id);
}

TEST(DataInterfaceTest, DanglingLine_1) {
  auto net = iidm::NetworkFactory::load("resources/dangling_line.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus1 = findBus(vl, "VL1_BUS1");
  auto danglingLine = net.getDanglingLine("DANGLING_LINE1").value();

  DanglingLineInterfaceIIDM danglingLineIfce(danglingLine);
  const std::shared_ptr<VoltageLevelInterface> vlItf = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  danglingLineIfce.setVoltageLevelInterface(vlItf);

  ASSERT_EQ(danglingLineIfce.getComponentVarIndex(std::string("p")), DanglingLineInterfaceIIDM::VAR_P);
  ASSERT_EQ(danglingLineIfce.getComponentVarIndex(std::string("wrongIndex")), -1);
  ASSERT_EQ(danglingLineIfce.getComponentVarIndex(std::string("q")), DanglingLineInterfaceIIDM::VAR_Q);
  ASSERT_EQ(danglingLineIfce.getComponentVarIndex(std::string("state")), DanglingLineInterfaceIIDM::VAR_STATE);

  ASSERT_EQ(danglingLineIfce.getID(), danglingLine.getId());

  ASSERT_TRUE(danglingLineIfce.getInitialConnected());
  ASSERT_TRUE(danglingLineIfce.isConnected());
  ASSERT_TRUE(danglingLineIfce.isPartiallyConnected());
  danglingLine.getTerminal().disconnect();
  ASSERT_FALSE(danglingLineIfce.isPartiallyConnected());
  ASSERT_FALSE(danglingLineIfce.isConnected());
  ASSERT_TRUE(danglingLineIfce.getInitialConnected());

  ASSERT_EQ(danglingLineIfce.getBusInterface().get(), nullptr);
  std::unique_ptr<BusInterface> busIfce = DYN::make_unique<BusInterfaceIIDM>(bus1, vl);
  danglingLineIfce.setBusInterface(std::move(busIfce));
  ASSERT_EQ(danglingLineIfce.getBusInterface().get()->getID(), "VL1_BUS1");

  ASSERT_FALSE(danglingLineIfce.hasInitialConditions());

  ASSERT_DOUBLE_EQ(danglingLineIfce.getP0(), 3.0);
  ASSERT_DOUBLE_EQ(danglingLineIfce.getP(), 0.0);
  danglingLine.getTerminal().setP(10.0);
  ASSERT_DOUBLE_EQ(danglingLineIfce.getP(), 10.0);

  ASSERT_DOUBLE_EQ(danglingLineIfce.getQ0(), 4.0);
  ASSERT_DOUBLE_EQ(danglingLineIfce.getQ(), 0.0);
  danglingLine.getTerminal().setQ(40.0);
  ASSERT_DOUBLE_EQ(danglingLineIfce.getQ(), 40.0);

  ASSERT_DOUBLE_EQ(danglingLineIfce.getB(), 1.0);
  ASSERT_DOUBLE_EQ(danglingLineIfce.getG(), 2.0);
  ASSERT_DOUBLE_EQ(danglingLineIfce.getR(), 5.0);
  ASSERT_DOUBLE_EQ(danglingLineIfce.getX(), 6.0);

  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  danglingLineIfce.setVoltageLevelInterface(voltageLevelIfce);
  ASSERT_DOUBLE_EQ(danglingLineIfce.getVNom(), 380);

  std::unique_ptr<CurrentLimitInterface> currentLimitIfce = DYN::make_unique<CurrentLimitInterfaceIIDM>(1.0, 99, false);
  danglingLineIfce.addCurrentLimitInterface(std::move(currentLimitIfce));
  ASSERT_EQ(danglingLineIfce.getCurrentLimitInterfaces().size(), 1);

  std::unique_ptr<CurrentLimitInterface> currentLimitIfceFictitious = DYN::make_unique<CurrentLimitInterfaceIIDM>(1.0,
      std::numeric_limits<unsigned long>::max(), true);
  danglingLineIfce.addCurrentLimitInterface(std::move(currentLimitIfceFictitious));
  ASSERT_EQ(danglingLineIfce.getCurrentLimitInterfaces().size(), 2);

  // DANGLING_LINE2 has p=10 q=40 in the file and x=0 → getX() returns 0.01
  auto danglingLine2 = net.getDanglingLine("DANGLING_LINE2").value();
  DanglingLineInterfaceIIDM danglingLineIfce2(danglingLine2);
  danglingLineIfce2.setVoltageLevelInterface(vlItf);
  ASSERT_DOUBLE_EQ(danglingLineIfce2.getX(), 0.01);
}

TEST(DataInterfaceTest, DanglingLine_2) {
  auto net = iidm::NetworkFactory::load("resources/dangling_line2.xiidm");
  auto danglingLine = net.getDanglingLine("DANGLING_LINE1").value();

  DanglingLineInterfaceIIDM danglingLineIfce(danglingLine);
  ASSERT_TRUE(danglingLineIfce.hasInitialConditions());
}

}  // namespace DYN
