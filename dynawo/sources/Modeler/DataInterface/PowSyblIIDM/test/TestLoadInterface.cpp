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

#include "DYNLoadInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNInjectorInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/Load.h>

#include "make_unique.hpp"
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

TEST(DataInterfaceTest, Load_1) {
  auto net = iidm::NetworkFactory::load("resources/load.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus1 = findBus(vl, "VL1_BUS1");
  auto load = net.getLoad("LOAD1").value();

  LoadInterfaceIIDM loadIfce(load);
  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  loadIfce.setVoltageLevelInterface(voltageLevelIfce);

  ASSERT_EQ(loadIfce.getComponentVarIndex(std::string("p")), LoadInterfaceIIDM::VAR_P);
  ASSERT_EQ(loadIfce.getComponentVarIndex(std::string("P1")), -1);
  ASSERT_EQ(loadIfce.getComponentVarIndex(std::string("q")), LoadInterfaceIIDM::VAR_Q);
  ASSERT_EQ(loadIfce.getComponentVarIndex(std::string("state")), LoadInterfaceIIDM::VAR_STATE);

  ASSERT_EQ(loadIfce.getID(), load.getId());
  ASSERT_DOUBLE_EQ(loadIfce.getPUnderVoltage(), 0.0);

  ASSERT_TRUE(loadIfce.getInitialConnected());
  ASSERT_TRUE(loadIfce.isConnected());
  ASSERT_TRUE(loadIfce.isPartiallyConnected());
  load.getTerminal().disconnect();
  ASSERT_FALSE(loadIfce.isPartiallyConnected());
  ASSERT_FALSE(loadIfce.isConnected());
  ASSERT_TRUE(loadIfce.getInitialConnected());

  ASSERT_EQ(loadIfce.getBusInterface().get(), nullptr);
  std::unique_ptr<BusInterface> busIfce = DYN::make_unique<BusInterfaceIIDM>(bus1, vl);
  loadIfce.setBusInterface(std::move(busIfce));
  ASSERT_EQ(loadIfce.getBusInterface().get()->getID(), "VL1_BUS1");

  ASSERT_FALSE(load.getTerminal().isConnected());
  ASSERT_FALSE(loadIfce.hasPInjector());
  ASSERT_FALSE(loadIfce.hasQInjector());

  ASSERT_DOUBLE_EQ(loadIfce.getP0(), 50.0);
  ASSERT_DOUBLE_EQ(loadIfce.getP(), 0.0);
  load.getTerminal().setP(1000.0);
  ASSERT_TRUE(loadIfce.hasPInjector());
  ASSERT_DOUBLE_EQ(loadIfce.getP(), 1000.0);

  ASSERT_DOUBLE_EQ(loadIfce.getQ0(), 40.0);
  ASSERT_DOUBLE_EQ(loadIfce.getQ(), 0.0);
  load.getTerminal().setQ(499.0);
  ASSERT_TRUE(loadIfce.hasQInjector());
  ASSERT_DOUBLE_EQ(loadIfce.getQ(), 499.0);

  ASSERT_FALSE(loadIfce.hasInitialConditions());

  ASSERT_TRUE(loadIfce.isFictitious());

  loadIfce.importStaticParameters();
  loadIfce.setBusInterface(nullptr);
  loadIfce.importStaticParameters();
  ASSERT_EQ(loadIfce.getCountry(), "");
  loadIfce.setCountry("AF");
  ASSERT_EQ(loadIfce.getCountry(), "AF");

  ASSERT_DOUBLE_EQ(loadIfce.getVNomInjector(), 382.0);
  ASSERT_EQ(loadIfce.getVoltageLevelInterfaceInjector(), voltageLevelIfce);

  // Test load with fictitious=false (loaded from separate file)
  auto net2 = iidm::NetworkFactory::load("resources/load_not_fictitious.xiidm");
  auto vl2 = findVL(net2, "VL1");
  auto loadNotFictitious = net2.getLoad("LOAD1").value();
  LoadInterfaceIIDM loadIfceNotFictitious(loadNotFictitious);
  loadIfceNotFictitious.setVoltageLevelInterface(std::make_shared<VoltageLevelInterfaceIIDM>(vl2));
  ASSERT_FALSE(loadIfceNotFictitious.isFictitious());
}

TEST(DataInterfaceTest, Load_2) {
  auto net = iidm::NetworkFactory::load("resources/load2.xiidm");
  auto vl = findVL(net, "VL1");
  auto load = net.getLoad("LOAD").value();

  LoadInterfaceIIDM loadIfce(load);
  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  loadIfce.setVoltageLevelInterface(voltageLevelIfce);
  ASSERT_EQ(loadIfce.getID(), "LOAD");

  load.getTerminal().disconnect();
  ASSERT_FALSE(loadIfce.getInitialConnected());

  ASSERT_FALSE(loadIfce.hasPInjector());
  ASSERT_FALSE(loadIfce.hasQInjector());
  ASSERT_DOUBLE_EQ(loadIfce.getP(), 0.0);
  ASSERT_DOUBLE_EQ(loadIfce.getQ(), 0.0);
  ASSERT_TRUE(loadIfce.isFictitious());
  ASSERT_FALSE(loadIfce.hasInitialConditions());
}

TEST(DataInterfaceTest, Load_3) {
  auto net = iidm::NetworkFactory::load("resources/load_initial_pq.xiidm");
  auto load = net.getLoad("LOAD1").value();

  LoadInterfaceIIDM loadIfce(load);
  ASSERT_TRUE(loadIfce.hasInitialConditions());
}

}  // namespace DYN
