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

#include "DYNBusInterfaceIIDM.h"
#include "DYNSwitchInterfaceIIDM.h"
#include "DYNDataInterfaceIIDM.h"

#include "make_unique.hpp"
#include "gtest_dynawo.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/Switch.h>

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

static iidm::Switch findSwitch(iidm::VoltageLevel& vl, const std::string& id) {
  for (auto& sw : vl.getBusBreakerView().getSwitches())
    if (sw.getId() == id) return sw;
  throw std::runtime_error("Switch not found: " + id);
}

namespace DYN {

TEST(DataInterfaceTest, Switch) {
  auto net = iidm::NetworkFactory::load("resources/switch.xiidm");
  auto vl = findVL(net, "VL1");
  auto b1 = findBus(vl, "BUS1");
  auto b2 = findBus(vl, "BUS2");
  auto sw = findSwitch(vl, "Sw");

  DYN::SwitchInterfaceIIDM swIface(sw);
  ASSERT_EQ(swIface.getID(), "Sw");
  ASSERT_FALSE(swIface.isOpen());
  ASSERT_TRUE(swIface.isConnected());
  ASSERT_TRUE(swIface.isPartiallyConnected());

  std::unique_ptr<BusInterface> x_b1 = DYN::make_unique<BusInterfaceIIDM>(b1, vl);
  std::unique_ptr<BusInterface> x_b2 = DYN::make_unique<BusInterfaceIIDM>(b2, vl);
  swIface.setBusInterface1(std::move(x_b1));
  swIface.setBusInterface2(std::move(x_b2));
  ASSERT_EQ(swIface.getBusInterface1()->getID(), "BUS1");
  ASSERT_EQ(swIface.getBusInterface2()->getID(), "BUS2");

  swIface.close();
  ASSERT_FALSE(swIface.isOpen());
  ASSERT_TRUE(swIface.isConnected());
  ASSERT_TRUE(swIface.isPartiallyConnected());
  swIface.open();
  ASSERT_TRUE(swIface.isOpen());
  ASSERT_FALSE(swIface.isConnected());
  ASSERT_FALSE(swIface.isPartiallyConnected());
  swIface.close();
  ASSERT_FALSE(swIface.isOpen());
  ASSERT_TRUE(swIface.isConnected());
  ASSERT_TRUE(swIface.isPartiallyConnected());

  ASSERT_EQ(swIface.getComponentVarIndex("state"), 0);
  ASSERT_EQ(swIface.getComponentVarIndex("others"), -1);
}

TEST(DataInterfaceTest, SwitchWithSameExtremities) {
  auto network = boost::make_shared<iidm::Network>(iidm::NetworkFactory::load("resources/switch_same_bus.xiidm"));
  boost::shared_ptr<DataInterfaceIIDM> data;
  DataInterfaceIIDM* ptr = new DataInterfaceIIDM(network);
  ptr->initFromIIDM();
  data.reset(ptr);
  ASSERT_THROW_DYNAWO(data->findComponent("SwSameBus"), Error::MODELER, KeyError_t::UnknownStaticComponent);
}

}  // namespace DYN
