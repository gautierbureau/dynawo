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
#include "DYNCalculatedBusInterfaceIIDM.h"

#include "gtest_dynawo.h"
#include "DYNCommon.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/BusbarSection.h>

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

namespace DYN {

TEST(DataInterfaceTest, testBusInterface) {
  auto net = iidm::NetworkFactory::load("resources/bus_bbr.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus = findBus(vl, "Bus1");

  BusInterfaceIIDM busIface(bus, vl);

  ASSERT_EQ(busIface.getID(), "Bus1");
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getVMax(), 420.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getVMin(), 380.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getV0(), 410.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getVNom(), 400.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getAngle0(), 3.14);
  ASSERT_FALSE(busIface.hasConnection());
  busIface.hasConnection(true);
  ASSERT_TRUE(busIface.hasConnection());
  ASSERT_EQ(busIface.getComponentVarIndex("v"), BusInterfaceIIDM::VAR_V);
  ASSERT_EQ(busIface.getComponentVarIndex("angle"), BusInterfaceIIDM::VAR_ANGLE);
  ASSERT_EQ(busIface.getComponentVarIndex("foo"), -1);
  ASSERT_EQ(busIface.getBusIndex(), 0);
  ASSERT_TRUE(busIface.getBusBarSectionIdentifiers().empty());
  ASSERT_FALSE(busIface.isFictitious());
  ASSERT_EQ(busIface.getCountry(), "");
  busIface.setCountry("AF");
  ASSERT_EQ(busIface.getCountry(), "AF");
  ASSERT_TRUE(busIface.hasInitialConditions());
}

TEST(DataInterfaceTest, testBusInterfaceCornerCases) {
  auto net = iidm::NetworkFactory::load("resources/bus_bbr_no_limits.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus = findBus(vl, "Bus1");

  BusInterfaceIIDM busIface(bus, vl);
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getV0(), 400.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getAngle0(), 0.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getVMax(), 480.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(busIface.getVMin(), 320.);
  ASSERT_FALSE(busIface.hasInitialConditions());
}

TEST(DataInterfaceTest, testCalculatedBusInterface) {
  auto net = iidm::NetworkFactory::load("resources/bus_nbr.xiidm");
  auto vl = findVL(net, "MyVoltageLevel");

  auto bss = vl.getNodeBreakerView().getBusbarSections();
  ASSERT_GE(bss.size(), 1U);

  CalculatedBusInterfaceIIDM bus(vl, "MyTest", 1);
  ASSERT_EQ(bus.getID(), "MyTest");
  ASSERT_DOUBLE_EQUALS_DYNAWO(bus.getVMax(), 420.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(bus.getVMin(), 380.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(bus.getV0(), 400.);
  bus.setU0(410.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(bus.getV0(), 410.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(bus.getVNom(), 400.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(bus.getAngle0(), 0.);
  bus.setAngle0(3.14);
  ASSERT_DOUBLE_EQUALS_DYNAWO(bus.getAngle0(), 3.14);
  ASSERT_FALSE(bus.hasConnection());
  bus.hasConnection(true);
  ASSERT_TRUE(bus.hasConnection());
  ASSERT_EQ(bus.getComponentVarIndex("v"), BusInterfaceIIDM::VAR_V);
  ASSERT_EQ(bus.getComponentVarIndex("angle"), BusInterfaceIIDM::VAR_ANGLE);
  ASSERT_EQ(bus.getComponentVarIndex("foo"), -1);
  ASSERT_EQ(bus.getBusIndex(), 1);
  ASSERT_EQ(bus.getBusBarSectionIdentifiers().size(), 0);
  bus.addBusBarSection(bss[0].getId());
  ASSERT_EQ(bus.getBusBarSectionIdentifiers().size(), 1);
  ASSERT_EQ(bus.getBusBarSectionIdentifiers()[0], "BBS");
  ASSERT_FALSE(bus.isFictitious());

  ASSERT_EQ(bus.getNodes().size(), 0);
  bus.addNode(8);
  ASSERT_EQ(bus.getNodes().size(), 1);
  ASSERT_TRUE(bus.hasNode(8));
  ASSERT_FALSE(bus.hasNode(2));
  ASSERT_TRUE(bus.hasInitialConditions());
}

}  // namespace DYN
