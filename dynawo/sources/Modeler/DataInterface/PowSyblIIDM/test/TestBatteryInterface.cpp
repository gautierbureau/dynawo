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

#include "DYNBatteryInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNCommon.h"
#include "DYNVoltageLevelInterfaceIIDM.h"
#include "make_unique.hpp"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/Battery.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>

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

TEST(DataInterfaceTest, Battery_1) {
  auto net = iidm::NetworkFactory::load("resources/battery_basic.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus1 = findBus(vl, "VL1_BUS1");
  auto bat = net.getBattery("BAT1").value();

  BatteryInterfaceIIDM batItf(bat);
  const std::shared_ptr<VoltageLevelInterface> vlItf = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  batItf.setVoltageLevelInterface(vlItf);
  ASSERT_EQ(batItf.getID(), "BAT1");

  ASSERT_EQ(batItf.getComponentVarIndex(std::string("p")), BatteryInterfaceIIDM::VAR_P);
  ASSERT_EQ(batItf.getComponentVarIndex(std::string("q")), BatteryInterfaceIIDM::VAR_Q);
  ASSERT_EQ(batItf.getComponentVarIndex(std::string("state")), BatteryInterfaceIIDM::VAR_STATE);
  ASSERT_EQ(batItf.getComponentVarIndex(std::string("invalid")), -1);

  batItf.importStaticParameters();

  ASSERT_EQ(batItf.getBusInterface().get(), nullptr);
  std::unique_ptr<BusInterface> busItf = DYN::make_unique<BusInterfaceIIDM>(bus1, vl);
  batItf.setBusInterface(std::move(busItf));
  ASSERT_EQ(batItf.getBusInterface().get()->getID(), "VL1_BUS1");

  batItf.importStaticParameters();

  const std::shared_ptr<VoltageLevelInterface> voltageLevelItf = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  batItf.setVoltageLevelInterface(voltageLevelItf);

  ASSERT_TRUE(batItf.getInitialConnected());

  ASSERT_EQ(batItf.getP(), 0.0);
  bat.getTerminal().setP(10.0);
  ASSERT_EQ(batItf.getP(), 10.0);

  ASSERT_EQ(batItf.getQ(), 0.0);
  bat.getTerminal().setQ(11.0);
  ASSERT_EQ(batItf.getQ(), 11.0);

  ASSERT_EQ(batItf.getPMin(), 3.0);
  ASSERT_EQ(batItf.getPMax(), 50.0);

  ASSERT_EQ(batItf.getTargetP(), 0.0);
  ASSERT_EQ(batItf.getTargetQ(), 0.0);
  ASSERT_EQ(batItf.getTargetV(), 0.0);
  ASSERT_EQ(batItf.getEnergySource(), GeneratorInterface::SOURCE_OTHER);

  ASSERT_FALSE(batItf.isVoltageRegulationOn());

  ASSERT_EQ(batItf.getReactiveCurvesPoints().size(), 0);

  ASSERT_TRUE(batItf.getCountry().empty());
  batItf.setCountry("FR");
  ASSERT_EQ(batItf.getCountry(), "FR");
  batItf.setCountry("");
  ASSERT_TRUE(batItf.getCountry().empty());

  // No reactive limits: returns 0.3 * maxP = 0.3 * 50 = 15
  ASSERT_EQ(batItf.getQMin(), -0.3 * 50.0);
  ASSERT_EQ(batItf.getQMax(), 0.3 * 50.0);
  ASSERT_EQ(batItf.getQNom(), 0.3 * 50.0);

  bat.getTerminal().disconnect();
  ASSERT_FALSE(batItf.hasActivePowerControl());
  ASSERT_FALSE(batItf.isParticipating());
  ASSERT_DOUBLE_EQUALS_DYNAWO(batItf.getActivePowerControlDroop(), 0.);
  ASSERT_FALSE(batItf.hasCoordinatedReactiveControl());
  ASSERT_DOUBLE_EQUALS_DYNAWO(batItf.getCoordinatedReactiveControlPercentage(), 0.);
}

TEST(DataInterfaceTest, Battery_MinMax) {
  auto net = iidm::NetworkFactory::load("resources/battery_minmax.xiidm");
  auto bat = net.getBattery("BAT1").value();

  BatteryInterfaceIIDM batItf(bat);
  ASSERT_EQ(batItf.getQMin(), 1.0);
  ASSERT_EQ(batItf.getQMax(), 2.0);
  ASSERT_EQ(batItf.getQNom(), 2.0);
}

// Curve P=1:15/25, P=2:10/20 with terminal P=10 → pGen=-10 < P[0]=1 → use first point
TEST(DataInterfaceTest, Battery_Curve1) {
  auto net = iidm::NetworkFactory::load("resources/battery_curve1.xiidm");
  auto vl = findVL(net, "VL1");
  auto bat = net.getBattery("BAT1").value();
  bat.getTerminal().setP(10.0);

  BatteryInterfaceIIDM batItf(bat);
  batItf.setVoltageLevelInterface(std::make_shared<VoltageLevelInterfaceIIDM>(vl));
  ASSERT_EQ(batItf.getQMin(), 15.0);
  ASSERT_EQ(batItf.getQMax(), 25.0);
  ASSERT_EQ(batItf.getQNom(), 25.0);
}

// Curve P=-30:15/25, P=-20:10/20 with terminal P=10 → pGen=-10 > P[-1]=-20 → use last point
TEST(DataInterfaceTest, Battery_Curve2) {
  auto net = iidm::NetworkFactory::load("resources/battery_curve2.xiidm");
  auto vl = findVL(net, "VL1");
  auto bat = net.getBattery("BAT1").value();
  bat.getTerminal().setP(10.0);

  BatteryInterfaceIIDM batItf(bat);
  batItf.setVoltageLevelInterface(std::make_shared<VoltageLevelInterfaceIIDM>(vl));
  ASSERT_EQ(batItf.getQMin(), 10.0);
  ASSERT_EQ(batItf.getQMax(), 20.0);
  ASSERT_EQ(batItf.getQNom(), 25.0);
}

// Curve P=-20:15/25, P=0:10/20 with terminal P=10 → pGen=-10 → interpolate t=0.5
TEST(DataInterfaceTest, Battery_Curve3) {
  auto net = iidm::NetworkFactory::load("resources/battery_curve3.xiidm");
  auto vl = findVL(net, "VL1");
  auto bat = net.getBattery("BAT1").value();
  bat.getTerminal().setP(10.0);

  BatteryInterfaceIIDM batItf(bat);
  batItf.setVoltageLevelInterface(std::make_shared<VoltageLevelInterfaceIIDM>(vl));
  ASSERT_EQ(batItf.getQMin(), 12.5);
  ASSERT_EQ(batItf.getQMax(), 22.5);
  ASSERT_EQ(batItf.getQNom(), 25.0);
  ASSERT_EQ(batItf.getReactiveCurvesPoints().size(), 2);
}

// Curve P=-10:-30/25, P=0:10/20 with terminal P=10 → pGen=-10 → use first point → qNom=30
TEST(DataInterfaceTest, Battery_Curve4) {
  auto net = iidm::NetworkFactory::load("resources/battery_curve4.xiidm");
  auto vl = findVL(net, "VL1");
  auto bat = net.getBattery("BAT1").value();
  bat.getTerminal().setP(10.0);

  BatteryInterfaceIIDM batItf(bat);
  batItf.setVoltageLevelInterface(std::make_shared<VoltageLevelInterfaceIIDM>(vl));
  ASSERT_EQ(batItf.getQNom(), 30.0);
}

TEST(DataInterfaceTest, Battery_APC) {
  auto net = iidm::NetworkFactory::load("resources/battery_apc.xiidm");
  auto bat = net.getBattery("BAT1").value();

  BatteryInterfaceIIDM batItfWithExtensions(bat);
  ASSERT_TRUE(batItfWithExtensions.hasActivePowerControl());
  ASSERT_TRUE(batItfWithExtensions.isParticipating());
  ASSERT_DOUBLE_EQUALS_DYNAWO(batItfWithExtensions.getActivePowerControlDroop(), 4.);
  ASSERT_FALSE(batItfWithExtensions.hasCoordinatedReactiveControl());
  ASSERT_DOUBLE_EQUALS_DYNAWO(batItfWithExtensions.getCoordinatedReactiveControlPercentage(), 0.);
}

}  // namespace DYN
