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

#include "DYNGeneratorInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNCommon.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/Generator.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>

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

TEST(DataInterfaceTest, Generator_1) {
  auto net = iidm::NetworkFactory::load("resources/generator_basic.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus1 = findBus(vl, "VL1_BUS1");
  auto gen = net.getGenerator("GEN1").value();

  GeneratorInterfaceIIDM genItf(gen);
  const std::shared_ptr<VoltageLevelInterface> vlItf = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  genItf.setVoltageLevelInterface(vlItf);
  ASSERT_EQ(genItf.getID(), "GEN1");

  ASSERT_FALSE(genItf.hasInitialConditions());

  ASSERT_EQ(genItf.getComponentVarIndex(std::string("p")), GeneratorInterfaceIIDM::VAR_P);
  ASSERT_EQ(genItf.getComponentVarIndex(std::string("q")), GeneratorInterfaceIIDM::VAR_Q);
  ASSERT_EQ(genItf.getComponentVarIndex(std::string("state")), GeneratorInterfaceIIDM::VAR_STATE);
  ASSERT_EQ(genItf.getComponentVarIndex(std::string("invalid")), -1);

  genItf.importStaticParameters();

  ASSERT_EQ(genItf.getBusInterface().get(), nullptr);
  std::unique_ptr<BusInterface> busItf = DYN::make_unique<BusInterfaceIIDM>(bus1, vl);
  genItf.setBusInterface(std::move(busItf));
  ASSERT_EQ(genItf.getBusInterface().get()->getID(), "VL1_BUS1");

  genItf.importStaticParameters();

  const std::shared_ptr<VoltageLevelInterface> voltageLevelItf = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  genItf.setVoltageLevelInterface(voltageLevelItf);

  ASSERT_TRUE(genItf.getInitialConnected());

  ASSERT_EQ(genItf.getP(), 0.0);
  gen.getTerminal().setP(10.0);
  ASSERT_EQ(genItf.getP(), 10.0);

  ASSERT_EQ(genItf.getQ(), 0.0);
  gen.getTerminal().setQ(11.0);
  ASSERT_EQ(genItf.getQ(), 11.0);

  ASSERT_EQ(genItf.getPMin(), 3.0);
  ASSERT_EQ(genItf.getPMax(), 50.0);

  ASSERT_EQ(genItf.getTargetP(), -45.0);
  ASSERT_EQ(genItf.getTargetQ(), -5.0);
  ASSERT_EQ(genItf.getTargetV(), 24.0);
  ASSERT_EQ(genItf.getEnergySource(), GeneratorInterface::SOURCE_WIND);

  ASSERT_TRUE(genItf.isVoltageRegulationOn());

  ASSERT_EQ(genItf.getReactiveCurvesPoints().size(), 0);

  ASSERT_TRUE(genItf.getCountry().empty());
  genItf.setCountry("FR");
  ASSERT_EQ(genItf.getCountry(), "FR");
  genItf.setCountry("");
  ASSERT_TRUE(genItf.getCountry().empty());

  // No reactive limits: returns 0.3 * maxP = 15
  ASSERT_EQ(genItf.getQMin(), -0.3 * 50.0);
  ASSERT_EQ(genItf.getQMax(), 0.3 * 50.0);
  ASSERT_EQ(genItf.getQNom(), 0.3 * 50.0);

  ASSERT_TRUE(genItf.isConnected());
  ASSERT_TRUE(genItf.isPartiallyConnected());
  gen.getTerminal().disconnect();
  ASSERT_FALSE(genItf.isPartiallyConnected());
  ASSERT_FALSE(genItf.isConnected());
  ASSERT_TRUE(genItf.getInitialConnected());
  ASSERT_FALSE(genItf.hasActivePowerControl());
  ASSERT_FALSE(genItf.isParticipating());
  ASSERT_DOUBLE_EQUALS_DYNAWO(genItf.getActivePowerControlDroop(), 0.);
  ASSERT_FALSE(genItf.hasCoordinatedReactiveControl());
  ASSERT_DOUBLE_EQUALS_DYNAWO(genItf.getCoordinatedReactiveControlPercentage(), 0.);
}

TEST(DataInterfaceTest, Generator_MinMax) {
  auto net = iidm::NetworkFactory::load("resources/generator_minmax.xiidm");
  auto gen = net.getGenerator("GEN1").value();

  GeneratorInterfaceIIDM genItf(gen);
  ASSERT_EQ(genItf.getQMin(), 1.0);
  ASSERT_EQ(genItf.getQMax(), 2.0);
  ASSERT_EQ(genItf.getQNom(), 2.0);
}

TEST(DataInterfaceTest, Generator_2) {
  auto net = iidm::NetworkFactory::load("resources/generator_not_connected.xiidm");
  auto vl = findVL(net, "VL1");
  auto gen = net.getGenerator("GEN1").value();

  GeneratorInterfaceIIDM genItf(gen);
  const std::shared_ptr<VoltageLevelInterface> vlItf = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  genItf.setVoltageLevelInterface(vlItf);
  ASSERT_EQ(genItf.getID(), "GEN1");
  ASSERT_EQ(genItf.getEnergySource(), GeneratorInterface::SOURCE_OTHER);

  ASSERT_FALSE(genItf.getInitialConnected());
  ASSERT_FALSE(genItf.isVoltageRegulationOn());
  ASSERT_EQ(genItf.getTargetV(), 0.0);

  // set targetV and voltageRegulatorOn via setters (they exist in new API)
  gen.setTargetV(24.0).setVoltageRegulatorOn(true);
  ASSERT_EQ(genItf.getTargetQ(), 0.0);
  genItf.importStaticParameters();
}

TEST(DataInterfaceTest, Generator_3) {
  auto net = iidm::NetworkFactory::load("resources/generator_initial_pq.xiidm");
  auto gen = net.getGenerator("GEN1").value();

  GeneratorInterfaceIIDM genItf(gen);
  ASSERT_TRUE(genItf.hasInitialConditions());
}

// Curve P=1:15/25, P=2:10/20 with terminal P=10 → pGen=-10 < P[0]=1 → use first point
TEST(DataInterfaceTest, Generator_Curve1) {
  auto net = iidm::NetworkFactory::load("resources/generator_curve1.xiidm");
  auto gen = net.getGenerator("GEN1").value();
  gen.getTerminal().setP(10.0);

  GeneratorInterfaceIIDM genItf(gen);
  ASSERT_EQ(genItf.getQMin(), 15.0);
  ASSERT_EQ(genItf.getQMax(), 25.0);
  ASSERT_EQ(genItf.getQNom(), 25.0);
}

// Curve P=-30:15/25, P=-20:10/20 with terminal P=10 → pGen=-10 > last P=-20 → use last point
TEST(DataInterfaceTest, Generator_Curve2) {
  auto net = iidm::NetworkFactory::load("resources/generator_curve2.xiidm");
  auto gen = net.getGenerator("GEN1").value();
  gen.getTerminal().setP(10.0);

  GeneratorInterfaceIIDM genItf(gen);
  ASSERT_EQ(genItf.getQMin(), 10.0);
  ASSERT_EQ(genItf.getQMax(), 20.0);
  ASSERT_EQ(genItf.getQNom(), 25.0);
}

// Curve P=-20:15/25, P=0:10/20 with terminal P=10 → pGen=-10 → interpolate t=0.5
TEST(DataInterfaceTest, Generator_Curve3) {
  auto net = iidm::NetworkFactory::load("resources/generator_curve3.xiidm");
  auto gen = net.getGenerator("GEN1").value();
  gen.getTerminal().setP(10.0);

  GeneratorInterfaceIIDM genItf(gen);
  ASSERT_EQ(genItf.getQMin(), 12.5);
  ASSERT_EQ(genItf.getQMax(), 22.5);
  ASSERT_EQ(genItf.getQNom(), 25.0);
  ASSERT_EQ(genItf.getReactiveCurvesPoints().size(), 2);
}

// Curve P=-10:-30/25, P=0:10/20 with terminal P=10 → pGen=-10 → first point → qNom=30
TEST(DataInterfaceTest, Generator_Curve4) {
  auto net = iidm::NetworkFactory::load("resources/generator_curve4.xiidm");
  auto gen = net.getGenerator("GEN1").value();
  gen.getTerminal().setP(10.0);

  GeneratorInterfaceIIDM genItf(gen);
  ASSERT_EQ(genItf.getQNom(), 30.0);
}

TEST(DataInterfaceTest, Generator_WithExt) {
  auto net = iidm::NetworkFactory::load("resources/generator_with_ext.xiidm");
  auto gen = net.getGenerator("GEN1").value();

  GeneratorInterfaceIIDM genItfWithExtensions(gen);
  ASSERT_TRUE(genItfWithExtensions.hasActivePowerControl());
  ASSERT_TRUE(genItfWithExtensions.isParticipating());
  ASSERT_DOUBLE_EQUALS_DYNAWO(genItfWithExtensions.getActivePowerControlDroop(), 4.);
  ASSERT_TRUE(genItfWithExtensions.hasCoordinatedReactiveControl());
  ASSERT_DOUBLE_EQUALS_DYNAWO(genItfWithExtensions.getCoordinatedReactiveControlPercentage(), 50.);
}

}  // namespace DYN
