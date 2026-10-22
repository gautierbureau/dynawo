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

#include "DYNStaticVarCompensatorInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNInjectorInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/StaticVarCompensator.h>
#include <iidm/Enums.h>

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

TEST(DataInterfaceTest, SVarC_1) {
  auto net = iidm::NetworkFactory::load("resources/svc_no_ext.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus1 = findBus(vl, "VL1_BUS1");
  auto svc = net.getStaticVarCompensator("SVC1").value();

  StaticVarCompensatorInterfaceIIDM svcInterface(svc);
  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  svcInterface.setVoltageLevelInterface(voltageLevelIfce);

  ASSERT_EQ(svcInterface.getComponentVarIndex(std::string("p")), StaticVarCompensatorInterfaceIIDM::VAR_P);
  ASSERT_EQ(svcInterface.getComponentVarIndex(std::string("P1")), -1);
  ASSERT_EQ(svcInterface.getComponentVarIndex(std::string("q")), StaticVarCompensatorInterfaceIIDM::VAR_Q);
  ASSERT_EQ(svcInterface.getComponentVarIndex(std::string("state")), StaticVarCompensatorInterfaceIIDM::VAR_STATE);
  ASSERT_EQ(svcInterface.getComponentVarIndex(std::string("regulatingMode")), -1);

  ASSERT_EQ(svcInterface.getID(), svc.getId());

  ASSERT_TRUE(svcInterface.getInitialConnected());
  ASSERT_TRUE(svcInterface.isConnected());
  ASSERT_TRUE(svcInterface.isPartiallyConnected());
  svc.getTerminal().disconnect();
  ASSERT_FALSE(svcInterface.isPartiallyConnected());
  ASSERT_FALSE(svcInterface.isConnected());
  ASSERT_TRUE(svcInterface.getInitialConnected());

  ASSERT_EQ(svcInterface.getBusInterface().get(), nullptr);
  svcInterface.importStaticParameters();
  std::unique_ptr<BusInterface> busIfce = DYN::make_unique<BusInterfaceIIDM>(bus1, vl);
  svcInterface.setBusInterface(std::move(busIfce));
  ASSERT_EQ(svcInterface.getBusInterface().get()->getID(), "VL1_BUS1");
  ASSERT_DOUBLE_EQ(svcInterface.getVNom(), 382.0);
  ASSERT_EQ(svcInterface.getVoltageLevelInterfaceInjector(), voltageLevelIfce);

  ASSERT_FALSE(svc.getTerminal().isConnected());

  ASSERT_DOUBLE_EQ(svcInterface.getBMin(), -0.01);
  ASSERT_DOUBLE_EQ(svcInterface.getBMax(), 0.02);

  ASSERT_FALSE(svcInterface.hasStandbyAutomaton());
  ASSERT_FALSE(svcInterface.isStandBy());
  ASSERT_FALSE(svcInterface.hasVoltagePerReactivePowerControl());
  ASSERT_DOUBLE_EQ(svcInterface.getSlope(), 0.);
  ASSERT_DOUBLE_EQ(svcInterface.getB0(), 0.);
  ASSERT_DOUBLE_EQ(svcInterface.getUMinActivation(), 0.);
  ASSERT_DOUBLE_EQ(svcInterface.getUMaxActivation(), 0.);
  ASSERT_DOUBLE_EQ(svcInterface.getUSetPointMin(), 0.);
  ASSERT_DOUBLE_EQ(svcInterface.getUSetPointMax(), 0.);

  ASSERT_DOUBLE_EQ(svcInterface.getVSetPoint(), 380.0);
  ASSERT_DOUBLE_EQ(svcInterface.getReactivePowerSetPoint(), 90.0);

  ASSERT_DOUBLE_EQ(svcInterface.getQ(), 0.0);
  svc.getTerminal().setQ(499.0);
  ASSERT_TRUE(svcInterface.hasQInjector());
  ASSERT_DOUBLE_EQ(svcInterface.getQ(), 499.0);

  ASSERT_DOUBLE_EQ(svcInterface.getPInjector(), 0.0);
  svc.getTerminal().setP(1.0);
  ASSERT_TRUE(svcInterface.hasPInjector());
  ASSERT_DOUBLE_EQ(svcInterface.getPInjector(), 1.0);

  ASSERT_FALSE(svcInterface.hasInitialConditions());

  ASSERT_EQ(svcInterface.getRegulationMode(), StaticVarCompensatorInterface::OFF);
  svc.setRegulationMode(iidm::StaticVarCompensatorRegulationMode::VOLTAGE);
  ASSERT_EQ(svcInterface.getRegulationMode(), StaticVarCompensatorInterface::RUNNING_V);
  svc.setRegulationMode(iidm::StaticVarCompensatorRegulationMode::REACTIVE_POWER);
  ASSERT_EQ(svcInterface.getRegulationMode(), StaticVarCompensatorInterface::RUNNING_Q);
}

TEST(DataInterfaceTest, SVarC_2) {
  auto net = iidm::NetworkFactory::load("resources/svc.xiidm");
  auto vl = findVL(net, "VL1");
  auto svc = net.getStaticVarCompensator("SVC1").value();

  StaticVarCompensatorInterfaceIIDM svcInterface(svc);
  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  svcInterface.setVoltageLevelInterface(voltageLevelIfce);
  ASSERT_EQ(svcInterface.getID(), "SVC1");

  svc.getTerminal().disconnect();
  ASSERT_FALSE(svcInterface.getInitialConnected());

  ASSERT_FALSE(svcInterface.hasPInjector());
  ASSERT_FALSE(svcInterface.hasQInjector());
  ASSERT_DOUBLE_EQ(svcInterface.getPInjector(), 0.0);
  ASSERT_DOUBLE_EQ(svcInterface.getQ(), 0.0);
  ASSERT_TRUE(svcInterface.hasVoltagePerReactivePowerControl());
  ASSERT_DOUBLE_EQ(svcInterface.getSlope(), 0.1);
  ASSERT_FALSE(svcInterface.hasInitialConditions());
}

TEST(DataInterfaceTest, SVarC_3) {
  auto net = iidm::NetworkFactory::load("resources/svc_initial_pq.xiidm");
  auto svc = net.getStaticVarCompensator("SVC1").value();

  StaticVarCompensatorInterfaceIIDM svcInterface(svc);
  ASSERT_TRUE(svcInterface.hasInitialConditions());
}

}  // namespace DYN
