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

#include "DYNShuntCompensatorInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNInjectorInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include "DYNCommon.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/ShuntCompensator.h>

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

TEST(DataInterfaceTest, ShuntCompensator_1) {
  auto net = iidm::NetworkFactory::load("resources/shunt.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus1 = findBus(vl, "VL1_BUS1");
  auto shuntCompensator = net.getShuntCompensator("SHUNT1").value();

  ShuntCompensatorInterfaceIIDM shuntCompensatorIfce(shuntCompensator);
  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  shuntCompensatorIfce.setVoltageLevelInterface(voltageLevelIfce);

  ASSERT_EQ(shuntCompensatorIfce.getComponentVarIndex(std::string("currentSection")), ShuntCompensatorInterfaceIIDM::VAR_CURRENTSECTION);
  ASSERT_EQ(shuntCompensatorIfce.getComponentVarIndex(std::string("wrongIndex")), -1);
  ASSERT_EQ(shuntCompensatorIfce.getComponentVarIndex(std::string("q")), ShuntCompensatorInterfaceIIDM::VAR_Q);
  ASSERT_EQ(shuntCompensatorIfce.getComponentVarIndex(std::string("state")), ShuntCompensatorInterfaceIIDM::VAR_STATE);

  ASSERT_EQ(shuntCompensatorIfce.getID(), shuntCompensator.getId());

  ASSERT_TRUE(shuntCompensatorIfce.getInitialConnected());
  ASSERT_TRUE(shuntCompensatorIfce.isConnected());
  ASSERT_TRUE(shuntCompensatorIfce.isPartiallyConnected());
  shuntCompensator.getTerminal().disconnect();
  ASSERT_FALSE(shuntCompensatorIfce.isPartiallyConnected());
  ASSERT_FALSE(shuntCompensatorIfce.isConnected());
  ASSERT_TRUE(shuntCompensatorIfce.getInitialConnected());

  ASSERT_EQ(shuntCompensatorIfce.getBusInterface().get(), nullptr);
  std::unique_ptr<BusInterface> busIfce = DYN::make_unique<BusInterfaceIIDM>(bus1, vl);
  shuntCompensatorIfce.setBusInterface(std::move(busIfce));
  ASSERT_EQ(shuntCompensatorIfce.getBusInterface().get()->getID(), "VL1_BUS1");

  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getCurrentSection(), 2);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getMaximumSection(), 3);
  shuntCompensator.getTerminal().setQ(4.0);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getQ(), 4.0);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getVNom(), 380);
  ASSERT_TRUE(shuntCompensatorIfce.isLinear());
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getB(0), 0.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getB(1), 12.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getB(2), 24.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce.getB(3), 36.);
  shuntCompensatorIfce.importStaticParameters();
  ASSERT_DOUBLE_EQUALS_DYNAWO(shuntCompensatorIfce.getStaticParameterValue<double>("v_pu"), 382.0/380.0);
  ASSERT_DOUBLE_EQUALS_DYNAWO(shuntCompensatorIfce.getStaticParameterValue<double>("angle_pu"), M_PI/2.0);

  // Loaded from file with voltageRegulatorOn=true and targetV=380.0
  ASSERT_DOUBLE_EQUALS_DYNAWO(shuntCompensatorIfce.getTargetV(), 380.);
  ASSERT_EQ(shuntCompensatorIfce.isVoltageRegulationOn(), true);

  shuntCompensatorIfce.setBusInterface(nullptr);
  shuntCompensatorIfce.importStaticParameters();
  ASSERT_DOUBLE_EQUALS_DYNAWO(shuntCompensatorIfce.getStaticParameterValue<double>("v_pu"), 0.);
  ASSERT_DOUBLE_EQUALS_DYNAWO(shuntCompensatorIfce.getStaticParameterValue<double>("angle_pu"), 0.);

  ASSERT_FALSE(shuntCompensatorIfce.hasInitialConditions());

  // Test SHUNT2 (non-linear)
  auto shuntCompensator_2 = net.getShuntCompensator("SHUNT2").value();
  ShuntCompensatorInterfaceIIDM shuntCompensatorIfce_2(shuntCompensator_2);
  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce_2 = std::make_shared<VoltageLevelInterfaceIIDM>(vl);
  shuntCompensatorIfce_2.setVoltageLevelInterface(voltageLevelIfce_2);
  ASSERT_FALSE(shuntCompensatorIfce_2.isLinear());
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce_2.getB(0), 0.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce_2.getB(1), 11.);
  ASSERT_DOUBLE_EQ(shuntCompensatorIfce_2.getB(2), 24.);

  // Test voltageRegulatorOn=false from SHUNT2
  ASSERT_EQ(shuntCompensatorIfce_2.isVoltageRegulationOn(), false);
}

TEST(DataInterfaceTest, ShuntCompensator_2) {
  auto net = iidm::NetworkFactory::load("resources/shunt2.xiidm");
  auto shuntCompensator = net.getShuntCompensator("SHUNT1").value();

  ShuntCompensatorInterfaceIIDM shuntCompensatorIfce(shuntCompensator);
  ASSERT_TRUE(shuntCompensatorIfce.hasInitialConditions());
}

}  // namespace DYN
