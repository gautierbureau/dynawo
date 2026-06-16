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

/**
 * @file  TestInjectorInterface.cpp
 *
 * @brief Test of the Injector interface base class for IIDM.
 * InjectorInterfaceIIDM<T> is a template; tests use LoadInterfaceIIDM
 * which inherits from InjectorInterfaceIIDM<iidm::Load>.
 */

#include "DYNLoadInterfaceIIDM.h"
#include "DYNInjectorInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/Load.h>

#include "make_unique.hpp"
#include "gtest_dynawo.h"

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

TEST(DataInterfaceTest, Injector_1) {
  auto net = iidm::NetworkFactory::load("resources/injector.xiidm");
  auto vl = findVL(net, "VL1");
  auto bus = findBus(vl, "VL1_BUS1");
  auto load = net.getLoad("LOAD1").value();

  DYN::LoadInterfaceIIDM Ifce(load);
  const std::shared_ptr<DYN::VoltageLevelInterface> voltageLevelIfce = std::make_shared<DYN::VoltageLevelInterfaceIIDM>(vl);
  Ifce.setVoltageLevelInterfaceInjector(voltageLevelIfce);
  ASSERT_EQ("LOAD1", Ifce.getIDInjector());

  ASSERT_EQ(Ifce.getBusInterfaceInjector().get(), nullptr);
  std::unique_ptr<DYN::BusInterface> busIfce = DYN::make_unique<DYN::BusInterfaceIIDM>(bus, vl);
  Ifce.setBusInterfaceInjector(std::move(busIfce));
  ASSERT_EQ(Ifce.getBusInterfaceInjector().get()->getID(), "VL1_BUS1");

  ASSERT_TRUE(Ifce.getInitialConnectedInjector());
  ASSERT_TRUE(Ifce.isConnectedInjector());
  load.getTerminal().disconnect();
  ASSERT_FALSE(Ifce.isConnectedInjector());
  ASSERT_TRUE(Ifce.getInitialConnectedInjector());

  ASSERT_FALSE(Ifce.hasPInjector());
  ASSERT_FALSE(Ifce.hasQInjector());

  ASSERT_DOUBLE_EQ(Ifce.getPInjector(), 0.0);
  load.getTerminal().setP(1000.0);
  ASSERT_TRUE(Ifce.hasPInjector());
  ASSERT_DOUBLE_EQ(Ifce.getPInjector(), 1000.0);

  ASSERT_DOUBLE_EQ(Ifce.getQInjector(), 0.0);
  load.getTerminal().setQ(499.0);
  ASSERT_TRUE(Ifce.hasQInjector());
  ASSERT_DOUBLE_EQ(Ifce.getQInjector(), 499.0);

  // Test "initially not connected" interface for LOAD2 (disconnected before first use)
  auto load2 = net.getLoad("LOAD2").value();
  load2.getTerminal().disconnect();
  DYN::LoadInterfaceIIDM IfceNC(load2);
  IfceNC.setVoltageLevelInterfaceInjector(voltageLevelIfce);
  ASSERT_FALSE(IfceNC.getInitialConnectedInjector());
  ASSERT_DOUBLE_EQ(IfceNC.getPInjector(), 0.0);
  ASSERT_DOUBLE_EQ(IfceNC.getQInjector(), 0.0);
  ASSERT_DOUBLE_EQ(Ifce.getVNomInjector(), 380);
  ASSERT_EQ(Ifce.getVoltageLevelInterfaceInjector(), voltageLevelIfce);
}

TEST(DataInterfaceTest, Injector_2) {
  auto net = iidm::NetworkFactory::load("resources/injector.xiidm");
  auto vl = findVL(net, "VL1");
  auto load2 = net.getLoad("LOAD2").value();
  load2.getTerminal().disconnect();

  DYN::LoadInterfaceIIDM IfceNC(load2);
  const std::shared_ptr<DYN::VoltageLevelInterface> voltageLevelIfce = std::make_shared<DYN::VoltageLevelInterfaceIIDM>(vl);
  IfceNC.setVoltageLevelInterfaceInjector(voltageLevelIfce);
  ASSERT_FALSE(IfceNC.getInitialConnectedInjector());
  ASSERT_DOUBLE_EQ(IfceNC.getPInjector(), 0.0);
  ASSERT_DOUBLE_EQ(IfceNC.getQInjector(), 0.0);
}
