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

#include "DYNLccConverterInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/LccConverterStation.h>

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

TEST(DataInterfaceTest, LccConverter) {
  auto net = iidm::NetworkFactory::load("resources/lcc.xiidm");
  auto vl1 = findVL(net, "VL1");
  auto lcc = net.getLccConverterStation("LCC1").value();

  LccConverterInterfaceIIDM Ifce(lcc);
  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce = std::make_shared<VoltageLevelInterfaceIIDM>(vl1);
  Ifce.setVoltageLevelInterfaceInjector(voltageLevelIfce);
  ASSERT_EQ(Ifce.getID(), "LCC1");
  ASSERT_EQ(Ifce.getComponentVarIndex("nothing"), -1);
  ASSERT_DOUBLE_EQ(Ifce.getLossFactor(), 2.0);
  // The new bridge stores powerFactor as a float on the JVM side, so the
  // round-trip back to double introduces a small ULP error. Compare with
  // float precision instead of ASSERT_DOUBLE_EQ.
  ASSERT_FLOAT_EQ(static_cast<float>(Ifce.getPowerFactor()), -0.2f);

  ASSERT_TRUE(Ifce.getInitialConnected());
  Ifce.exportStateVariablesUnitComponent();
  Ifce.importStaticParameters();

  auto bus1 = findBus(vl1, "BUS1");
  const std::shared_ptr<BusInterface> x_b1 = std::make_shared<BusInterfaceIIDM>(bus1, vl1);

  ASSERT_EQ(Ifce.getBusInterface(), nullptr);
  Ifce.setBusInterface(x_b1);
  ASSERT_EQ(Ifce.getBusInterface(), x_b1);
  ASSERT_EQ(Ifce.getBusInterface()->getID(), "BUS1");
  Ifce.setBusInterface(nullptr);
  ASSERT_EQ(Ifce.getBusInterface(), nullptr);
  Ifce.setBusInterface(x_b1);

  ASSERT_FALSE(Ifce.hasP());
  ASSERT_FALSE(Ifce.hasQ());

  ASSERT_DOUBLE_EQ(Ifce.getP(), 0.0);
  lcc.getTerminal().setP(1000.0);
  ASSERT_TRUE(Ifce.hasP());
  ASSERT_DOUBLE_EQ(Ifce.getP(), 1000.0);

  ASSERT_DOUBLE_EQ(Ifce.getQ(), 0.0);
  lcc.getTerminal().setQ(499.0);
  ASSERT_TRUE(Ifce.hasQ());
  ASSERT_DOUBLE_EQ(Ifce.getQ(), 499.0);

  ASSERT_FALSE(Ifce.hasInitialConditions());

  ASSERT_EQ(Ifce.getLccIIDM().getHvdcLine().getId(), "HVDC1");
  Ifce.importStaticParameters();

  ASSERT_EQ(Ifce.getLccIIDM().getId(), lcc.getId());

  ASSERT_DOUBLE_EQ(Ifce.getVNom(), 380);
  ASSERT_EQ(Ifce.getVoltageLevelInterfaceInjector(), voltageLevelIfce);

  ASSERT_TRUE(Ifce.isConnected());
  ASSERT_TRUE(Ifce.isPartiallyConnected());
  lcc.getTerminal().disconnect();
  ASSERT_FALSE(Ifce.isPartiallyConnected());
  ASSERT_FALSE(Ifce.isConnected());
  ASSERT_TRUE(Ifce.getInitialConnected());
}

TEST(DataInterfaceTest, LccConverter_2) {
  auto net = iidm::NetworkFactory::load("resources/lcc2.xiidm");
  auto lcc = net.getLccConverterStation("LCC1").value();

  LccConverterInterfaceIIDM Ifce(lcc);
  ASSERT_TRUE(Ifce.hasInitialConditions());
}

}  // namespace DYN
