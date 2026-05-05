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

#include "DYNVscConverterInterfaceIIDM.h"

#include "DYNBusInterfaceIIDM.h"
#include "DYNVoltageLevelInterfaceIIDM.h"

#include <iidm/Network.h>
#include <iidm/NetworkFactory.h>
#include <iidm/VoltageLevel.h>
#include <iidm/Bus.h>
#include <iidm/VscConverterStation.h>

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

TEST(DataInterfaceTest, VscConverter_1) {
  auto net = iidm::NetworkFactory::load("resources/vsc.xiidm");
  auto vl1 = findVL(net, "VL1");
  auto vsc = net.getVscConverterStation("VSC1").value();

  VscConverterInterfaceIIDM Ifce(vsc);
  const std::shared_ptr<VoltageLevelInterface> voltageLevelIfce = std::make_shared<VoltageLevelInterfaceIIDM>(vl1);
  Ifce.setVoltageLevelInterface(voltageLevelIfce);
  ASSERT_EQ(Ifce.getID(), "VSC1");
  ASSERT_EQ(Ifce.getComponentVarIndex("nothing"), -1);
  ASSERT_DOUBLE_EQ(Ifce.getLossFactor(), 3.0);
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
  vsc.getTerminal().setP(1000.0);
  ASSERT_TRUE(Ifce.hasP());
  ASSERT_DOUBLE_EQ(Ifce.getP(), 1000.0);

  ASSERT_DOUBLE_EQ(Ifce.getQ(), 0.0);
  vsc.getTerminal().setQ(499.0);
  ASSERT_TRUE(Ifce.hasQ());
  ASSERT_DOUBLE_EQ(Ifce.getQ(), 499.0);

  ASSERT_FALSE(Ifce.hasInitialConditions());

  ASSERT_TRUE(Ifce.getVoltageRegulatorOn());
  ASSERT_DOUBLE_EQ(Ifce.getVoltageSetpoint(), 4.0);
  ASSERT_DOUBLE_EQ(Ifce.getReactivePowerSetpoint(), 5.0);

  auto vsc2 = net.getVscConverterStation("VSC2").value();
  VscConverterInterfaceIIDM Ifce2(vsc2);
  ASSERT_FALSE(Ifce2.getVoltageRegulatorOn());
  ASSERT_DOUBLE_EQ(Ifce2.getVoltageSetpoint(), -4.0);
  ASSERT_DOUBLE_EQ(Ifce2.getReactivePowerSetpoint(), -5.0);

  // VSC1 already has reactive capability curve in vsc.xiidm
  // With terminal P=1000 → pGen=-1000, between curve points (-2000,10,10) and (0,10,20)
  // t=0.5 → QMax=15, QMin=10
  ASSERT_DOUBLE_EQ(Ifce.getQMax(), 15.0);

  ASSERT_EQ(Ifce.getVscIIDM().getHvdcLine().getId(), "HVDC1");
  Ifce.importStaticParameters();

  ASSERT_EQ(Ifce.getVscIIDM().getId(), vsc.getId());
  ASSERT_DOUBLE_EQ(Ifce.getVNom(), 388);
  ASSERT_EQ(Ifce.getVoltageLevelInterfaceInjector(), voltageLevelIfce);

  ASSERT_TRUE(Ifce.isConnected());
  ASSERT_TRUE(Ifce.isPartiallyConnected());
  vsc.getTerminal().disconnect();
  ASSERT_FALSE(Ifce.isConnected());
  ASSERT_FALSE(Ifce.isPartiallyConnected());
  ASSERT_TRUE(Ifce.getInitialConnected());
  ASSERT_DOUBLE_EQ(Ifce.getQMin(), 10.0);
  const auto& points = Ifce.getReactiveCurvesPoints();
  ASSERT_EQ(points.size(), 3);
  ASSERT_EQ(points.at(0).p, -2000);
  ASSERT_EQ(points.at(0).qmax, 10);
  ASSERT_EQ(points.at(0).qmin, 10);
  ASSERT_EQ(points.at(1).p, 0.);
  ASSERT_EQ(points.at(1).qmax, 20);
  ASSERT_EQ(points.at(1).qmin, 10);
  ASSERT_EQ(points.at(2).p, 2000);
  ASSERT_EQ(points.at(2).qmax, 10);
  ASSERT_EQ(points.at(2).qmin, 10);
}

TEST(DataInterfaceTest, VscConverter_2) {
  auto net = iidm::NetworkFactory::load("resources/vsc_initial_pq.xiidm");
  auto vsc = net.getVscConverterStation("VSC1").value();

  VscConverterInterfaceIIDM Ifce(vsc);
  ASSERT_TRUE(Ifce.hasInitialConditions());
}

}  // namespace DYN
