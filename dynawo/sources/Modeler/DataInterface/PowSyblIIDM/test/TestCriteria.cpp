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

#include "gtest_dynawo.h"
#include "DYNCriteria.h"
#include "CRTCriteriaParams.h"
#include "CRTCriteriaParamsFactory.h"

#include <memory>
#include <string>

using criteria::CriteriaParams;
using criteria::CriteriaParamsFactory;

TEST(DataInterfaceIIDMTest, Timeline) {
  // TODO: rewrite for iidm-bridge-cpp
}

namespace DYN {

class LineInterfaceStub {
 public:
  /**
  * @param id   Component id returned by getID()
  * @param i1   Current on side 1 returned by getStateVarI1()
  * @param i2   Current on side 2 returned by getStateVarI2()
  */
  explicit LineInterfaceStub(std::string id, double i1 = 0., double i2 = 0.) :
        id_(id),
        i1_(i1),
        i2_(i2) {}

  /** Used by QuadripoleCriteria::checkCriteria to compare against I1Max. */
  double getStateVarI1() const { return i1_; }

  /** Used by QuadripoleCriteria::checkCriteria to compare against I2Max. */
  double getStateVarI2() const { return i2_; }

  /** Used by QuadripoleFailingCriteria to build the timeline / log message. */
  const std::string& getID() const { return id_; }

 private:
  std::string id_;
  double      i1_;
  double      i2_;
};

static std::shared_ptr<LineInterfaceStub>
createLine(const std::string& id, double i1, double i2) {
  return std::make_shared<LineInterfaceStub>(id, i1, i2);
}

TEST(DataInterfaceIIDMTest, testQuadripoleCriteria) {
  std::shared_ptr<CriteriaParams> criteriap1 = CriteriaParamsFactory::newCriteriaParams();
  criteriap1->setType(CriteriaParams::LOCAL_VALUE);
  ASSERT_FALSE(QuadripoleCriteria<LineInterfaceStub>::criteriaEligibleForQuadripole(criteriap1));
  criteriap1->setScope(CriteriaParams::DYNAMIC);
  ASSERT_FALSE(QuadripoleCriteria<LineInterfaceStub>::criteriaEligibleForQuadripole(criteriap1));
  criteriap1->setI1Max(100.);
  ASSERT_TRUE(QuadripoleCriteria<LineInterfaceStub>::criteriaEligibleForQuadripole(criteriap1));
  criteriap1->setI2Max(100.);
  ASSERT_TRUE(QuadripoleCriteria<LineInterfaceStub>::criteriaEligibleForQuadripole(criteriap1));
  criteriap1->setId("quadripole_criteria");

  QuadripoleCriteria<LineInterfaceStub> LineCriteriaOK(criteriap1);
  ASSERT_TRUE(LineCriteriaOK.empty());
  ASSERT_TRUE(LineCriteriaOK.checkCriteria(0., false));
  std::shared_ptr<LineInterfaceStub> lineOK = createLine("MY_LINE_3", 50., 50.);
  LineCriteriaOK.addQuadripole(lineOK);
  ASSERT_TRUE(LineCriteriaOK.checkCriteria(0., false));
  ASSERT_TRUE(LineCriteriaOK.getFailingCriteria().empty());

  std::shared_ptr<LineInterfaceStub> lineSide1 = createLine("MY_LINE_1", 200., 50.);
  QuadripoleCriteria<LineInterfaceStub> LineCriteriaSide1(criteriap1);
  LineCriteriaSide1.addQuadripole(lineSide1);
  ASSERT_FALSE(LineCriteriaSide1.checkCriteria(0., false));
  ASSERT_EQ(1, LineCriteriaSide1.getFailingCriteria().size());

  std::shared_ptr<LineInterfaceStub> lineSide2 = createLine("MY_LINE_2", 50., 200.);
  QuadripoleCriteria<LineInterfaceStub> LineCriteriaSide2(criteriap1);
  LineCriteriaSide2.addQuadripole(lineSide2);
  ASSERT_FALSE(LineCriteriaSide2.checkCriteria(0., false));
  ASSERT_EQ(1, LineCriteriaSide2.getFailingCriteria().size());


  std::shared_ptr<LineInterfaceStub> lineBothSides = createLine("MY_LINE_4", 220., 200.);
  QuadripoleCriteria<LineInterfaceStub> LineCriteriaBothSides(criteriap1);
  LineCriteriaBothSides.addQuadripole(lineBothSides);
  ASSERT_FALSE(LineCriteriaBothSides.checkCriteria(0., false));
  ASSERT_EQ(2, LineCriteriaBothSides.getFailingCriteria().size());

  criteriap1->setScope(CriteriaParams::FINAL);
  QuadripoleCriteria<LineInterfaceStub> LineCriteriaFinal(criteriap1);
  LineCriteriaFinal.addQuadripole(lineBothSides);
  ASSERT_TRUE(LineCriteriaFinal.checkCriteria(0., false));
  ASSERT_FALSE(LineCriteriaFinal.checkCriteria(0., true));
}

}  // namespace DYN

TEST(DataInterfaceIIDMTest, testBusCriteria) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testBusCriteriaDataIIDM) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testLoadCriteriaLocalValue) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testLoadCriteriaSum) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testLoadCriteriaDataIIDMLocalValue) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testLoadCriteriaDataIIDMSum) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testDontTestFictitiousLoadsInCriteria) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testGeneratorCriteriaLocalValue) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testGeneratorCriteriaSum) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testGeneratorCriteriaDataIIDMLocalValue) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, testGeneratorCriteriaDataIIDMSum) {
  // TODO: rewrite for iidm-bridge-cpp
}

TEST(DataInterfaceIIDMTest, NoVoltageLevelInCriteria) {
  // TODO: rewrite for iidm-bridge-cpp
}
