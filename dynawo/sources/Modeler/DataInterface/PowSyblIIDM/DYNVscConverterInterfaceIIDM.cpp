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

//======================================================================
/**
 * @file  DYNVscConverterInterfaceIIDM.cpp
 *
 * @brief Vsc Converter data interface : implementation file for IIDM interface
 *
 */
//======================================================================

#include "DYNVscConverterInterfaceIIDM.h"

#include "DYNInjectorInterfaceIIDM.h"

#include <iidm/HvdcLine.h>
#include <iidm/VscConverterStation.h>
#include <iidm/ReactiveCapabilityCurve.h>
#include <iidm/MinMaxReactiveLimits.h>
#include <iidm/VoltageLevel.h>

#include <string>

using std::shared_ptr;

namespace DYN {

VscConverterInterfaceIIDM::VscConverterInterfaceIIDM(iidm::VscConverterStation& vsc) : VscConverterInterface(false),
                                                                                                InjectorInterfaceIIDM(vsc, vsc.getId()),
                                                                                                vscConverterIIDM_(vsc) {
  if (hasQInjector() || hasPInjector()) {
    hasInitialConditions(true);
  }

  setType(ComponentInterface::VSC_CONVERTER);
}

void
VscConverterInterfaceIIDM::importStaticParameters() {
  staticParameters_.clear();
  staticParameters_.insert(std::make_pair("p_pu", StaticParameter("p_pu", StaticParameter::DOUBLE).setValue(-1 * getP() / SNREF)));
  staticParameters_.insert(std::make_pair("q_pu", StaticParameter("q_pu", StaticParameter::DOUBLE).setValue(-1 * getQ() / SNREF)));
  staticParameters_.insert(std::make_pair("targetQ_pu", StaticParameter("targetQ_pu",
      StaticParameter::DOUBLE).setValue(-1 * getReactivePowerSetpoint() / SNREF)));
  staticParameters_.insert(std::make_pair("qMax_pu", StaticParameter("qMax_pu", StaticParameter::DOUBLE).setValue(getQMax() / SNREF)));
  staticParameters_.insert(std::make_pair("p", StaticParameter("p", StaticParameter::DOUBLE).setValue(-1 * getP())));
  staticParameters_.insert(std::make_pair("q", StaticParameter("q", StaticParameter::DOUBLE).setValue(-1 * getQ())));
  staticParameters_.insert(std::make_pair("targetQ", StaticParameter("targetQ", StaticParameter::DOUBLE).setValue(-1 * getReactivePowerSetpoint())));
  staticParameters_.insert(std::make_pair("qMax", StaticParameter("qMax", StaticParameter::DOUBLE).setValue(getQMax())));
  if (getBusInterface()) {
    double U0 = getBusInterface()->getV0();
    double vNom = vscConverterIIDM_.getHvdcLine().getNominalV();
    double theta = getBusInterface()->getAngle0();
    staticParameters_.insert(std::make_pair("v_pu", StaticParameter("v_pu", StaticParameter::DOUBLE).setValue(U0 / vNom)));
    staticParameters_.insert(std::make_pair("angle_pu", StaticParameter("angle_pu", StaticParameter::DOUBLE).setValue(theta * M_PI / 180)));
    staticParameters_.insert(std::make_pair("targetV_pu", StaticParameter("targetV_pu", StaticParameter::DOUBLE).setValue(getVoltageSetpoint() / getVNom())));
    staticParameters_.insert(std::make_pair("v", StaticParameter("v", StaticParameter::DOUBLE).setValue(U0)));
    staticParameters_.insert(std::make_pair("angle", StaticParameter("angle", StaticParameter::DOUBLE).setValue(theta)));
    staticParameters_.insert(std::make_pair("targetV", StaticParameter("targetV", StaticParameter::DOUBLE).setValue(getVoltageSetpoint())));
  } else {
    staticParameters_.insert(std::make_pair("v_pu", StaticParameter("v_pu", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("angle_pu", StaticParameter("angle_pu", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("targetV_pu", StaticParameter("targetV_pu", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("v", StaticParameter("v", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("angle", StaticParameter("angle", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("targetV", StaticParameter("targetV", StaticParameter::DOUBLE).setValue(0.)));
  }
}

void
VscConverterInterfaceIIDM::setBusInterface(const shared_ptr<BusInterface>& busInterface) {
  setBusInterfaceInjector(busInterface);
}

void
VscConverterInterfaceIIDM::setVoltageLevelInterface(const shared_ptr<VoltageLevelInterface>& voltageLevelInterface) {
  setVoltageLevelInterfaceInjector(voltageLevelInterface);
}

shared_ptr<BusInterface>
VscConverterInterfaceIIDM::getBusInterface() const {
  return getBusInterfaceInjector();
}

bool
VscConverterInterfaceIIDM::getInitialConnected() {
  return getInitialConnectedInjector();
}

bool
VscConverterInterfaceIIDM::isConnected() const {
  return isConnectedInjector();
}

double
VscConverterInterfaceIIDM::getVNom() const {
  return getVNomInjector();
}

bool
VscConverterInterfaceIIDM::hasP() {
  return hasPInjector();
}

bool
VscConverterInterfaceIIDM::hasQ() {
  return hasQInjector();
}

double
VscConverterInterfaceIIDM::getP() {
  return getPInjector();
}

double
VscConverterInterfaceIIDM::getQMax() {
  if (vscConverterIIDM_.hasReactiveCapabilityCurve()) {
    const auto curve = vscConverterIIDM_.getReactiveCapabilityCurve();
    const double p = -1 * getP();
    const auto points = curve.getPoints();
    if (points.empty()) return 0.;
    // Interpolate maxQ against p; fallback to nearest endpoint.
    if (p <= points.front().p) return points.front().maxQ;
    if (p >= points.back().p) return points.back().maxQ;
    for (size_t i = 1; i < points.size(); ++i) {
      if (p <= points[i].p) {
        const double t = (p - points[i - 1].p) / (points[i].p - points[i - 1].p);
        return points[i - 1].maxQ + t * (points[i].maxQ - points[i - 1].maxQ);
      }
    }
    return points.back().maxQ;
  }
  if (vscConverterIIDM_.hasMinMaxReactiveLimits()) {
    return vscConverterIIDM_.getMinMaxReactiveLimits().getMaxQ();
  }
  return 0.;
}

double
VscConverterInterfaceIIDM::getQMin() {
  if (vscConverterIIDM_.hasReactiveCapabilityCurve()) {
    const auto curve = vscConverterIIDM_.getReactiveCapabilityCurve();
    const double p = -1 * getP();
    const auto points = curve.getPoints();
    if (points.empty()) return 0.;
    if (p <= points.front().p) return points.front().minQ;
    if (p >= points.back().p) return points.back().minQ;
    for (size_t i = 1; i < points.size(); ++i) {
      if (p <= points[i].p) {
        const double t = (p - points[i - 1].p) / (points[i].p - points[i - 1].p);
        return points[i - 1].minQ + t * (points[i].minQ - points[i - 1].minQ);
      }
    }
    return points.back().minQ;
  }
  if (vscConverterIIDM_.hasMinMaxReactiveLimits()) {
    return vscConverterIIDM_.getMinMaxReactiveLimits().getMinQ();
  }
  return 0.;
}

std::vector<VscConverterInterface::ReactiveCurvePoint> VscConverterInterfaceIIDM::getReactiveCurvesPoints() const {
  std::vector<VscConverterInterface::ReactiveCurvePoint> ret;
  if (vscConverterIIDM_.hasReactiveCapabilityCurve()) {
    const auto reactiveCurve = vscConverterIIDM_.getReactiveCapabilityCurve();
    for (const auto& point : reactiveCurve.getPoints()) {
      ret.emplace_back(point.p, point.minQ, point.maxQ);
    }
  }

  return ret;
}

double
VscConverterInterfaceIIDM::getQ() {
  return getQInjector();
}

const std::string&
VscConverterInterfaceIIDM::getID() const {
  return getIDInjector();
}

double
VscConverterInterfaceIIDM::getLossFactor() const {
  return vscConverterIIDM_.getLossFactor();
}

bool
VscConverterInterfaceIIDM::getVoltageRegulatorOn() const {
  return vscConverterIIDM_.isVoltageRegulatorOn();
}

double
VscConverterInterfaceIIDM::getReactivePowerSetpoint() const {
  return vscConverterIIDM_.getReactivePowerSetpoint();
}

double
VscConverterInterfaceIIDM::getVoltageSetpoint() const {
  return vscConverterIIDM_.getVoltageSetpoint();
}

iidm::VscConverterStation&
VscConverterInterfaceIIDM::getVscIIDM() {
  return vscConverterIIDM_;
}

int
VscConverterInterfaceIIDM::getComponentVarIndex(const std::string& /*varName*/) const {
  return -1;
}

}  // namespace DYN
