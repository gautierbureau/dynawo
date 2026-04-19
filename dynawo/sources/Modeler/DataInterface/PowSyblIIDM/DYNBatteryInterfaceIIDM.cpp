//
// Copyright (c) 2021, RTE (http://www.rte-france.com)
// See AUTHORS.txt
// All rights reserved.
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, you can obtain one at http://mozilla.org/MPL/2.0/.
// SPDX-License-Identifier: MPL-2.0
//
// This file is part of Dynawo, an hybrid C++/Modelica open source suite of simulation tools
// for power systems.
//


//======================================================================
/**
 * @file  DYNBatteryInterfaceIIDM.cpp
 *
 * @brief Battery data interface : implementation file for IIDM interface
 *
 */
//======================================================================

#include "DYNBatteryInterfaceIIDM.h"

#include <iidm/VoltageLevel.h>

using std::string;
using std::vector;

namespace DYN {

BatteryInterfaceIIDM::BatteryInterfaceIIDM(iidm::Battery& battery) :
InjectorInterfaceIIDM(battery, battery.getId()),
batteryIIDM_(battery) {
  setType(ComponentInterface::GENERATOR);
  const bool neededForCriteriaCheck = true;
  stateVariables_.resize(3);
  stateVariables_[VAR_P] = StateVariable("p", StateVariable::DOUBLE, neededForCriteriaCheck);  // P
  stateVariables_[VAR_Q] = StateVariable("q", StateVariable::DOUBLE);  // Q
  stateVariables_[VAR_STATE] = StateVariable("state", StateVariable::INT);   // connectionState
  // TODO(iidm-bridge): Battery does not expose extension accessors in iidm-bridge yet.
  hasActivePowerControl_ = false;
}

int
BatteryInterfaceIIDM::getComponentVarIndex(const std::string& varName) const {
  int index = -1;
  if ( varName == "p" )
    index = VAR_P;
  else if ( varName == "q" )
    index = VAR_Q;
  else if ( varName == "state" )
    index = VAR_STATE;
  return index;
}

void
BatteryInterfaceIIDM::exportStateVariablesUnitComponent() {
  bool connected = (getValue<int>(VAR_STATE) == CLOSED);
  batteryIIDM_.getTerminal().setP(-1 * getValue<double>(VAR_P) * SNREF);
  batteryIIDM_.getTerminal().setQ(-1 * getValue<double>(VAR_Q) * SNREF);

  if (getVoltageLevelInterfaceInjector()->isNodeBreakerTopology()) {
    // should be removed once a solution has been found to propagate switches (de)connection
    // following component (de)connection (only Modelica models)
    if (connected && !getInitialConnected())
      getVoltageLevelInterfaceInjector()->connectNode(static_cast<unsigned int>(batteryIIDM_.getTerminal().getNodeBreakerView().getNode()));
    else if (!connected && getInitialConnected())
      getVoltageLevelInterfaceInjector()->disconnectNode(static_cast<unsigned int>(batteryIIDM_.getTerminal().getNodeBreakerView().getNode()));
  } else {
    if (connected)
      batteryIIDM_.getTerminal().connect();
    else
      batteryIIDM_.getTerminal().disconnect();
  }
}

void
BatteryInterfaceIIDM::importStaticParameters() {
  staticParameters_.clear();
  double pMax = getPMax();
  double qMax = getQMax();
  double qMin = getQMin();
  staticParameters_.insert(std::make_pair("p_pu", StaticParameter("p_pu", StaticParameter::DOUBLE).setValue(getP() / SNREF)));
  staticParameters_.insert(std::make_pair("q_pu", StaticParameter("q_pu", StaticParameter::DOUBLE).setValue(getQ() / SNREF)));
  staticParameters_.insert(std::make_pair("pMin_pu", StaticParameter("pMin_pu", StaticParameter::DOUBLE).setValue(getPMin() / SNREF)));
  staticParameters_.insert(std::make_pair("pMax_pu", StaticParameter("pMax_pu", StaticParameter::DOUBLE).setValue(pMax / SNREF)));
  staticParameters_.insert(std::make_pair("qMax_pu", StaticParameter("qMax_pu", StaticParameter::DOUBLE).setValue(qMax / SNREF)));
  staticParameters_.insert(std::make_pair("qMin_pu", StaticParameter("qMin_pu", StaticParameter::DOUBLE).setValue(qMin / SNREF)));
  staticParameters_.insert(std::make_pair("p", StaticParameter("p", StaticParameter::DOUBLE).setValue(getP())));
  staticParameters_.insert(std::make_pair("q", StaticParameter("q", StaticParameter::DOUBLE).setValue(getQ())));
  staticParameters_.insert(std::make_pair("pMin", StaticParameter("pMin", StaticParameter::DOUBLE).setValue(getPMin())));
  staticParameters_.insert(std::make_pair("pMax", StaticParameter("pMax", StaticParameter::DOUBLE).setValue(pMax)));
  staticParameters_.insert(std::make_pair("qMax", StaticParameter("qMax", StaticParameter::DOUBLE).setValue(qMax)));
  staticParameters_.insert(std::make_pair("qMin", StaticParameter("qMin", StaticParameter::DOUBLE).setValue(qMin)));
  staticParameters_.insert(std::make_pair("targetP_pu", StaticParameter("targetP_pu", StaticParameter::DOUBLE).setValue(getTargetP() / SNREF)));
  staticParameters_.insert(std::make_pair("targetP", StaticParameter("targetP", StaticParameter::DOUBLE).setValue(getTargetP())));
  double qNom = getQNom();
  double sNom = sqrt(pMax * pMax + qNom * qNom);
  staticParameters_.insert(std::make_pair("sNom", StaticParameter("sNom", StaticParameter::DOUBLE).setValue(sNom)));
  if (getBusInterface()) {
    double U0 = getBusInterface()->getV0();
    double vNom;
    if (batteryIIDM_.getTerminal().getVoltageLevel().getNominalV() > 0)
      vNom = batteryIIDM_.getTerminal().getVoltageLevel().getNominalV();
    else
      throw DYNError(Error::MODELER, UndefinedNominalV, batteryIIDM_.getTerminal().getVoltageLevel().getId());

    double theta = getBusInterface()->getAngle0();
    staticParameters_.insert(std::make_pair("v_pu", StaticParameter("v_pu", StaticParameter::DOUBLE).setValue(U0 / vNom)));
    staticParameters_.insert(std::make_pair("angle_pu", StaticParameter("angle_pu", StaticParameter::DOUBLE).setValue(theta * M_PI / 180)));
    staticParameters_.insert(std::make_pair("uc_pu", StaticParameter("uc", StaticParameter::DOUBLE).setValue(U0 / vNom)));
    staticParameters_.insert(std::make_pair("v", StaticParameter("v", StaticParameter::DOUBLE).setValue(U0)));
    staticParameters_.insert(std::make_pair("uc", StaticParameter("uc", StaticParameter::DOUBLE).setValue(U0)));
    staticParameters_.insert(std::make_pair("angle", StaticParameter("angle", StaticParameter::DOUBLE).setValue(theta)));
    staticParameters_.insert(std::make_pair("vNom", StaticParameter("vNom", StaticParameter::DOUBLE).setValue(vNom)));
    staticParameters_.insert(std::make_pair("targetV_pu", StaticParameter("targetV_pu", StaticParameter::DOUBLE).setValue(getTargetV() / vNom)));
    staticParameters_.insert(std::make_pair("targetV", StaticParameter("targetV", StaticParameter::DOUBLE).setValue(getTargetV())));
    staticParameters_.insert(std::make_pair("targetQ_pu", StaticParameter("targetQ_pu", StaticParameter::DOUBLE).setValue(getTargetQ() / SNREF)));
    staticParameters_.insert(std::make_pair("targetQ", StaticParameter("targetQ", StaticParameter::DOUBLE).setValue(getTargetQ())));
  } else {
    staticParameters_.insert(std::make_pair("v_pu", StaticParameter("v_pu", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("angle_pu", StaticParameter("angle_pu", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("uc_pu", StaticParameter("uc", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("v", StaticParameter("v", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("angle", StaticParameter("angle", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("uc", StaticParameter("uc", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("vNom", StaticParameter("vNom", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("targetV_pu", StaticParameter("targetV_pu", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("targetV", StaticParameter("targetV", StaticParameter::DOUBLE).setValue(0.)));
  }
}

void
BatteryInterfaceIIDM::setBusInterface(const std::shared_ptr<BusInterface>& busInterface) {
  setBusInterfaceInjector(busInterface);
}

void
BatteryInterfaceIIDM::setVoltageLevelInterface(const std::shared_ptr<VoltageLevelInterface>& voltageLevelInterface) {
  setVoltageLevelInterfaceInjector(voltageLevelInterface);
}

std::shared_ptr<BusInterface>
BatteryInterfaceIIDM::getBusInterface() const {
  return getBusInterfaceInjector();
}

bool
BatteryInterfaceIIDM::getInitialConnected() {
  return getInitialConnectedInjector();
}

double
BatteryInterfaceIIDM::getP() {
  return getPInjector();
}

double
BatteryInterfaceIIDM::getStateVarP() {
  return getValue<double>(VAR_P);
}

double
BatteryInterfaceIIDM::getPMin() {
  return batteryIIDM_.getMinP();
}

double
BatteryInterfaceIIDM::getPMax() {
  return batteryIIDM_.getMaxP();
}

double
BatteryInterfaceIIDM::getTargetP() {
  return 0.;
}


double
BatteryInterfaceIIDM::getQ() {
  return getQInjector();
}

double
BatteryInterfaceIIDM::getQMax() {
  // TODO(iidm-bridge): Battery does not expose reactive-limit accessors; using default fallback.
  return 0.3 * getPMax();
}

double
BatteryInterfaceIIDM::getQNom() {
  // TODO(iidm-bridge): Battery does not expose reactive-limit accessors; using default fallback.
  return 0.3 * getPMax();
}

double
BatteryInterfaceIIDM::getQMin() {
  // TODO(iidm-bridge): Battery does not expose reactive-limit accessors; using default fallback.
  return -0.3 * getPMax();
}

double
BatteryInterfaceIIDM::getTargetQ() {
  return 0.;
}

double
BatteryInterfaceIIDM::getTargetV() {
  return 0.;
}


const std::string&
BatteryInterfaceIIDM::getID() const {
  return getIDInjector();
}

vector<GeneratorInterface::ReactiveCurvePoint> BatteryInterfaceIIDM::getReactiveCurvesPoints() const {
  // TODO(iidm-bridge): Battery does not expose reactive-capability curve accessors in the new API.
  return vector<GeneratorInterface::ReactiveCurvePoint>();
}

bool BatteryInterfaceIIDM::isVoltageRegulationOn() const {
  return false;
}

bool
BatteryInterfaceIIDM::hasActivePowerControl() const {
  return hasActivePowerControl_;
}

bool
BatteryInterfaceIIDM::isParticipating() const {
  // TODO(iidm-bridge): Battery ActivePowerControl extension not yet supported.
  return false;
}

double
BatteryInterfaceIIDM::getActivePowerControlDroop() const {
  // TODO(iidm-bridge): Battery ActivePowerControl extension not yet supported.
  return 0.;
}

bool
BatteryInterfaceIIDM::hasCoordinatedReactiveControl() const {
  return false;
}

double
BatteryInterfaceIIDM::getCoordinatedReactiveControlPercentage() const {
  return 0.;
}

GeneratorInterface::EnergySource_t
BatteryInterfaceIIDM::getEnergySource() const {
  return SOURCE_OTHER;
}

}  // namespace DYN
