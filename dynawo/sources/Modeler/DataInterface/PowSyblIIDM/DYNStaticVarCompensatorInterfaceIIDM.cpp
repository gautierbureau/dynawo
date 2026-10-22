//
// Copyright (c) 2015-2019, RTE (http://www.rte-france.com)
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
 * @file  DYNStaticVarCompensatorInterfaceIIDM.cpp
 *
 * @brief Static Var Compensator data interface : implementation file for IIDM interface
 *
 */
//======================================================================
#include "DYNStaticVarCompensatorInterfaceIIDM.h"
#include "DYNExecUtils.h"
#include "DYNFileSystemUtils.h"
#include <iidm/StaticVarCompensator.h>
#include <iidm/VoltageLevel.h>
#include <iidm/VoltagePerReactivePowerControl.h>
#include <iostream>

using iidm::StaticVarCompensator;
using std::string;
using std::shared_ptr;

namespace DYN {

StaticVarCompensatorInterfaceIIDM::~StaticVarCompensatorInterfaceIIDM() = default;

StaticVarCompensatorInterfaceIIDM::StaticVarCompensatorInterfaceIIDM(const StaticVarCompensator& svc) :
StaticVarCompensatorInterface(false),
InjectorInterfaceIIDM(svc, svc.getId()),
staticVarCompensatorIIDM_(svc) {
  if (hasQInjector() || hasPInjector()) {
    hasInitialConditions(true);
  }

  setType(ComponentInterface::SVC);

  hasVoltagePerReactivePowerControl_ = svc.hasVoltagePerReactivePowerControl();

  stateVariables_.resize(3);
  stateVariables_[VAR_P] = StateVariable("p", StateVariable::DOUBLE);  // P
  stateVariables_[VAR_Q] = StateVariable("q", StateVariable::DOUBLE);  // Q
  stateVariables_[VAR_STATE] = StateVariable("state", StateVariable::INT);  // connectionState
  // The PowSyBl StandbyAutomaton extension is not exposed by iidm-bridge yet,
  // so the regulating-mode state variable is not registered (hasStandbyAutomaton()
  // always returns false below).
}

int
StaticVarCompensatorInterfaceIIDM::getComponentVarIndex(const std::string& varName) const {
  if ( varName == "p" ) {
    return VAR_P;
  } else if  ( varName == "q" ) {
    return VAR_Q;
  } else if  ( varName == "state" ) {
    return VAR_STATE;
  } else if ( varName == "regulatingMode" ) {
    if ( stateVariables_.size() > VAR_REGULATINGMODE )
      return VAR_REGULATINGMODE;
    else
      Trace::warn() << DYNLog(RegulModeReqdNoSA, getID()) << Trace::endline;
  }
  return -1;
}

int
StaticVarCompensatorInterfaceIIDM::createComponentVarIndex(const std::string& varName) {
  if ((varName == "regulatingMode") && (stateVariables_.size() <= VAR_REGULATINGMODE)) {
    stateVariables_.resize(VAR_REGULATINGMODE+1);
    stateVariables_[VAR_REGULATINGMODE] = StateVariable("regulatingMode", StateVariable::INT);
    return VAR_REGULATINGMODE;
  }
  return -1;
}

void
StaticVarCompensatorInterfaceIIDM::exportStateVariablesUnitComponent() {
  staticVarCompensatorIIDM_.getTerminal().setP(-1 * getValue<double>(VAR_P) * SNREF);
  staticVarCompensatorIIDM_.getTerminal().setQ(-1 * getValue<double>(VAR_Q) * SNREF);
  bool connected = (getValue<int>(VAR_STATE) == CLOSED);
  // StandbyAutomaton extension not yet exposed by iidm-bridge - the
  // regulatingMode state variable is never registered, so nothing to export.

  if (getVoltageLevelInterfaceInjector()->isNodeBreakerTopology()) {
    // should be removed once a solution has been found to propagate switches (de)connection
    // following component (de)connection (only Modelica models)
    if (connected && !getInitialConnected())
      getVoltageLevelInterfaceInjector()->connectNode(static_cast<unsigned int>(staticVarCompensatorIIDM_.getTerminal().getNodeBreakerView().getNode()));
    else if (!connected && getInitialConnected())
      getVoltageLevelInterfaceInjector()->disconnectNode(static_cast<unsigned int>(staticVarCompensatorIIDM_.getTerminal().getNodeBreakerView().getNode()));
  } else {
    if (connected)
      staticVarCompensatorIIDM_.getTerminal().connect();
    else
      staticVarCompensatorIIDM_.getTerminal().disconnect();
  }
}

void
StaticVarCompensatorInterfaceIIDM::importStaticParameters() {
  staticParameters_.clear();
  staticParameters_.insert(std::make_pair("p", StaticParameter("p", StaticParameter::DOUBLE).setValue(getPInjector())));
  staticParameters_.insert(std::make_pair("q", StaticParameter("q", StaticParameter::DOUBLE).setValue(getQ())));
  staticParameters_.insert(std::make_pair("p_pu", StaticParameter("p_pu", StaticParameter::DOUBLE).setValue(getPInjector() / SNREF)));
  staticParameters_.insert(std::make_pair("q_pu", StaticParameter("q_pu", StaticParameter::DOUBLE).setValue(getQ() / SNREF)));
  int regulatingMode = getRegulationMode();
  staticParameters_.insert(std::make_pair("regulatingMode", StaticParameter("regulatingMode", StaticParameter::INT).setValue(regulatingMode)));
  if (getBusInterface()) {
    double U0 = getBusInterface()->getV0();
    double vNom;
    if (staticVarCompensatorIIDM_.getTerminal().getVoltageLevel().getNominalV() > 0)
      vNom = staticVarCompensatorIIDM_.getTerminal().getVoltageLevel().getNominalV();
    else
      throw DYNError(Error::MODELER, UndefinedNominalV, staticVarCompensatorIIDM_.getTerminal().getVoltageLevel().getId());

    double theta = getBusInterface()->getAngle0();
    staticParameters_.insert(std::make_pair("v", StaticParameter("v", StaticParameter::DOUBLE).setValue(U0)));
    staticParameters_.insert(std::make_pair("angle", StaticParameter("angle", StaticParameter::DOUBLE).setValue(theta)));
    staticParameters_.insert(std::make_pair("v_pu", StaticParameter("v_pu", StaticParameter::DOUBLE).setValue(U0 / vNom)));
    staticParameters_.insert(std::make_pair("angle_pu", StaticParameter("angle_pu", StaticParameter::DOUBLE).setValue(theta * M_PI / 180)));
    staticParameters_.insert(std::make_pair("UNom", StaticParameter("UNom", StaticParameter::DOUBLE).setValue(vNom)));
  } else {
    staticParameters_.insert(std::make_pair("v", StaticParameter("v", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("angle", StaticParameter("angle", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("v_pu", StaticParameter("v_pu", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("angle_pu", StaticParameter("angle_pu", StaticParameter::DOUBLE).setValue(0.)));
    staticParameters_.insert(std::make_pair("UNom", StaticParameter("UNom", StaticParameter::DOUBLE).setValue(0.)));
  }
}

void
StaticVarCompensatorInterfaceIIDM::setBusInterface(const shared_ptr<BusInterface>& busInterface) {
  setBusInterfaceInjector(busInterface);
}

shared_ptr<BusInterface>
StaticVarCompensatorInterfaceIIDM::getBusInterface() const {
  return getBusInterfaceInjector();
}

void
StaticVarCompensatorInterfaceIIDM::setVoltageLevelInterface(const shared_ptr<VoltageLevelInterface>& voltageLevelInterface) {
  setVoltageLevelInterfaceInjector(voltageLevelInterface);
}

bool
StaticVarCompensatorInterfaceIIDM::getInitialConnected() {
  return getInitialConnectedInjector();
}

bool
StaticVarCompensatorInterfaceIIDM::isConnected() const {
  return isConnectedInjector();
}

double
StaticVarCompensatorInterfaceIIDM::getVNom() const {
  return getVNomInjector();
}

const std::string&
StaticVarCompensatorInterfaceIIDM::getID() const {
  return getIDInjector();
}

double
StaticVarCompensatorInterfaceIIDM::getBMin() const {
  return staticVarCompensatorIIDM_.getBMin();
}

double
StaticVarCompensatorInterfaceIIDM::getBMax() const {
  return staticVarCompensatorIIDM_.getBMax();
}

double
StaticVarCompensatorInterfaceIIDM::getP() {
  return getPInjector();
}

double
StaticVarCompensatorInterfaceIIDM::getQ() {
  return getQInjector();
}

bool
StaticVarCompensatorInterfaceIIDM::hasVoltagePerReactivePowerControl() const {
  return hasVoltagePerReactivePowerControl_;
}

double
StaticVarCompensatorInterfaceIIDM::getSlope() const {
  if (hasVoltagePerReactivePowerControl_) {
    return staticVarCompensatorIIDM_.getVoltagePerReactivePowerControl().getSlope();
  }
  return 0.;
}

double
StaticVarCompensatorInterfaceIIDM::getVSetPoint() const {
  return staticVarCompensatorIIDM_.getVoltageSetpoint();
}

double
StaticVarCompensatorInterfaceIIDM::getReactivePowerSetPoint() const {
  return staticVarCompensatorIIDM_.getReactivePowerSetpoint();
}

// StandbyAutomaton extension is not exposed by iidm-bridge yet; return defaults
// until the bridge surfaces it. PowSyBl reference: StandbyAutomaton extension
// on StaticVarCompensator (UMinActivation, UMaxActivation, UMin/MaxSetPoint,
// standby flag, B0).
double
StaticVarCompensatorInterfaceIIDM::getUMinActivation() const { return 0.0; }

double
StaticVarCompensatorInterfaceIIDM::getUMaxActivation() const { return 0.0; }

double
StaticVarCompensatorInterfaceIIDM::getUSetPointMin() const { return 0.0; }

double
StaticVarCompensatorInterfaceIIDM::getUSetPointMax() const { return 0.0; }

bool
StaticVarCompensatorInterfaceIIDM::hasStandbyAutomaton() const { return false; }

bool
StaticVarCompensatorInterfaceIIDM::isStandBy() const { return false; }

double
StaticVarCompensatorInterfaceIIDM::getB0() const { return 0.0; }

StaticVarCompensatorInterface::RegulationMode_t StaticVarCompensatorInterfaceIIDM::getRegulationMode() const {
  iidm::StaticVarCompensatorRegulationMode regMode = staticVarCompensatorIIDM_.getRegulationMode();
  switch (regMode) {
    case iidm::StaticVarCompensatorRegulationMode::VOLTAGE:
      return StaticVarCompensatorInterface::RUNNING_V;
    case iidm::StaticVarCompensatorRegulationMode::REACTIVE_POWER:
      return StaticVarCompensatorInterface::RUNNING_Q;
    case iidm::StaticVarCompensatorRegulationMode::OFF:
      return StaticVarCompensatorInterface::OFF;
    default:
      return StaticVarCompensatorInterface::OFF;
  }
}

}  // namespace DYN
