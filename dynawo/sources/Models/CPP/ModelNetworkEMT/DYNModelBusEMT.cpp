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

/**
 * @file  DYNModelBusEMT.cpp
 */
#include <cmath>
#include "DYNModelBusEMT.h"
#include "DYNVariable.h"
#include "DYNVariableNative.h"
#include "DYNVariableNativeFactory.h"
#include "DYNVariableAlias.h"
#include "DYNVariableAliasFactory.h"
#include "DYNElement.h"
#include "DYNCommonModeler.h"

namespace DYN {

ModelBusEMT::ModelBusEMT(double cPu, double rShunt, const double u0[3]) :
cPu_(cPu), hasLocalRef_(cPu > 0.), switchedOff_(false), subNetwork_(-1) {
  for (int k = 0; k < 3; ++k) { u0_[k] = u0[k]; inj_[k] = 0.; shuntG_[k] = (rShunt > 0 ? 1.0 / rShunt : 0.); }
}

void
ModelBusEMT::resetInjection() {
  for (int k = 0; k < 3; ++k) inj_[k] = 0.;
}

void
ModelBusEMT::registerBranchCurrent(int phase, int iIdx, double sign) {
  branchCur_[phase].push_back(std::make_pair(iIdx, sign));
}

void
ModelBusEMT::updateBranchCurrentCoeff(int phase, int iIdx, double coeff) {
  for (size_t b = 0; b < branchCur_[phase].size(); ++b)
    if (branchCur_[phase][b].first == iIdx) { branchCur_[phase][b].second = coeff; return; }
}

void
ModelBusEMT::evalF(double /*t*/, const double* y, const double* yp, double* f) {
  if (switchedOff_) {                                            // dead island: force v = 0
    for (int k = 0; k < 3; ++k) f[fOff_ + k] = y[off_ + k];     // residual at fOff_, voltage state at off_
    return;
  }
  // KCL: sum of injected currents - shunt-conductance draw = C * v'  (C = 0 -> algebraic node)
  // a connection bus adds its abc current through the in-block ACPIN FLOW vars at off_+3..off_+5.
  // Every EMT model (the machine, a fault, a load) renders abc current in the SAME standard
  // convention as this network (peak = sqrt(2)*Irms), so flowScale_ is 1; it is kept as a per-bus
  // hook only in case a model in a different per-unit base is ever attached.
  for (int k = 0; k < 3; ++k) {
    f[fOff_ + k] = inj_[k] - (shuntG_[k] + faultG_[k]) * y[off_ + k] - cPu_ * yp[off_ + k];
    if (hasExtConn_) f[fOff_ + k] += flowScale_ * y[off_ + 3 + k];
  }
}

void
ModelBusEMT::evalJt(double cj, int rowOffset, SparseMatrix& jt) {
  if (switchedOff_) {                                            // d(v)/dv = 1
    for (int k = 0; k < 3; ++k) { jt.changeCol(); jt.addTerm(off_ + k + rowOffset, 1.0); }
    return;
  }
  for (int k = 0; k < 3; ++k) {
    jt.changeCol();
    jt.addTerm(off_ + k + rowOffset, -shuntG_[k] - faultG_[k] - cj * cPu_);    // d(KCL)/dV
    for (size_t b = 0; b < branchCur_[k].size(); ++b)             // d(KCL)/d(i_branch) = +/-1
      jt.addTerm(branchCur_[k][b].first + rowOffset, branchCur_[k][b].second);
    if (hasExtConn_)                                              // d(KCL)/d(in-block ACPIN FLOW current)
      jt.addTerm(off_ + 3 + k + rowOffset, flowScale_);
  }
}

void
ModelBusEMT::evalJtPrim(int rowOffset, SparseMatrix& jt) {
  // one column per equation (keeps the global column alignment); the C term only if capacitive and live
  for (int k = 0; k < 3; ++k) { jt.changeCol(); if (cPu_ > 0 && !switchedOff_) jt.addTerm(off_ + k + rowOffset, -cPu_); }
}

void
ModelBusEMT::getY0(double /*t*/, double* y, double* yp) {
  for (int k = 0; k < 3; ++k) { y[off_ + k] = (switchedOff_ ? 0. : u0_[k]); yp[off_ + k] = 0.; }
  if (hasExtConn_)                                              // in-block ACPIN FLOW currents start at 0
    for (int k = 0; k < 3; ++k) { y[off_ + 3 + k] = 0.; yp[off_ + 3 + k] = 0.; }
}

void
ModelBusEMT::evalStaticYType(propertyContinuousVar_t* yType) {
  // a node with a shunt C carries a voltage state (differential); without C it is algebraic
  for (int k = 0; k < 3; ++k) yType[off_ + k] = (cPu_ > 0 ? DIFFERENTIAL : ALGEBRAIC);
  if (hasExtConn_) for (int k = 0; k < 3; ++k) yType[off_ + 3 + k] = ALGEBRAIC;   // ACPIN FLOW currents
}

void
ModelBusEMT::evalStaticFType(propertyF_t* fType) {
  // only the 3 KCL residuals (the FLOW currents' equation is the connector's, not the bus's)
  for (int k = 0; k < 3; ++k) fType[fOff_ + k] = (cPu_ > 0 ? DIFFERENTIAL_EQ : ALGEBRAIC_EQ);
}

void
ModelBusEMT::evalDynamicYType(const double* /*z*/, propertyContinuousVar_t* yType) {
  // a switched-off bus is pinned to 0 (algebraic) whatever its shunt C
  for (int k = 0; k < 3; ++k) yType[off_ + k] = ((cPu_ > 0 && !switchedOff_) ? DIFFERENTIAL : ALGEBRAIC);
  if (hasExtConn_) for (int k = 0; k < 3; ++k) yType[off_ + 3 + k] = ALGEBRAIC;
}

void
ModelBusEMT::evalDynamicFType(const double* /*z*/, propertyF_t* fType) {
  for (int k = 0; k < 3; ++k) fType[fOff_ + k] = ((cPu_ > 0 && !switchedOff_) ? DIFFERENTIAL_EQ : ALGEBRAIC_EQ);
}

double
ModelBusEMT::evalCalcVar(int /*i*/, const double* y) const {
  const double s = y[off_] * y[off_] + y[off_ + 1] * y[off_ + 1] + y[off_ + 2] * y[off_ + 2];
  return std::sqrt(s / 3.);   // RMS phase voltage (= |V| for a balanced abc set)
}

void
ModelBusEMT::calcVarIndexes(int /*i*/, std::vector<int>& idx) const {
  idx.push_back(off_); idx.push_back(off_ + 1); idx.push_back(off_ + 2);
}

void
ModelBusEMT::evalJCalcVar(int /*i*/, const double* y, std::vector<double>& res) const {
  const double s = y[off_] * y[off_] + y[off_ + 1] * y[off_ + 1] + y[off_ + 2] * y[off_ + 2];
  const double u = std::sqrt(s / 3.);
  res.resize(3);
  for (int k = 0; k < 3; ++k) res[k] = (u > 0. ? y[off_ + k] / (3. * u) : 0.);  // d/dv_k of sqrt(S/3)
}

void
ModelBusEMT::instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) {
  static const char* ph[3] = {"a", "b", "c"};
  for (int k = 0; k < 3; ++k)
    variables.push_back(VariableNativeFactory::createState(id_ + "_v_" + ph[k], CONTINUOUS));
  // a connection bus's 3 abc ACPIN FLOW currents sit IN its y-block, right after the voltages
  // (off_+3..off_+5), with v aliases so the connector equates the terminal voltage to the bus voltage
  if (hasExtConn_) {
    for (int k = 0; k < 3; ++k)
      variables.push_back(VariableNativeFactory::createState(id_ + "_ACPIN_i_" + std::to_string(k) + "_", FLOW));
    for (int k = 0; k < 3; ++k)
      variables.push_back(VariableAliasFactory::create(id_ + "_ACPIN_v_" + std::to_string(k) + "_", id_ + "_v_" + ph[k]));
  }
  variables.push_back(VariableNativeFactory::createCalculated(id_ + "_Urms", CONTINUOUS));
}

void
ModelBusEMT::defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                            const std::string& parentName, const std::string& parentType) {
  static const char* ph[3] = {"a", "b", "c"};
  // bus abc voltages are LEAF terminals (element id = variable name) so an external machine's
  // connector leaf (machine_terminal_v_k_) matches them directly
  for (int k = 0; k < 3; ++k)
    addElement(id_ + "_v_" + ph[k], Element::TERMINAL, elements, mapElement);
  // abc ACPIN for a connection bus: <id>_ACPIN { v_k_ (potential -> bus voltage), i_k_ (flow) },
  // matching the machine's EmtTerminal so connect(machine_terminal, <bus>_ACPIN) binds v<->v, i<->i
  if (hasExtConn_) {
    const std::string ac = id_ + "_ACPIN";
    addElement(ac, Element::STRUCTURE, elements, mapElement);
    for (int k = 0; k < 3; ++k) {
      addSubElement("v_" + std::to_string(k) + "_", ac, Element::TERMINAL, parentName, parentType, elements, mapElement);
      addSubElement("i_" + std::to_string(k) + "_", ac, Element::TERMINAL, parentName, parentType, elements, mapElement);
    }
  }
}

void
ModelBusEMT::linkNeighbor(NodeEMT* other) {
  if (other->imposesVoltage()) hasLocalRef_ = true;   // adjacent source -> this bus is referenced
  ModelBusEMT* b = other->asBus();
  if (b) addNeighbor(b);                               // bus-to-bus adjacency for the island BFS
}

void
ModelBusEMT::resetConnectivity() {
  neighbors_.clear();
  subNetwork_ = -1;
  hasLocalRef_ = (cPu_ > 0.);   // keep only the permanent (own shunt C) reference; sources re-mark via linkNeighbor
}

void
ModelBusEMT::exploreNeighbors(int numSubNetwork, std::vector<ModelBusEMT*>& island) {
  for (size_t i = 0; i < neighbors_.size(); ++i) {
    ModelBusEMT* bus = neighbors_[i];
    if (!bus->numSubNetworkSet()) {
      bus->setNumSubNetwork(numSubNetwork);
      island.push_back(bus);
      bus->exploreNeighbors(numSubNetwork, island);
    }
  }
}

// ---------------------------------------------------------------------------
//  SubNetworkEMT
// ---------------------------------------------------------------------------
bool
SubNetworkEMT::hasReference() const {
  for (size_t i = 0; i < buses_.size(); ++i)
    if (buses_[i]->hasLocalReference()) return true;
  return false;
}

void
SubNetworkEMT::shutDown() {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->switchOff();
}

void
SubNetworkEMT::turnOn() {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->switchOn();
}

// ---------------------------------------------------------------------------
//  ModelBusContainerEMT
// ---------------------------------------------------------------------------
void
ModelBusContainerEMT::resetConnectivity() {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->resetConnectivity();
}

void
ModelBusContainerEMT::exploreNeighbors() {
  subNetworks_.clear();
  int num = 0;
  for (size_t i = 0; i < buses_.size(); ++i) {
    if (!buses_[i]->numSubNetworkSet()) {
      SubNetworkEMT island;
      buses_[i]->setNumSubNetwork(num);
      std::vector<ModelBusEMT*> members;
      members.push_back(buses_[i]);
      buses_[i]->exploreNeighbors(num, members);
      for (size_t m = 0; m < members.size(); ++m) island.addBus(members[m]);
      subNetworks_.push_back(island);
      ++num;
    }
  }
}

void
ModelBusContainerEMT::analyseIslands() {
  // an island with no voltage reference (no source, no shunt C) is singular -> shut it off
  for (size_t i = 0; i < subNetworks_.size(); ++i) {
    if (subNetworks_[i].hasReference()) subNetworks_[i].turnOn();
    else                                subNetworks_[i].shutDown();
  }
}

}  // namespace DYN
