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
 * @file  DYNModelSwitchEMT.cpp
 */
#include "DYNModelSwitchEMT.h"
#include "DYNModelBusEMT.h"

namespace DYN {

ModelSwitchEMT::ModelSwitchEMT(NodeEMT* bus1, NodeEMT* bus2, bool closed) :
bus1_(bus1), bus2_(bus2), closed_(closed) { }

namespace {
/// a node is "dead" if it is a bus that has been switched off
bool nodeSwitchedOff(NodeEMT* n) {
  ModelBusEMT* b = n->asBus();
  return b != nullptr && b->isSwitchedOff();
}
}  // namespace

bool
ModelSwitchEMT::effectiveClosed() const {
  return closed_ && !nodeSwitchedOff(bus1_) && !nodeSwitchedOff(bus2_);
}

void
ModelSwitchEMT::evalF(double t, const double* y, const double* yp, double* f) {
  if (effectiveClosed()) {
    for (int k = 0; k < 3; ++k) {
      double i = y[off_ + k];
      f[fOff_ + k] = bus1_->v(k, t, y) - bus2_->v(k, t, y);  // voltage equality
      bus1_->addInjection(k, -i);   // current leaves bus1
      bus2_->addInjection(k, i);    // and enters bus2
    }
  } else {
    for (int k = 0; k < 3; ++k) f[fOff_ + k] = y[off_ + k];  // open: i = 0
  }
}

void
ModelSwitchEMT::evalJt(double cj, int rowOffset, SparseMatrix& jt) {
  if (effectiveClosed()) {
    for (int k = 0; k < 3; ++k) {
      jt.changeCol();
      int v1 = bus1_->vIndex(k), v2 = bus2_->vIndex(k);
      if (v1 >= 0) jt.addTerm(v1 + rowOffset, 1.0);    // d(v1 - v2)/dv1
      if (v2 >= 0) jt.addTerm(v2 + rowOffset, -1.0);   // d(v1 - v2)/dv2
    }
  } else {
    for (int k = 0; k < 3; ++k) { jt.changeCol(); jt.addTerm(off_ + k + rowOffset, 1.0); }  // d(i)/di
  }
}

void
ModelSwitchEMT::evalJtPrim(int rowOffset, SparseMatrix& jt) {
  for (int k = 0; k < 3; ++k) jt.changeCol();  // no derivative terms (algebraic)
}

void
ModelSwitchEMT::getY0(double /*t*/, double* y, double* yp) {
  for (int k = 0; k < 3; ++k) { y[off_ + k] = 0.; yp[off_ + k] = 0.; }
}

void
ModelSwitchEMT::getZ0(double* z) {
  z[zOff_] = closed_ ? CONN_CLOSED : CONN_OPEN;
}

StateChangeEMT
ModelSwitchEMT::evalZ(double /*t*/, double* z) {
  bool wanted = zIsClosed(z[zOff_]);
  if (wanted != closed_) { closed_ = wanted; return NC_TOPO_CHANGE; }
  return NC_NO_CHANGE;
}

void
ModelSwitchEMT::evalStaticYType(propertyContinuousVar_t* yType) {
  for (int k = 0; k < 3; ++k) yType[off_ + k] = ALGEBRAIC;
}

void
ModelSwitchEMT::evalStaticFType(propertyF_t* fType) {
  for (int k = 0; k < 3; ++k) fType[fOff_ + k] = ALGEBRAIC_EQ;
}

void
ModelSwitchEMT::wire() {
  for (int k = 0; k < 3; ++k) {
    bus1_->registerBranchCurrent(k, off_ + k, -1.0);   // leaves bus1
    bus2_->registerBranchCurrent(k, off_ + k, 1.0);    // enters bus2
  }
}

void
ModelSwitchEMT::addBusNeighbors() {
  if (effectiveClosed()) {
    bus1_->linkNeighbor(bus2_);
    bus2_->linkNeighbor(bus1_);
  }
}

}  // namespace DYN
