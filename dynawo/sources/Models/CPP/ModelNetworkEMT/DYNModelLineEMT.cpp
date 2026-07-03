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
 * @file  DYNModelLineEMT.cpp
 */
#include <cmath>
#include "DYNModelLineEMT.h"

namespace DYN {

ModelLineEMT::ModelLineEMT(NodeEMT* from, NodeEMT* to, double r, double l, const double i0[3], bool connected) :
from_(from), to_(to), r_(r), l_(l), connected_(connected), factorPuToA_(1.) {
  for (int k = 0; k < 3; ++k) i0_[k] = i0[k];
}

void
ModelLineEMT::setCurrentLimits(std::unique_ptr<ModelCurrentLimits> cl, const std::string& id, double factorPuToA) {
  currentLimits_ = std::move(cl);
  id_ = id;
  factorPuToA_ = (factorPuToA > 0. ? factorPuToA : 1.);
}

void
ModelLineEMT::evalGLimit(double t, const double* y, state_g* g) {
  const double s = y[off_] * y[off_] + y[off_ + 1] * y[off_ + 1] + y[off_ + 2] * y[off_ + 2];
  const double iRmsA = std::sqrt(s / 3.) * factorPuToA_;            // RMS current in amps
  currentLimits_->evalG(t, iRmsA, connected_ ? 0. : 1., &g[gOff_]); // desactivate the roots when already open
}

StateChangeEMT
ModelLineEMT::evalZLimit(double t, const state_g* g, double* z, SubModel* network) {
  const ModelCurrentLimits::state_t st =
      currentLimits_->evalZ(id_, t, &g[gOff_], connected_ ? 0. : 1., "Line", network);
  if (st == ModelCurrentLimits::COMPONENT_OPEN && connected_) {
    connected_ = false;
    z[zOff_] = CONN_OPEN;   // latch the open state into the connection z so evalZ keeps it open
    return NC_TOPO_CHANGE;
  }
  return NC_NO_CHANGE;
}

void
ModelLineEMT::evalF(double t, const double* y, const double* yp, double* f) {
  if (!connected_) {                                        // open line: i = 0, no injection
    for (int k = 0; k < 3; ++k) f[fOff_ + k] = y[off_ + k];
    return;
  }
  for (int k = 0; k < 3; ++k) {
    double i = y[off_ + k];
    f[fOff_ + k] = from_->v(k, t, y) - to_->v(k, t, y) - r_ * i - l_ * yp[off_ + k];  // branch v-i
    from_->addInjection(k, -i);   // leaves the 'from' node
    to_->addInjection(k, i);      // enters the 'to' node
  }
}

void
ModelLineEMT::evalJt(double cj, int rowOffset, SparseMatrix& jt) {
  if (!connected_) {                                        // d(i)/di = 1
    for (int k = 0; k < 3; ++k) { jt.changeCol(); jt.addTerm(off_ + k + rowOffset, 1.0); }
    return;
  }
  for (int k = 0; k < 3; ++k) {
    jt.changeCol();
    jt.addTerm(off_ + k + rowOffset, -r_ - cj * l_);                 // d/di
    int vf = from_->vIndex(k), vt = to_->vIndex(k);
    if (vf >= 0) jt.addTerm(vf + rowOffset, 1.0);                    // d/dvFrom
    if (vt >= 0) jt.addTerm(vt + rowOffset, -1.0);                   // d/dvTo
  }
}

void
ModelLineEMT::evalJtPrim(int rowOffset, SparseMatrix& jt) {
  for (int k = 0; k < 3; ++k) { jt.changeCol(); if (connected_) jt.addTerm(off_ + k + rowOffset, -l_); }
}

void
ModelLineEMT::getZ0(double* z) { z[zOff_] = connected_ ? CONN_CLOSED : CONN_OPEN; }

StateChangeEMT
ModelLineEMT::evalZ(double /*t*/, double* z) {
  bool wanted = zIsClosed(z[zOff_]);
  if (wanted != connected_) { connected_ = wanted; return NC_TOPO_CHANGE; }
  return NC_NO_CHANGE;
}

void
ModelLineEMT::evalDynamicYType(const double* /*z*/, propertyContinuousVar_t* yType) {
  // currents are differential when carrying L*di/dt, algebraic (i = 0) when open
  for (int k = 0; k < 3; ++k) yType[off_ + k] = (connected_ ? DIFFERENTIAL : ALGEBRAIC);
}

void
ModelLineEMT::evalDynamicFType(const double* /*z*/, propertyF_t* fType) {
  for (int k = 0; k < 3; ++k) fType[fOff_ + k] = (connected_ ? DIFFERENTIAL_EQ : ALGEBRAIC_EQ);
}

void
ModelLineEMT::getY0(double /*t*/, double* y, double* yp) {
  for (int k = 0; k < 3; ++k) { y[off_ + k] = i0_[k]; yp[off_ + k] = 0.; }
}

void
ModelLineEMT::evalStaticYType(propertyContinuousVar_t* yType) {
  for (int k = 0; k < 3; ++k) yType[off_ + k] = DIFFERENTIAL;
}

void
ModelLineEMT::evalStaticFType(propertyF_t* fType) {
  for (int k = 0; k < 3; ++k) fType[fOff_ + k] = DIFFERENTIAL_EQ;
}

void
ModelLineEMT::wire() {
  for (int k = 0; k < 3; ++k) {
    from_->registerBranchCurrent(k, off_ + k, -1.0);   // leaves 'from'
    to_->registerBranchCurrent(k, off_ + k, 1.0);      // enters 'to'
  }
}

void
ModelLineEMT::addBusNeighbors() {
  if (!connected_) return;   // an open line does not connect its two ends
  from_->linkNeighbor(to_);
  to_->linkNeighbor(from_);
}

double
ModelLineEMT::evalCalcVar(int /*i*/, const double* y) const {
  const double s = y[off_] * y[off_] + y[off_ + 1] * y[off_ + 1] + y[off_ + 2] * y[off_ + 2];
  return std::sqrt(s / 3.);   // RMS branch current
}

void
ModelLineEMT::calcVarIndexes(int /*i*/, std::vector<int>& idx) const {
  idx.push_back(off_); idx.push_back(off_ + 1); idx.push_back(off_ + 2);
}

void
ModelLineEMT::evalJCalcVar(int /*i*/, const double* y, std::vector<double>& res) const {
  const double s = y[off_] * y[off_] + y[off_ + 1] * y[off_ + 1] + y[off_ + 2] * y[off_ + 2];
  const double im = std::sqrt(s / 3.);
  res.resize(3);
  for (int k = 0; k < 3; ++k) res[k] = (im > 0. ? y[off_ + k] / (3. * im) : 0.);
}

}  // namespace DYN
