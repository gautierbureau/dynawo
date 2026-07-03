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
 * @file  DYNModelTwoWindingsTransformerEMT.cpp
 */
#include <cmath>
#include "DYNModelTwoWindingsTransformerEMT.h"

namespace DYN {

ModelTwoWindingsTransformerEMT::ModelTwoWindingsTransformerEMT(NodeEMT* primary, NodeEMT* secondary,
                                                              double rTfo, double r, double l, const double i0[3], bool connected) :
primary_(primary), secondary_(secondary), rTfo_(rTfo), r_(r), l_(l), connected_(connected), alpha_(0.),
monitoredNode_(nullptr), vNomMonitored_(1.) {
  for (int k = 0; k < 3; ++k) i0_[k] = i0[k];
}

void
ModelTwoWindingsTransformerEMT::setRatioTapChanger(std::unique_ptr<ModelRatioTapChanger> tc, NodeEMT* monitored, double vNomMonitored) {
  ratioTapChanger_ = std::move(tc);
  monitoredNode_ = monitored;
  vNomMonitored_ = (vNomMonitored > 0. ? vNomMonitored : 1.);
  // the tap changer's rho is the secondary/primary voltage ratio (phasor U2 = rho*U1);
  // the EMT branch ratio is the inverse N1/N2, so rTfo = 1/rho
  rTfo_ = (ratioTapChanger_->getCurrentStep().getRho() > 0. ? 1.0 / ratioTapChanger_->getCurrentStep().getRho() : rTfo_);
}

void
ModelTwoWindingsTransformerEMT::setPhaseTapChanger(std::unique_ptr<ModelPhaseTapChanger> tc) {
  phaseTapChanger_ = std::move(tc);
  const TapChangerStep& step = phaseTapChanger_->getCurrentStep();
  if (step.getRho() > 0.) rTfo_ = 1.0 / step.getRho();
  alpha_ = step.getAlpha();   // phase shift in rad
}

void
ModelTwoWindingsTransformerEMT::applyTapTiming(double t1stTHT, double tNextTHT, double t1stHT, double tNextHT,
                                               bool hasTolV, double tolVRel) {
  // pick the THT delays for a VHV<->HV transformer (one end >= VHV_THRESHOLD, the other in [HV, VHV)),
  // else the HT delays -- the exact rule the phasor uses
  const bool end1VHV = (vNom1_ >= VHV_THRESHOLD), end1HV = (vNom1_ >= HV_THRESHOLD && vNom1_ < VHV_THRESHOLD);
  const bool end2VHV = (vNom2_ >= VHV_THRESHOLD), end2HV = (vNom2_ >= HV_THRESHOLD && vNom2_ < VHV_THRESHOLD);
  const bool tht = (end1VHV && end2HV) || (end2VHV && end1HV);
  const double tFirst = tht ? t1stTHT : t1stHT;
  const double tNext  = tht ? tNextTHT : tNextHT;
  if (ratioTapChanger_) {
    ratioTapChanger_->setTFirst(tFirst);
    ratioTapChanger_->setTNext(tNext);
    if (hasTolV) ratioTapChanger_->setTolV(tolVRel * vNomMonitored_);   // relative tol -> kV on the monitored side
  }
  if (phaseTapChanger_) {
    phaseTapChanger_->setTFirst(tFirst);
    phaseTapChanger_->setTNext(tNext);
  }
}

void
ModelTwoWindingsTransformerEMT::updateSecondaryKcl() {
  // secondary KCL injection is rTfo * R(-alpha) * i (3x3 rotation); coefficients change with rTfo/alpha
  const double c = rTfo_ * std::cos(alpha_);
  const double s = rTfo_ * std::sin(alpha_) / std::sqrt(3.0);
  for (int j = 0; j < 3; ++j) {
    secondary_->updateBranchCurrentCoeff(j, off_ + j, c);                  // d(KCL_j)/d i_j
    secondary_->updateBranchCurrentCoeff(j, off_ + (j + 2) % 3, -s);       // d(KCL_j)/d i_{j+2}
    secondary_->updateBranchCurrentCoeff(j, off_ + (j + 1) % 3, s);        // d(KCL_j)/d i_{j+1}
  }
}

void
ModelTwoWindingsTransformerEMT::evalGTap(double t, const double* y, state_g* g) {
  int off = gOff_;
  if (ratioTapChanger_) {                                  // ratio: monitored RMS voltage (kV)
    double sv = 0.;
    for (int k = 0; k < 3; ++k) { const double v = monitoredNode_->v(k, t, y); sv += v * v; }
    const double uKv = std::sqrt(sv / 3.) * vNomMonitored_;
    ratioTapChanger_->evalG(t, uKv, false, 0., 0., connected_, 0., &g[off]);
    off += ratioTapChanger_->sizeG();
  }
  if (phaseTapChanger_) {                                  // phase: branch RMS current
    const double si = y[off_] * y[off_] + y[off_ + 1] * y[off_ + 1] + y[off_ + 2] * y[off_ + 2];
    const double iRms = std::sqrt(si / 3.);
    phaseTapChanger_->evalG(t, iRms, false, 0., 0., connected_, &g[off]);
  }
}

StateChangeEMT
ModelTwoWindingsTransformerEMT::evalZTap(double t, const double* y, const state_g* g, SubModel* network) {
  bool moved = false;
  int off = gOff_;
  if (ratioTapChanger_) {
    ratioTapChanger_->evalZ(t, &g[off], 0., false, 0., connected_, network);
    if (ratioTapChanger_->getNextStepIndex() != -1) {
      ratioTapChanger_->applyStep();
      const double rho = ratioTapChanger_->getCurrentStep().getRho();
      if (rho > 0.) rTfo_ = 1.0 / rho;
      moved = true;
    }
    off += ratioTapChanger_->sizeG();
  }
  if (phaseTapChanger_) {
    // active-power direction at the primary (Sum vPrim_k * i_k): > 0 -> power flows 1 -> 2
    double p1 = 0.;
    for (int k = 0; k < 3; ++k) p1 += primary_->v(k, t, y) * y[off_ + k];
    phaseTapChanger_->evalZ(t, &g[off], 0., p1 > 0., 0., connected_, network);
    if (phaseTapChanger_->getNextStepIndex() != -1) {
      phaseTapChanger_->applyStep();
      const TapChangerStep& step = phaseTapChanger_->getCurrentStep();
      if (step.getRho() > 0.) rTfo_ = 1.0 / step.getRho();
      alpha_ = step.getAlpha();
      moved = true;
    }
  }
  if (moved) { updateSecondaryKcl(); return NC_STATE_CHANGE; }
  return NC_NO_CHANGE;
}

void
ModelTwoWindingsTransformerEMT::evalF(double t, const double* y, const double* yp, double* f) {
  if (!connected_) {                                       // open: i = 0, no injection
    for (int k = 0; k < 3; ++k) f[fOff_ + k] = y[off_ + k];
    return;
  }
  const double c = std::cos(alpha_);
  const double sn = std::sin(alpha_) / std::sqrt(3.0);
  for (int k = 0; k < 3; ++k) {
    const double i = y[off_ + k];
    // R(alpha) applied to vSec: cos(a)*vSec_k + sin(a)*(vSec_{k+2} - vSec_{k+1})/sqrt(3)
    const double rotV = c * secondary_->v(k, t, y)
                      + sn * (secondary_->v((k + 2) % 3, t, y) - secondary_->v((k + 1) % 3, t, y));
    f[fOff_ + k] = primary_->v(k, t, y) - rTfo_ * rotV - r_ * i - l_ * yp[off_ + k];
    primary_->addInjection(k, -i);                         // primary current enters the transformer
  }
  // secondary injection is rTfo * R(-alpha) * i (transpose rotation, power-conserving)
  for (int j = 0; j < 3; ++j) {
    const double rotI = c * y[off_ + j] - sn * (y[off_ + (j + 2) % 3] - y[off_ + (j + 1) % 3]);
    secondary_->addInjection(j, rTfo_ * rotI);
  }
}

void
ModelTwoWindingsTransformerEMT::evalJt(double cj, int rowOffset, SparseMatrix& jt) {
  if (!connected_) {
    for (int k = 0; k < 3; ++k) { jt.changeCol(); jt.addTerm(off_ + k + rowOffset, 1.0); }
    return;
  }
  const double c = rTfo_ * std::cos(alpha_);
  const double s = rTfo_ * std::sin(alpha_) / std::sqrt(3.0);
  for (int k = 0; k < 3; ++k) {
    jt.changeCol();
    jt.addTerm(off_ + k + rowOffset, -r_ - cj * l_);                   // d/di
    int vp = primary_->vIndex(k);
    if (vp >= 0) jt.addTerm(vp + rowOffset, 1.0);                      // d/dvPrim
    int vsk = secondary_->vIndex(k);
    if (vsk >= 0) jt.addTerm(vsk + rowOffset, -c);                     // d/dvSec_k
    int vs2 = secondary_->vIndex((k + 2) % 3);
    if (vs2 >= 0) jt.addTerm(vs2 + rowOffset, -s);                     // d/dvSec_{k+2}
    int vs1 = secondary_->vIndex((k + 1) % 3);
    if (vs1 >= 0) jt.addTerm(vs1 + rowOffset, s);                      // d/dvSec_{k+1}
  }
}

void
ModelTwoWindingsTransformerEMT::evalJtPrim(int rowOffset, SparseMatrix& jt) {
  for (int k = 0; k < 3; ++k) { jt.changeCol(); if (connected_) jt.addTerm(off_ + k + rowOffset, -l_); }
}

void
ModelTwoWindingsTransformerEMT::getZ0(double* z) { z[zOff_] = connected_ ? CONN_CLOSED : CONN_OPEN; }

StateChangeEMT
ModelTwoWindingsTransformerEMT::evalZ(double /*t*/, double* z) {
  bool wanted = zIsClosed(z[zOff_]);
  if (wanted != connected_) { connected_ = wanted; return NC_TOPO_CHANGE; }
  return NC_NO_CHANGE;
}

void
ModelTwoWindingsTransformerEMT::evalDynamicYType(const double* /*z*/, propertyContinuousVar_t* yType) {
  for (int k = 0; k < 3; ++k) yType[off_ + k] = (connected_ ? DIFFERENTIAL : ALGEBRAIC);
}

void
ModelTwoWindingsTransformerEMT::evalDynamicFType(const double* /*z*/, propertyF_t* fType) {
  for (int k = 0; k < 3; ++k) fType[fOff_ + k] = (connected_ ? DIFFERENTIAL_EQ : ALGEBRAIC_EQ);
}

void
ModelTwoWindingsTransformerEMT::getY0(double /*t*/, double* y, double* yp) {
  for (int k = 0; k < 3; ++k) { y[off_ + k] = i0_[k]; yp[off_ + k] = 0.; }
}

void
ModelTwoWindingsTransformerEMT::evalStaticYType(propertyContinuousVar_t* yType) {
  for (int k = 0; k < 3; ++k) yType[off_ + k] = DIFFERENTIAL;
}

void
ModelTwoWindingsTransformerEMT::evalStaticFType(propertyF_t* fType) {
  for (int k = 0; k < 3; ++k) fType[fOff_ + k] = DIFFERENTIAL_EQ;
}

void
ModelTwoWindingsTransformerEMT::wire() {
  const double c = rTfo_ * std::cos(alpha_);
  const double s = rTfo_ * std::sin(alpha_) / std::sqrt(3.0);
  for (int k = 0; k < 3; ++k)
    primary_->registerBranchCurrent(k, off_ + k, -1.0);     // -i into primary KCL
  // secondary KCL gets rTfo * R(-alpha) * i: three terms per phase (i_j, i_{j+2}, i_{j+1})
  for (int j = 0; j < 3; ++j) {
    secondary_->registerBranchCurrent(j, off_ + j, c);
    secondary_->registerBranchCurrent(j, off_ + (j + 2) % 3, -s);
    secondary_->registerBranchCurrent(j, off_ + (j + 1) % 3, s);
  }
}

void
ModelTwoWindingsTransformerEMT::addBusNeighbors() {
  if (!connected_) return;
  primary_->linkNeighbor(secondary_);
  secondary_->linkNeighbor(primary_);
}

double
ModelTwoWindingsTransformerEMT::evalCalcVar(int /*i*/, const double* y) const {
  const double s = y[off_] * y[off_] + y[off_ + 1] * y[off_ + 1] + y[off_ + 2] * y[off_ + 2];
  return std::sqrt(s / 3.);   // RMS primary current
}

void
ModelTwoWindingsTransformerEMT::calcVarIndexes(int /*i*/, std::vector<int>& idx) const {
  idx.push_back(off_); idx.push_back(off_ + 1); idx.push_back(off_ + 2);
}

void
ModelTwoWindingsTransformerEMT::evalJCalcVar(int /*i*/, const double* y, std::vector<double>& res) const {
  const double s = y[off_] * y[off_] + y[off_ + 1] * y[off_ + 1] + y[off_ + 2] * y[off_ + 2];
  const double im = std::sqrt(s / 3.);
  res.resize(3);
  for (int k = 0; k < 3; ++k) res[k] = (im > 0. ? y[off_ + k] / (3. * im) : 0.);
}

}  // namespace DYN
