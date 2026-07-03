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
 * @file  DYNModelGeneratorEMT.cpp
 */
#include <cmath>
#include "DYNModelGeneratorEMT.h"

namespace DYN {

ModelGeneratorInjectionEMT::ModelGeneratorInjectionEMT(NodeEMT* node, double iPeak, double phase0, double fNom, bool connected) :
node_(node), iPeak_(iPeak), phase0_(phase0), fNom_(fNom), connected_(connected) { }

double
ModelGeneratorInjectionEMT::inj(double t, int phase) const {
  static const double shift[3] = {0., -2. * M_PI / 3., 2. * M_PI / 3.};
  return iPeak_ * std::cos(2. * M_PI * fNom_ * t + phase0_ + shift[phase]);
}

void
ModelGeneratorInjectionEMT::evalF(double t, const double* /*y*/, const double* /*yp*/, double* /*f*/) {
  if (!connected_) return;                                          // disconnected: no injection
  for (int k = 0; k < 3; ++k) node_->addInjection(k, inj(t, k));    // current into the node
}

void
ModelGeneratorInjectionEMT::getZ0(double* z) { z[zOff_] = connected_ ? CONN_CLOSED : CONN_OPEN; }

StateChangeEMT
ModelGeneratorInjectionEMT::evalZ(double /*t*/, double* z) {
  bool wanted = zIsClosed(z[zOff_]);
  if (wanted != connected_) { connected_ = wanted; return NC_STATE_CHANGE; }
  return NC_NO_CHANGE;
}

}  // namespace DYN
