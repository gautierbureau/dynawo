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
 * @file  DYNModelLoadEMT.cpp
 */
#include "DYNModelLoadEMT.h"

namespace DYN {

ModelLoadEMT::ModelLoadEMT(NodeEMT* node, double rPu, bool connected) :
node_(node), gPu_(rPu > 0 ? 1.0 / rPu : 0.), connected_(connected) { }

void
ModelLoadEMT::wire() {
  if (connected_)
    for (int k = 0; k < 3; ++k) node_->registerShuntG(k, gPu_);   // draw only when connected
}

void
ModelLoadEMT::getZ0(double* z) { z[zOff_] = connected_ ? CONN_CLOSED : CONN_OPEN; }

StateChangeEMT
ModelLoadEMT::evalZ(double /*t*/, double* z) {
  // a constant-impedance load is a shunt conductance on its node; (dis)connecting
  // it just adds / removes that conductance from the node's accumulated shunt G
  bool wanted = zIsClosed(z[zOff_]);
  if (wanted == connected_) return NC_NO_CHANGE;
  const double delta = wanted ? gPu_ : -gPu_;
  for (int k = 0; k < 3; ++k) node_->registerShuntG(k, delta);
  connected_ = wanted;
  return NC_STATE_CHANGE;   // node Jacobian/residual values change, structure unchanged
}

}  // namespace DYN
