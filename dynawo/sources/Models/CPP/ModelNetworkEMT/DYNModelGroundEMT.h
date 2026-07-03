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
 * @file  DYNModelGroundEMT.h
 * @brief zero-volt ground reference node used internally as the return point of
 *        shunt elements (e.g. a load's shunt reactor, an inductive SVC branch).
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELGROUNDEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELGROUNDEMT_H_

#include "DYNNetworkComponentEMT.h"

namespace DYN {

/**
 * @brief the network's internal 0 V ground: a node whose three phase voltages are
 *        fixed at zero. It is NOT a source and carries no state; it only gives shunt
 *        branches (shunt reactors, inductive SVCs) a return reference. It is purely
 *        internal -- the network never imposes a non-zero voltage. Any non-zero
 *        voltage reference (a slack / infinite bus) is an external Modelica model
 *        attached to a bus through the ACPIN connector, exactly as in the phasor world.
 */
class ModelGroundEMT : public NetworkComponentEMT, public NodeEMT {
 public:
  double v(int /*phase*/, double /*t*/, const double* /*y*/) const override { return 0.; }
  bool imposesVoltage() const override { return true; }  ///< a fixed 0 V reference node
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELGROUNDEMT_H_
