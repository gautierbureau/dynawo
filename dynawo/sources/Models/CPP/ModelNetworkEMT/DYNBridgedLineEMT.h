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
 * @file  DYNBridgedLineEMT.h
 * @brief connectivity bridge for a line replaced by an external dynamic model.
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNBRIDGEDLINEEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNBRIDGEDLINEEMT_H_

#include "DYNNetworkComponentEMT.h"

namespace DYN {

/**
 * @brief the network-side bridge of a line whose branch is built by an external
 *        dynamic model (e.g. a LineFault). It contributes NOTHING electrically --
 *        the external model carries the current through the two end-bus ACPINs --
 *        but it owns the line's connection state and links its two end buses in the
 *        connectivity graph while closed. When the external model trips the line,
 *        an event drives its "<id>_state_value" discrete (as for any switch/branch);
 *        evalZ then reports a topology change so the network recomputes its islands.
 *        This is the abc/EMT analogue of the phasor NetworkBridgeQuadripole.
 */
class BridgedLineEMT : public NetworkComponentEMT {
 public:
  BridgedLineEMT(NodeEMT* bus1, NodeEMT* bus2) : bus1_(bus1), bus2_(bus2) { }

  int nbZ() const override { return 1; }  ///< one connection-state discrete, no continuous state

  void getZ0(double* z) override { z[zOff_] = CONN_CLOSED; }  ///< starts closed

  StateChangeEMT evalZ(double /*t*/, double* z) override {
    const bool wanted = zIsClosed(z[zOff_]);
    if (wanted != closed_) { closed_ = wanted; return NC_TOPO_CHANGE; }
    return NC_NO_CHANGE;
  }

  void addBusNeighbors() override {
    if (!closed_) return;  // an open (tripped) line does not connect its two ends
    bus1_->linkNeighbor(bus2_);
    bus2_->linkNeighbor(bus1_);
  }

  // Variable/Element: the connection-state discrete only (the line itself is the external model's)
  void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) override { instantiateStateZ(variables); }
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                      const std::string& parentName, const std::string& parentType) override {
    defineStateElement(elements, mapElement, parentName, parentType);
  }
  // internal state: the connection flag (mirrors z, but z alone does not re-sync the C++ member)
  unsigned getNbInternalVariables() const override { return 1; }
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override { os << closed_; }
  void loadInternalVariables(boost::archive::binary_iarchive& is) override { is >> closed_; }
  // the bridged line contributes nothing electrically (the external model carries the current); its
  // connection state is never read by a discrete equation, so it is silent
  void collectSilentZ(BitMask* silentZTable) override { silentZTable[0].setFlags(NotUsedInDiscreteEquations); }

 private:
  NodeEMT* bus1_;        ///< end bus 1
  NodeEMT* bus2_;        ///< end bus 2
  bool closed_ = true;   ///< current connection state (mirrors z)
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNBRIDGEDLINEEMT_H_
