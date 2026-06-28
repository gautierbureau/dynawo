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
 * @file  DYNModelSwitchEMT.h
 * @brief three-phase ideal switch (abc analogue of ModelSwitch): closed imposes
 *        v1 = v2 per phase with the switch current as the unknown; open forces
 *        the current to zero. The open/closed state is a discrete (z) variable so
 *        a topology change recomputes the network connectivity.
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELSWITCHEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELSWITCHEMT_H_

#include "DYNNetworkComponentEMT.h"

namespace DYN {

/**
 * @brief an ideal three-phase switch between two nodes, owning its 3 abc currents.
 *        closed: vFrom - vTo = 0 (current free); open: i = 0. The current is
 *        injected into both end nodes' KCL exactly like a branch.
 */
class ModelSwitchEMT : public NetworkComponentEMT {
 public:
  ModelSwitchEMT(NodeEMT* bus1, NodeEMT* bus2, bool closed);

  // states: 3 abc currents; discrete: 1 connection state
  int nbStates() const override { return 3; }
  int nbZ() const override { return 1; }

  void evalF(double t, const double* y, const double* yp, double* f) override;
  void evalJt(double cj, int rowOffset, SparseMatrix& jt) override;
  void evalJtPrim(int rowOffset, SparseMatrix& jt) override;
  void getY0(double t, double* y, double* yp) override;
  void getZ0(double* z) override;
  StateChangeEMT evalZ(double t, double* z) override;
  void evalStaticYType(propertyContinuousVar_t* yType) override;
  void evalStaticFType(propertyF_t* fType) override;

  void wire() override;             ///< register the switch current into its end nodes' KCL
  void addBusNeighbors() override;  ///< link the two nodes only when the switch is closed

  // Variable/Element: 3 abc current states (<id>_i_{a,b,c}) + the connection-state discrete (<id>_state)
  void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) override {
    instantiateCurrentStates(variables); instantiateStateZ(variables);
  }
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                      const std::string& parentName, const std::string& parentType) override {
    defineCurrentElements(elements, mapElement, parentName, parentType);
    defineStateElement(elements, mapElement, parentName, parentType);
  }
  // internal state: the connection flag (mirrors z, but z alone does not re-sync the C++ member)
  unsigned getNbInternalVariables() const override { return 1; }
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override { os << closed_; }
  void loadInternalVariables(boost::archive::binary_iarchive& is) override { is >> closed_; }

 private:
  bool effectiveClosed() const;     ///< closed AND neither end switched off

  NodeEMT* bus1_;   ///< 'from' node
  NodeEMT* bus2_;   ///< 'to' node
  bool closed_;     ///< current connection state (mirrors z)
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELSWITCHEMT_H_
