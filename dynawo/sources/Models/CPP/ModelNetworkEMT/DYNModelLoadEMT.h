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
 * @file  DYNModelLoadEMT.h
 * @brief default simplified load: a constant-impedance shunt (abc analogue of ModelLoad).
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELLOADEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELLOADEMT_H_

#include "DYNNetworkComponentEMT.h"

namespace DYN {

/**
 * @brief the DEFAULT simplified load (abc analogue of ModelLoad): a constant
 *        per-phase shunt conductance G = 1/R to ground, registered into its node.
 *        It owns no state; the node carries the -G*v draw in its KCL. A
 *        voltage/frequency-dependent or dynamic load is a Modelica model instead.
 */
class ModelLoadEMT : public NetworkComponentEMT {
 public:
  ModelLoadEMT(NodeEMT* node, double rPu, bool connected = true);
  int nbZ() const override { return 1; }
  /// register the load conductance into its node (call after offsets are set)
  void wire() override;
  void getZ0(double* z) override;
  StateChangeEMT evalZ(double t, double* z) override;
  // Variable/Element: the connection-state discrete only (no continuous state)
  void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) override { instantiateStateZ(variables); }
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                      const std::string& parentName, const std::string& parentType) override {
    defineStateElement(elements, mapElement, parentName, parentType);
  }
  // internal state: the connection flag (mirrors z, but z alone does not re-sync the C++ member)
  unsigned getNbInternalVariables() const override { return 1; }
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override { os << connected_; }
  void loadInternalVariables(boost::archive::binary_iarchive& is) override { is >> connected_; }
  // the connection state appears only in the node's continuous KCL (shunt G), never in a discrete equation
  void collectSilentZ(BitMask* silentZTable) override { silentZTable[0].setFlags(NotUsedInDiscreteEquations); }

 private:
  NodeEMT* node_;   ///< node the load is connected to
  double gPu_;      ///< per-phase conductance in pu (1 / RPu)
  bool connected_;  ///< connection state (mirrors z); disconnected -> no shunt draw
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELLOADEMT_H_
