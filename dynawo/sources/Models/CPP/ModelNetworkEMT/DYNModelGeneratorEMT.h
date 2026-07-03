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
 * @file  DYNModelGeneratorEMT.h
 * @brief default simplified generator: a constant balanced current injection.
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELGENERATOREMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELGENERATOREMT_H_

#include "DYNNetworkComponentEMT.h"

namespace DYN {

/**
 * @brief the DEFAULT simplified generator (abc analogue of ModelGenerator's PQ
 *        injection): a constant balanced current injected into its node. A
 *        detailed machine is NOT this -- it stays in Modelica and attaches to the
 *        node through the abc terminal.
 */
class ModelGeneratorInjectionEMT : public NetworkComponentEMT {
 public:
  ModelGeneratorInjectionEMT(NodeEMT* node, double iPeak, double phase0, double fNom, bool connected = true);
  int nbZ() const override { return 1; }
  void evalF(double t, const double* y, const double* yp, double* f) override;
  void getZ0(double* z) override;
  StateChangeEMT evalZ(double t, double* z) override;
  // Variable/Element: the connection-state discrete only (PQ injector, no continuous state)
  void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) override { instantiateStateZ(variables); }
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                      const std::string& parentName, const std::string& parentType) override {
    defineStateElement(elements, mapElement, parentName, parentType);
  }
  // internal state: the connection flag (mirrors z, but z alone does not re-sync the C++ member)
  unsigned getNbInternalVariables() const override { return 1; }
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override { os << connected_; }
  void loadInternalVariables(boost::archive::binary_iarchive& is) override { is >> connected_; }
  // the connection state appears only in the node's continuous KCL (injection), never in a discrete equation
  void collectSilentZ(BitMask* silentZTable) override { silentZTable[0].setFlags(NotUsedInDiscreteEquations); }

 private:
  double inj(double t, int phase) const;
  NodeEMT* node_;  ///< node the generator injects into
  double iPeak_;   ///< injected peak current in pu
  double phase0_;  ///< phase-a angle at t = 0 in rad
  double fNom_;    ///< nominal frequency in Hz
  bool connected_; ///< connection state (mirrors z); disconnected -> no injection
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELGENERATOREMT_H_
