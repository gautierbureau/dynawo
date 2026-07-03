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
 * @file  DYNModelLineEMT.h
 * @brief three-phase series R-L branch (abc analogue of ModelLine).
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELLINEEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELLINEEMT_H_

#include <memory>
#include <string>
#include "DYNNetworkComponentEMT.h"
#include "DYNModelCurrentLimits.h"   // reused phasor overcurrent automaton

namespace DYN {

class SubModel;

/**
 * @brief a three-phase series R-L branch owning 3 current states; it injects its
 *        current into both end nodes (KCL) and stamps the branch v-i residual.
 */
class ModelLineEMT : public NetworkComponentEMT {
 public:
  ModelLineEMT(NodeEMT* from, NodeEMT* to, double r, double l, const double i0[3], bool connected = true);

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
  void evalDynamicYType(const double* z, propertyContinuousVar_t* yType) override;
  void evalDynamicFType(const double* z, propertyF_t* fType) override;

  /// register this branch's currents into its end nodes' KCL (call after offsets are set)
  void wire() override;
  /// link the two end nodes for the island BFS (only when the line is connected)
  void addBusNeighbors() override;
  // calculated variable: RMS line current Irms = sqrt((ia^2 + ib^2 + ic^2) / 3)
  int nbCalcVars() const override { return 1; }
  double evalCalcVar(int i, const double* y) const override;
  void calcVarIndexes(int i, std::vector<int>& idx) const override;
  void evalJCalcVar(int i, const double* y, std::vector<double>& res) const override;
  int iIndex(int phase) const { return off_ + phase; }

  // Variable/Element: 3 abc current states + connection-state discrete + the Irms calculated var
  void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) override {
    instantiateCurrentStates(variables); instantiateStateZ(variables); instantiateIrms(variables);
  }
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                      const std::string& parentName, const std::string& parentType) override {
    defineCurrentElements(elements, mapElement, parentName, parentType);
    defineStateElement(elements, mapElement, parentName, parentType);
  }
  // internal state: the connection flag (mirrors z, but z alone does not re-sync the C++ member)
  unsigned getNbInternalVariables() const override { return 1; }
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override { os << connected_; }
  void loadInternalVariables(boost::archive::binary_iarchive& is) override { is >> connected_; }
  // the connection state appears only in the continuous branch v-i residual, never in a discrete equation
  void collectSilentZ(BitMask* silentZTable) override { silentZTable[0].setFlags(NotUsedInDiscreteEquations); }

  // ---- steady-state (snubber-consistent) init seed correction (opt-in) ----
  bool initSeedBranch(NodeEMT*& from, NodeEMT*& to, double& r, double& l, double& ratio) const override {
    if (!connected_) return false;   // an open line carries no current and no stamp
    from = from_; to = to_; r = r_; l = l_; ratio = 1.0; return true;
  }
  void setInitSeedCurrent(const double i0[3]) override { for (int k = 0; k < 3; ++k) i0_[k] = i0[k]; }

  // ---- current limits (reused phasor overcurrent automaton) ----
  /// attach an overcurrent protection; id labels timeline/constraint events,
  /// factorPuToA converts the pu RMS current to amps (the limit unit)
  void setCurrentLimits(std::unique_ptr<ModelCurrentLimits> cl, const std::string& id, double factorPuToA);
  bool hasCurrentLimits() const { return currentLimits_ != nullptr; }
  int nbG() const override { return currentLimits_ ? currentLimits_->sizeG() : 0; }
  /// fill the overcurrent root functions from the RMS current (in amps)
  void evalGLimit(double t, const double* y, state_g* g);
  /// trip (disconnect) the line if a limit timed out; returns TOPO_CHANGE if it opened.
  /// also drives the connection-state z to OPEN so the trip persists (the connection
  /// evalZ would otherwise re-read z and re-close the line on the next step).
  StateChangeEMT evalZLimit(double t, const state_g* g, double* z, SubModel* network);

 private:
  NodeEMT* from_;     ///< 'from' node
  NodeEMT* to_;       ///< 'to' node
  double r_;          ///< series resistance per phase in pu
  double l_;          ///< series inductance per phase in pu
  double i0_[3];      ///< initial abc branch currents
  bool connected_;    ///< connection state (mirrors z); disconnected -> open (i = 0)

  std::unique_ptr<ModelCurrentLimits> currentLimits_;  ///< reused phasor overcurrent automaton (null if none)
  std::string id_;        ///< line id (for timeline/constraint events on a trip)
  double factorPuToA_;    ///< pu RMS current -> amps (so the limit threshold is in amps)
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELLINEEMT_H_
