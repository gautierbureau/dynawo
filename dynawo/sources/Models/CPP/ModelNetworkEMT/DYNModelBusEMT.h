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
 * @file  DYNModelBusEMT.h
 * @brief three-phase capacitive bus node (abc analogue of ModelBus), plus the
 *        SubNetworkEMT / ModelBusContainerEMT helpers that compute the AC-connected
 *        islands (the abc analogue of DYNModelBus.h's SubNetwork/ModelBusContainer).
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELBUSEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELBUSEMT_H_

#include <vector>
#include <utility>
#include <complex>
#include <cmath>
#include "DYNNetworkComponentEMT.h"

namespace DYN {

/**
 * @brief a three-phase bus node owning 3 abc voltage states with a small shunt C
 *        (and optional shunt G); branches inject their currents into it (KCL).
 *        It can be switched off (forced to 0) when it falls in a dead island.
 */
class ModelBusEMT : public NetworkComponentEMT, public NodeEMT {
 public:
  ModelBusEMT(double cPu, double rShunt, const double u0[3]);

  // NetworkComponentEMT
  int nbStates() const override { return 3; }
  void evalF(double t, const double* y, const double* yp, double* f) override;
  void evalJt(double cj, int rowOffset, SparseMatrix& jt) override;
  void evalJtPrim(int rowOffset, SparseMatrix& jt) override;
  void getY0(double t, double* y, double* yp) override;
  void evalStaticYType(propertyContinuousVar_t* yType) override;
  void evalStaticFType(propertyF_t* fType) override;
  void evalDynamicYType(const double* z, propertyContinuousVar_t* yType) override;
  void evalDynamicFType(const double* z, propertyF_t* fType) override;
  // calculated variable: RMS phase voltage Urms = sqrt((va^2 + vb^2 + vc^2) / 3)
  int nbCalcVars() const override { return 1; }
  double evalCalcVar(int i, const double* y) const override;
  void calcVarIndexes(int i, std::vector<int>& idx) const override;
  void evalJCalcVar(int i, const double* y, std::vector<double>& res) const override;
  // Variable/Element: 3 abc voltage states (<id>_v_{a,b,c}) + the Urms calculated var; for a
  // connection bus also the abc ACPIN element {v_k_ (potential), i_k_ (flow)}. The ACPIN FLOW
  // variables themselves live at the end of y (declared by the network) -- elements bind by name.
  void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) override;
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                      const std::string& parentName, const std::string& parentType) override;
  // internal state a (dis)connecting shunt / load / fault has accumulated into this node: the shunt
  // capacitance, the shunt conductance and the fault conductance (none of which live in y/z/g)
  unsigned getNbInternalVariables() const override { return 7; }
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override {
    os << cPu_;
    for (int k = 0; k < 3; ++k) os << shuntG_[k];
    for (int k = 0; k < 3; ++k) os << faultG_[k];
  }
  void loadInternalVariables(boost::archive::binary_iarchive& is) override {
    is >> cPu_;
    for (int k = 0; k < 3; ++k) is >> shuntG_[k];
    for (int k = 0; k < 3; ++k) is >> faultG_[k];
  }

  // NodeEMT
  int vIndex(int phase) const override { return off_ + phase; }
  double v(int phase, double t, const double* y) const override { return y[off_ + phase]; }
  void resetInjection() override;
  void addInjection(int phase, double i) override { inj_[phase] += i; }
  void registerBranchCurrent(int phase, int iIdx, double sign) override;
  /// update the KCL coefficient of an already-registered branch current (e.g. when a
  /// transformer ratio / phase shift changes); matches on (phase, global current index)
  void updateBranchCurrentCoeff(int phase, int iIdx, double coeff) override;
  void registerShuntG(int phase, double g) override { shuntG_[phase] += g; }
  /// per-phase fault conductance to ground (a bus fault toggles this during [tBegin, tEnd]);
  /// it enters the KCL exactly like a shunt G but is switched on/off by the fault automaton
  void setFaultG(int phase, double g) { faultG_[phase] = g; }
  /// add shunt capacitance to the node (e.g. a shunt compensator or a capacitive SVC)
  void addShuntC(double c) { cPu_ += c; hasLocalRef_ = (cPu_ > 0.); }
  /// remove shunt capacitance previously added (a capacitive shunt compensator disconnecting),
  /// flooring at the bus snubber C so the node stays a (stiff) differential, non-singular reference
  /// rather than collapsing to an algebraic node -- mirrors the snubber baseline pass 2 gives a C-less bus
  void releaseShuntC(double c) { cPu_ = (cPu_ - c > 1e-4 ? cPu_ - c : 1e-4); hasLocalRef_ = true; }
  /// set the initial abc node voltage (e.g. the load-flow operating point of a machine bus)
  void setInitialVoltage(const double u0[3]) { for (int k = 0; k < 3; ++k) u0_[k] = u0[k]; }

  // ---- steady-state (snubber-consistent) init seed correction (opt-in) ----
  /// record this bus's load-flow abc voltage (kept even when the seed u0_ starts at 0 for
  /// meshed-init robustness), so the snubber correction can rebuild the operating point
  void setLoadFlowVoltage(const double u0[3]) { for (int k = 0; k < 3; ++k) u0LF_[k] = u0[k]; }
  /// the load-flow node voltage as a (peak) phasor: V = v_a + j*(v_b - v_c)/sqrt(3) for a balanced abc triple
  std::complex<double> phasorLF() const { return std::complex<double>(u0LF_[0], (u0LF_[1] - u0LF_[2]) / std::sqrt(3.0)); }
  double cPu() const { return cPu_; }               ///< TOTAL snubber capacitance per phase in pu
  double shuntGPhase() const { return shuntG_[0]; }  ///< TOTAL shunt conductance per phase (balanced)
  /// record an EMT-ADDED shunt (one NOT present in the load flow: the snubber-C regularisation, the
  /// machine-bus damping G). Only these drive the seed correction's RHS -- physical shunts (loads,
  /// IIDM G/B, reactors) are in the load flow already, so they belong on the matrix diagonal only.
  void addEmtShunt(double g, double c) { gEmt_ += g; cEmt_ += c; }
  /// the EMT-added shunt as a phasor admittance g + j*omega*c (the part the load flow did not supply)
  std::complex<double> emtShuntY(double omega) const { return std::complex<double>(gEmt_, omega * cEmt_); }

  // ---- external connection (black-box machine, or any model on a hasConnection bus) ----
  /// mark this bus as a connection point: it then carries 3 abc FLOW current variables whose
  /// connector-summed value is injected into the KCL. flowScale bridges the connected model's
  /// current convention to this network's standard abc convention -- 1 for a standard model
  /// (load, fault), 3 for the power-invariant-Park machine (Cdq = sqrt(2)/3).
  void enableExternalConnection(double flowScale = 1.0) { hasExtConn_ = true; hasLocalRef_ = true; flowScale_ = flowScale; }
  bool hasExternalConnection() const { return hasExtConn_; }
  /// a connection bus owns 3 extra y entries (the abc ACPIN FLOW currents) right after its 3
  /// voltage states, so its y-block is 6 wide (3 states + 3 flows); the flows have no residual
  /// of their own (the connector provides it), so nbStates stays 3 -- hence nbY != nbStates
  int nbY() const override { return hasExtConn_ ? 6 : 3; }

  ModelBusEMT* asBus() override { return this; }
  void linkNeighbor(NodeEMT* other) override;

  // ---- connectivity / islanding ----
  /// reset neighbor list and the per-pass reference flag (keeps the permanent C reference)
  void resetConnectivity();
  void addNeighbor(ModelBusEMT* bus) { neighbors_.push_back(bus); }
  /// recursively assign every reachable not-yet-visited bus to this island
  void exploreNeighbors(int numSubNetwork, std::vector<ModelBusEMT*>& island);
  bool numSubNetworkSet() const { return subNetwork_ >= 0; }
  void setNumSubNetwork(int n) { subNetwork_ = n; }
  void clearNumSubNetwork() { subNetwork_ = -1; }
  /// this bus has a local voltage reference (its own shunt C, or an adjacent source)
  bool hasLocalReference() const { return hasLocalRef_; }
  /// force the bus voltage to 0 (dead island) / release it
  void switchOff() { switchedOff_ = true; }
  void switchOn() { switchedOff_ = false; }
  bool isSwitchedOff() const { return switchedOff_; }

 private:
  double cPu_;        ///< shunt capacitance per phase in pu (0 -> the node is algebraic)
  double u0_[3];      ///< initial abc node voltages
  double u0LF_[3] = {0., 0., 0.};  ///< load-flow abc node voltage (for the snubber-consistent seed correction)
  double gEmt_ = 0.;   ///< EMT-added shunt conductance per phase (damping G) -- not in the load flow
  double cEmt_ = 0.;   ///< EMT-added shunt capacitance per phase (snubber-C regularisation) -- not in the load flow
  double inj_[3];     ///< accumulated injected current per phase (KCL)
  double shuntG_[3];  ///< accumulated shunt conductance per phase (own shunt R + registered loads)
  double faultG_[3] = {0., 0., 0.};  ///< per-phase fault-to-ground conductance (0 unless a fault is active)
  std::vector<std::pair<int, double> > branchCur_[3];  ///< (global current index, KCL sign) per phase

  std::vector<ModelBusEMT*> neighbors_;  ///< AC-connected buses (via closed branches/switches)
  bool hasLocalRef_;                     ///< has its own voltage reference (shunt C or adjacent source)
  bool switchedOff_;                     ///< voltage forced to 0 (dead island)
  int subNetwork_;                       ///< island index (-1 if not yet assigned)
  bool hasExtConn_ = false;              ///< driven by an external black-box (machine / fault / load)
  double flowScale_ = 1.0;               ///< FLOW current convention bridge (1 standard, 3 machine Park)
};

/**
 * @brief an AC-connected island: the buses reachable from one another through
 *        closed branches/switches (abc analogue of SubNetwork).
 */
class SubNetworkEMT {
 public:
  void addBus(ModelBusEMT* bus) { buses_.push_back(bus); }
  unsigned nbBus() const { return static_cast<unsigned>(buses_.size()); }
  /// true if any bus owns a voltage reference (source or shunt C) -> well-posed
  bool hasReference() const;
  void shutDown();  ///< switch off every bus (dead island)
  void turnOn();    ///< release every bus

 private:
  std::vector<ModelBusEMT*> buses_;  ///< buses within the island
};

/**
 * @brief owns all bus nodes and computes the AC islands each time the topology
 *        changes (abc analogue of ModelBusContainer).
 */
class ModelBusContainerEMT {
 public:
  void add(ModelBusEMT* bus) { buses_.push_back(bus); }
  void resetConnectivity();                  ///< clear neighbors + island ids on every bus
  void exploreNeighbors();                   ///< BFS: build the list of islands
  void analyseIslands();                     ///< keep islands with a reference, shut the rest off
  const std::vector<SubNetworkEMT>& subNetworks() const { return subNetworks_; }

 private:
  std::vector<ModelBusEMT*> buses_;         ///< every bus node (not the sources)
  std::vector<SubNetworkEMT> subNetworks_;   ///< AC islands, rebuilt on each topology change
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELBUSEMT_H_
