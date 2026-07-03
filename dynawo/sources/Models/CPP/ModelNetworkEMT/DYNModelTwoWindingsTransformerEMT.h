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
 * @file  DYNModelTwoWindingsTransformerEMT.h
 * @brief three-phase YgYg two-winding transformer (abc analogue of ModelTwoWindingsTransformer).
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELTWOWINDINGSTRANSFORMEREMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELTWOWINDINGSTRANSFORMEREMT_H_

#include <memory>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include "DYNNetworkComponentEMT.h"
#include "DYNModelRatioTapChanger.h"   // reused phasor regulation logic
#include "DYNModelPhaseTapChanger.h"   // reused phasor regulation logic

namespace DYN {

class SubModel;

/**
 * @brief a grounded-wye / grounded-wye transformer: a turns-ratio-coupled R-L
 *        branch with the primary leakage referred to the primary winding. Owns 3
 *        primary-current states; per phase, with ratio rTfo = N1/N2:
 *          vPrim - rTfo * vSec = R * i + L * i'        (branch v-i)
 *          iSec  = - rTfo * i                          (ampere-turn balance)
 *        It injects -i into the primary node and +rTfo*i into the secondary node.
 */
class ModelTwoWindingsTransformerEMT : public NetworkComponentEMT {
 public:
  ModelTwoWindingsTransformerEMT(NodeEMT* primary, NodeEMT* secondary,
                                 double rTfo, double r, double l, const double i0[3], bool connected = true);

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

  /// register the primary current into the two nodes' KCL (call after offsets are set)
  void wire() override;
  /// link the two end nodes for the island BFS (only when connected)
  void addBusNeighbors() override;

  // ---- tap changers (reused phasor automata, EMT physics) ----
  /// attach a ratio tap changer regulating the monitored (secondary) node's RMS voltage
  void setRatioTapChanger(std::unique_ptr<ModelRatioTapChanger> tc, NodeEMT* monitored, double vNomMonitored);
  /// attach a phase tap changer regulating the branch current (RMS); rho/alpha per step
  void setPhaseTapChanger(std::unique_ptr<ModelPhaseTapChanger> tc);
  bool hasTapChanger() const { return ratioTapChanger_ != nullptr || phaseTapChanger_ != nullptr; }
  /// the two end nominal voltages (kV), used to pick the VHV/HV tap-changer timing set
  void setVNoms(double vNom1, double vNom2) { vNom1_ = vNom1; vNom2_ = vNom2; }
  /// apply the PAR-driven tap-changer action delays (mirrors the phasor): tFirst/tNext are taken from
  /// the THT pair when this is a VHV<->HV transformer, else the HT pair; the ratio changer also gets
  /// the (relative) voltage tolerance scaled by its monitored nominal voltage when tolV is provided.
  void applyTapTiming(double t1stTHT, double tNextTHT, double t1stHT, double tNextHT, bool hasTolV, double tolVRel);
  int nbG() const override {
    return (ratioTapChanger_ ? ModelRatioTapChanger::sizeG() : 0) + (phaseTapChanger_ ? phaseTapChanger_->sizeG() : 0);
  }
  /// fill the tap changers' root functions (ratio: monitored RMS voltage; phase: branch RMS current)
  void evalGTap(double t, const double* y, state_g* g);
  /// step the tap(s) if a root fired; update ratio / phase shift. Returns STATE_CHANGE if any moved.
  StateChangeEMT evalZTap(double t, const double* y, const state_g* g, SubModel* network);
  // calculated variable: RMS primary current Irms = sqrt((ia^2 + ib^2 + ic^2) / 3)
  int nbCalcVars() const override { return 1; }
  double evalCalcVar(int i, const double* y) const override;
  void calcVarIndexes(int i, std::vector<int>& idx) const override;
  void evalJCalcVar(int i, const double* y, std::vector<double>& res) const override;

  // Variable/Element: 3 abc primary-current states + connection-state discrete + the Irms calculated var
  void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) override {
    instantiateCurrentStates(variables); instantiateStateZ(variables); instantiateIrms(variables);
  }
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                      const std::string& parentName, const std::string& parentType) override {
    defineCurrentElements(elements, mapElement, parentName, parentType);
    defineStateElement(elements, mapElement, parentName, parentType);
  }
  // internal state: the connection flag plus the tap changers' positions (reused phasor automata)
  unsigned getNbInternalVariables() const override {
    return 1 + (ratioTapChanger_ ? ratioTapChanger_->getNbInternalVariables() : 0)
             + (phaseTapChanger_ ? phaseTapChanger_->getNbInternalVariables() : 0);
  }
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override {
    os << connected_;
    if (ratioTapChanger_) ratioTapChanger_->dumpInternalVariables(os);
    if (phaseTapChanger_) phaseTapChanger_->dumpInternalVariables(os);
  }
  void loadInternalVariables(boost::archive::binary_iarchive& is) override {
    is >> connected_;
    if (ratioTapChanger_) ratioTapChanger_->loadInternalVariables(is);
    if (phaseTapChanger_) phaseTapChanger_->loadInternalVariables(is);
  }
  // the connection state appears only in the continuous branch v-i residual, never in a discrete equation
  void collectSilentZ(BitMask* silentZTable) override { silentZTable[0].setFlags(NotUsedInDiscreteEquations); }

  // ---- steady-state (snubber-consistent) init seed correction (opt-in) ----
  // Reports the current turns ratio so the seed matches the running model; the phasor
  // stamp is i = (vPrim - rTfo*vSec)/z into the primary, -rTfo*i into the secondary.
  bool initSeedBranch(NodeEMT*& from, NodeEMT*& to, double& r, double& l, double& ratio) const override {
    if (!connected_) return false;
    from = primary_; to = secondary_; r = r_; l = l_; ratio = rTfo_; return true;
  }
  void setInitSeedCurrent(const double i0[3]) override { for (int k = 0; k < 3; ++k) i0_[k] = i0[k]; }

 private:
  NodeEMT* primary_;    ///< primary (HV) node
  NodeEMT* secondary_;  ///< secondary (LV) node
  double rTfo_;         ///< turns ratio N1/N2
  double r_;            ///< leakage resistance referred to primary in pu
  double l_;            ///< leakage inductance referred to primary in pu
  double i0_[3];        ///< initial abc primary currents
  bool connected_;      ///< connection state (mirrors z); disconnected -> open (i = 0)
  double alpha_;        ///< phase shift in rad (0 for a pure ratio transformer)

  /// recompute the secondary-KCL current coefficients from the current rTfo / alpha
  void updateSecondaryKcl();

  std::unique_ptr<ModelRatioTapChanger> ratioTapChanger_;  ///< reused phasor ratio automaton (null if none)
  std::unique_ptr<ModelPhaseTapChanger> phaseTapChanger_;  ///< reused phasor phase automaton (null if none)
  NodeEMT* monitoredNode_;   ///< node whose RMS voltage the ratio tap changer regulates
  double vNomMonitored_;     ///< nominal voltage of the monitored node (kV), to scale RMS -> kV
  double vNom1_ = 0.;        ///< primary nominal voltage (kV), for the VHV/HV tap-timing selection
  double vNom2_ = 0.;        ///< secondary nominal voltage (kV), for the VHV/HV tap-timing selection
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELTWOWINDINGSTRANSFORMEREMT_H_
