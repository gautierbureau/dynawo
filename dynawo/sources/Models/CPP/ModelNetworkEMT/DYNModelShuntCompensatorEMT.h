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
 * @file  DYNModelShuntCompensatorEMT.h
 * @brief switchable capacitive shunt compensator (abc analogue of ModelShuntCompensator).
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELSHUNTCOMPENSATOREMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELSHUNTCOMPENSATOREMT_H_

#include "DYNModelBusEMT.h"

namespace DYN {

/**
 * @brief a switchable capacitive shunt compensator (abc analogue of ModelShuntCompensator).
 *        Its per-phase capacitance C sits in its node's KCL (the node carries the C*v' term),
 *        so the component owns no continuous state -- only a connection-state discrete (z).
 *        (Dis)connecting it adds / removes its C from the node, exactly as a constant-impedance
 *        load (dis)connecting adds / removes its conductance. An INDUCTIVE shunt (B < 0) is not
 *        built as this component: it is a reactor to ground, realised as a switchable ModelLineEMT
 *        (the same branch-to-ground the inductive SVC uses), which is already a first-class branch.
 */
class ModelShuntCompensatorEMT : public NetworkComponentEMT {
 public:
  /// @param node the bus the compensator is connected to
  /// @param cPu  per-phase capacitance in pu (already folded into the node's C when connected at build)
  /// @param connected initial connection state (from the IIDM)
  ModelShuntCompensatorEMT(ModelBusEMT* node, double cPu, bool connected = true) :
    node_(node), cPu_(cPu), connected_(connected) { }

  int nbZ() const override { return 1; }                       ///< one connection-state discrete, no continuous state
  void getZ0(double* z) override { z[zOff_] = connected_ ? CONN_CLOSED : CONN_OPEN; }

  StateChangeEMT evalZ(double t, double* z) override {
    // (dis)connecting just adds / removes the capacitor bank from the node's shunt C; the
    // node stays differential (releaseShuntC floors at the snubber C), so this is a value change.
    const bool wanted = zIsClosed(z[zOff_]);
    if (wanted == connected_) return NC_NO_CHANGE;
    if (!wanted) {                                  // open: drop the bank, start the no-reclosing timer
      node_->releaseShuntC(cPu_);
      connected_ = false;
      tLastOpening_ = t;
      pendingClose_ = false;
      return NC_STATE_CHANGE;
    }
    // close commanded: honour it only if the no-reclosing lockout has expired, else defer it (a
    // capacitor bank must stay open for no_reclosing_delay after tripping, to avoid rapid cycling).
    // The deferred close is applied later by evalZReclose when the reclosing-permitted root fires.
    if (reclosingPermitted(t)) {
      node_->addShuntC(cPu_);
      connected_ = true;
      pendingClose_ = false;
      return NC_STATE_CHANGE;
    }
    pendingClose_ = true;
    return NC_NO_CHANGE;        // still open; reclose held off until the lockout expires
  }

  // ---- no-reclosing delay (abc analogue of ModelShuntCompensator's _no_reclosing_delay) ----
  /// minimum time the bank must stay open after a trip before it may reclose (0 -> no lockout)
  void setReclosingDelay(double d) { noReclosingDelay_ = d; }
  int nbG() const override { return noReclosingDelay_ > 0. ? 1 : 0; }   ///< one "reclosing permitted" root when a lockout exists
  /// root: UP once reclosing is permitted (never opened, or the lockout has elapsed), DOWN during lockout
  void evalGReclose(double t, state_g* g) const {
    g[gOff_] = reclosingPermitted(t) ? ROOT_UP : ROOT_DOWN;
  }
  /// apply a deferred reclose once the lockout has expired (called by the network on a root crossing)
  StateChangeEMT evalZReclose(double t, double* z) {
    if (!pendingClose_ || connected_ || !reclosingPermitted(t)) return NC_NO_CHANGE;
    node_->addShuntC(cPu_);
    connected_ = true;
    pendingClose_ = false;
    z[zOff_] = CONN_CLOSED;     // keep the connection discrete consistent with the applied reclose
    return NC_STATE_CHANGE;
  }

  // Variable/Element: the connection-state discrete only (the C lives in the node, no own state)
  void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) override { instantiateStateZ(variables); }
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                      const std::string& parentName, const std::string& parentType) override {
    defineStateElement(elements, mapElement, parentName, parentType);
  }
  // internal state: the connection flag + the last-opening time and the pending-reclose flag (the
  // C contribution itself lives in the node, dumped by the bus)
  unsigned getNbInternalVariables() const override { return 3; }
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override {
    os << connected_; os << tLastOpening_; os << pendingClose_;
  }
  void loadInternalVariables(boost::archive::binary_iarchive& is) override {
    is >> connected_; is >> tLastOpening_; is >> pendingClose_;
  }
  // the connection state appears only in the node's continuous KCL (shunt C), never in a discrete equation
  void collectSilentZ(BitMask* silentZTable) override { silentZTable[0].setFlags(NotUsedInDiscreteEquations); }

 private:
  /// true if the bank may (re)close now: it never opened, there is no lockout, or the lockout elapsed
  bool reclosingPermitted(double t) const {
    return noReclosingDelay_ <= 0. || tLastOpening_ < -1e29 || t >= tLastOpening_ + noReclosingDelay_;
  }

  ModelBusEMT* node_;  ///< node the compensator is connected to
  double cPu_;         ///< per-phase capacitance in pu
  bool connected_;     ///< connection state (mirrors z)
  double noReclosingDelay_ = 0.;   ///< min open time before a reclose (s); 0 -> no lockout
  double tLastOpening_ = -1e30;    ///< time of the last trip (sentinel < -1e29 = never opened)
  bool pendingClose_ = false;      ///< a reclose was commanded during lockout and is waiting
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELSHUNTCOMPENSATOREMT_H_
