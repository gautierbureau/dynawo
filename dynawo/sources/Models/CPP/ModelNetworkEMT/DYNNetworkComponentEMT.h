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
 * @file  DYNNetworkComponentEMT.h
 * @brief Base classes of the abc EMT network component family (mirrors
 *        DYNNetworkComponent): the NodeEMT node abstraction and the
 *        NetworkComponentEMT base every EMT network sub-model derives from.
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNNETWORKCOMPONENTEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNNETWORKCOMPONENTEMT_H_

#include <vector>
#include <map>
#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include "DYNEnumUtils.h"
#include "DYNModelConstants.h"
#include "DYNSparseMatrix.h"
#include "DYNBitMask.h"

namespace DYN {

class ModelBusEMT;
class Variable;
class Element;

/**
 * @brief a three-phase node (the abc analogue of ModelBus): branches read its
 *        phase voltage and add their phase currents into it.
 */
class NodeEMT {
 public:
  virtual ~NodeEMT();
  /// instantaneous phase voltage seen at the node (state for a bus, source for a slack)
  virtual double v(int phase, double t, const double* y) const = 0;
  /// global y index of the phase voltage (-1 if the node imposes its voltage, e.g. a slack)
  virtual int vIndex(int phase) const { return -1; }
  /// current injection accumulation (KCL), used by buses with a state voltage
  virtual void resetInjection() { }
  virtual void addInjection(int phase, double i) { }
  /// branches register their current (global index + KCL sign) into the node so it
  /// can stamp d(KCL)/d(i_branch) -- the abc analogue of ModelBus::irAdd/iiAdd wiring
  virtual void registerBranchCurrent(int phase, int iIdx, double sign) { }
  /// update an already-registered branch-current coefficient (ratio / phase-shift change)
  virtual void updateBranchCurrentCoeff(int phase, int iIdx, double coeff) { }
  /// constant-impedance shunts (e.g. a load) register their conductance into the node
  virtual void registerShuntG(int phase, double g) { }

  // ---- connectivity (island computation) ----
  /// true if this node imposes its own voltage (a slack / infinite bus = a reference)
  virtual bool imposesVoltage() const { return false; }
  /// downcast helper: the bus self, or nullptr for a source node
  virtual ModelBusEMT* asBus() { return nullptr; }
  /// a branch links its two end nodes: a bus records the other as an AC neighbor and,
  /// if the other imposes voltage, marks itself as having a local voltage reference
  virtual void linkNeighbor(NodeEMT* other) { }
};

/**
 * @brief base class for every EMT network component (mirrors NetworkComponent).
 *
 * The meta-model owns the global y/yp/f arrays and assigns each component the
 * global offset of its first state; components contribute their DAE residuals and
 * Jacobian into those arrays.
 */
/**
 * @brief result of a discrete (evalZ/evalState) re-evaluation, mirroring
 *        NetworkComponent::StateChange_t: NO_CHANGE, a plain state change, or a
 *        topology change (which forces the network to recompute connectivity).
 */
enum StateChangeEMT {
  NC_NO_CHANGE = 0,    ///< nothing changed
  NC_STATE_CHANGE,     ///< a discrete state changed (Jacobian values, no topology)
  NC_TOPO_CHANGE       ///< connectivity changed (recompute islands + Jacobian structure)
};

/**
 * @brief connection-state discrete convention for every switchable EMT component, matching
 *        Dynawo's Modelica Constants.state (Open = 1, Closed = 2). A network event such as
 *        EventQuadripoleDisconnection drives a component's <id>_state_value to these exact
 *        integer codes, so the components MUST read/write them (a plain 0/1 convention silently
 *        ignores an event writing Open = 1, since 1 > 0.5 would read as "connected").
 */
enum ConnectionStateEMT {
  CONN_OPEN = 1,    ///< Constants.state.Open: component disconnected
  CONN_CLOSED = 2   ///< Constants.state.Closed: component connected
};
/// a connection-state discrete denotes "connected" iff it is Closed (2) or higher (a partial
/// side-closed code); Open (1) and 0 are disconnected. Threshold 1.5 separates the two.
inline bool zIsClosed(double z) { return z > 1.5; }

class NetworkComponentEMT {
 public:
  virtual ~NetworkComponentEMT();
  /// number of states (y entries) this component owns
  virtual int nbStates() const { return 0; }
  /// number of discrete variables (z entries) this component owns
  virtual int nbZ() const { return 0; }
  /// number of root functions (g entries) this component owns
  virtual int nbG() const { return 0; }
  /// the meta-model assigns the global y-offset of this component's first state. Components with a
  /// FLOW connector (a bus's ACPIN) own MORE y entries (nbY) than residual equations (nbStates),
  /// so the y-offset (off_) and the residual/f-offset (fOff_) diverge once such a component precedes
  /// in the global order -- mirroring the phasor's distinct offsetY / offsetF.
  virtual void setOffset(int off) { off_ = off; }
  /// the meta-model assigns the global residual (f) offset of this component's first equation
  virtual void setFOffset(int fOff) { fOff_ = fOff; }
  /// the meta-model assigns the global offset of this component's first z / g
  virtual void setZOffset(int zOff) { zOff_ = zOff; }
  virtual void setGOffset(int gOff) { gOff_ = gOff; }
  /// residual contributions into the global f (branches also inject into their nodes)
  virtual void evalF(double t, const double* y, const double* yp, double* f) { }
  /// Jacobian contributions dF/dy + cj dF/dy' (one changeCol per owned equation)
  virtual void evalJt(double cj, int rowOffset, SparseMatrix& jt) { }
  /// dF/dy' contributions
  virtual void evalJtPrim(int rowOffset, SparseMatrix& jt) { }
  /// consistent start (energisation or load-flow seed) into the global y/yp
  virtual void getY0(double t, double* y, double* yp) { }
  /// initial discrete state into the global z (offsets already assigned)
  virtual void getZ0(double* z) { }
  /// re-evaluate discrete variables from z; report whether topology/state changed
  virtual StateChangeEMT evalZ(double t, double* z) { return NC_NO_CHANGE; }
  /// flag this component's z entries that are "silent" -- present in the continuous residual (F) but
  /// not read by any discrete (g / z) equation, so the solver can skip re-evaluating the discrete
  /// system when only they change. silentZTable is the component-local view (index 0 = first z).
  /// Default: flag nothing (always correct, just no optimization). Mirrors NetworkComponent::collectSilentZ.
  virtual void collectSilentZ(BitMask* silentZTable) { }
  /// root functions into the global g (zero-crossing detection for events)
  virtual void evalG(double t, const double* y, const double* yp, const double* z, state_g* g) { }
  /// y / f property (DIFFERENTIAL vs ALGEBRAIC) for the component's states/equations
  virtual void evalStaticYType(propertyContinuousVar_t* yType) { }
  virtual void evalStaticFType(propertyF_t* fType) { }
  /// dynamic y/f property, re-evaluated after a topology change (e.g. a switched-off
  /// bus becomes algebraic); defaults to the static classification
  virtual void evalDynamicYType(const double* z, propertyContinuousVar_t* yType) { evalStaticYType(yType); }
  virtual void evalDynamicFType(const double* z, propertyF_t* fType) { evalStaticFType(fType); }
  /// register this component's end buses as AC neighbors of each other (closed
  /// branches/switches only) so the bus container can compute connected islands
  virtual void addBusNeighbors() { }
  /// register the component into its node(s) once every offset is assigned
  /// (branches push their current, shunts push their conductance); no-op by default
  virtual void wire() { }

  // ---- steady-state (snubber-consistent) init seed correction (opt-in) ----
  /// If this component is a series branch, report its two end nodes, its pu series
  /// resistance/inductance (z = r + j*omega*l), and the turns ratio (1 for a line);
  /// the phasor current is i = (v_from - ratio*v_to)/z. Returns false for non-branches.
  virtual bool initSeedBranch(NodeEMT*& from, NodeEMT*& to, double& r, double& l, double& ratio) const { return false; }
  /// overwrite this branch's abc initial current from the solved steady-state seed
  virtual void setInitSeedCurrent(const double i0[3]) { }

  // ---- calculated (output) variables: pure functions of the state vector ----
  /// number of calculated variables this component exposes (e.g. an RMS magnitude)
  virtual int nbCalcVars() const { return 0; }
  /// value of the i-th calculated variable from the current state
  virtual double evalCalcVar(int i, const double* y) const { return 0.; }
  /// global y indexes the i-th calculated variable depends on (for its Jacobian)
  virtual void calcVarIndexes(int i, std::vector<int>& idx) const { }
  /// d(calcVar_i)/dy for each index returned by calcVarIndexes (same order)
  virtual void evalJCalcVar(int i, const double* y, std::vector<double>& res) const { }

  // ---- Dynawo Variable / Element framework (mirrors NetworkComponent) ----
  /// total number of y entries this component owns (continuous states PLUS any FLOW
  /// connector currents); defaults to nbStates() for components with no connector
  virtual int nbY() const { return nbStates(); }
  /// declare this component's named variables (states, FLOW currents, calculated, discrete)
  /// in the SAME order as its y / calc / z blocks, so curves and <connect> resolve by name
  /// (the network calls this on every component, in the offset order = the y/z/calc order)
  virtual void instantiateVariables(std::vector<boost::shared_ptr<Variable> >& variables) { }
  /// declare this component's connectable elements (the abc ACPIN terminals, the state
  /// structure, the calculated-variable leaves) -- the abc analogue of NetworkComponent::defineElements.
  /// parentName / parentType are the owning network model's name() / modelType() (every EMT element
  /// is owned by the single ModelNetworkEMT, unlike the phasor where each component is its own model)
  virtual void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                              const std::string& parentName, const std::string& parentType) { }

  // ---- internal-variable dump / restore (mirrors NetworkComponent) ----
  // The y/yp/z/g arrays are checkpointed by ModelCPP::dumpVariables; these hooks persist the
  // component's INTERNAL C++ state that lives outside those arrays -- the connection flag, a tap
  // position, and (for a bus) the accumulated node quantities a (dis)connecting shunt/load/fault
  // has mutated (cPu_, shuntG_, faultG_). They must read/write in the SAME order.
  /// number of internal (non y/z/g) scalars this component serialises
  virtual unsigned getNbInternalVariables() const { return 0; }
  virtual void dumpInternalVariables(boost::archive::binary_oarchive& os) const { }
  virtual void loadInternalVariables(boost::archive::binary_iarchive& is) { }

 protected:
  /// add a "<name> { value }" structure whose leaf TERMINAL binds to the variable <name>_value
  /// (helper for calculated-variable / state elements, mirroring NetworkComponent::addElementWithValue)
  void addElementWithValue(const std::string& name, const std::string& parentName, const std::string& parentType,
                           std::vector<Element>& elements, std::map<std::string, int>& mapElement) const;

  // ---- shared instantiateVariables / defineElements building blocks (current-bearing components) ----
  void instantiateCurrentStates(std::vector<boost::shared_ptr<Variable> >& variables) const;   ///< <id>_i_{a,b,c} (CONTINUOUS)
  void instantiateStateZ(std::vector<boost::shared_ptr<Variable> >& variables) const;           ///< <id>_state_value (INTEGER)
  void instantiateIrms(std::vector<boost::shared_ptr<Variable> >& variables) const;             ///< <id>_Irms (CALCULATED)
  void defineCurrentElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                             const std::string& parentName, const std::string& parentType) const;  ///< <id>_i_{a,b,c} { value }
  void defineStateElement(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                          const std::string& parentName, const std::string& parentType) const;     ///< <id>_state { value }

 public:
  /// the network assigns each component its id (= its IIDM/static id), used to name its
  /// variables (<id>_v_a, <id>_i_a, <id>_state_value, <id>_Urms ...) and elements
  void setId(const std::string& id) { id_ = id; }
  const std::string& id() const { return id_; }
  /// global offset of this component's first residual (f) / first root function (g); used by the
  /// network to name each equation in setFequations / setGequations (init-solver diagnostics)
  int fOffset() const { return fOff_; }
  int gOffset() const { return gOff_; }

 protected:
  int off_ = 0;   ///< global index of this component's first state (y)
  int fOff_ = 0;  ///< global index of this component's first residual equation (f); diverges from off_ once a FLOW-bearing component precedes
  int zOff_ = 0;  ///< global index of this component's first discrete variable
  int gOff_ = 0;  ///< global index of this component's first root function
  std::string id_;  ///< component id (= IIDM static id); set by the network after construction
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNNETWORKCOMPONENTEMT_H_
