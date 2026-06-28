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
 * @file  DYNModelNetworkEMT.h
 *
 * @brief Native instantaneous three-phase (abc) EMT network model.
 *
 * A separate, first-class network model alongside the phasor DYNModelNetwork:
 * same ModelCPP + SubModelFactory design and the same loading path (selected in
 * the jobs by <network lib="DYNModelNetworkEMT"/>), but the unknowns are
 * instantaneous abc node voltages and differential branch currents, solved as a
 * DAE by the Dynawo solver (residuals + Jacobian, no embedded integrator). The
 * abc residuals are the ones validated standalone in emt-exploration/cpp.
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELNETWORKEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELNETWORKEMT_H_

#include <vector>
#include <string>
#include "DYNModelCPP.h"
#include "DYNSubModelFactory.h"
#include "DYNNetworkComponentEMT.h"
#include "DYNModelBusEMT.h"

namespace DYN {

class ModelSwitchEMT;
class ModelVoltageLevelEMT;
class ModelTwoWindingsTransformerEMT;
class ModelLineEMT;
class ModelShuntCompensatorEMT;

/**
 * @brief ModelNetworkEMT factory (mirrors ModelNetworkFactory)
 */
class ModelNetworkEMTFactory : public SubModelFactory {
 public:
  ModelNetworkEMTFactory() { }
  ~ModelNetworkEMTFactory() { }
  SubModel* create() const override;
  void destroy(SubModel*) const override;
};

/**
 * @brief Native abc EMT network model (ModelCPP), DAE formulation.
 */
class ModelNetworkEMT : public ModelCPP {
 public:
  ModelNetworkEMT();
  ~ModelNetworkEMT() override;

  // ---- ModelCPP interface (mirrors DYNModelNetwork / DYNModelOmegaRef) ----
  void init(double t0) override;
  void getSize() override;
  void evalF(double t, propertyF_t type) override;
  void evalG(double t) override;
  void evalZ(double t) override;
  void collectSilentZ(BitMask* silentZTable) override;  ///< flag connection-state z's that no discrete equation reads
  modeChangeType_t evalMode(double t) override;
  void evalJt(double t, double cj, int rowOffset, SparseMatrix& jt) override;
  void evalJtPrim(double t, double cj, int rowOffset, SparseMatrix& jt) override;
  void evalCalculatedVars() override;
  void evalStaticFType() override;
  void evalDynamicFType() override;
  void getY0() override;
  void evalStaticYType() override;
  void evalDynamicYType() override;
  void getIndexesOfVariablesUsedForCalculatedVarI(unsigned iCalculatedVar, std::vector<int>& indexes) const override;
  void evalJCalculatedVarI(unsigned iCalculatedVar, std::vector<double>& res) const override;
  double evalCalculatedVarI(unsigned iCalculatedVar) const override;
  void setSubModelParameters() override;  ///< read the optional bus-fault PAR parameters
  void defineElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement) override;
  void dumpUserReadableElementList(const std::string& nameElement) const override { }
  void defineVariables(std::vector<boost::shared_ptr<Variable> >& variables) override;
  void defineParameters(std::vector<ParameterModeler>& parameters) override;  ///< optional bus-fault parameters
  std::string getCheckSum() const override;
  /// checkpoint / restore the per-component internal state (connection flags, tap positions, and the
  /// node-accumulated shunt C / shunt G / fault G) that lives outside the y/yp/z/g arrays ModelCPP dumps
  void dumpInternalVariables(boost::archive::binary_oarchive& os) const override;
  void loadInternalVariables(boost::archive::binary_iarchive& is) override;
  void initializeStaticData() override { /* not needed */ }
  void initializeFromData(const boost::shared_ptr<DataInterface>& data) override;
  void setFequations() override;        ///< name every abc residual (KCL / branch v-i / switch) for init diagnostics
  void setGequations() override;        ///< name every root function (tap changer, current limit, bus fault)
  void initParams() override { /* not needed */ }
  void checkDataCoherence(double t) override { }

 private:
  void assignIds();            ///< give each component its id (zip the typed component lists with the parallel id lists)
  void assignOffsets();        ///< assign each component its global y / z offset (buses, switches, branches)
  void computeConnectivity();  ///< rebuild AC neighbors, the islands, and the switched-off buses
  void buildCalcVarMap();      ///< flatten the calculated-variable owners (buses, then branches)
  /// opt-in: reseed node voltages / branch currents from the AC steady state that INCLUDES the EMT
  /// snubber shunts, so KCL closes at t=0 and the start is flat (no snubber-LC charging ring). Solves
  /// the complex nodal correction (Y_branch + G_shunt + j*omega*C)*dV = -(G_shunt + j*omega*C)*V_LF
  /// from the load-flow voltages -- data the network already has, no external load flow.
  void applySnubberSeedCorrection();

  // meta-model containers (mirroring DYNModelNetwork's component collections)
  std::vector<NodeEMT*> nodes_;                  ///< all nodes (buses + slack), owned
  std::vector<ModelBusEMT*> buses_;              ///< nodes that own voltage states + a KCL equation
  std::vector<ModelSwitchEMT*> switches_;        ///< switches (own abc currents + a connection-state z)
  std::vector<NetworkComponentEMT*> branches_;   ///< current-bearing components (lines, transformers)
  std::vector<ModelTwoWindingsTransformerEMT*> tapTfos_;  ///< transformers carrying a tap changer (own root functions)
  std::vector<ModelLineEMT*> limitLines_;        ///< lines carrying a current-limit automaton (own root functions)
  std::vector<NetworkComponentEMT*> loads_;      ///< constant-impedance loads (a connection-state z)
  std::vector<ModelShuntCompensatorEMT*> shunts_;  ///< switchable capacitive shunt compensators (a connection-state z)
  std::vector<ModelShuntCompensatorEMT*> reclosingShunts_;  ///< the subset carrying a no-reclosing-delay lockout (own a root)
  std::vector<NetworkComponentEMT*> injectors_;  ///< zero-state injectors (default generators), inject only
  std::vector<NetworkComponentEMT*> bridgedLines_;  ///< lines replaced by an external dynamic model: connectivity + state only, no electrical contribution
  std::vector<ModelVoltageLevelEMT*> voltageLevels_;  ///< voltage-level grouping, owned
  std::vector<NetworkComponentEMT*> owned_;      ///< everything to delete
  ModelBusContainerEMT busContainer_;            ///< owns the bus list; computes the AC islands

  // buses driven by an external black-box machine (generator carrying a dynamic model):
  // each gets 3 abc FLOW current variables (connector-summed) injected into its KCL, in the
  // same order, appended after the branch currents in y
  std::vector<ModelBusEMT*> connectedBuses_;     ///< buses with an external-machine connection
  std::vector<std::string> connectedBusIds_;     ///< their ids (<id>_ACPIN_i_{a,b,c} flow vars)

  // curve/element identifiers, in the same order as the states (buses, switches, branches)
  std::vector<std::string> busIds_;     ///< id of each bus node (its 3 voltage states are <id>_v_{a,b,c})
  std::vector<std::string> switchIds_;  ///< id of each switch (its 3 current states are <id>_i_{a,b,c})
  std::vector<std::string> branchIds_;  ///< id of each branch (its 3 current states are <id>_i_{a,b,c})
  std::vector<std::string> loadIds_;    ///< id of each load (its connection state is <id>_state)
  std::vector<std::string> shuntIds_;   ///< id of each shunt compensator (its connection state is <id>_state)
  std::vector<std::string> genIds_;     ///< id of each PQ generator (its connection state is <id>_state)
  std::vector<std::string> bridgedLineIds_;  ///< id of each bridged line (its connection state is <id>_state)

  // calculated-variable routing: global calc index -> (component, local index)
  std::vector<NetworkComponentEMT*> calcVarComp_;   ///< owner component of each calculated variable
  std::vector<int> calcVarLocal_;                   ///< local index of the calculated variable in its owner

  bool pendingModeChange_;  ///< set by evalZ on a topology change, consumed by evalMode
  double FNom_;   ///< nominal frequency in Hz (EMT modeling default; not carried by the IIDM)
  bool ssInitSeed_ = false;  ///< opt-in snubber-consistent init seed (steady_state_init_seed PAR flag)

  // optional bus fault (PAR-driven): a per-phase shunt-to-ground (RfPu) on faultBus_, switched on
  // during [tBegin, tEnd]. abc/per-phase, so it supports unbalanced faults (faultedPhase select).
  ModelBusEMT* faultBus_;     ///< faulted bus (null if no fault configured)
  double faultRfPu_;          ///< fault on-state resistance to ground per phase in pu
  double faultTBegin_;        ///< fault inception time (s)
  double faultTEnd_;          ///< fault clearing time (s)
  bool faultedPhase_[3];      ///< phases shorted to ground (a, b, c) -> unbalanced faults
  int faultGOff_;             ///< g (root) offset of the fault's 2 timing roots (-1 if none)
  bool faultActive_;          ///< current fault on/off state
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELNETWORKEMT_H_
