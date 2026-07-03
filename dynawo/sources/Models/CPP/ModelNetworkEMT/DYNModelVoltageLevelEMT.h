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
 * @file  DYNModelVoltageLevelEMT.h
 * @brief container of all the sub-models of one voltage level (abc analogue of
 *        ModelVoltageLevel): its buses, switches and injectors, plus the
 *        topology kind. The network aggregates the voltage levels and flattens
 *        their components for the DAE dispatch.
 */
#ifndef MODELS_CPP_MODELNETWORKEMT_DYNMODELVOLTAGELEVELEMT_H_
#define MODELS_CPP_MODELNETWORKEMT_DYNMODELVOLTAGELEVELEMT_H_

#include <string>
#include <vector>

namespace DYN {

class ModelBusEMT;
class ModelSwitchEMT;
class NetworkComponentEMT;

/**
 * @brief one voltage level: groups its buses, switches and injectors. Bus-breaker
 *        for now; the node-breaker graph (bus-bar sections, shortest-path switch
 *        operations) will live here, mirroring ModelVoltageLevel.
 */
class ModelVoltageLevelEMT {
 public:
  enum TopologyKind { BUS_BREAKER, NODE_BREAKER };

  ModelVoltageLevelEMT(const std::string& id, TopologyKind kind) : id_(id), topologyKind_(kind) { }

  const std::string& id() const { return id_; }
  TopologyKind topologyKind() const { return topologyKind_; }

  void addBus(ModelBusEMT* bus) { buses_.push_back(bus); }
  void addSwitch(ModelSwitchEMT* sw) { switches_.push_back(sw); }
  void addInjector(NetworkComponentEMT* inj) { injectors_.push_back(inj); }

  const std::vector<ModelBusEMT*>& buses() const { return buses_; }
  const std::vector<ModelSwitchEMT*>& switches() const { return switches_; }
  const std::vector<NetworkComponentEMT*>& injectors() const { return injectors_; }

 private:
  std::string id_;                                   ///< voltage level id
  TopologyKind topologyKind_;                        ///< bus-breaker or node-breaker
  std::vector<ModelBusEMT*> buses_;                  ///< buses of this voltage level
  std::vector<ModelSwitchEMT*> switches_;            ///< switches of this voltage level
  std::vector<NetworkComponentEMT*> injectors_;      ///< loads / generators / shunts on this voltage level
};

}  // namespace DYN

#endif  // MODELS_CPP_MODELNETWORKEMT_DYNMODELVOLTAGELEVELEMT_H_
