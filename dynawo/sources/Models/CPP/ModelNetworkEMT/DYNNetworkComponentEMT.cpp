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
 * @file  DYNNetworkComponentEMT.cpp
 * @brief out-of-line anchors for the EMT network component base classes.
 */
#include "DYNNetworkComponentEMT.h"
#include "DYNVariable.h"
#include "DYNVariableNative.h"
#include "DYNVariableNativeFactory.h"
#include "DYNElement.h"
#include "DYNCommonModeler.h"

namespace DYN {

NodeEMT::~NodeEMT() { }

NetworkComponentEMT::~NetworkComponentEMT() { }

void
NetworkComponentEMT::addElementWithValue(const std::string& name, const std::string& parentName, const std::string& parentType,
                                         std::vector<Element>& elements, std::map<std::string, int>& mapElement) const {
  addElement(name, Element::STRUCTURE, elements, mapElement);
  addSubElement("value", name, Element::TERMINAL, parentName, parentType, elements, mapElement);
}

void
NetworkComponentEMT::instantiateCurrentStates(std::vector<boost::shared_ptr<Variable> >& variables) const {
  static const char* ph[3] = {"a", "b", "c"};
  for (int k = 0; k < 3; ++k)
    variables.push_back(VariableNativeFactory::createState(id_ + "_i_" + ph[k], CONTINUOUS));
}

void
NetworkComponentEMT::instantiateStateZ(std::vector<boost::shared_ptr<Variable> >& variables) const {
  variables.push_back(VariableNativeFactory::createState(id_ + "_state_value", INTEGER));
}

void
NetworkComponentEMT::instantiateIrms(std::vector<boost::shared_ptr<Variable> >& variables) const {
  variables.push_back(VariableNativeFactory::createCalculated(id_ + "_Irms", CONTINUOUS));
}

void
NetworkComponentEMT::defineCurrentElements(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                                           const std::string& parentName, const std::string& parentType) const {
  static const char* ph[3] = {"a", "b", "c"};
  for (int k = 0; k < 3; ++k)
    addElementWithValue(id_ + "_i_" + ph[k], parentName, parentType, elements, mapElement);
}

void
NetworkComponentEMT::defineStateElement(std::vector<Element>& elements, std::map<std::string, int>& mapElement,
                                        const std::string& parentName, const std::string& parentType) const {
  addElementWithValue(id_ + "_state", parentName, parentType, elements, mapElement);
}

}  // namespace DYN
