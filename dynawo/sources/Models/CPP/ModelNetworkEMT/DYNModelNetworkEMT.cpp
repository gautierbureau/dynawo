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
 * @file  DYNModelNetworkEMT.cpp
 * @brief Native abc EMT network META-MODEL (the same role as the phasor
 *        DYNModelNetwork): aggregates one abc sub-model per IIDM element grouped
 *        by voltage level, computes the AC-connected islands (shutting down the
 *        ones without a voltage reference), and dispatches the DAE residuals /
 *        Jacobian. Switches are explicit ModelSwitchEMT components with a discrete
 *        (z) open/closed state; a topology change recomputes the connectivity.
 *        Detailed components stay Modelica and attach to the abc node.
 */
#include <sstream>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <limits>
#include <complex>
#include <utility>
#include <klu.h>

#include "DYNModelNetworkEMT.h"
#include "DYNModelNetworkEMT.hpp"
#include "DYNModelLineEMT.h"
#include "DYNModelTwoWindingsTransformerEMT.h"
#include "DYNModelLoadEMT.h"
#include "DYNModelShuntCompensatorEMT.h"
#include "DYNModelGroundEMT.h"
#include "DYNBridgedLineEMT.h"
#include "DYNModelGeneratorEMT.h"
#include "DYNModelSwitchEMT.h"
#include "DYNModelVoltageLevelEMT.h"
#include "DYNModelConstants.h"
#include "DYNSparseMatrix.h"
#include "DYNElement.h"
#include "DYNCommonModeler.h"
#include "DYNVariableForModel.h"
#include "DYNVariableAliasFactory.h"
#include "DYNParameter.h"

#include "DYNDataInterface.h"
#include "DYNNetworkInterface.h"
#include "DYNVoltageLevelInterface.h"
#include "DYNBusInterface.h"
#include "DYNSwitchInterface.h"
#include "DYNLoadInterface.h"
#include "DYNGeneratorInterface.h"
#include "DYNShuntCompensatorInterface.h"
#include "DYNLineInterface.h"
#include "DYNTwoWTransformerInterface.h"
#include "DYNStaticVarCompensatorInterface.h"
#include "DYNHvdcLineInterface.h"
#include "DYNConverterInterface.h"
#include "DYNDanglingLineInterface.h"
#include "DYNCurrentLimitInterface.h"
#include "DYNModelCurrentLimits.h"
#include "DYNRatioTapChangerInterface.h"
#include "DYNPhaseTapChangerInterface.h"
#include "DYNStepInterface.h"
#include "DYNModelRatioTapChanger.h"
#include "DYNModelPhaseTapChanger.h"
#include "DYNModelTapChangerStep.h"

using std::vector;
using std::string;
using std::map;
using boost::shared_ptr;

extern "C" DYN::SubModelFactory* getFactory() { return (new DYN::ModelNetworkEMTFactory()); }
extern "C" void deleteFactory(DYN::SubModelFactory* factory) { delete factory; }
extern "C" DYN::SubModel* DYN::ModelNetworkEMTFactory::create() const { return (new DYN::ModelNetworkEMT()); }
extern "C" void DYN::ModelNetworkEMTFactory::destroy(DYN::SubModel* model) const { delete model; }

namespace DYN {

// abc instantaneous current at t=0 for a series R-X branch carrying the load-flow
// phasor current I = (Vfrom - Vto)/(R + jX). The buses are given as rms-pu magnitude
// and angle (rad); the result is rendered cosine-referenced, exactly like the bus
// voltages (i_k(0) = sqrt(2)*|I|*cos(arg(I) - 2*pi*k/3)), so a branch fed by buses at
// their load-flow point starts with di/dt = 0 (a true flat start, no charging transient).
static void branchInitCurrent(double uFrom, double thFrom, double uTo, double thTo,
                              double r, double x, double out[3]) {
  const double dvr = uFrom * std::cos(thFrom) - uTo * std::cos(thTo);
  const double dvi = uFrom * std::sin(thFrom) - uTo * std::sin(thTo);
  const double zden = r * r + x * x;
  if (zden < 1e-12) { for (int k = 0; k < 3; ++k) out[k] = 0.; return; }
  const double ir = (dvr * r + dvi * x) / zden;   // I = dV / (r + jx)
  const double ii = (dvi * r - dvr * x) / zden;
  const double imag = std::sqrt(ir * ir + ii * ii);
  const double iang = std::atan2(ii, ir);
  for (int k = 0; k < 3; ++k) out[k] = std::sqrt(2.) * imag * std::cos(iang - 2. * M_PI * k / 3.);
}

ModelNetworkEMT::ModelNetworkEMT() :
ModelCPP("networkEMT"),
pendingModeChange_(false), FNom_(50.),
faultBus_(nullptr), faultRfPu_(0.), faultTBegin_(0.), faultTEnd_(0.),
faultedPhase_{true, true, true}, faultGOff_(-1), faultActive_(false) {
}

ModelNetworkEMT::~ModelNetworkEMT() {
  for (size_t i = 0; i < owned_.size(); ++i) delete owned_[i];
  for (size_t i = 0; i < voltageLevels_.size(); ++i) delete voltageLevels_[i];
}

void
ModelNetworkEMT::initializeFromData(const shared_ptr<DataInterface>& data) {
  // Build one abc sub-model per IIDM element, grouped by voltage level (mirroring
  // DYNModelNetwork::initializeFromData). Per-unit conversions use the same base
  // as the phasor model: coeff = VNom^2 / SNREF, and the EMT inductance /
  // capacitance follow from the reactance / susceptance at FNom:
  //   RPu = R/coeff   LPu = (X/coeff)/(2*pi*FNom)   CPu = (B*coeff)/(2*pi*FNom)
  static const double zero3[3] = {0., 0., 0.};
  const double w = 2.0 * M_PI * FNom_;
  const auto network = data->getNetwork();
  map<string, NodeEMT*> nodeByBus;
  map<string, ModelVoltageLevelEMT*> vlById;
  std::map<string, std::shared_ptr<BusInterface> > extMachineBuses;  ///< buses whose generator is an external black-box
  std::map<string, std::shared_ptr<BusInterface> > extLineBuses;     ///< end buses of a line replaced by an external black-box (e.g. a LineFault)

  // voltage-level containers
  for (const auto& vl : network->getVoltageLevels()) {
    ModelVoltageLevelEMT::TopologyKind kind =
        vl->isNodeBreakerTopology() ? ModelVoltageLevelEMT::NODE_BREAKER : ModelVoltageLevelEMT::BUS_BREAKER;
    ModelVoltageLevelEMT* mvl = new ModelVoltageLevelEMT(vl->getID(), kind);
    vlById[vl->getID()] = mvl;
    voltageLevels_.push_back(mvl);
  }

  // pass 1: a generator bound to an EXTERNAL Modelica model -- a detailed machine, or
  // an InfiniteBus voltage reference -- exposes its bus for the ACPIN connection. The
  // network itself NEVER imposes a node voltage: exactly as in the phasor world, the
  // infinite bus (and any voltage-regulating generator) is a separate Modelica model
  // bound to a bus, not synthesised inside the network. Generators that stay in the
  // network are only there for constant (PQ) current injections (pass 3b below). A
  // bus with an external model stays a normal node (pass 2) that the external model
  // drives through the KCL.
  for (const auto& vl : network->getVoltageLevels()) {
    for (const auto& gen : vl->getGenerators()) {
      if (!gen->hasDynamicModel()) continue;
      const auto bus = gen->getBusInterface();
      if (!bus) continue;
      extMachineBuses[bus->getID()] = bus;
    }
  }

  // pass 2: remaining buses -> abc nodes; shunt compensators set the node's C
  // (B > 0 -> capacitive). A node with no C is algebraic (handled by ModelBusEMT).
  for (const auto& vl : network->getVoltageLevels()) {
    ModelVoltageLevelEMT* mvl = vlById[vl->getID()];
    const double coeff = vl->getVNom() * vl->getVNom() / SNREF;
    map<string, double> cByBus;
    for (const auto& shunt : vl->getShuntCompensators()) {
      const auto bus = shunt->getBusInterface();
      if (!bus) continue;
      const double bPu = shunt->getB(shunt->getCurrentSection()) * coeff;  // susceptance in pu
      // fold ONLY a connected capacitive bank into the node's build-time C (an initially-open one
      // is added later by its switchable ModelShuntCompensatorEMT if it closes); inductive banks
      // (B < 0) are reactors to ground built in pass 2c, not a node C
      if (bPu > 0. && shunt->getInitialConnected()) cByBus[bus->getID()] += bPu / w;  // CPu = BPu / (2*pi*FNom)
    }
    for (const auto& bus : vl->getBuses()) {
      if (nodeByBus.count(bus->getID())) continue;  // already a source node
      double cPu = (cByBus.count(bus->getID()) ? cByBus[bus->getID()] : 0.);
      // Every passive node is fed only by line/transformer currents, which are differential states;
      // with no shunt C its abc voltage would be absent from its own (algebraic) KCL at the
      // restoration -> structurally singular. Give every C-less node a tiny snubber C (the EMT bus
      // stray capacitance) so it becomes a very stiff differential node -- the index regularisation.
      const bool cIsEmtReg = (cPu <= 0.);   // no physical shunt C here -> the C is the EMT regularisation
      if (cPu <= 0.) cPu = 1e-4;
      // A regular meshed bus starts at 0 (robust: the global init then converges from a clean
      // guess on any grid). Only a voltage-DETERMINED node -- an adjacent source, or a bus driven
      // by an external machine -- is initialised to its load-flow voltage (done where that bus is
      // built: pass 1 for sources, the external-machine pass for machine buses), so its connection
      // can flat-start without a charging transient while the rest of the grid stays robust.
      ModelBusEMT* node = new ModelBusEMT(cPu, 0., zero3);
      // record the load-flow abc voltage (kept even though the seed u0_ starts at 0 for
      // meshed-init robustness) so the opt-in snubber-consistent seed correction can use it
      const double uPuLF = (vl->getVNom() > 0. ? bus->getV0() / vl->getVNom() : 1.);
      const double phaseLF = bus->getAngle0() * M_PI / 180.;
      double u0LF[3];
      for (int k = 0; k < 3; ++k) u0LF[k] = std::sqrt(2.) * uPuLF * std::cos(phaseLF - 2. * M_PI * k / 3.);
      node->setLoadFlowVoltage(u0LF);
      if (cIsEmtReg) node->addEmtShunt(0., cPu);   // the snubber-C regularisation is EMT-added (not in the LF)
      nodeByBus[bus->getID()] = node;
      nodes_.push_back(node); buses_.push_back(node); owned_.push_back(node);
      busContainer_.add(node); mvl->addBus(node);
      busIds_.push_back(bus->getID());
    }
  }

  // pass 2b: switches -> explicit ModelSwitchEMT (closed: v1 = v2; open: i = 0)
  for (const auto& vl : network->getVoltageLevels()) {
    ModelVoltageLevelEMT* mvl = vlById[vl->getID()];
    for (const auto& sw : vl->getSwitches()) {
      const auto b1 = sw->getBusInterface1();
      const auto b2 = sw->getBusInterface2();
      if (!b1 || !b2) continue;
      NodeEMT* n1 = nodeByBus[b1->getID()];
      NodeEMT* n2 = nodeByBus[b2->getID()];
      if (!n1 || !n2) continue;
      ModelSwitchEMT* msw = new ModelSwitchEMT(n1, n2, !sw->isOpen());
      switches_.push_back(msw); owned_.push_back(msw); mvl->addSwitch(msw);
      switchIds_.push_back(sw->getID());
    }
  }

  // shared 0 V ground node for shunt reactors (loads' inductive part, inductive SVCs)
  ModelGroundEMT* ground = nullptr;

  // pass 2c: shunt compensators as switchable components (abc analogue of ModelShuntCompensator).
  // A CAPACITIVE bank (B > 0) is a ModelShuntCompensatorEMT owning a connection-state z; its C is
  // already folded into the node above when initially connected, and the component (dis)connects it
  // at runtime. An INDUCTIVE bank (B < 0) is a reactor to ground, realised as a switchable
  // ModelLineEMT (R = 0, L = 1/(w*|B|)) -- the same branch-to-ground the inductive SVC uses.
  for (const auto& vl : network->getVoltageLevels()) {
    const double coeff = vl->getVNom() * vl->getVNom() / SNREF;
    for (const auto& shunt : vl->getShuntCompensators()) {
      const auto bus = shunt->getBusInterface();
      if (!bus || !nodeByBus.count(bus->getID())) continue;
      ModelBusEMT* node = nodeByBus[bus->getID()]->asBus();
      if (!node) continue;                                   // shunt on a source node: skip (source imposes V)
      const double bPu = shunt->getB(shunt->getCurrentSection()) * coeff;
      if (bPu > 0.) {                                        // capacitive bank
        ModelShuntCompensatorEMT* sh = new ModelShuntCompensatorEMT(node, bPu / w, shunt->getInitialConnected());
        shunts_.push_back(sh); owned_.push_back(sh); shuntIds_.push_back(shunt->getID());
      } else if (bPu < 0.) {                                 // inductive bank -> reactor to ground
        if (!ground) { ground = new ModelGroundEMT(); nodes_.push_back(ground); owned_.push_back(ground); }
        ModelLineEMT* reactor = new ModelLineEMT(node, ground, 0., 1.0 / (w * (-bPu)), zero3, shunt->getInitialConnected());
        branches_.push_back(reactor); owned_.push_back(reactor); branchIds_.push_back(shunt->getID());
      }
    }
  }

  // pass 3: loads -> constant-impedance shunts (default simplified load):
  //   R = U0^2 / P0 (active, G into the node) and B = Q0 / U0^2 (reactive): an inductive load
  //   (Q0 > 0, absorbing) is a shunt reactor to ground, a capacitive one (Q0 < 0) a shunt C.
  //   Modelling the reactive draw is what lets a realistic grid (e.g. IEEE14, ~80 MVAr of load
  //   Q) reproduce its load flow and therefore flat-start; a pure-G load drops all of it.
  for (const auto& vl : network->getVoltageLevels()) {
    ModelVoltageLevelEMT* mvl = vlById[vl->getID()];
    for (const auto& load : vl->getLoads()) {
      const auto bus = load->getBusInterface();
      if (!bus) continue;
      const double vNom = bus->getVNom();
      const double uPu = (vNom > 0. ? bus->getV0() / vNom : 1.);
      const double p0Pu = load->getP0() / SNREF;
      const double rPu = (std::fabs(p0Pu) > 1e-9 ? uPu * uPu / p0Pu : 0.);
      ModelLoadEMT* l = new ModelLoadEMT(nodeByBus[bus->getID()], rPu, load->getInitialConnected());
      owned_.push_back(l); mvl->addInjector(l); loads_.push_back(l); loadIds_.push_back(load->getID());
      // reactive part: B = Q0/U0^2 at the load-flow voltage
      const double q0Pu = load->getQ0() / SNREF;
      ModelBusEMT* lnode = nodeByBus.count(bus->getID()) ? nodeByBus[bus->getID()]->asBus() : nullptr;
      const double bPu = (uPu > 0. ? q0Pu / (uPu * uPu) : 0.);
      if (lnode && std::fabs(bPu) > 1e-9 && load->getInitialConnected()) {
        if (bPu > 0.) {   // inductive (absorbing Q): shunt reactor to ground, X = 1/B -> L = 1/(w*B)
          if (!ground) { ground = new ModelGroundEMT(); nodes_.push_back(ground); owned_.push_back(ground); }
          // the load bus is a regular (0-init) node, so the reactor starts at 0 current too
          ModelLineEMT* reactor = new ModelLineEMT(lnode, ground, 0., 1.0 / (w * bPu), zero3);
          branches_.push_back(reactor); owned_.push_back(reactor);
          branchIds_.push_back(load->getID() + "_reactor");
        } else {          // capacitive (producing Q): shunt C = |B| / w
          lnode->addShuntC(-bPu / w);
        }
      }
    }
  }

  // pass 3b: a generator that stays IN the network is only a constant (PQ) current
  // injection into its bus (abc analogue of the phasor ModelGenerator's PQ injection):
  // I = conj(S/V), |I| = |S|/U, angle = theta - atan2(Q, P). A voltage-regulating
  // generator, or one bound to an external Modelica model (machine / InfiniteBus,
  // handled in pass 1), is NOT a network injection -- it is modelled outside.
  for (const auto& vl : network->getVoltageLevels()) {
    ModelVoltageLevelEMT* mvl = vlById[vl->getID()];
    for (const auto& gen : vl->getGenerators()) {
      if (gen->isVoltageRegulationOn() || gen->hasDynamicModel()) continue;
      const auto bus = gen->getBusInterface();
      if (!bus) continue;
      const double vNom = bus->getVNom();
      const double uPu = (vNom > 0. ? bus->getV0() / vNom : 1.);
      const double theta = bus->getAngle0() * M_PI / 180.;
      const double pPu = gen->getTargetP() / SNREF;   // produced (generator convention)
      const double qPu = gen->getTargetQ() / SNREF;
      if (uPu <= 0.) continue;
      const double iRms = std::sqrt(pPu * pPu + qPu * qPu) / uPu;
      const double phase0 = theta - std::atan2(qPu, pPu);
      ModelGeneratorInjectionEMT* g = new ModelGeneratorInjectionEMT(nodeByBus[bus->getID()], std::sqrt(2.) * iRms, phase0, FNom_, gen->getInitialConnected());
      injectors_.push_back(g); owned_.push_back(g); mvl->addInjector(g); genIds_.push_back(gen->getID());
    }
  }

  // a bus whose abc voltage is DETERMINED at init: a bus driven by an external Modelica
  // model -- a machine or an InfiniteBus reference -- set to its load-flow voltage. A
  // branch whose BOTH ends are determined can flat-start to its load-flow current
  // (di/dt = 0); a branch touching a regular meshed bus (which starts at 0) keeps a
  // 0 current so the global init stays robust.
  auto vDet = [&](const std::string& id) {
    return extMachineBuses.count(id) > 0 || extLineBuses.count(id) > 0;
  };

  // pass 4: lines -> series R-L branches between their two nodes
  for (const auto& line : network->getLines()) {
    if (!line->getBusInterface1() || !line->getBusInterface2()) continue;
    // a line carrying its own dynamic model (an external black-box, e.g. a LineFault)
    // is NOT built here: the external Modelica model replaces the branch and drives
    // both end buses through their ACPIN connectors (the EMT analogue of the phasor
    // network's quadripole bridge). Expose both end buses for the connection below.
    if (line->hasDynamicModel()) {
      const string b1 = line->getBusInterface1()->getID(), b2 = line->getBusInterface2()->getID();
      extLineBuses[b1] = line->getBusInterface1();
      extLineBuses[b2] = line->getBusInterface2();
      // network-side connectivity bridge: links the two end buses while closed and
      // forwards the external model's trip (its <id>_state_value) as a topology change,
      // the abc analogue of the phasor NetworkBridgeQuadripole. No electrical contribution.
      BridgedLineEMT* bridge = new BridgedLineEMT(nodeByBus[b1], nodeByBus[b2]);
      bridgedLines_.push_back(bridge); owned_.push_back(bridge);
      bridgedLineIds_.push_back(line->getID());
      continue;
    }
    const double coeff = line->getVNom1() * line->getVNom1() / SNREF;
    NodeEMT* n1 = nodeByBus[line->getBusInterface1()->getID()];
    NodeEMT* n2 = nodeByBus[line->getBusInterface2()->getID()];
    const bool conn = line->getInitialConnected1() && line->getInitialConnected2();
    const double rLine = line->getR() / coeff, xLine = line->getX() / coeff;
    // flat-start branch current = load-flow current (Vbus1 - Vbus2)/(R + jX), only when both end
    // buses are voltage-determined (else the regular bus starts at 0 -> keep a 0 current).
    const auto b1 = line->getBusInterface1(), b2 = line->getBusInterface2();
    const double u1 = (b1->getVNom() > 0. ? b1->getV0() / b1->getVNom() : 0.);
    const double u2 = (b2->getVNom() > 0. ? b2->getV0() / b2->getVNom() : 0.);
    double iLine0[3] = {0., 0., 0.};
    if (conn && vDet(b1->getID()) && vDet(b2->getID()) && std::isfinite(u1) && std::isfinite(u2) && u1 > 0. && u2 > 0.)
      branchInitCurrent(u1, b1->getAngle0() * M_PI / 180., u2, b2->getAngle0() * M_PI / 180., rLine, xLine, iLine0);
    ModelLineEMT* br = new ModelLineEMT(n1, n2, rLine, xLine / w, iLine0, conn);
    branches_.push_back(br); owned_.push_back(br);
    branchIds_.push_back(line->getID());

    // current limits (reused phasor overcurrent automaton): monitor the RMS current in
    // amps and trip the line if a limit times out. Side-1 limits, like the phasor model.
    const auto& cls = line->getCurrentLimitInterfaces1();
    if (!cls.empty()) {
      std::unique_ptr<ModelCurrentLimits> cl(new ModelCurrentLimits());
      cl->setMaxTimeOperation(std::numeric_limits<double>::max());
      for (const auto& c : cls)
        cl->addLimit(line->getID(), c->getLimit(), c->getAcceptableDuration(), false);
      const double vNom1 = line->getVNom1();
      const double factorPuToA = (vNom1 > 0. ? 1000. * SNREF / (std::sqrt(3.) * vNom1) : 1.);
      br->setCurrentLimits(std::move(cl), line->getID(), factorPuToA);
      limitLines_.push_back(br);
    }
  }

  // pass 5: two-winding transformers -> ratio-coupled R-L branches.
  // R/X are referred to side 2 (the secondary), and rTfo = ratedU1/ratedU2.
  for (const auto& tfo : network->getTwoWTransformers()) {
    if (!tfo->getBusInterface1() || !tfo->getBusInterface2()) continue;
    const double coeff = tfo->getVNom2() * tfo->getVNom2() / SNREF;
    // Both end buses carry per-unit abc voltages (of their own nominal), so the coupling ratio is
    // the PER-UNIT ratio, not the physical turns ratio: rTfo = (ratedU1/ratedU2)*(VNom2/VNom1).
    // With nominal data (ratedU == VNom on each side) this is 1 (an ideal pu transformer is 1:1);
    // using the raw ratedU1/ratedU2 would scale the pu voltage across each voltage level (the
    // source of the IEEE14 1.85 / 0.37 pu buses).
    const double rTfo = (tfo->getRatedU2() > 0. && tfo->getVNom1() > 0.
                         ? (tfo->getRatedU1() / tfo->getRatedU2()) * (tfo->getVNom2() / tfo->getVNom1()) : 1.);
    NodeEMT* n1 = nodeByBus[tfo->getBusInterface1()->getID()];
    NodeEMT* n2 = nodeByBus[tfo->getBusInterface2()->getID()];
    const bool conn = tfo->getInitialConnected1() && tfo->getInitialConnected2();
    const double rT = tfo->getR() / coeff, xT = tfo->getX() / coeff;
    // flat-start primary current = (Vprim - rTfo*Vsec)/(R + jX) (the branch v-i at di/dt = 0,
    // with the tap phase shift taken as 0 at init), only when both end buses are voltage-determined
    // (a regular or a 3WT's fictitious star node starts at 0 -> keep a 0 current).
    const auto bp = tfo->getBusInterface1(), bs = tfo->getBusInterface2();
    const double up = (bp->getVNom() > 0. ? bp->getV0() / bp->getVNom() : 0.);
    const double us = (bs->getVNom() > 0. ? bs->getV0() / bs->getVNom() : 0.);
    double iTfo0[3] = {0., 0., 0.};
    if (conn && vDet(bp->getID()) && vDet(bs->getID()) && std::isfinite(up) && std::isfinite(us) && up > 0. && us > 0.)
      branchInitCurrent(up, bp->getAngle0() * M_PI / 180., rTfo * us, bs->getAngle0() * M_PI / 180., rT, xT, iTfo0);
    ModelTwoWindingsTransformerEMT* t = new ModelTwoWindingsTransformerEMT(
        n1, n2, rTfo, rT, xT / w, iTfo0, conn);
    t->setVNoms(tfo->getVNom1(), tfo->getVNom2());   // for the VHV/HV tap-timing selection in setSubModelParameters
    branches_.push_back(t); owned_.push_back(t);
    branchIds_.push_back(tfo->getID());

    // ratio tap changer: reuse the phasor regulation automaton verbatim, only the
    // physics (rho -> the abc ratio rTfo) is EMT. Monitors the secondary node's RMS.
    const auto& rtc = tfo->getRatioTapChanger();
    if (rtc && rtc->hasLoadTapChangingCapabilities() && !rtc->getSteps().empty()) {
      const int low = rtc->getLowPosition();
      std::unique_ptr<ModelRatioTapChanger> tc(new ModelRatioTapChanger(tfo->getID(), rtc->getTerminalRefSide(), low));
      for (const auto& step : rtc->getSteps())
        tc->addStep(TapChangerStep(step->getRho(), 0., 0., 0., 0., 0.));   // rho carries the abc ratio; r/x/g/b deltas unused
      tc->setHighStepIndex(low + static_cast<int>(rtc->getSteps().size()) - 1);
      tc->setCurrentStepIndex(rtc->getCurrentPosition());
      tc->setTargetV(rtc->getTargetV());
      tc->setTolV(0.01 * tfo->getVNom2());   // deadband ~1% of the monitored nominal voltage (kV)
      tc->setRegulating(true);
      tc->setTFirst(0.01); tc->setTNext(0.01);   // default timers; overridden by PAR t1st/tNext in setSubModelParameters
      t->setRatioTapChanger(std::move(tc), n2, tfo->getVNom2());
      tapTfos_.push_back(t);
    }

    // phase tap changer: same reused automaton (monitors the branch RMS current); the
    // EMT physics is a phase shift alpha applied as an abc rotation of the coupling.
    const auto& ptc = tfo->getPhaseTapChanger();
    if (ptc && !ptc->getSteps().empty()) {
      const int low = ptc->getLowPosition();
      std::unique_ptr<ModelPhaseTapChanger> pc(new ModelPhaseTapChanger(tfo->getID(), low));
      for (const auto& step : ptc->getSteps())
        pc->addStep(TapChangerStep(step->getRho(), step->getAlpha() * M_PI / 180., 0., 0., 0., 0.));  // alpha deg -> rad
      pc->setHighStepIndex(low + static_cast<int>(ptc->getSteps().size()) - 1);
      pc->setCurrentStepIndex(ptc->getCurrentPosition());
      pc->setThresholdI(ptc->getRegulationValue());   // current threshold (same unit as the monitored RMS, here pu)
      pc->setRegulating(true);
      pc->setTFirst(0.01); pc->setTNext(0.01);   // default timers; overridden by PAR t1st/tNext in setSubModelParameters
      t->setPhaseTapChanger(std::move(pc));
      if (tapTfos_.empty() || tapTfos_.back() != t) tapTfos_.push_back(t);
    }
  }

  // NB: three-winding transformers need no dedicated pass. Dynawo's IIDM importer
  // already decomposes every 3WT into a fictitious star bus (<id>_FictBUS, at the
  // ratedU0 voltage) and three fictitious two-winding transformers (<id>_1/_2/_3,
  // star <-> each terminal), which surface through getTwoWTransformers() and are
  // built by pass 5 above. The star bus is an ordinary algebraic node (pass 2 over
  // the fictitious voltage level). This matches the phasor model, whose own 3WT
  // component is an inert stub for the same reason.

  // pass 6: static var compensators (simplified, as in phasor): a shunt susceptance B
  // at its operating point. B > 0 (capacitive) -> shunt C on the node; B < 0 (inductive)
  // -> an R-L branch to a 0 V ground node (reused ModelLineEMT). A detailed SVC is Modelica.
  for (const auto& vl : network->getVoltageLevels()) {
    const double coeff = vl->getVNom() * vl->getVNom() / SNREF;
    for (const auto& svc : vl->getStaticVarCompensators()) {
      const auto bus = svc->getBusInterface();
      if (!bus || svc->getRegulationMode() == StaticVarCompensatorInterface::OFF) continue;
      // operating susceptance: from the load-flow reactive output (Q = B*U^2, producing Q ->
      // capacitive -> B > 0), falling back to the standby susceptance B0. A regulating SVC's
      // detailed B(t) is Modelica; here it is fixed at its operating point.
      const double u0 = (bus->getVNom() > 0. ? bus->getV0() / bus->getVNom() : 1.);
      const double q = svc->getQ();
      const double bPu = (!std::isnan(q) && u0 > 0.) ? (-q / SNREF) / (u0 * u0) : svc->getB0() * coeff;
      ModelBusEMT* node = nodeByBus.count(bus->getID()) ? nodeByBus[bus->getID()]->asBus() : nullptr;
      if (!node) continue;
      if (bPu > 0.) {                                 // capacitive: add shunt C
        node->addShuntC(bPu / w);
      } else if (bPu < 0.) {                          // inductive: shunt reactor to ground
        if (!ground) { ground = new ModelGroundEMT(); nodes_.push_back(ground); owned_.push_back(ground); }
        ModelLineEMT* reactor = new ModelLineEMT(node, ground, 0., 1.0 / (w * (-bPu)), zero3);
        branches_.push_back(reactor); owned_.push_back(reactor);
        branchIds_.push_back(svc->getID() + "_reactor");
      }
    }
  }

  // pass 7: HVDC links (simplified, as in phasor): two decoupled PQ current injections,
  // one per converter, with no AC connection between the two buses. Converter 1 absorbs
  // the active set-point, converter 2 injects it minus the converter losses. A detailed
  // HVDC (droop, limits, blocking) is Modelica.
  for (const auto& hvdc : network->getHvdcLines()) {
    const auto& c1 = hvdc->getConverter1();
    const auto& c2 = hvdc->getConverter2();
    if (!c1 || !c2 || !c1->getBusInterface() || !c2->getBusInterface()) continue;
    NodeEMT* n1 = nodeByBus[c1->getBusInterface()->getID()];
    NodeEMT* n2 = nodeByBus[c2->getBusInterface()->getID()];
    if (!n1 || !n2) continue;
    const double pPu = hvdc->getActivePowerSetpoint() / SNREF;
    const double loss = (1. - c1->getLossFactor() / 100.) * (1. - c2->getLossFactor() / 100.);
    // converter 1 absorbs P (produced = -P); converter 2 injects P * (1 - losses)
    const double u1 = (c1->getBusInterface()->getVNom() > 0. ? c1->getBusInterface()->getV0() / c1->getBusInterface()->getVNom() : 1.);
    const double u2 = (c2->getBusInterface()->getVNom() > 0. ? c2->getBusInterface()->getV0() / c2->getBusInterface()->getVNom() : 1.);
    const double th1 = c1->getBusInterface()->getAngle0() * M_PI / 180.;
    const double th2 = c2->getBusInterface()->getAngle0() * M_PI / 180.;
    if (u1 > 0. && std::fabs(pPu) > 1e-9) {            // |I| = |P|/U, angle = theta - atan2(Q=0, P)
      ModelGeneratorInjectionEMT* g1 = new ModelGeneratorInjectionEMT(n1, std::sqrt(2.) * pPu / u1, th1 - M_PI, FNom_);  // absorb (P<0 produced)
      injectors_.push_back(g1); owned_.push_back(g1); genIds_.push_back(hvdc->getID() + "_conv1");
    }
    if (u2 > 0. && std::fabs(pPu) > 1e-9) {
      ModelGeneratorInjectionEMT* g2 = new ModelGeneratorInjectionEMT(n2, std::sqrt(2.) * pPu * loss / u2, th2, FNom_);   // inject
      injectors_.push_back(g2); owned_.push_back(g2); genIds_.push_back(hvdc->getID() + "_conv2");
    }
  }

  // pass 8: dangling lines (simplified): an R-L branch from the real bus to a fictitious
  // boundary node carrying the boundary load P0 (reused bus + line + load).
  for (const auto& vl : network->getVoltageLevels()) {
    ModelVoltageLevelEMT* mvl = vlById[vl->getID()];
    const double coeff = vl->getVNom() * vl->getVNom() / SNREF;
    for (const auto& dl : vl->getDanglingLines()) {
      const auto bus = dl->getBusInterface();
      if (!bus || !nodeByBus.count(bus->getID())) continue;
      NodeEMT* real = nodeByBus[bus->getID()];
      ModelBusEMT* fict = new ModelBusEMT(0., 0., zero3);   // boundary node (algebraic)
      nodes_.push_back(fict); buses_.push_back(fict); owned_.push_back(fict);
      busContainer_.add(fict); mvl->addBus(fict);
      busIds_.push_back(dl->getID() + "_fict");
      const double rBr = dl->getR() / coeff, lBr = (dl->getX() / coeff) / w;
      ModelLineEMT* br = new ModelLineEMT(real, fict, rBr, lBr, zero3, dl->getInitialConnected());
      branches_.push_back(br); owned_.push_back(br);
      branchIds_.push_back(dl->getID());
      const double uPu = (bus->getVNom() > 0. ? bus->getV0() / bus->getVNom() : 1.);
      const double p0Pu = dl->getP0() / SNREF;
      const double gLoad = (std::fabs(p0Pu) > 1e-9) ? p0Pu / (uPu * uPu) : 0.;
      if (gLoad > 0.) {
        ModelLoadEMT* l = new ModelLoadEMT(fict, 1.0 / gLoad, dl->getInitialConnected());
        owned_.push_back(l); loads_.push_back(l); loadIds_.push_back(dl->getID() + "_boundary");
      }
      // seed the fictitious boundary node's load-flow voltage: the real-bus voltage divided across
      // the dangling branch onto the boundary load to ground (Vf = Vr * y / (y + gLoad)), so the
      // steady-state seed correction places it and reseeds the branch current instead of leaving both
      // at 0. Without this the boundary node has no load-flow voltage and the correction mis-seeds it.
      const std::complex<double> Vr = std::polar(std::sqrt(2.) * uPu, bus->getAngle0() * M_PI / 180.);
      const std::complex<double> yBr = std::complex<double>(1., 0.) / std::complex<double>(rBr, w * lBr);
      const std::complex<double> Vf = Vr * yBr / (yBr + gLoad);
      double u0f[3];
      for (int k = 0; k < 3; ++k) u0f[k] = std::abs(Vf) * std::cos(std::arg(Vf) - 2. * M_PI * k / 3.);
      fict->setLoadFlowVoltage(u0f);
    }
  }

  // NB: three-winding transformers are intentionally NOT modelled here, mirroring the phasor
  // DYNModelNetwork. The phasor's ModelThreeWindingsTransformer is a pure no-op placeholder
  // (sizeY = 0; evalF/evalJt/evalNodeInjection/addBusNeighbors all empty) because the C++
  // ThreeWTransformerInterface exposes only the three bus interfaces and connection flags -- no leg
  // R/X/ratedU/ratio -- so the network has nothing to build a physical model from. A detailed
  // three-winding transformer is therefore an external Modelica model bound to the three buses (its
  // abc terminals connect to the three bus ACPINs, exactly like the detailed machine). To give the
  // EMT network its OWN star + three RL-leg model one would first have to extend the DataInterface to
  // surface the per-leg impedances; until then there is no C++ behaviour to port. The three end buses
  // still exist as ordinary abc nodes (built in pass 2), so an external model can drive them.

  // external-machine buses: flag each so it carries the 3 abc FLOW currents the black-box
  // machine injects, and give it a small shunt C so its abc voltage is a (differential) state
  // the machine can drive (avoids the algebraic stator-current / node-voltage index problem)
  for (const auto& kv : extMachineBuses) {
    const string& busId = kv.first;
    if (!nodeByBus.count(busId)) continue;
    ModelBusEMT* node = nodeByBus[busId]->asBus();
    if (!node) continue;
    node->enableExternalConnection(1.0);   // standard abc convention (the machine now renders Cdq=sqrt(2))
    // The bus already carries the snubber C every passive node gets (above), which makes its
    // abc voltage a differential state the machine current charges. Add a small damping G (large
    // damping resistor R = 100), mirroring smibfull's DAMPGEN, to damp the C/stator-L resonance
    // that the machine's excitation would otherwise drive. It is tiny (draws ~1% pu), so it barely
    // perturbs the load-flow operating point.
    for (int k = 0; k < 3; ++k) node->registerShuntG(k, 1e-2);
    node->addEmtShunt(1e-2, 0.);   // the damping G is EMT-added (not in the LF) -> drives the seed correction
    // initialize the node to the load-flow operating point so the machine starts consistent
    const auto& bus = kv.second;
    const double vNom = bus->getVNom();
    const double uPu = (vNom > 0. ? bus->getV0() / vNom : 1.);
    const double phase0 = bus->getAngle0() * M_PI / 180.;
    double u0[3];
    for (int k = 0; k < 3; ++k) u0[k] = std::sqrt(2.) * uPu * std::cos(phase0 - 2. * M_PI * k / 3.);
    node->setInitialVoltage(u0);
    connectedBuses_.push_back(node);
    connectedBusIds_.push_back(busId);
  }

  // external-line buses: the two end buses of a line replaced by an external black-box
  // (a LineFault). Expose each for the ACPIN connection (the black-box's terminal1 /
  // terminal2 connect to the two bus ACPINs) and seed it to its load-flow voltage, so
  // the external model's segment currents can flat-start. No damping G here -- these are
  // ordinary network buses, not a machine terminal.
  std::set<string> already(connectedBusIds_.begin(), connectedBusIds_.end());
  for (const auto& kv : extLineBuses) {
    const string& busId = kv.first;
    if (already.count(busId) || !nodeByBus.count(busId)) continue;
    ModelBusEMT* node = nodeByBus[busId]->asBus();
    if (!node) continue;
    node->enableExternalConnection(1.0);
    const auto& bus = kv.second;
    const double vNom = bus->getVNom();
    const double uPu = (vNom > 0. ? bus->getV0() / vNom : 1.);
    const double phase0 = bus->getAngle0() * M_PI / 180.;
    double u0[3];
    for (int k = 0; k < 3; ++k) u0[k] = std::sqrt(2.) * uPu * std::cos(phase0 - 2. * M_PI * k / 3.);
    node->setInitialVoltage(u0);
    connectedBuses_.push_back(node);
    connectedBusIds_.push_back(busId);
    already.insert(busId);
  }

  // give each component its id now that every typed list is populated, so it can name its
  // own variables/elements (instantiateVariables/defineElements). The id lists are pushed in
  // lock-step with the component lists, so a positional zip is exact.
  assignIds();

  // NB: offsets / wiring / connectivity are finalised in getSize(), which runs AFTER
  // setSubModelParameters() -- so components created from PAR parameters (e.g. a bus fault)
  // are included. See assignOffsets()/getSize().
}

void
ModelNetworkEMT::assignIds() {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->setId(busIds_[i]);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->setId(switchIds_[i]);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->setId(branchIds_[i]);
  for (size_t i = 0; i < loads_.size(); ++i) loads_[i]->setId(loadIds_[i]);
  for (size_t i = 0; i < shunts_.size(); ++i) shunts_[i]->setId(shuntIds_[i]);
  for (size_t i = 0; i < injectors_.size(); ++i) injectors_[i]->setId(genIds_[i]);
  for (size_t i = 0; i < bridgedLines_.size(); ++i) bridgedLines_[i]->setId(bridgedLineIds_[i]);
}

void
ModelNetworkEMT::defineParameters(vector<ParameterModeler>& parameters) {
  // optional bus fault: a per-phase shunt-to-ground on a chosen bus, active over [tBegin, tEnd].
  // All EXTERNAL_PARAMETER and optional (guarded by hasParameterDynamic in setSubModelParameters).
  parameters.push_back(ParameterModeler("fault_bus", VAR_TYPE_STRING, EXTERNAL_PARAMETER));
  parameters.push_back(ParameterModeler("fault_RfPu", VAR_TYPE_DOUBLE, EXTERNAL_PARAMETER));
  parameters.push_back(ParameterModeler("fault_tBegin", VAR_TYPE_DOUBLE, EXTERNAL_PARAMETER));
  parameters.push_back(ParameterModeler("fault_tEnd", VAR_TYPE_DOUBLE, EXTERNAL_PARAMETER));
  parameters.push_back(ParameterModeler("fault_faultedPhase_0_", VAR_TYPE_BOOL, EXTERNAL_PARAMETER));
  parameters.push_back(ParameterModeler("fault_faultedPhase_1_", VAR_TYPE_BOOL, EXTERNAL_PARAMETER));
  parameters.push_back(ParameterModeler("fault_faultedPhase_2_", VAR_TYPE_BOOL, EXTERNAL_PARAMETER));
  // per-bus connection flag (as in the phasor network): a true "<busId>_hasConnection" exposes that
  // bus's abc ACPIN so any external Modelica model (fault, load, injector) can attach to it.
  // "<busId>_hasShortCircuitCapabilities" is the phasor's synonym (ModelBus gates the same ir/ii
  // injection terminal on either flag), so it is declared too and opens the same ACPIN here.
  for (size_t i = 0; i < busIds_.size(); ++i) {
    parameters.push_back(ParameterModeler(busIds_[i] + "_hasConnection", VAR_TYPE_BOOL, EXTERNAL_PARAMETER));
    parameters.push_back(ParameterModeler(busIds_[i] + "_hasShortCircuitCapabilities", VAR_TYPE_BOOL, EXTERNAL_PARAMETER));
  }
  // tap-changer action delays (as in the phasor): declared both shared ("transformer_<name>") and
  // per-transformer ("<id>_<name>"), so setSubModelParameters can read them. A parameter must be
  // DECLARED here for hasParameterDynamic() to see it -- without this, applyTapTiming never runs.
  static const char* tapPars[5] = {"t1st_THT", "tNext_THT", "t1st_HT", "tNext_HT", "tolV"};
  for (int p = 0; p < 5; ++p)
    parameters.push_back(ParameterModeler(string("transformer_") + tapPars[p], VAR_TYPE_DOUBLE, EXTERNAL_PARAMETER));
  for (size_t i = 0; i < tapTfos_.size(); ++i)
    for (int p = 0; p < 5; ++p)
      parameters.push_back(ParameterModeler(tapTfos_[i]->id() + "_" + tapPars[p], VAR_TYPE_DOUBLE, EXTERNAL_PARAMETER));
  // per-shunt no-reclosing delay (as in the phasor ModelShuntCompensator); optional, default 0 = no lockout
  for (size_t i = 0; i < shuntIds_.size(); ++i)
    parameters.push_back(ParameterModeler(shuntIds_[i] + "_no_reclosing_delay", VAR_TYPE_DOUBLE, EXTERNAL_PARAMETER));
  // opt-in snubber-consistent init seed: reseed node voltages / branch currents from the AC steady
  // state that includes the EMT snubber shunts (default off -> committed cases stay byte-identical)
  parameters.push_back(ParameterModeler("steady_state_init_seed", VAR_TYPE_BOOL, EXTERNAL_PARAMETER));
}

void
ModelNetworkEMT::setSubModelParameters() {
  // A parameter declared in defineParameters always "exists"; only a PAR <par> gives it a value.
  auto hasVal = [&](const string& n) { return hasParameterDynamic(n) && findParameterDynamic(n).hasValue(); };

  // per-bus connection flag: expose the abc ACPIN on each "<busId>_hasConnection = true" bus so an
  // external Modelica model can attach to it (the EMT analogue of the phasor's per-bus ACPIN). The
  // FLOW is in this network's standard abc convention (flowScale 1); the machine buses already added
  // above keep their factor-3 Park bridge. Skip a bus already wired as an external-machine bus.
  // "<busId>_hasShortCircuitCapabilities" is accepted as a synonym: in the phasor ModelBus that flag
  // and hasConnection gate the IDENTICAL machinery (the bus gains its ir/ii injection terminal), so a
  // bus that may host a short-circuit / node-fault model exposes its terminal exactly like a connected
  // bus. Either flag therefore opens the abc ACPIN here.
  auto wantsConnection = [&](const string& busId) {
    const string c = busId + "_hasConnection", sc = busId + "_hasShortCircuitCapabilities";
    return (hasVal(c) && findParameterDynamic(c).getValue<bool>()) ||
           (hasVal(sc) && findParameterDynamic(sc).getValue<bool>());
  };
  std::set<string> alreadyConnected(connectedBusIds_.begin(), connectedBusIds_.end());
  for (size_t i = 0; i < busIds_.size(); ++i) {
    if (!wantsConnection(busIds_[i])) continue;
    if (alreadyConnected.count(busIds_[i])) continue;
    buses_[i]->enableExternalConnection(1.0);            // standard abc convention, no Park bridge
    connectedBuses_.push_back(buses_[i]);
    connectedBusIds_.push_back(busIds_[i]);
  }

  // PAR-driven tap-changer action delays (mirrors the phasor ModelTwoWindingsTransformer): a tap
  // changer's t1st/tNext are read from the PAR -- per-transformer "<id>_t1st_THT" or the shared
  // "transformer_t1st_THT" -- and applied, picking the THT or HT pair by the transformer's voltages.
  // Absent in the PAR -> the construction-time default timers are kept (so a case with no tap params
  // is unchanged). Mirrors getParameterDynamic's {id, "transformer"} lookup, but optional (no throw).
  if (!tapTfos_.empty()) {
    auto valOr = [&](const string& id, const string& suffix, double def) {
      const string perId = id + "_" + suffix, shared = "transformer_" + suffix;
      if (hasVal(perId)) return findParameterDynamic(perId).getValue<double>();
      if (hasVal(shared)) return findParameterDynamic(shared).getValue<double>();
      return def;
    };
    for (size_t i = 0; i < tapTfos_.size(); ++i) {
      const string& id = tapTfos_[i]->id();
      const bool hasTiming = hasVal(id + "_t1st_THT") || hasVal("transformer_t1st_THT") ||
                             hasVal(id + "_t1st_HT")  || hasVal("transformer_t1st_HT");
      if (!hasTiming) continue;   // no PAR timing for this transformer -> keep its default timers
      const double t1stTHT = valOr(id, "t1st_THT", 60.), tNextTHT = valOr(id, "tNext_THT", 10.);
      const double t1stHT  = valOr(id, "t1st_HT", 60.),  tNextHT  = valOr(id, "tNext_HT", 10.);
      const bool hasTolV = hasVal(id + "_tolV") || hasVal("transformer_tolV");
      const double tolV = valOr(id, "tolV", 0.);
      tapTfos_[i]->applyTapTiming(t1stTHT, tNextTHT, t1stHT, tNextHT, hasTolV, tolV);
    }
  }

  // per-shunt no-reclosing delay (mirrors the phasor ModelShuntCompensator): a true ">0" delay makes
  // the bank own a "reclosing permitted" root and defer a too-early reclose. Absent / 0 -> no lockout.
  reclosingShunts_.clear();
  for (size_t i = 0; i < shunts_.size(); ++i) {
    const string pn = shuntIds_[i] + "_no_reclosing_delay";
    if (!hasVal(pn)) continue;
    const double d = findParameterDynamic(pn).getValue<double>();
    if (d <= 0.) continue;
    shunts_[i]->setReclosingDelay(d);
    reclosingShunts_.push_back(shunts_[i]);
  }

  // opt-in snubber-consistent init seed (default off); consumed in init(t0) after connectivity
  ssInitSeed_ = hasVal("steady_state_init_seed") && findParameterDynamic("steady_state_init_seed").getValue<bool>();

  // wire up the optional bus fault from the PAR (no fault unless fault_bus / fault_RfPu are set).
  if (!hasVal("fault_bus") || !hasVal("fault_RfPu")) return;
  const string busId = findParameterDynamic("fault_bus").getValue<string>();
  ModelBusEMT* node = nullptr;
  for (size_t i = 0; i < busIds_.size(); ++i)
    if (busIds_[i] == busId) { node = buses_[i]; break; }
  if (!node) return;   // unknown bus -> no fault
  faultBus_ = node;
  faultRfPu_ = findParameterDynamic("fault_RfPu").getValue<double>();
  faultTBegin_ = hasVal("fault_tBegin") ? findParameterDynamic("fault_tBegin").getValue<double>() : 1e9;
  faultTEnd_ = hasVal("fault_tEnd") ? findParameterDynamic("fault_tEnd").getValue<double>() : 1e9;
  for (int k = 0; k < 3; ++k) {
    std::ostringstream pn; pn << "fault_faultedPhase_" << k << "_";
    faultedPhase_[k] = hasVal(pn.str()) ? findParameterDynamic(pn.str()).getValue<bool>() : true;
  }
}

void
ModelNetworkEMT::assignOffsets() {
  // global y offsets: buses, then switches, then branches (so evalJt columns follow y). A
  // connection bus owns nbY() = 6 entries (3 voltage states + 3 in-block abc ACPIN FLOW currents),
  // so its flows sit right after its states -- mirroring the phasor ModelBus's offsetY layout.
  int off = 0;
  for (size_t i = 0; i < buses_.size(); ++i) { buses_[i]->setOffset(off); off += buses_[i]->nbY(); }
  for (size_t i = 0; i < switches_.size(); ++i) { switches_[i]->setOffset(off); off += switches_[i]->nbY(); }
  for (size_t i = 0; i < branches_.size(); ++i) { branches_[i]->setOffset(off); off += branches_[i]->nbY(); }
  // global residual (f) offsets, in the SAME component order as the y offsets but counting only
  // nbStates() (FLOW currents have no residual of their own -- the connector provides it), so fOff_
  // diverges from off_ once a connection bus precedes -- mirroring the phasor's distinct offsetF.
  int foff = 0;
  for (size_t i = 0; i < buses_.size(); ++i) { buses_[i]->setFOffset(foff); foff += buses_[i]->nbStates(); }
  for (size_t i = 0; i < switches_.size(); ++i) { switches_[i]->setFOffset(foff); foff += switches_[i]->nbStates(); }
  for (size_t i = 0; i < branches_.size(); ++i) { branches_[i]->setFOffset(foff); foff += branches_[i]->nbStates(); }
  // global z offsets, in the same order as the discrete variables are declared:
  // switches, branches, loads, generators (each carries one connection-state z)
  int zoff = 0;
  for (size_t i = 0; i < switches_.size(); ++i) { switches_[i]->setZOffset(zoff); zoff += switches_[i]->nbZ(); }
  for (size_t i = 0; i < branches_.size(); ++i) { branches_[i]->setZOffset(zoff); zoff += branches_[i]->nbZ(); }
  for (size_t i = 0; i < loads_.size(); ++i) { loads_[i]->setZOffset(zoff); zoff += loads_[i]->nbZ(); }
  for (size_t i = 0; i < shunts_.size(); ++i) { shunts_[i]->setZOffset(zoff); zoff += shunts_[i]->nbZ(); }
  for (size_t i = 0; i < injectors_.size(); ++i) { injectors_[i]->setZOffset(zoff); zoff += injectors_[i]->nbZ(); }
  for (size_t i = 0; i < bridgedLines_.size(); ++i) { bridgedLines_[i]->setZOffset(zoff); zoff += bridgedLines_[i]->nbZ(); }
  // global g (root function) offsets: tap-changer transformers, then current-limit lines
  int goff = 0;
  for (size_t i = 0; i < tapTfos_.size(); ++i) { tapTfos_[i]->setGOffset(goff); goff += tapTfos_[i]->nbG(); }
  for (size_t i = 0; i < limitLines_.size(); ++i) { limitLines_[i]->setGOffset(goff); goff += limitLines_[i]->nbG(); }
  for (size_t i = 0; i < reclosingShunts_.size(); ++i) { reclosingShunts_[i]->setGOffset(goff); goff += reclosingShunts_[i]->nbG(); }
  if (faultBus_) { faultGOff_ = goff; goff += 2; }   // bus fault: 2 timing roots (tBegin, tEnd)
}

void
ModelNetworkEMT::computeConnectivity() {
  // rebuild the AC neighbor graph from the currently-closed branches/switches,
  // split it into islands, and switch off the islands with no voltage reference.
  busContainer_.resetConnectivity();
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->addBusNeighbors();
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->addBusNeighbors();
  for (size_t i = 0; i < bridgedLines_.size(); ++i) bridgedLines_[i]->addBusNeighbors();
  busContainer_.exploreNeighbors();
  busContainer_.analyseIslands();
}

void ModelNetworkEMT::init(const double /*t0*/) {
  computeConnectivity();
  if (ssInitSeed_) applySnubberSeedCorrection();
}

void
ModelNetworkEMT::applySnubberSeedCorrection() {
  // Reseed every node voltage / branch current from the AC steady state that INCLUDES the EMT
  // snubber shunts, so KCL closes at t=0 and the start is flat (no snubber-LC charging ring).
  //
  // The classical load flow the IIDM carries is snubber-UNAWARE: the branch flows do not supply
  // each node's snubber current i = j*omega*C*V (nor the EMT damping G*V), so at t=0 the KCL
  // residual = that missing current, which kicks the C-L modes. The fix is one linear nodal solve
  // for the voltage shift dV that lets the branches supply it, from data the network already has
  // (branch admittances + snubber C + shunt G + the load-flow voltages) -- no external load flow:
  //     (Y_branch + diag(G_sh + j*omega*C)) * dV = -(G_emt + j*omega*C_emt) * V_LF ,
  // where the MATRIX diagonal carries the TOTAL shunt but the RHS carries only the EMT-ADDED shunt
  // (snubber-C regularisation + damping G) -- the physical shunts are already balanced by V_LF.
  // The system is restricted to the FREE (non-external, live) buses: external-connection nodes are
  // driven by their own Modelica model, so they are held at V_LF (dV = 0) and drop out of the rows
  // and columns. The snubber shunts put an admittance-to-ground on every node, so it is non-singular
  // with no slack. It is sparse (each bus touches only its neighbours) and is solved with KLU
  // (SuiteSparse), the same sparse solver Dynawo uses elsewhere -- so this scales to a real grid.
  typedef std::complex<double> cx;
  const double w = 2.0 * M_PI * FNom_;
  const size_t n = buses_.size();
  if (n == 0) return;

  std::vector<cx> V0(n);
  std::vector<int> rid(n, -1);   // reduced (free-bus) index, or -1 if held fixed
  int m = 0;
  for (size_t i = 0; i < n; ++i) {
    V0[i] = buses_[i]->phasorLF();
    if (!buses_[i]->hasExternalConnection() && !buses_[i]->isSwitchedOff()) rid[i] = m++;
  }
  std::map<ModelBusEMT*, int> busPos;                 // bus -> position in buses_
  for (size_t i = 0; i < n; ++i) busPos[buses_[i]] = static_cast<int>(i);

  std::vector<cx> dV(n, cx(0., 0.));                  // 0 on fixed / dead nodes
  if (m > 0) {
    // assemble the reduced matrix column-by-column (sorted rows via std::map) and the RHS
    std::vector<std::map<int, cx> > col(m);
    std::vector<cx> rhs(m, cx(0., 0.));
    for (size_t i = 0; i < n; ++i) {
      if (rid[i] < 0) continue;
      col[rid[i]][rid[i]] += cx(buses_[i]->shuntGPhase(), w * buses_[i]->cPu());  // total shunt (diagonal)
      rhs[rid[i]] += -buses_[i]->emtShuntY(w) * V0[i];                            // only EMT-added drives dV
    }
    for (size_t b = 0; b < branches_.size(); ++b) {
      NodeEMT* fn = nullptr; NodeEMT* tn = nullptr; double r = 0., l = 0., ratio = 1.;
      if (!branches_[b]->initSeedBranch(fn, tn, r, l, ratio)) continue;
      ModelBusEMT* fb = fn ? fn->asBus() : nullptr;
      ModelBusEMT* tb = tn ? tn->asBus() : nullptr;
      const int fp = (fb && busPos.count(fb)) ? busPos[fb] : -1;
      const int tp = (tb && busPos.count(tb)) ? busPos[tb] : -1;
      const int rf = fp >= 0 ? rid[fp] : -1, rt = tp >= 0 ? rid[tp] : -1;  // -1 = ground / fixed
      const cx y = cx(1., 0.) / cx(r, w * l);
      // ideal-transformer primitive: [[y, -ratio*y],[-ratio*y, ratio^2*y]]; entries touching a
      // fixed / ground node (reduced index < 0, dV = 0) are dropped -- leaving the free-end self term.
      if (rf >= 0) col[rf][rf] += y;
      if (rt >= 0) col[rt][rt] += ratio * ratio * y;
      if (rf >= 0 && rt >= 0) { col[rt][rf] += -ratio * y; col[rf][rt] += -ratio * y; }
    }

    // pack to compressed-column (Ap/Ai) with interleaved complex values (Ax: re,im per entry) for KLU
    std::vector<int> Ap(m + 1), Ai;
    std::vector<double> Ax, b(2 * m);
    for (int j = 0; j < m; ++j) {
      Ap[j] = static_cast<int>(Ai.size());
      for (std::map<int, cx>::const_iterator it = col[j].begin(); it != col[j].end(); ++it) {
        Ai.push_back(it->first); Ax.push_back(it->second.real()); Ax.push_back(it->second.imag());
      }
      b[2 * j] = rhs[j].real(); b[2 * j + 1] = rhs[j].imag();
    }
    Ap[m] = static_cast<int>(Ai.size());

    klu_common Common;
    klu_defaults(&Common);
    klu_symbolic* Symbolic = klu_analyze(m, &Ap[0], &Ai[0], &Common);
    klu_numeric* Numeric = Symbolic ? klu_z_factor(&Ap[0], &Ai[0], &Ax[0], Symbolic, &Common) : nullptr;
    bool ok = (Numeric != nullptr);
    if (ok) ok = (klu_z_solve(Symbolic, Numeric, m, 1, &b[0], &Common) == 1);   // b <- solution
    if (Symbolic) klu_free_symbolic(&Symbolic, &Common);      // NB: the free calls null out the handles,
    if (Numeric) klu_z_free_numeric(&Numeric, &Common);       //     so success must be captured BEFORE them
    if (!ok) return;                                          // factorisation/solve failed -> keep the LF seed (safe)
    for (size_t i = 0; i < n; ++i)
      if (rid[i] >= 0) dV[i] = cx(b[2 * rid[i]], b[2 * rid[i] + 1]);
  }

  // write the corrected phasor voltages back as abc seeds
  std::vector<cx> V(n);
  for (size_t i = 0; i < n; ++i) {
    V[i] = V0[i] + dV[i];   // dV = 0 on external / dead nodes, so V = V0 there
    if (buses_[i]->isSwitchedOff() || buses_[i]->hasExternalConnection()) continue;  // keep their own seed
    const double mag = std::abs(V[i]), ang = std::arg(V[i]);
    double u0[3];
    for (int k = 0; k < 3; ++k) u0[k] = mag * std::cos(ang - 2. * M_PI * k / 3.);
    buses_[i]->setInitialVoltage(u0);
  }
  // and the corrected branch currents i = (V_from - ratio*V_to)/(r + j*omega*l). A bus-to-ground
  // reactor uses V = 0 for the ground end -- this reseeds its i0 (else 0) to its true V/(j*omega*L),
  // removing that branch's own charging transient too.
  for (size_t b = 0; b < branches_.size(); ++b) {
    NodeEMT* fn = nullptr; NodeEMT* tn = nullptr; double r = 0., l = 0., ratio = 1.;
    if (!branches_[b]->initSeedBranch(fn, tn, r, l, ratio)) continue;
    ModelBusEMT* fb = fn ? fn->asBus() : nullptr;
    ModelBusEMT* tb = tn ? tn->asBus() : nullptr;
    const int fp = (fb && busPos.count(fb)) ? busPos[fb] : -1;
    const int tp = (tb && busPos.count(tb)) ? busPos[tb] : -1;
    if (fp < 0 && tp < 0) continue;           // neither end a solved bus -> nothing to seed
    const cx Vf = fp >= 0 ? V[fp] : cx(0., 0.);
    const cx Vt = tp >= 0 ? V[tp] : cx(0., 0.);
    const cx Iph = (Vf - ratio * Vt) / cx(r, w * l);
    const double mag = std::abs(Iph), ang = std::arg(Iph);
    double i0[3];
    for (int k = 0; k < 3; ++k) i0[k] = mag * std::cos(ang - 2. * M_PI * k / 3.);
    branches_[b]->setInitSeedCurrent(i0);
  }
}

void
ModelNetworkEMT::buildCalcVarMap() {
  // flatten the calculated-variable owners in the SAME order as defineVariables:
  // every bus (Urms), then every branch (Irms)
  calcVarComp_.clear(); calcVarLocal_.clear();
  for (size_t i = 0; i < buses_.size(); ++i)
    for (int k = 0; k < buses_[i]->nbCalcVars(); ++k) { calcVarComp_.push_back(buses_[i]); calcVarLocal_.push_back(k); }
  for (size_t i = 0; i < branches_.size(); ++i)
    for (int k = 0; k < branches_[i]->nbCalcVars(); ++k) { calcVarComp_.push_back(branches_[i]); calcVarLocal_.push_back(k); }
}

void
ModelNetworkEMT::getSize() {
  // finalise offsets and wiring here (after setSubModelParameters, so PAR-driven components count)
  assignOffsets();
  for (size_t i = 0; i < owned_.size(); ++i) owned_[i]->wire();  // register currents/conductances now offsets are set
  // sizeY counts every y entry (nbY: states PLUS in-block ACPIN FLOW currents); sizeF counts only
  // residual equations (nbStates: a FLOW current's equation comes from the connector, not the model),
  // so sizeY > sizeF whenever a connection bus exists -- mirroring the phasor meta-model.
  int ny = 0, nf = 0, nz = 0;
  for (size_t i = 0; i < buses_.size(); ++i) { ny += buses_[i]->nbY(); nf += buses_[i]->nbStates(); }
  for (size_t i = 0; i < switches_.size(); ++i) { ny += switches_[i]->nbY(); nf += switches_[i]->nbStates(); nz += switches_[i]->nbZ(); }
  for (size_t i = 0; i < branches_.size(); ++i) { ny += branches_[i]->nbY(); nf += branches_[i]->nbStates(); nz += branches_[i]->nbZ(); }
  for (size_t i = 0; i < loads_.size(); ++i) nz += loads_[i]->nbZ();
  for (size_t i = 0; i < shunts_.size(); ++i) nz += shunts_[i]->nbZ();
  for (size_t i = 0; i < injectors_.size(); ++i) nz += injectors_[i]->nbZ();
  for (size_t i = 0; i < bridgedLines_.size(); ++i) nz += bridgedLines_[i]->nbZ();
  int ng = 0;
  for (size_t i = 0; i < tapTfos_.size(); ++i) ng += tapTfos_[i]->nbG();
  for (size_t i = 0; i < limitLines_.size(); ++i) ng += limitLines_[i]->nbG();
  for (size_t i = 0; i < reclosingShunts_.size(); ++i) ng += reclosingShunts_[i]->nbG();
  if (faultBus_) ng += 2;   // bus fault timing roots (tBegin, tEnd)
  buildCalcVarMap();
  sizeF_ = nf; sizeY_ = ny; sizeZ_ = nz; sizeG_ = ng; sizeMode_ = 0;
  sizeCalculatedVar_ = static_cast<int>(calcVarComp_.size());
  calculatedVars_.assign(sizeCalculatedVar_, 0.);
}

void
ModelNetworkEMT::evalCalculatedVars() {
  for (size_t g = 0; g < calcVarComp_.size(); ++g)
    calculatedVars_[g] = calcVarComp_[g]->evalCalcVar(calcVarLocal_[g], yLocal_);
}

double
ModelNetworkEMT::evalCalculatedVarI(unsigned i) const {
  return calcVarComp_[i]->evalCalcVar(calcVarLocal_[i], yLocal_);
}

void
ModelNetworkEMT::getIndexesOfVariablesUsedForCalculatedVarI(unsigned i, std::vector<int>& indexes) const {
  calcVarComp_[i]->calcVarIndexes(calcVarLocal_[i], indexes);
}

void
ModelNetworkEMT::evalJCalculatedVarI(unsigned i, std::vector<double>& res) const {
  calcVarComp_[i]->evalJCalcVar(calcVarLocal_[i], yLocal_, res);
}

void
ModelNetworkEMT::evalF(double t, propertyF_t /*type*/) {
  for (size_t i = 0; i < nodes_.size(); ++i) nodes_[i]->resetInjection();                               // 1. reset injections
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->evalF(t, yLocal_, ypLocal_, fLocal_);     // 2. branches
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->evalF(t, yLocal_, ypLocal_, fLocal_);     //    switches
  for (size_t i = 0; i < injectors_.size(); ++i) injectors_[i]->evalF(t, yLocal_, ypLocal_, fLocal_);   //    generators
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->evalF(t, yLocal_, ypLocal_, fLocal_);           // 3. nodes (KCL)
}

void
ModelNetworkEMT::evalJt(const double /*t*/, const double cj, const int rowOffset, SparseMatrix& jt) {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->evalJt(cj, rowOffset, jt);          // columns in y order:
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->evalJt(cj, rowOffset, jt);    // buses, switches, branches,
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->evalJt(cj, rowOffset, jt);
  // NB: the external-machine FLOW columns are NOT filled here -- like the phasor ModelBus, the
  // flow connector owns the FLOW variable's Jacobian (its row and its coupling into the bus KCL)
}

void
ModelNetworkEMT::evalJtPrim(const double /*t*/, const double /*cj*/, const int rowOffset, SparseMatrix& jt) {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->evalJtPrim(rowOffset, jt);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->evalJtPrim(rowOffset, jt);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->evalJtPrim(rowOffset, jt);
}

void
ModelNetworkEMT::getY0() {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->getY0(0., yLocal_, ypLocal_);
  for (size_t i = 0; i < switches_.size(); ++i) { switches_[i]->getY0(0., yLocal_, ypLocal_); switches_[i]->getZ0(zLocal_); }
  for (size_t i = 0; i < branches_.size(); ++i) { branches_[i]->getY0(0., yLocal_, ypLocal_); branches_[i]->getZ0(zLocal_); }
  for (size_t i = 0; i < loads_.size(); ++i) loads_[i]->getZ0(zLocal_);
  for (size_t i = 0; i < shunts_.size(); ++i) shunts_[i]->getZ0(zLocal_);
  for (size_t i = 0; i < injectors_.size(); ++i) injectors_[i]->getZ0(zLocal_);
  for (size_t i = 0; i < bridgedLines_.size(); ++i) bridgedLines_[i]->getZ0(zLocal_);
  // NB: a connection bus seeds its own in-block ACPIN FLOW currents to 0 inside its getY0()
}

void
ModelNetworkEMT::evalZ(double t) {
  bool topoChange = false, stateChange = false;
  for (size_t i = 0; i < switches_.size(); ++i)
    if (switches_[i]->evalZ(t, zLocal_) == NC_TOPO_CHANGE) topoChange = true;
  for (size_t i = 0; i < branches_.size(); ++i)
    if (branches_[i]->evalZ(t, zLocal_) == NC_TOPO_CHANGE) topoChange = true;
  for (size_t i = 0; i < loads_.size(); ++i)
    if (loads_[i]->evalZ(t, zLocal_) == NC_STATE_CHANGE) stateChange = true;
  for (size_t i = 0; i < shunts_.size(); ++i)
    if (shunts_[i]->evalZ(t, zLocal_) == NC_STATE_CHANGE) stateChange = true;
  for (size_t i = 0; i < injectors_.size(); ++i)
    if (injectors_[i]->evalZ(t, zLocal_) == NC_STATE_CHANGE) stateChange = true;
  for (size_t i = 0; i < bridgedLines_.size(); ++i)                        // external line trip -> recompute connectivity
    if (bridgedLines_[i]->evalZ(t, zLocal_) == NC_TOPO_CHANGE) topoChange = true;
  for (size_t i = 0; i < tapTfos_.size(); ++i)                              // tap changers: step on a root
    if (tapTfos_[i]->evalZTap(t, yLocal_, gLocal_, this) == NC_STATE_CHANGE) stateChange = true;
  for (size_t i = 0; i < limitLines_.size(); ++i)                          // current limits: trip on a root
    if (limitLines_[i]->evalZLimit(t, gLocal_, zLocal_, this) == NC_TOPO_CHANGE) topoChange = true;
  for (size_t i = 0; i < reclosingShunts_.size(); ++i)                     // shunts: apply a deferred reclose once the lockout expires
    if (reclosingShunts_[i]->evalZReclose(t, zLocal_) == NC_STATE_CHANGE) stateChange = true;
  if (faultBus_ && faultGOff_ >= 0) {                                       // bus fault: apply/clear shunt-to-ground
    const bool active = (t >= faultTBegin_ && t < faultTEnd_);
    if (active != faultActive_) {
      faultActive_ = active;
      const double gf = (active && faultRfPu_ > 0.) ? 1.0 / faultRfPu_ : 0.;
      for (int k = 0; k < 3; ++k) faultBus_->setFaultG(k, faultedPhase_[k] ? gf : 0.);
      stateChange = true;   // the bus conductance changed -> the Jacobian values change
    }
  }
  if (topoChange) { computeConnectivity(); pendingModeChange_ = true; }    // connectivity + J update
  else if (stateChange) { pendingModeChange_ = true; }                     // J values only
}

void
ModelNetworkEMT::evalG(double t) {
  for (size_t i = 0; i < tapTfos_.size(); ++i) tapTfos_[i]->evalGTap(t, yLocal_, gLocal_);
  for (size_t i = 0; i < limitLines_.size(); ++i) limitLines_[i]->evalGLimit(t, yLocal_, gLocal_);
  for (size_t i = 0; i < reclosingShunts_.size(); ++i) reclosingShunts_[i]->evalGReclose(t, gLocal_);
  if (faultBus_ && faultGOff_ >= 0) {                          // bus fault: cross tBegin then tEnd
    gLocal_[faultGOff_]     = (t - faultTBegin_ >= 0) ? ROOT_UP : ROOT_DOWN;
    gLocal_[faultGOff_ + 1] = (t - faultTEnd_   >= 0) ? ROOT_UP : ROOT_DOWN;
  }
}

modeChangeType_t
ModelNetworkEMT::evalMode(double /*t*/) {
  if (pendingModeChange_) { pendingModeChange_ = false; return ALGEBRAIC_J_UPDATE_MODE; }
  return NO_MODE;
}

void
ModelNetworkEMT::evalStaticYType() {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->evalStaticYType(yType_);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->evalStaticYType(yType_);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->evalStaticYType(yType_);
  // NB: a connection bus marks its own in-block ACPIN FLOW currents ALGEBRAIC inside its evalStaticYType
}

void
ModelNetworkEMT::evalStaticFType() {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->evalStaticFType(fType_);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->evalStaticFType(fType_);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->evalStaticFType(fType_);
}

void
ModelNetworkEMT::evalDynamicYType() {
  // re-evaluated after a topology change: a switched-off bus becomes algebraic
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->evalDynamicYType(zLocal_, yType_);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->evalDynamicYType(zLocal_, yType_);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->evalDynamicYType(zLocal_, yType_);
  // NB: a connection bus marks its own in-block ACPIN FLOW currents ALGEBRAIC inside its evalDynamicYType
}

void
ModelNetworkEMT::evalDynamicFType() {
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->evalDynamicFType(zLocal_, fType_);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->evalDynamicFType(zLocal_, fType_);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->evalDynamicFType(zLocal_, fType_);
}

void
ModelNetworkEMT::defineVariables(vector<shared_ptr<Variable> >& variables) {
  // per-component emission (the abc analogue of DYNModelNetwork::defineVariables looping
  // component->instantiateVariables): each component declares its own variables IN-BLOCK, in the same
  // order as its y / z / calc entries -- so the declared order matches assignOffsets exactly. A
  // connection bus emits its 3 voltage states, then its 3 in-block ACPIN FLOW currents (and their
  // v aliases), then its Urms; the FLOW currents thus sit right after the bus states in y, exactly
  // where assignOffsets placed them (nbY = 6). This mirrors the phasor ModelBus emitting its own
  // FLOW i in instantiateVariables rather than the network appending them at the end.
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->instantiateVariables(variables);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->instantiateVariables(variables);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->instantiateVariables(variables);
  for (size_t i = 0; i < loads_.size(); ++i) loads_[i]->instantiateVariables(variables);
  for (size_t i = 0; i < shunts_.size(); ++i) shunts_[i]->instantiateVariables(variables);
  for (size_t i = 0; i < injectors_.size(); ++i) injectors_[i]->instantiateVariables(variables);
  for (size_t i = 0; i < bridgedLines_.size(); ++i) bridgedLines_[i]->instantiateVariables(variables);
}

void
ModelNetworkEMT::defineElements(vector<Element>& elements, map<string, int>& mapElement) {
  // per-component emission (the abc analogue of DYNModelNetwork::defineElements looping
  // component->defineElements): each component declares its own connectable elements -- the bus its
  // abc voltage terminals and (for a connection bus) its ACPIN, the current-bearing components their
  // <id>_i_{a,b,c} { value } and <id>_state { value }, the injectors/loads/bridged their state. Every
  // element is owned by this single network model, so each gets name()/modelType() as its parent.
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->defineElements(elements, mapElement, name(), modelType());
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->defineElements(elements, mapElement, name(), modelType());
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->defineElements(elements, mapElement, name(), modelType());
  for (size_t i = 0; i < loads_.size(); ++i) loads_[i]->defineElements(elements, mapElement, name(), modelType());
  for (size_t i = 0; i < shunts_.size(); ++i) shunts_[i]->defineElements(elements, mapElement, name(), modelType());
  for (size_t i = 0; i < injectors_.size(); ++i) injectors_[i]->defineElements(elements, mapElement, name(), modelType());
  for (size_t i = 0; i < bridgedLines_.size(); ++i) bridgedLines_[i]->defineElements(elements, mapElement, name(), modelType());
}

void
ModelNetworkEMT::dumpInternalVariables(boost::archive::binary_oarchive& os) const {
  // persist the internal C++ state outside y/yp/z/g (ModelCPP::dumpVariables handles those), in a
  // FIXED component order that loadInternalVariables reads back identically: each bus's accumulated
  // shunt C / shunt G / fault G, then every switchable component's connection flag (and a
  // transformer's tap positions), then the network-level fault-active flag.
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->dumpInternalVariables(os);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->dumpInternalVariables(os);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->dumpInternalVariables(os);
  for (size_t i = 0; i < loads_.size(); ++i) loads_[i]->dumpInternalVariables(os);
  for (size_t i = 0; i < shunts_.size(); ++i) shunts_[i]->dumpInternalVariables(os);
  for (size_t i = 0; i < injectors_.size(); ++i) injectors_[i]->dumpInternalVariables(os);
  for (size_t i = 0; i < bridgedLines_.size(); ++i) bridgedLines_[i]->dumpInternalVariables(os);
  os << faultActive_;
}

void
ModelNetworkEMT::loadInternalVariables(boost::archive::binary_iarchive& is) {
  // read back in the SAME order as dumpInternalVariables, then rebuild the AC islands from the
  // restored connection flags (recomputes each bus's switchedOff_), so the network resumes in the
  // exact discrete/topology state it was checkpointed in.
  for (size_t i = 0; i < buses_.size(); ++i) buses_[i]->loadInternalVariables(is);
  for (size_t i = 0; i < switches_.size(); ++i) switches_[i]->loadInternalVariables(is);
  for (size_t i = 0; i < branches_.size(); ++i) branches_[i]->loadInternalVariables(is);
  for (size_t i = 0; i < loads_.size(); ++i) loads_[i]->loadInternalVariables(is);
  for (size_t i = 0; i < shunts_.size(); ++i) shunts_[i]->loadInternalVariables(is);
  for (size_t i = 0; i < injectors_.size(); ++i) injectors_[i]->loadInternalVariables(is);
  for (size_t i = 0; i < bridgedLines_.size(); ++i) bridgedLines_[i]->loadInternalVariables(is);
  is >> faultActive_;
  computeConnectivity();
}

void
ModelNetworkEMT::collectSilentZ(BitMask* silentZTable) {
  // walk the components in the SAME order their z-offsets were assigned (switches, branches, loads,
  // shunts, injectors, bridged lines), handing each a component-local view of the table. A connection
  // state that appears only in the continuous residual is flagged NotUsedInDiscreteEquations by the
  // component; the switch leaves its z unflagged (its open/closed drives the topology recompute),
  // mirroring the phasor (ModelLine/ModelLoad flag silent, ModelSwitch does not).
  int zoff = 0;
  for (size_t i = 0; i < switches_.size(); ++i) { if (switches_[i]->nbZ() > 0) switches_[i]->collectSilentZ(&silentZTable[zoff]); zoff += switches_[i]->nbZ(); }
  for (size_t i = 0; i < branches_.size(); ++i) { if (branches_[i]->nbZ() > 0) branches_[i]->collectSilentZ(&silentZTable[zoff]); zoff += branches_[i]->nbZ(); }
  for (size_t i = 0; i < loads_.size(); ++i) { if (loads_[i]->nbZ() > 0) loads_[i]->collectSilentZ(&silentZTable[zoff]); zoff += loads_[i]->nbZ(); }
  for (size_t i = 0; i < shunts_.size(); ++i) { if (shunts_[i]->nbZ() > 0) shunts_[i]->collectSilentZ(&silentZTable[zoff]); zoff += shunts_[i]->nbZ(); }
  for (size_t i = 0; i < injectors_.size(); ++i) { if (injectors_[i]->nbZ() > 0) injectors_[i]->collectSilentZ(&silentZTable[zoff]); zoff += injectors_[i]->nbZ(); }
  for (size_t i = 0; i < bridgedLines_.size(); ++i) { if (bridgedLines_[i]->nbZ() > 0) bridgedLines_[i]->collectSilentZ(&silentZTable[zoff]); zoff += bridgedLines_[i]->nbZ(); }
}

void
ModelNetworkEMT::setFequations() {
  // name every residual (for the init-solver diagnostics and dumpInitValues), keyed by the global
  // f-index each component was assigned (fOffset). Every EMT residual is one abc-phase equation;
  // the lists are walked in the same order as assignOffsets so the names line up with f.
  static const char* ph[3] = {"a", "b", "c"};
  for (size_t i = 0; i < buses_.size(); ++i)
    for (int k = 0; k < 3; ++k)
      fEquationIndex_[buses_[i]->fOffset() + k] =
        busIds_[i] + "_phase_" + ph[k] + " : KCL (sum of injected abc currents = (G + Gfault) v + C dv/dt)";
  for (size_t i = 0; i < switches_.size(); ++i)
    for (int k = 0; k < 3; ++k)
      fEquationIndex_[switches_[i]->fOffset() + k] =
        switchIds_[i] + "_phase_" + ph[k] + " : closed -> v1 - v2 = 0 ; open -> i = 0";
  for (size_t i = 0; i < branches_.size(); ++i)
    for (int k = 0; k < 3; ++k)
      fEquationIndex_[branches_[i]->fOffset() + k] =
        branchIds_[i] + "_phase_" + ph[k] + " : branch v-i (v_from - v_to = R i + L di/dt) ; open -> i = 0";
}

void
ModelNetworkEMT::setGequations() {
  // name every root function (tap-changer step, overcurrent limit, bus-fault timing), keyed by the
  // global g-index each owner was assigned (gOffset), in the same order as assignOffsets's g pass.
  for (size_t i = 0; i < tapTfos_.size(); ++i)
    for (int j = 0; j < tapTfos_[i]->nbG(); ++j)
      gEquationIndex_[tapTfos_[i]->gOffset() + j] = tapTfos_[i]->id() + " : tap-changer root " + std::to_string(j);
  for (size_t i = 0; i < limitLines_.size(); ++i)
    for (int j = 0; j < limitLines_[i]->nbG(); ++j)
      gEquationIndex_[limitLines_[i]->gOffset() + j] = limitLines_[i]->id() + " : overcurrent-limit root " + std::to_string(j);
  if (faultBus_ && faultGOff_ >= 0) {
    gEquationIndex_[faultGOff_]     = faultBus_->id() + " : bus-fault begin (t >= tBegin)";
    gEquationIndex_[faultGOff_ + 1] = faultBus_->id() + " : bus-fault end (t >= tEnd)";
  }
}


}  // namespace DYN
