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
 * @file  DYNSimulationLauncher.h
 *
 * @brief function to launch a Dynawo simulation
 *
 */

#ifndef SIMULATION_DYNSIMULATIONLAUNCHER_H_
#define SIMULATION_DYNSIMULATIONLAUNCHER_H_

#include <memory>
#include <string>

#include <boost/shared_ptr.hpp>

namespace DYN {
class DataInterface;
class InputFiles;
}

/**
 * @brief launch a simulation thanks to a description contains in jobsFileName
 *
 *
 * @param jobsFileName file describing the job(s) to launch
 * @param isInteractive true if simulation in interactive or real-time mode
 */
void launchSimu(const std::string& jobsFileName, bool isInteractive = false);

/**
 * @brief launch a simulation thanks to a description contains in jobsFileName, on a network given in memory instead of
 * the iidm file of the job(s)
 *
 * The final values of the state variables are written into that network, as for the IIDM file of the final state
 * but without writing it. A failed simulation leaves the network as it was.
 *
 * @param jobsFileName file describing the job(s) to launch
 * @param data data interface of the network to simulate
 */
void launchSimu(const std::string& jobsFileName, const boost::shared_ptr<DYN::DataInterface>& data);

/**
 * @brief launch a simulation thanks to a description contains in jobsFileName, on a network given in memory, with input
 * files given in memory
 *
 * As launchSimu(jobsFileName, data), the input files given in memory (the jobs file, and the dyd, par, curves, final
 * state values and criteria files it refers to) being read from their content rather than from the file system, where
 * they need not be. The outputs are written in the directory of the jobs file.
 *
 * @param jobsFileName file describing the job(s) to launch
 * @param data data interface of the network to simulate
 * @param inputFiles input files given in memory
 */
void launchSimu(const std::string& jobsFileName, const boost::shared_ptr<DYN::DataInterface>& data,
    const std::shared_ptr<const DYN::InputFiles>& inputFiles);

#endif  // SIMULATION_DYNSIMULATIONLAUNCHER_H_
