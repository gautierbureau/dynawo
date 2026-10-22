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
class OutputFiles;
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

/**
 * @brief launch a simulation thanks to a description contains in jobsFileName, on a network given in memory, with input
 * files given in memory and output files written in memory
 *
 * As launchSimu(jobsFileName, data, inputFiles), the logs, timeline, curves, final state values, constraints and lost
 * equipments being written in memory rather than in the directory of the jobs file, where only the other outputs, the
 * dump files of the states for instance, are written. A failed simulation writes there its outputs as far as it went.
 *
 * @param jobsFileName file describing the job(s) to launch
 * @param data data interface of the network to simulate
 * @param inputFiles input files given in memory
 * @param outputFiles output files written in memory
 */
void launchSimu(const std::string& jobsFileName, const boost::shared_ptr<DYN::DataInterface>& data,
    const std::shared_ptr<const DYN::InputFiles>& inputFiles, const std::shared_ptr<DYN::OutputFiles>& outputFiles);

#endif  // SIMULATION_DYNSIMULATIONLAUNCHER_H_
