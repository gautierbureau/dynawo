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
 * @file  DYNSimulationLauncher.cpp
 *
 * @brief SimulationLauncher implementation
 *
 */
#include <xml/sax/parser/ParserFactory.h>

#include "DYNSimulationLauncher.h"
#include "DYNMacrosMessage.h"
#include "DYNTrace.h"
#include "DYNSimulation.h"
#include "DYNSimulationRT.h"
#include "DYNSimulationContext.h"
#include "DYNFileSystemUtils.h"
#include "DYNTimer.h"
#include "DYNExecUtils.h"
#include "JOBXmlImporter.h"
#include "JOBJobsCollection.h"
#include "JOBJobEntry.h"
#include "JOBOutputsEntry.h"

#include <string>
#include <memory>

namespace parser = xml::sax::parser;

using DYN::Trace;
using DYN::Simulation;
using DYN::SimulationRT;
using DYN::SimulationContext;

// If logging is disabled, Trace::info has no effect so we also print on standard output to have basic information
template<class T>
static void print(const T& output, const DYN::SeverityLevel level = DYN::INFO) {
  DYN::TraceStream ss;
  switch (level) {
    case DYN::DEBUG:
      ss = Trace::debug();
      break;
    case DYN::INFO:
      ss = Trace::info();
      break;
    case DYN::WARN:
      ss = Trace::warn();
      break;
    case DYN::ERROR:
      ss = Trace::error();
      break;
    default:
      // impossible case by definition of the enum
      return;
  }
  ss << output << Trace::endline;
  if (!Trace::isLoggingEnabled()) {
    std::clog << output << std::endl;
  }
}

/**
 * @brief terminate a simulation that failed
 * @param simulation simulation that failed
 * @param data data interface of the network given by the caller, if any: a failed simulation leaves that network as it
 * was, without writing its final state, an IIDM file of it or the lost equipments, that would write into it
 */
static void terminateFailed(const std::shared_ptr<Simulation>& simulation, const boost::shared_ptr<DYN::DataInterface>& data) {
  if (data) {
    simulation->disableExportIIDM();
    simulation->setExportFinalStateToNetwork(false);
    simulation->setLostEquipmentsExportMode(Simulation::EXPORT_LOSTEQUIPMENTS_NONE);
  }
  simulation->terminate();
}

/**
 * @brief launch the job(s) of a jobs file
 * @param jobsFileName file describing the job(s) to launch
 * @param isInteractive true if simulation in interactive or real-time mode
 * @param data data interface of the network to simulate, null to read the iidm file of the job(s)
 * @param inputFiles input files given in memory: the jobs file and the files it refers to that are read from their
 * content rather than from the file system
 */
static void launchJobs(const std::string& jobsFileName, bool isInteractive, const boost::shared_ptr<DYN::DataInterface>& data,
    const std::shared_ptr<const DYN::InputFiles>& inputFiles) {
#if defined(_DEBUG_) || defined(PRINT_TIMERS)
  DYN::Timer timer("Main::LaunchSimu");
#endif

  job::XmlImporter importer;
  const std::shared_ptr<job::JobsCollection>& jobsCollection = inputFiles->importFile(importer, jobsFileName);
  const std::string prefixJobFile = absolute(removeFileName(jobsFileName));
  if (jobsCollection->getJobs().empty())
    throw DYNError(DYN::Error::SIMULATION, NoJobDefined);
  Trace::init();

  for (const auto& job : jobsCollection->getJobs()) {
    print(DYNLog(LaunchingJob, job->getName()));

    const auto context = std::make_shared<SimulationContext>();
    context->setResourcesDirectory(getMandatoryEnvVar("DYNAWO_RESOURCES_DIR"));
    context->setLocale(getMandatoryEnvVar("DYNAWO_LOCALE"));
    context->setInputDirectory(prefixJobFile);
    context->setWorkingDirectory(prefixJobFile);
    context->setInputFiles(inputFiles);

    std::shared_ptr<Simulation> simulation;
    try {
      if (isInteractive)
        simulation = std::unique_ptr<SimulationRT>(new SimulationRT(job, context, data));
      else
        simulation = std::unique_ptr<Simulation>(new Simulation(job, context, data));
      if (data)
        simulation->setExportFinalStateToNetwork(true);

      simulation->init();
    } catch (const DYN::Error& err) {
      print(err.what(), DYN::ERROR);
      throw;
    } catch (const DYN::MessageError& e) {
      print(e.what(), DYN::ERROR);
      throw;
    } catch (const char* s) {
      print(s, DYN::ERROR);
      throw;
    } catch (const std::string& Msg) {
      print(Msg, DYN::ERROR);
      throw;
    } catch (const std::exception& exc) {
      print(exc.what(), DYN::ERROR);
      throw;
    }

    try {
      simulation->simulate();
      simulation->terminate();
    } catch (const DYN::Error& err) {
      // Needed as otherwise terminate might crash due to missing staticRef variables
      if (err.key() == DYN::KeyError_t::StateVariableNoReference) {
        simulation->disableExportIIDM();
        simulation->setLostEquipmentsExportMode(Simulation::EXPORT_LOSTEQUIPMENTS_NONE);
      }
      print(err.what(), DYN::ERROR);
      terminateFailed(simulation, data);
      throw;
    } catch (const DYN::Terminate& e) {
      print(e.what(), DYN::ERROR);
      terminateFailed(simulation, data);
      throw;
    } catch (const DYN::MessageError& e) {
      print(e.what(), DYN::ERROR);
      terminateFailed(simulation, data);
      throw;
    } catch (const char* s) {
      print(s, DYN::ERROR);
      terminateFailed(simulation, data);
      throw;
    } catch (const std::string& Msg) {
      print(Msg, DYN::ERROR);
      terminateFailed(simulation, data);
      throw;
    } catch (const std::exception& exc) {
      print(exc.what(), DYN::ERROR);
      terminateFailed(simulation, data);
      throw;
    }
    simulation->clean();
    print(DYNLog(EndOfJob, job->getName()));
    Trace::resetCustomAppenders();
    Trace::init();
    print(DYNLog(JobSuccess, job->getName()));
    if (job->getOutputsEntry()) {
      std::string outputsDirectory = createAbsolutePath(job->getOutputsEntry()->getOutputsDirectory(), context->getWorkingDirectory());
      print(DYNLog(ResultFolder, outputsDirectory));
    }
  }
}

void launchSimu(const std::string& jobsFileName, bool isInteractive) {
  launchJobs(jobsFileName, isInteractive, boost::shared_ptr<DYN::DataInterface>(), std::make_shared<DYN::InputFiles>());
}

void launchSimu(const std::string& jobsFileName, const boost::shared_ptr<DYN::DataInterface>& data) {
  launchJobs(jobsFileName, false, data, std::make_shared<DYN::InputFiles>());
}

void launchSimu(const std::string& jobsFileName, const boost::shared_ptr<DYN::DataInterface>& data,
    const std::shared_ptr<const DYN::InputFiles>& inputFiles) {
  launchJobs(jobsFileName, false, data, inputFiles ? inputFiles : std::make_shared<DYN::InputFiles>());
}
