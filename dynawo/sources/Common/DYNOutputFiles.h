//
// Copyright (c) 2026, RTE (http://www.rte-france.com)
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
 * @file  DYNOutputFiles.h
 *
 * @brief Output files of a simulation kept in memory: header
 *
 */
#ifndef COMMON_DYNOUTPUTFILES_H_
#define COMMON_DYNOUTPUTFILES_H_

#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace DYN {

/**
 * @class OutputFiles
 * @brief Output files of a simulation kept in memory, by the paths the simulation would write them to
 *
 * The logs, timeline, curves, final state values, constraints and lost equipments of a simulation run by a caller that
 * reads its outputs from memory, such as a JVM, are written there rather than on the file system. The other outputs,
 * the dump of a state for instance, are written on the file system.
 */
class OutputFiles {
 public:
  /**
   * @brief open an output file, emptied as a file opened for writing: the writers of a same path share its stream
   * @param path path of the file, relative to the current directory if not absolute
   * @return the stream the file is written to
   */
  std::shared_ptr<std::ostringstream> open(const std::string& path);

  /**
   * @brief the output files
   * @return the output files, by their absolute and normalized paths, with their contents
   */
  std::vector<std::pair<std::string, std::string> > getFiles() const;

 private:
  std::map<std::string, std::shared_ptr<std::ostringstream> > streams_;  ///< streams of the files, by their normalized path
};

}  // namespace DYN

#endif  // COMMON_DYNOUTPUTFILES_H_
