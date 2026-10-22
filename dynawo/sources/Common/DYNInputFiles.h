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
 * @file  DYNInputFiles.h
 *
 * @brief Input files of a simulation given in memory: header
 *
 */
#ifndef COMMON_DYNINPUTFILES_H_
#define COMMON_DYNINPUTFILES_H_

#include <istream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include "DYNError.h"
#include "DYNMacrosMessage.h"

namespace DYN {

/**
 * @class InputFiles
 * @brief Input files of a simulation given in memory, by the paths the simulation reads them from
 *
 * A file given here is read from its content, without being looked for on the file system: the jobs file, and the dyd,
 * par, curves, final state values and criteria files of a simulation run by a caller that holds all its inputs, such
 * as a JVM. The other files are read from the file system.
 */
class InputFiles {
 public:
  /**
   * @brief add a file
   * @param path path of the file, relative to the current directory if not absolute
   * @param content content of the file
   */
  void addFile(const std::string& path, const std::string& content);

  /**
   * @brief whether a file is given here
   * @param path path of the file, relative to the current directory if not absolute
   * @return @b true if the file is given here
   */
  bool contains(const std::string& path) const;

  /**
   * @brief whether a file exists, given here or on the file system
   * @param path path of the file
   * @return @b true if the file exists
   */
  bool exists(const std::string& path) const;

  /**
   * @brief the canonical path of a file: normalized for a file given here, that need not exist on the file system, as
   * canonical() for another one
   * @param path path of the file, relative to the root path if not absolute
   * @param rootPath root path
   * @return the canonical path of the file
   */
  std::string canonical(const std::string& path, const std::string& rootPath) const;

  /**
   * @brief open a file: its content if it is given here, the file of the file system otherwise
   * @param path path of the file
   * @return the stream of the file, in a failed state if the file cannot be opened
   */
  std::unique_ptr<std::istream> open(const std::string& path) const;

  /**
   * @brief import a file with an XML importer: from its content if it is given here, from the file system otherwise
   * @param importer importer of the file, with an importFromFile and an importFromStream method
   * @param path path of the file
   * @return what the importer imports from the file
   */
  template <typename Importer>
  auto importFile(const Importer& importer, const std::string& path) const -> decltype(importer.importFromFile(path)) {
    const auto content = contents_.find(normalize(path));
    if (content == contents_.end())
      return importer.importFromFile(path);

    std::istringstream stream(content->second);
    try {
      return importer.importFromStream(stream);
    } catch (const Error& exp) {
      throw DYNError(Error::API, XmlFileParsingError, path, exp.what());
    }
  }

 private:
  /**
   * @brief the path a file is given by here: absolute and normalized
   * @param path path of the file
   * @return the absolute and normalized path of the file
   */
  static std::string normalize(const std::string& path);

  std::unordered_map<std::string, std::string> contents_;  ///< contents of the files, by their normalized path
};

}  // namespace DYN

#endif  // COMMON_DYNINPUTFILES_H_
