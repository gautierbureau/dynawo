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
 * @file  DYNInputFiles.cpp
 *
 * @brief Input files of a simulation given in memory: implementation
 *
 */
#include "DYNInputFiles.h"

#include <fstream>

#include <boost/filesystem.hpp>

#include "DYNFileSystemUtils.h"

namespace fs = boost::filesystem;

namespace DYN {

void
InputFiles::addFile(const std::string& path, const std::string& content) {
  contents_[normalize(path)] = content;
}

bool
InputFiles::contains(const std::string& path) const {
  return contents_.find(normalize(path)) != contents_.end();
}

bool
InputFiles::exists(const std::string& path) const {
  return contains(path) || ::exists(path);
}

std::string
InputFiles::canonical(const std::string& path, const std::string& rootPath) const {
  const std::string absolutePath = fs::absolute(fs::path(path), fs::path(rootPath)).string();
  if (contains(absolutePath))
    return normalize(absolutePath);
  return ::canonical(path, rootPath);
}

std::unique_ptr<std::istream>
InputFiles::open(const std::string& path) const {
  const auto content = contents_.find(normalize(path));
  if (content != contents_.end())
    return std::unique_ptr<std::istream>(new std::istringstream(content->second));
  return std::unique_ptr<std::istream>(new std::ifstream(path.c_str()));
}

std::string
InputFiles::normalize(const std::string& path) {
  return fs::absolute(fs::path(path)).lexically_normal().string();
}

}  // namespace DYN
