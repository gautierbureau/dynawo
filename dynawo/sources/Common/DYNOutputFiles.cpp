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
 * @file  DYNOutputFiles.cpp
 *
 * @brief Output files of a simulation kept in memory: implementation
 *
 */
#include "DYNOutputFiles.h"

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace DYN {

std::shared_ptr<std::ostringstream>
OutputFiles::open(const std::string& path) {
  std::shared_ptr<std::ostringstream>& stream = streams_[fs::absolute(fs::path(path)).lexically_normal().string()];
  if (stream)
    stream->str(std::string());
  else
    stream = std::make_shared<std::ostringstream>();
  return stream;
}

std::vector<std::pair<std::string, std::string> >
OutputFiles::getFiles() const {
  std::vector<std::pair<std::string, std::string> > files;
  for (const auto& stream : streams_)
    files.emplace_back(stream.first, stream.second->str());
  return files;
}

}  // namespace DYN
