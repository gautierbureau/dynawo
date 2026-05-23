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
 * @file  CRVBinaryCurves.h
 *
 * @brief Streaming binary writer for the full solution vector
 *
 * File layout (all integers little-endian):
 *
 *   Header:
 *     magic    : u8[4]  = {'M','O','D','C'}
 *     version  : u32    = 1
 *     n_vars   : u32    (number of tracked variables, NOT counting time)
 *     for i in [0, n_vars):
 *       name_len : u32
 *       name     : u8[name_len]  (UTF-8, no null terminator)
 *
 *   Records (one per record() call):
 *     time   : f64
 *     values : f64[n_vars]
 */
#ifndef API_CRV_CRVBINARYCURVES_H_
#define API_CRV_CRVBINARYCURVES_H_

#include <cstddef>
#include <fstream>
#include <string>
#include <vector>

namespace curves {

/**
 * @brief Streams the full solution vector to a compact binary file.
 *
 * The output file is opened and the header flushed at construction time; each
 * call to record() appends one time-stamped row. The stream is closed either
 * explicitly via close() or automatically by the destructor.
 */
class BinaryCurves {
 public:
  /**
   * @brief Open @p filename for writing and emit the header.
   * @param filename Output file path.
   * @param names    Variable names in y[] order (size = number of variables).
   * @throws std::runtime_error if the file cannot be opened.
   */
  BinaryCurves(const std::string& filename, const std::vector<std::string>& names);

  ~BinaryCurves();

  BinaryCurves(const BinaryCurves&) = delete;
  BinaryCurves& operator=(const BinaryCurves&) = delete;

  /**
   * @brief Append one record (time + full y vector) to the stream.
   * @param t Current simulation time.
   * @param y Solution vector; must hold at least n_vars entries.
   * @throws std::out_of_range if y.size() < n_vars.
   */
  void record(double t, const std::vector<double>& y);

  /**
   * @brief Flush and close the stream. No-op if already closed.
   */
  void close();

 private:
  void writeHeader(const std::vector<std::string>& names);

  // stream_buf_ MUST be declared before stream_ so it is destroyed last:
  // ~ofstream flushes through this buffer, and using freed memory there
  // silently produces an empty file.
  std::vector<char> stream_buf_;
  std::ofstream stream_;
  std::size_t nVars_;
  std::vector<char> buf_;
};

}  // namespace curves

#endif  // API_CRV_CRVBINARYCURVES_H_
