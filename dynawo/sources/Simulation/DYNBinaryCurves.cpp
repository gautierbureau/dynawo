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
 * @file  DYNBinaryCurves.cpp
 *
 * @brief Streaming binary writer for the full solution vector
 */
#include "DYNBinaryCurves.h"

#include <cstdint>
#include <cstring>
#include <stdexcept>

namespace DYN {

namespace {

void write_u32_le(std::ostream& os, uint32_t v) {
  const unsigned char buf[4] = {
      static_cast<unsigned char>(v & 0xFFu),
      static_cast<unsigned char>((v >> 8) & 0xFFu),
      static_cast<unsigned char>((v >> 16) & 0xFFu),
      static_cast<unsigned char>((v >> 24) & 0xFFu),
  };
  os.write(reinterpret_cast<const char*>(buf), 4);
}

void encode_f64_le(char* out, double v) {
  static_assert(sizeof(double) == 8, "double must be 8 bytes");
  uint64_t bits;
  std::memcpy(&bits, &v, 8);
  for (int i = 0; i < 8; ++i)
    out[i] = static_cast<char>((bits >> (8 * i)) & 0xFFu);
}

}  // namespace

BinaryCurves::BinaryCurves(const std::string& filename, const std::vector<std::string>& names)
  : stream_(filename, std::ios::binary),
    nVars_(names.size()),
    buf_((1 + names.size()) * sizeof(double)) {
  if (!stream_)
    throw std::runtime_error("BinaryCurves: cannot open '" + filename + "' for writing");
  writeHeader(names);
}

BinaryCurves::~BinaryCurves() {
  // Destructors must not throw; swallow any flush error.
  try {
    if (stream_.is_open())
      stream_.flush();
  } catch (...) {
    // Intentionally ignored.
  }
}

void BinaryCurves::record(double t, const std::vector<double>& y) {
  if (y.size() < nVars_)
    throw std::out_of_range("BinaryCurves::record: y vector smaller than registered variables");

  encode_f64_le(buf_.data(), t);
  for (std::size_t i = 0; i < nVars_; ++i)
    encode_f64_le(buf_.data() + (1 + i) * sizeof(double), y[i]);
  stream_.write(buf_.data(), static_cast<std::streamsize>(buf_.size()));
}

void BinaryCurves::close() {
  if (stream_.is_open()) {
    stream_.flush();
    stream_.close();
  }
}

void BinaryCurves::writeHeader(const std::vector<std::string>& names) {
  stream_.write("MODC", 4);
  write_u32_le(stream_, 1u);
  write_u32_le(stream_, static_cast<uint32_t>(names.size()));
  for (const auto& name : names) {
    write_u32_le(stream_, static_cast<uint32_t>(name.size()));
    stream_.write(name.data(), static_cast<std::streamsize>(name.size()));
  }
}

}  // namespace DYN
