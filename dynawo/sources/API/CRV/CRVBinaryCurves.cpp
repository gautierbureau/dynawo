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
 * @file  CRVBinaryCurves.cpp
 *
 * @brief Streaming binary writer for the full solution vector
 */
#include "CRVBinaryCurves.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include "DYNCommon.h"

namespace curves {

namespace {

#if defined(_WIN32) || (defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__)
constexpr bool kHostIsLittleEndian = true;
#else
constexpr bool kHostIsLittleEndian = false;
#endif

// Pick an I/O buffer size for the ofstream. The default filebuf is ~8 KB
// and an os.write() that exceeds the buffer is still copied through it
// (libstdc++/libc++), so for large rows we must hand the stream a buffer
// at least one row wide; small rows just want enough headroom to coalesce
// many writes into one syscall.
std::size_t pickStreamBufSize(std::size_t nVars) {
  const std::size_t rowBytes = (1u + nVars) * sizeof(double);
  constexpr std::size_t kMin = 64u * 1024u;         // 64 KiB floor
  constexpr std::size_t kMax = 8u * 1024u * 1024u;  // 8 MiB cap
  const std::size_t want = rowBytes * 16u;          // ~16 rows per syscall
  return std::min(kMax, std::max(kMin, want));
}

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

// Round @p v through DYN::double2String so the encoded value has the same
// decimal precision as the CSV/XML exporters (fixed decimals below 1e6,
// scientific notation above).
double roundToCsvPrecision(double v) {
  return std::stod(DYN::double2String(v));
}

}  // namespace

BinaryCurves::BinaryCurves(const std::string& filename,
                           const std::vector<std::string>& names,
                           Precision precision)
  : stream_buf_(pickStreamBufSize(names.size())),
    stream_(),
    nVars_(names.size()),
    buf_((1 + names.size()) * sizeof(double)),
    precision_(precision) {
  // pubsetbuf only takes effect before the file is opened.
  stream_.rdbuf()->pubsetbuf(stream_buf_.data(), static_cast<std::streamsize>(stream_buf_.size()));
  stream_.open(filename, std::ios::binary);
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

  if (precision_ == Precision::CSV_ROUNDED) {
    const double rt = roundToCsvPrecision(t);
    if (kHostIsLittleEndian) {
      // On little-endian hosts the on-disk layout is byte-identical to the
      // in-memory double, so memcpy the rounded doubles straight into the row
      // buffer and skip the shift/mask encode entirely.
      std::memcpy(buf_.data(), &rt, sizeof(double));
      for (std::size_t i = 0; i < nVars_; ++i) {
        const double v = roundToCsvPrecision(y[i]);
        std::memcpy(buf_.data() + (1 + i) * sizeof(double), &v, sizeof(double));
      }
    } else {
      encode_f64_le(buf_.data(), rt);
      for (std::size_t i = 0; i < nVars_; ++i)
        encode_f64_le(buf_.data() + (1 + i) * sizeof(double), roundToCsvPrecision(y[i]));
    }
  } else {  // Precision::FULL: store raw IEEE-754 bits, no rounding.
    if (kHostIsLittleEndian) {
      std::memcpy(buf_.data(), &t, sizeof(double));
      std::memcpy(buf_.data() + sizeof(double), y.data(), nVars_ * sizeof(double));
    } else {
      encode_f64_le(buf_.data(), t);
      for (std::size_t i = 0; i < nVars_; ++i)
        encode_f64_le(buf_.data() + (1 + i) * sizeof(double), y[i]);
    }
  }
  stream_.write(buf_.data(), static_cast<std::streamsize>(buf_.size()));
}

void BinaryCurves::close() {
  if (stream_.is_open()) {
    stream_.flush();
    stream_.close();
  }
}

void BinaryCurves::writeHeader(const std::vector<std::string>& names) {
  stream_.write("DYNB", 4);
  write_u32_le(stream_, 1u);
  write_u32_le(stream_, static_cast<uint32_t>(names.size()));
  for (const auto& name : names) {
    write_u32_le(stream_, static_cast<uint32_t>(name.size()));
    stream_.write(name.data(), static_cast<std::streamsize>(name.size()));
  }
}

}  // namespace curves
