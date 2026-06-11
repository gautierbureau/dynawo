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
 * @file API/CRV/test/TestBinaryCurves.cpp
 * @brief Unit tests for the BinaryCurves streaming writer
 */

#include "gtest_dynawo.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "CRVBinaryCurves.h"
#include "DYNCommon.h"

namespace curves {

namespace {

// Mirrors the anonymous-namespace helper inside CRVBinaryCurves.cpp so tests
// can compute the exact value the writer stores after CSV-precision rounding.
double expectedRounded(double v) {
  return std::stod(DYN::double2String(v));
}

std::vector<unsigned char> readFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream oss;
  oss << in.rdbuf();
  const std::string s = oss.str();
  return std::vector<unsigned char>(s.begin(), s.end());
}

uint32_t readU32LE(const unsigned char* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

double readF64LE(const unsigned char* p) {
  uint64_t bits = 0;
  for (int i = 0; i < 8; ++i)
    bits |= static_cast<uint64_t>(p[i]) << (8 * i);
  double v;
  std::memcpy(&v, &bits, sizeof(double));
  return v;
}

// Walks a file buffer and decodes header + records. Returns the offset
// immediately after the header; fills @p nVars and @p namesOut.
std::size_t parseHeader(const std::vector<unsigned char>& buf,
                        uint32_t* nVarsOut,
                        std::vector<std::string>* namesOut) {
  EXPECT_GE(buf.size(), 12u);
  EXPECT_EQ(buf[0], 'D');
  EXPECT_EQ(buf[1], 'Y');
  EXPECT_EQ(buf[2], 'N');
  EXPECT_EQ(buf[3], 'B');
  EXPECT_EQ(readU32LE(&buf[4]), 1u);
  const uint32_t n = readU32LE(&buf[8]);
  *nVarsOut = n;
  std::size_t off = 12;
  for (uint32_t i = 0; i < n; ++i) {
    EXPECT_LE(off + 4, buf.size());
    const uint32_t len = readU32LE(&buf[off]);
    off += 4;
    EXPECT_LE(off + len, buf.size());
    namesOut->emplace_back(reinterpret_cast<const char*>(&buf[off]), len);
    off += len;
  }
  return off;
}

}  // namespace

//-----------------------------------------------------
// Header is emitted on construction with the documented layout.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesHeaderOnly) {
  const std::string path = "TestBinaryCurves_headerOnly.bin";
  std::vector<std::string> names = {"alpha", "beta", "gamma_long_name"};
  { BinaryCurves bc(path, names); }  // destructor flushes and closes

  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  const std::size_t off = parseHeader(buf, &n, &got);

  ASSERT_EQ(n, names.size());
  ASSERT_EQ(got, names);
  ASSERT_EQ(off, buf.size());  // no records yet
  std::remove(path.c_str());
}

//-----------------------------------------------------
// Zero variables: header is well-formed, records contain time only.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesEmptyVarList) {
  const std::string path = "TestBinaryCurves_empty.bin";
  std::vector<std::string> names;
  {
    BinaryCurves bc(path, names);
    bc.record(0.5, std::vector<double>());
    bc.record(1.5, std::vector<double>());
  }

  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  std::size_t off = parseHeader(buf, &n, &got);
  ASSERT_EQ(n, 0u);
  ASSERT_TRUE(got.empty());
  ASSERT_EQ(buf.size() - off, 2u * sizeof(double));
  ASSERT_DOUBLE_EQ(readF64LE(&buf[off]), expectedRounded(0.5));
  ASSERT_DOUBLE_EQ(readF64LE(&buf[off + 8]), expectedRounded(1.5));
  std::remove(path.c_str());
}

//-----------------------------------------------------
// Each record() appends time + n_vars little-endian doubles in order.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesRecordLayout) {
  const std::string path = "TestBinaryCurves_records.bin";
  std::vector<std::string> names = {"v0", "v1", "v2"};
  const std::vector<std::vector<double>> rows = {
      {1.0,  2.0,  3.0},
      {0.0, -1.5,  4.25},
      {1234.5, -9.875, 1e-3},
  };
  const std::vector<double> times = {0.0, 0.1, 0.25};

  {
    BinaryCurves bc(path, names);
    for (std::size_t i = 0; i < rows.size(); ++i)
      bc.record(times[i], rows[i]);
  }

  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  std::size_t off = parseHeader(buf, &n, &got);
  ASSERT_EQ(n, names.size());

  const std::size_t rowBytes = (1 + names.size()) * sizeof(double);
  ASSERT_EQ(buf.size() - off, rows.size() * rowBytes);

  for (std::size_t r = 0; r < rows.size(); ++r) {
    ASSERT_DOUBLE_EQ(readF64LE(&buf[off]), expectedRounded(times[r]));
    for (std::size_t i = 0; i < names.size(); ++i)
      ASSERT_DOUBLE_EQ(readF64LE(&buf[off + (1 + i) * sizeof(double)]),
                       expectedRounded(rows[r][i]));
    off += rowBytes;
  }
  std::remove(path.c_str());
}

//-----------------------------------------------------
// y vectors larger than n_vars are tolerated; only the first n_vars are
// written so the row stride stays constant.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesAcceptsLargerYVector) {
  const std::string path = "TestBinaryCurves_largerY.bin";
  std::vector<std::string> names = {"a", "b"};
  std::vector<double> y = {10.0, 20.0, 99.0, -1.0};
  {
    BinaryCurves bc(path, names);
    bc.record(0.0, y);
  }
  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  const std::size_t off = parseHeader(buf, &n, &got);
  ASSERT_EQ(buf.size() - off, 3u * sizeof(double));  // time + 2 vars only
  ASSERT_DOUBLE_EQ(readF64LE(&buf[off + 1 * sizeof(double)]), expectedRounded(10.0));
  ASSERT_DOUBLE_EQ(readF64LE(&buf[off + 2 * sizeof(double)]), expectedRounded(20.0));
  std::remove(path.c_str());
}

//-----------------------------------------------------
// y smaller than n_vars throws.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesRejectsSmallerYVector) {
  const std::string path = "TestBinaryCurves_smallY.bin";
  std::vector<std::string> names = {"a", "b", "c"};
  BinaryCurves bc(path, names);
  std::vector<double> y = {1.0, 2.0};
  ASSERT_THROW(bc.record(0.0, y), std::out_of_range);
  std::remove(path.c_str());
}

//-----------------------------------------------------
// Opening an unwritable path raises runtime_error.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesUnwritablePath) {
  std::vector<std::string> names = {"a"};
  // A path under a non-existent directory cannot be created by ofstream.
  ASSERT_THROW(BinaryCurves("/no/such/dir/binary.bin", names), std::runtime_error);
}

//-----------------------------------------------------
// close() is idempotent and the file is flushed once closed.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesCloseIsIdempotent) {
  const std::string path = "TestBinaryCurves_close.bin";
  std::vector<std::string> names = {"x"};
  BinaryCurves bc(path, names);
  bc.record(0.0, std::vector<double>{1.0});
  bc.close();
  bc.close();  // must not throw or crash

  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  const std::size_t off = parseHeader(buf, &n, &got);
  ASSERT_EQ(buf.size() - off, 2u * sizeof(double));
  std::remove(path.c_str());
}

//-----------------------------------------------------
// CSV-precision rounding: a value with more decimals than the exporter
// precision is stored rounded (matches CSV/XML formatting).
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesAppliesCsvRounding) {
  const std::string path = "TestBinaryCurves_round.bin";
  std::vector<std::string> names = {"x"};
  const double raw = 1234.123456789;
  {
    BinaryCurves bc(path, names);
    bc.record(0.0, std::vector<double>{raw});
  }
  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  const std::size_t off = parseHeader(buf, &n, &got);
  const double stored = readF64LE(&buf[off + sizeof(double)]);
  ASSERT_DOUBLE_EQ(stored, expectedRounded(raw));
  // Sanity: the rounding must actually change the value here.
  ASSERT_NE(stored, raw);
  std::remove(path.c_str());
}

//-----------------------------------------------------
// Large workload: stride is exact, last record round-trips. Exercises the
// 4 MiB I/O buffer path with many records spanning multiple buffer flushes.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesLargeStreamStride) {
  const std::string path = "TestBinaryCurves_large.bin";
  const std::size_t kVars = 200;
  const std::size_t kSteps = 5000;
  std::vector<std::string> names;
  names.reserve(kVars);
  for (std::size_t i = 0; i < kVars; ++i)
    names.emplace_back("var_" + std::to_string(i));
  std::vector<double> y(kVars);
  for (std::size_t i = 0; i < kVars; ++i)
    y[i] = static_cast<double>(i) * 0.5;

  {
    BinaryCurves bc(path, names);
    for (std::size_t s = 0; s < kSteps; ++s)
      bc.record(static_cast<double>(s) * 0.01, y);
  }

  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  const std::size_t off = parseHeader(buf, &n, &got);
  ASSERT_EQ(n, kVars);

  const std::size_t rowBytes = (1 + kVars) * sizeof(double);
  ASSERT_EQ(buf.size() - off, kSteps * rowBytes);

  // Spot-check the first, middle, and last records to confirm the stride
  // and that the I/O buffer flushes did not drop or reorder bytes.
  const std::size_t indices[] = {0, kSteps / 2, kSteps - 1};
  for (std::size_t idx : indices) {
    const std::size_t base = off + idx * rowBytes;
    ASSERT_DOUBLE_EQ(readF64LE(&buf[base]),
                     expectedRounded(static_cast<double>(idx) * 0.01));
    for (std::size_t i = 0; i < kVars; ++i)
      ASSERT_DOUBLE_EQ(readF64LE(&buf[base + (1 + i) * sizeof(double)]),
                       expectedRounded(y[i]));
  }
  std::remove(path.c_str());
}

//-----------------------------------------------------
// Precision::FULL stores raw IEEE-754 bits — no rounding, lossless.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesFullPrecisionLossless) {
  const std::string path = "TestBinaryCurves_full.bin";
  std::vector<std::string> names = {"x", "y"};
  // Values that demonstrably do NOT survive the CSV round-trip.
  const double t = 0.123456789012345;
  const std::vector<double> row = {1234.123456789012, 9.876543210987e-7};
  {
    BinaryCurves bc(path, names, Precision::FULL);
    bc.record(t, row);
  }
  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  const std::size_t off = parseHeader(buf, &n, &got);
  // Exact bit-for-bit equality, not ASSERT_DOUBLE_EQ (which allows 4 ULPs).
  ASSERT_EQ(readF64LE(&buf[off]), t);
  ASSERT_EQ(readF64LE(&buf[off + sizeof(double)]), row[0]);
  ASSERT_EQ(readF64LE(&buf[off + 2 * sizeof(double)]), row[1]);
  std::remove(path.c_str());
}

//-----------------------------------------------------
// CSV_ROUNDED and FULL produce different bytes for a value that the CSV
// rounding would change, and agree on a value that survives the round-trip.
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesPrecisionModesDiffer) {
  std::vector<std::string> names = {"x"};
  const double raw = 1234.123456789;  // changed by CSV rounding
  const double safe = 0.5;             // exactly representable, survives rounding

  const std::string pCsv = "TestBinaryCurves_csvmode.bin";
  const std::string pFull = "TestBinaryCurves_fullmode.bin";
  {
    BinaryCurves bc(pCsv, names, Precision::CSV_ROUNDED);
    bc.record(safe, std::vector<double>{raw});
    BinaryCurves bf(pFull, names, Precision::FULL);
    bf.record(safe, std::vector<double>{raw});
  }
  const auto csv = readFile(pCsv);
  const auto full = readFile(pFull);
  ASSERT_EQ(csv.size(), full.size());
  uint32_t n = 0;
  std::vector<std::string> got;
  const std::size_t off = parseHeader(csv, &n, &got);

  // Time field is identical (safe value).
  ASSERT_EQ(readF64LE(&csv[off]), readF64LE(&full[off]));
  ASSERT_EQ(readF64LE(&csv[off]), safe);

  // Value field differs because CSV rounding actually changes raw.
  const double csvVal = readF64LE(&csv[off + sizeof(double)]);
  const double fullVal = readF64LE(&full[off + sizeof(double)]);
  ASSERT_EQ(fullVal, raw);
  ASSERT_NE(csvVal, raw);
  ASSERT_NE(csvVal, fullVal);
  std::remove(pCsv.c_str());
  std::remove(pFull.c_str());
}

//-----------------------------------------------------
// CSV_ROUNDED is the default (preserves existing behavior at call sites
// that haven't been updated).
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesCsvRoundedIsDefault) {
  std::vector<std::string> names = {"x"};
  const double raw = 1234.123456789;

  const std::string pDef = "TestBinaryCurves_default.bin";
  const std::string pExp = "TestBinaryCurves_explicit.bin";
  {
    BinaryCurves bc(pDef, names);  // no precision arg
    bc.record(0.0, std::vector<double>{raw});
    BinaryCurves bx(pExp, names, Precision::CSV_ROUNDED);
    bx.record(0.0, std::vector<double>{raw});
  }
  const auto a = readFile(pDef);
  const auto b = readFile(pExp);
  ASSERT_EQ(a, b);
  std::remove(pDef.c_str());
  std::remove(pExp.c_str());
}

//-----------------------------------------------------
// Variable names with zero length are encoded as just a u32(0).
//-----------------------------------------------------
TEST(APICRVTest, BinaryCurvesEmptyNameString) {
  const std::string path = "TestBinaryCurves_emptyName.bin";
  std::vector<std::string> names = {"", "tail"};
  { BinaryCurves bc(path, names); }
  const auto buf = readFile(path);
  uint32_t n = 0;
  std::vector<std::string> got;
  parseHeader(buf, &n, &got);
  ASSERT_EQ(n, 2u);
  ASSERT_EQ(got[0], "");
  ASSERT_EQ(got[1], "tail");
  std::remove(path.c_str());
}

}  // namespace curves
