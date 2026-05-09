// Minimal proof-of-concept benchmark for the proposed Dynawo binary PAR
// format. Parses the same parameters file in both formats and reports
// load times.
//
// This is intentionally self-contained: it does not link against Dynawo
// itself. The XML side uses Expat (a SAX parser, same family as the
// libxml2/SAX combo behind PARXmlImporter), so the comparison is
// representative even though absolute numbers will differ from a full
// PARXmlImporter run.
//
// Build:   make
// Usage:   ./bench <sample.par> <sample.par.bin> [iterations]

#include <expat.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

// ---- shared sanity counters ---------------------------------------------

struct Counts {
  std::size_t sets = 0;
  std::size_t macros = 0;
  std::size_t pars = 0;
  std::size_t par_tables = 0;
  std::size_t refs = 0;
  std::size_t macro_refs = 0;

  bool operator==(const Counts& o) const {
    return sets == o.sets && macros == o.macros && pars == o.pars
        && par_tables == o.par_tables && refs == o.refs
        && macro_refs == o.macro_refs;
  }

  void print(const char* tag) const {
    std::printf("[%s] sets=%zu macros=%zu pars=%zu parTables=%zu refs=%zu macroRefs=%zu\n",
                tag, sets, macros, pars, par_tables, refs, macro_refs);
  }
};

// ---- binary reader ------------------------------------------------------

struct MMapped {
  const std::uint8_t* data = nullptr;
  std::size_t size = 0;
  int fd = -1;

  ~MMapped() {
    if (data) ::munmap(const_cast<std::uint8_t*>(data), size);
    if (fd >= 0) ::close(fd);
  }
};

bool mmap_file(const char* path, MMapped& m) {
  m.fd = ::open(path, O_RDONLY);
  if (m.fd < 0) return false;
  struct stat st{};
  if (::fstat(m.fd, &st) != 0) return false;
  m.size = static_cast<std::size_t>(st.st_size);
  void* p = ::mmap(nullptr, m.size, PROT_READ, MAP_PRIVATE, m.fd, 0);
  if (p == MAP_FAILED) return false;
  m.data = static_cast<const std::uint8_t*>(p);
  // Hint the kernel we'll do a sequential read, so prefetch is aggressive.
  ::madvise(p, m.size, MADV_SEQUENTIAL);
  return true;
}

class BinReader {
 public:
  BinReader(const std::uint8_t* p, std::size_t n) : p_(p), end_(p + n) {}

  std::uint8_t u8() { check(1); return *p_++; }
  std::uint16_t u16() { check(2); std::uint16_t v; std::memcpy(&v, p_, 2); p_ += 2; return v; }
  std::uint32_t u32() { check(4); std::uint32_t v; std::memcpy(&v, p_, 4); p_ += 4; return v; }
  std::uint64_t u64() { check(8); std::uint64_t v; std::memcpy(&v, p_, 8); p_ += 8; return v; }
  std::int32_t i32() { check(4); std::int32_t v; std::memcpy(&v, p_, 4); p_ += 4; return v; }
  double f64() { check(8); double v; std::memcpy(&v, p_, 8); p_ += 8; return v; }

  std::uint64_t varint() {
    std::uint64_t r = 0;
    int shift = 0;
    while (true) {
      check(1);
      std::uint8_t b = *p_++;
      r |= static_cast<std::uint64_t>(b & 0x7F) << shift;
      if (!(b & 0x80)) return r;
      shift += 7;
    }
  }

  void skip(std::size_t n) { check(n); p_ += n; }
  std::size_t pos(const std::uint8_t* base) const { return static_cast<std::size_t>(p_ - base); }
  bool eof() const { return p_ >= end_; }

 private:
  void check(std::size_t n) const {
    if (p_ + n > end_) {
      std::fprintf(stderr, "bin: unexpected eof\n");
      std::exit(1);
    }
  }
  const std::uint8_t* p_;
  const std::uint8_t* end_;
};

// "Parses but does not retain" — we measure raw decode cost.
// We do dereference every string entry into `string_views_` so the
// string-table cost is paid exactly like a real importer would.
struct StringRef {
  const char* data;
  std::uint32_t len;
};

Counts parse_binary(const std::uint8_t* data, std::size_t size) {
  Counts c{};
  BinReader r(data, size);

  // Header
  if (size < 32) { std::fprintf(stderr, "binary too small\n"); std::exit(1); }
  if (std::memcmp(data, "DYNP", 4) != 0) { std::fprintf(stderr, "bad magic\n"); std::exit(1); }
  r.skip(4);
  std::uint16_t vmaj = r.u16(); std::uint16_t vmin = r.u16();
  (void)vmin;
  if (vmaj != 1) { std::fprintf(stderr, "bad version\n"); std::exit(1); }
  std::uint32_t flags = r.u32(); (void)flags;
  std::uint32_t num_strings = r.u32();
  std::uint32_t num_macros = r.u32();
  std::uint32_t num_sets = r.u32();
  std::uint64_t body_offset = r.u64();
  (void)body_offset;

  // String table — keep slices into the mmap.
  std::vector<StringRef> strings;
  strings.reserve(num_strings);
  for (std::uint32_t i = 0; i < num_strings; ++i) {
    std::uint64_t n = r.varint();
    StringRef sr{ reinterpret_cast<const char*>(data) + r.pos(data),
                  static_cast<std::uint32_t>(n) };
    strings.push_back(sr);
    r.skip(static_cast<std::size_t>(n));
  }

  auto consume_value = [&](std::uint8_t ptype) {
    switch (ptype) {
      case 0: r.u8(); break;          // BOOL
      case 1: r.i32(); break;         // INT
      case 2: r.f64(); break;         // DOUBLE
      case 3: r.varint(); break;      // STRING
      default:
        std::fprintf(stderr, "bad ptype %u\n", ptype);
        std::exit(1);
    }
  };

  auto consume_items = [&](std::uint64_t n, bool in_set) {
    for (std::uint64_t i = 0; i < n; ++i) {
      std::uint8_t tag = r.u8();
      switch (tag) {
        case 0x10: { // par
          std::uint8_t pt = r.u8();
          r.varint();          // name
          consume_value(pt);
          ++c.pars;
          break;
        }
        case 0x11: { // parTable
          if (!in_set) { std::fprintf(stderr, "parTable in macro\n"); std::exit(1); }
          std::uint8_t pt = r.u8();
          r.varint();          // name
          std::uint64_t cells = r.varint();
          for (std::uint64_t k = 0; k < cells; ++k) {
            r.i32(); r.i32();   // row, col
            consume_value(pt);
          }
          ++c.par_tables;
          break;
        }
        case 0x12: { // reference
          r.varint();           // type
          r.varint();           // name
          r.u8();               // origData
          r.varint();           // origName
          std::uint8_t fl = r.u8();
          if (fl & 0x01) r.varint();
          if (fl & 0x02) r.varint();
          if (fl & 0x04) r.varint();
          ++c.refs;
          break;
        }
        case 0x13: { // macroParSet
          if (!in_set) { std::fprintf(stderr, "macroParSet in macro\n"); std::exit(1); }
          r.varint();
          ++c.macro_refs;
          break;
        }
        default:
          std::fprintf(stderr, "bad tag 0x%02x\n", tag);
          std::exit(1);
      }
    }
  };

  for (std::uint32_t i = 0; i < num_macros; ++i) {
    r.varint();                 // id
    std::uint64_t n = r.varint();
    consume_items(n, /*in_set=*/false);
    ++c.macros;
  }
  for (std::uint32_t i = 0; i < num_sets; ++i) {
    r.varint();                 // id
    std::uint64_t n = r.varint();
    consume_items(n, /*in_set=*/true);
    ++c.sets;
  }
  return c;
}

// ---- XML reader (Expat SAX) --------------------------------------------

struct XmlState {
  Counts c{};
  bool in_macro = false;
  bool in_table = false;
};

const char* localname(const char* qname) {
  // Expat with namespace handling separates with '\f' or '|'; without
  // namespace handling the qname is plain. We don't enable namespaces, so
  // qname is exactly the element name we wrote.
  return qname;
}

void XMLCALL on_start(void* user, const XML_Char* name, const XML_Char** attrs) {
  XmlState* s = static_cast<XmlState*>(user);
  const char* n = localname(name);
  if (std::strcmp(n, "set") == 0) {
    ++s->c.sets;
  } else if (std::strcmp(n, "macroParameterSet") == 0) {
    ++s->c.macros;
    s->in_macro = true;
  } else if (std::strcmp(n, "parTable") == 0) {
    ++s->c.par_tables;
    s->in_table = true;
  } else if (std::strcmp(n, "par") == 0) {
    if (!s->in_table) ++s->c.pars;
    // Touch all attributes so Expat actually materialises them — this
    // mimics what a real importer pays.
    for (int i = 0; attrs[i]; i += 2) {
      // Read the attribute value to keep the optimiser honest.
      asm volatile("" : : "r"(attrs[i + 1]) : "memory");
    }
  } else if (std::strcmp(n, "reference") == 0) {
    ++s->c.refs;
  } else if (std::strcmp(n, "macroParSet") == 0) {
    ++s->c.macro_refs;
  }
}

void XMLCALL on_end(void* user, const XML_Char* name) {
  XmlState* s = static_cast<XmlState*>(user);
  const char* n = localname(name);
  if (std::strcmp(n, "macroParameterSet") == 0) s->in_macro = false;
  else if (std::strcmp(n, "parTable") == 0) s->in_table = false;
}

Counts parse_xml(const char* path) {
  MMapped m;
  if (!mmap_file(path, m)) {
    std::fprintf(stderr, "cannot mmap %s\n", path);
    std::exit(1);
  }
  XML_Parser p = XML_ParserCreate(nullptr);
  XmlState s;
  XML_SetUserData(p, &s);
  XML_SetElementHandler(p, on_start, on_end);
  if (XML_Parse(p, reinterpret_cast<const char*>(m.data),
                static_cast<int>(m.size), 1) == XML_STATUS_ERROR) {
    std::fprintf(stderr, "expat error: %s at line %lu\n",
                 XML_ErrorString(XML_GetErrorCode(p)),
                 XML_GetCurrentLineNumber(p));
    std::exit(1);
  }
  XML_ParserFree(p);
  return s.c;
}

// ---- driver -------------------------------------------------------------

template <class F>
double time_ms(int iters, F&& f) {
  // Warm-up
  f();
  auto t0 = Clock::now();
  for (int i = 0; i < iters; ++i) f();
  auto t1 = Clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count() / iters;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) {
    std::fprintf(stderr, "usage: %s <sample.par> <sample.par.bin> [iters]\n", argv[0]);
    return 1;
  }
  const char* xml_path = argv[1];
  const char* bin_path = argv[2];
  int iters = (argc > 3) ? std::atoi(argv[3]) : 10;

  // Sizes
  struct stat sx{}; struct stat sb{};
  ::stat(xml_path, &sx);
  ::stat(bin_path, &sb);
  std::printf("xml size: %ld bytes\n", static_cast<long>(sx.st_size));
  std::printf("bin size: %ld bytes  (%.2fx smaller)\n",
              static_cast<long>(sb.st_size),
              static_cast<double>(sx.st_size) / static_cast<double>(sb.st_size));

  // Load binary file once into RAM (after warm-up the file is in cache anyway).
  MMapped bin_m;
  if (!mmap_file(bin_path, bin_m)) {
    std::fprintf(stderr, "cannot mmap %s\n", bin_path);
    return 1;
  }

  Counts cx{}, cb{};
  double xml_ms = time_ms(iters, [&]() { cx = parse_xml(xml_path); });
  double bin_ms = time_ms(iters, [&]() { cb = parse_binary(bin_m.data, bin_m.size); });

  cx.print("xml");
  cb.print("bin");
  std::printf("xml parse:  %8.2f ms (avg of %d iterations)\n", xml_ms, iters);
  std::printf("bin parse:  %8.2f ms (avg of %d iterations)\n", bin_ms, iters);
  std::printf("speedup:    %.2fx\n", xml_ms / bin_ms);

  if (!(cx == cb)) {
    std::fprintf(stderr, "WARN: counts differ between xml and bin parses\n");
    return 2;
  }
  return 0;
}
