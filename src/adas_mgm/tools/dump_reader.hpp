#ifndef ADAS_MGM_TOOLS_DUMP_READER_HPP_
#define ADAS_MGM_TOOLS_DUMP_READER_HPP_
#include <cstdio>
#include <istream>
#include "tools/dump_format.hpp"
namespace adas_mgm {
// Both replay tools use exactly the same contract. Never fill missing parameters
// with invented defaults or silently treat a partial record as a successful run.
inline bool read_dump_header(std::istream & in, DumpHeader & h) {
  uint32_t fixed[4]{};
  if (!in.read(reinterpret_cast<char *>(fixed), sizeof(fixed))) {
    std::fprintf(stderr, "dump header truncated\n"); return false;
  }
  h.magic=fixed[0]; h.version=fixed[1]; h.snapshot_size=fixed[2]; h.params_size=fixed[3];
  if (h.magic != kDumpMagic || h.version != kDumpVersion ||
      h.snapshot_size != sizeof(CoreSnapshot) || h.params_size != sizeof(CoreParams)) {
    std::fprintf(stderr,
      "dump contract mismatch: recorded v%u snapshot=%u params=%u; reader v%u snapshot=%zu params=%zu. "
      "Use the recording build's replay tool; v32 has two incompatible variants.\n",
      h.version,h.snapshot_size,h.params_size,kDumpVersion,sizeof(CoreSnapshot),sizeof(CoreParams));
    return false;
  }
  if (!in.read(reinterpret_cast<char *>(&h.params), sizeof(h.params))) {
    std::fprintf(stderr, "dump parameters truncated\n"); return false;
  }
  const auto start=in.tellg();
  in.seekg(0,std::ios::end);
  const auto end=in.tellg();
  if (start < 0 || end < start || (end-start) % sizeof(CoreSnapshot) != 0) {
    std::fprintf(stderr, "dump snapshot records truncated or unreadable\n"); return false;
  }
  in.seekg(start);
  return bool(in);
}
}
#endif
