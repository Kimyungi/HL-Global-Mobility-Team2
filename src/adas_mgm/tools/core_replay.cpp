// tools/core_replay — 스냅샷 덤프를 mgm_step에 오프라인 재생 → CoreOutput CSV (ROS 무관)
//
// back-to-back 검증 (CLAUDE.md §5.5):
//   같은 덤프를 ① 레퍼런스 코어 ② Simulink 생성 코드에 각각 재생 → CSV diff.
//   레퍼런스 코어를 두 번 재생하면 diff가 0이어야 한다 (결정론).
//
// 사용법: core_replay <dump.bin> <out.csv>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>

#include "core/mgm_step.hpp"
#include "tools/dump_format.hpp"

using namespace adas_mgm;

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::fprintf(stderr, "usage: core_replay <dump.bin> <out.csv>\n");
    return 1;
  }

  std::ifstream in(argv[1], std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "cannot open dump: %s\n", argv[1]);
    return 1;
  }
  DumpHeader h{};
  in.read(reinterpret_cast<char *>(&h), sizeof(h));
  if (!in || h.magic != kDumpMagic || h.version != kDumpVersion ||
    h.snapshot_size != sizeof(CoreSnapshot) || h.params_size != sizeof(CoreParams))
  {
    std::fprintf(stderr,
      "dump header mismatch (magic/version/layout) — 기록한 빌드와 같은 ABI로 재생할 것\n");
    return 1;
  }

  std::ofstream out(argv[2], std::ios::trunc);
  if (!out) {
    std::fprintf(stderr, "cannot open csv: %s\n", argv[2]);
    return 1;
  }
  out << "tick,state,path_source,immediate_stop,v_ref";
  for (int i = 0; i < MGM_NUM_POINTS; ++i) {
    out << ",x" << i << ",y" << i << ",yaw" << i << ",k" << i;
  }
  out << "\n";

  CoreState st;
  mgm_init(st, h.params);  // 기록 당시 파라미터로 동일 조건 재생

  char buf[96];
  CoreSnapshot s{};
  int64_t tick = 0;
  while (in.read(reinterpret_cast<char *>(&s), sizeof(s))) {
    const CoreOutput o = mgm_step(s, st);
    out << tick << ',' << static_cast<int>(o.state) << ',' << static_cast<int>(o.path_source)
        << ',' << (o.immediate_stop ? 1 : 0);
    std::snprintf(buf, sizeof(buf), ",%.9g", static_cast<double>(o.v_ref));
    out << buf;
    for (int i = 0; i < MGM_NUM_POINTS; ++i) {
      std::snprintf(buf, sizeof(buf), ",%.9g,%.9g,%.9g,%.9g",
        static_cast<double>(o.ref_points[i].x), static_cast<double>(o.ref_points[i].y),
        static_cast<double>(o.ref_points[i].yaw), static_cast<double>(o.ref_points[i].curvature));
      out << buf;
    }
    out << '\n';
    ++tick;
  }

  std::fprintf(stderr, "replayed %lld ticks (%.1f s) → %s\n",
    static_cast<long long>(tick), tick * 0.01, argv[2]);
  return 0;
}
