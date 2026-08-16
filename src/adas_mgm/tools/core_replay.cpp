// tools/core_replay — 스냅샷 덤프를 mgm_step에 오프라인 재생 → CoreOutput CSV (ROS 무관)
//
// back-to-back 검증 (CLAUDE.md §5.5):
//   같은 덤프를 ① 레퍼런스 코어 ② Simulink 생성 코드에 각각 재생 → CSV diff.
//   레퍼런스 코어를 두 번 재생하면 diff가 0이어야 한다 (결정론).
//
// 사용법: core_replay <dump.bin> <out.csv> [key=value ...]
//   파라미터 오버라이드로 같은 run을 다른 임계에 재생해 튜닝을 비교할 수 있다:
//     core_replay run/mgm_snapshots.bin new.csv lane_conf_return=0.7 n_cycles=50
//   옛 덤프(작은 CoreParams)도 재생 가능 — 뒤에 추가된 필드는 0으로 채운다.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <cstdlib>
#include <string>

#include "core/mgm_step.hpp"
#include "tools/dump_format.hpp"

using namespace adas_mgm;

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr,
      "usage: core_replay <dump.bin> <out.csv> [key=value ...]\n"
      "  오버라이드 가능: lane_conf_exit lane_conf_return n_cycles "
      "avoid_return_hold_cycles v_base\n");
    return 1;
  }

  std::ifstream in(argv[1], std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "cannot open dump: %s\n", argv[1]);
    return 1;
  }
  // 헤더는 고정부(uint32 4개) + CoreParams 로 나눠 읽는다. CoreParams가 커져도
  // 옛 덤프를 재생할 수 있어야 하기 때문 — 통째로 읽으면 늘어난 크기만큼
  // 앞당겨 읽어 뒤따르는 스냅샷 레코드가 어긋난다 (2026-08-14).
  uint32_t fixed[4]{};
  in.read(reinterpret_cast<char *>(fixed), sizeof(fixed));
  DumpHeader h{};
  h.magic = fixed[0]; h.version = fixed[1]; h.snapshot_size = fixed[2]; h.params_size = fixed[3];
  if (!in || h.magic != kDumpMagic || h.version != kDumpVersion ||
    h.snapshot_size != sizeof(CoreSnapshot))
  {
    std::fprintf(stderr,
      "dump header mismatch (magic/version/layout) — 기록한 빌드와 같은 ABI로 재생할 것\n");
    return 1;
  }
  if (h.params_size > sizeof(CoreParams)) {
    std::fprintf(stderr, "dump params(%u B)가 현재 CoreParams(%zu B)보다 큼 — 재생 불가\n",
      h.params_size, sizeof(CoreParams));
    return 1;
  }
  in.read(reinterpret_cast<char *>(&h.params), h.params_size);   // 나머지는 0(기본값)
  if (h.params_size < sizeof(CoreParams)) {
    std::fprintf(stderr,
      "주의: 옛 덤프(params %u B < %zu B) — 뒤에 추가된 파라미터는 0으로 재생됩니다\n",
      h.params_size, sizeof(CoreParams));
  }

  // 파라미터 오버라이드 — 같은 run을 다른 임계로 재생해 튜닝 비교 (2026-08-14)
  for (int i = 3; i < argc; ++i) {
    const char * eq = std::strchr(argv[i], '=');
    if (eq == nullptr) {
      std::fprintf(stderr, "무시: %s (key=value 형식이 아님)\n", argv[i]);
      continue;
    }
    const std::string key(argv[i], eq - argv[i]);
    const double val = std::atof(eq + 1);
    if (key == "lane_conf_exit") {h.params.lane_conf_exit = static_cast<float>(val);} else if (
      key == "lane_conf_return") {h.params.lane_conf_return = static_cast<float>(val);} else if (
      key == "n_cycles") {h.params.n_cycles = static_cast<int32_t>(val);} else if (
      key == "avoid_return_hold_cycles") {
      h.params.avoid_return_hold_cycles = static_cast<int32_t>(val);
    } else if (key == "v_base") {h.params.v_base = static_cast<float>(val);} else {
      std::fprintf(stderr, "무시: 알 수 없는 파라미터 %s\n", key.c_str());
      continue;
    }
    std::fprintf(stderr, "override %s=%g\n", key.c_str(), val);
  }

  std::ofstream out(argv[2], std::ios::trunc);
  if (!out) {
    std::fprintf(stderr, "cannot open csv: %s\n", argv[2]);
    return 1;
  }
  out << "tick,state,path_source,immediate_stop,v_ref,n_points";
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
    out << buf << ',' << o.n_points;
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
