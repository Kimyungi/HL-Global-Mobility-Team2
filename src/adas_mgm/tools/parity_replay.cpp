// tools/parity_replay — 실차 덤프 하나를 **두 구현에 동시 재생**해 차이를 찾는다.
//
//   레퍼런스 C++ 코어 (mgm_step)  ↔  Simulink 생성 C (ADAS_MGR2 v1.88)
//
// CLAUDE.md §5.5 back-to-back 의 실차판이다. core_replay 는 한쪽만 재생해 CSV 를
// 남기므로 두 번 돌려 diff 해야 하고, "어느 틱에서 왜 갈렸나"가 CSV 줄 번호로만
// 남는다. 이 도구는 매 틱 두 구현을 나란히 돌려 **전이 이력과 필드별 불일치**를
// 바로 보여 준다 — 시험 목적이 "전이 조건이 같은가" 이기 때문.
//
// v1.88 은 LANE/WAYPOINT/AVOID/PARKING 4상태와 지정 정지·회피/GPS 전용 구간,
// 종점·역방향 래치까지 구현한다. 따라서 후진 탈출이 꺼진 덤프는 모든 틱을
// 비교한다. v1.88 생성 뒤 production 코어에 추가된 rear-escape만 범위 밖이다.
// escape_after_cycles != 0인 덤프는 일부 틱을 임의로 빼지 않고 시작 전에 명확히
// 거절한다. 그래야 실제 escape 진입 전 구간만 우연히 통과한 결과를 패리티로
// 오해하지 않는다.
//
// 사용법: parity_replay <dump.bin> [diff.csv]
//   diff.csv 를 주면 불일치 틱만 CSV 로 남긴다 (양쪽 값을 나란히).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#include "core/mgm_step.hpp"
#include "src/generated_adapter.hpp"
#include "tools/dump_format.hpp"

using namespace adas_mgm;

namespace
{

// 패리티 테스트와 같은 허용오차 (test/generated_lane_waypoint_parity_test.cpp)
bool near(float a, float b)
{
  return std::fabs(a - b) <= 3e-5f;
}

const char * stateName(uint8_t s)
{
  switch (s) {
    case MGM_STATE_LANE: return "LANE";
    case MGM_STATE_WAYPOINT: return "WAYPOINT";
    case MGM_STATE_AVOID: return "AVOID";
    case MGM_STATE_PARKING: return "PARKING";
    default: return "?";
  }
}

struct Transition
{
  int64_t tick;
  uint8_t state;
};

void printTimeline(const char * label, const std::vector<Transition> & t)
{
  std::printf("  %s : ", label);
  if (t.empty()) {
    std::printf("(없음)\n");
    return;
  }
  for (size_t i = 0; i < t.size(); ++i) {
    if (i) {std::printf(" → ");}
    std::printf("%s@%.2fs", stateName(t[i].state), t[i].tick * 0.01);
  }
  std::printf("\n");
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(
      stderr,
      "usage: parity_replay <dump.bin> [diff.csv]\n"
      "  실차 덤프를 레퍼런스 코어와 생성 C 에 동시 재생해 전이·출력을 비교한다.\n");
    return 2;
  }

  std::ifstream in(argv[1], std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "덤프를 열 수 없음: %s\n", argv[1]);
    return 2;
  }

  // 헤더는 core_replay 와 같은 규약 — 고정부 4개 + CoreParams(옛 덤프 호환).
  uint32_t fixed[4]{};
  in.read(reinterpret_cast<char *>(fixed), sizeof(fixed));
  DumpHeader h{};
  h.magic = fixed[0]; h.version = fixed[1];
  h.snapshot_size = fixed[2]; h.params_size = fixed[3];
  if (!in || h.magic != kDumpMagic || h.version != kDumpVersion ||
    h.snapshot_size != sizeof(CoreSnapshot))
  {
    std::fprintf(
      stderr,
      "덤프 헤더 불일치 — 기록한 빌드와 같은 ABI 로 재생할 것\n");
    return 2;
  }
  if (h.params_size > sizeof(CoreParams)) {
    std::fprintf(
      stderr, "덤프 params(%u B)가 현재 CoreParams(%zu B)보다 큼 — 재생 불가\n",
      h.params_size, sizeof(CoreParams));
    return 2;
  }
  in.read(reinterpret_cast<char *>(&h.params), h.params_size);

  std::ofstream diff;
  const bool want_csv = argc >= 3;
  if (want_csv) {
    diff.open(argv[2], std::ios::trunc);
    if (!diff) {
      std::fprintf(stderr, "CSV 를 열 수 없음: %s\n", argv[2]);
      return 2;
    }
    diff << "tick,t_s,field,reference,generated\n";
  }

  if (h.params.escape_after_cycles != 0) {
    std::fprintf(
      stderr,
      "비교 불가: 이 덤프는 rear-escape가 활성화되어 있음 "
      "(escape_after_cycles=%d). ADAS_MGR2 v1.88에는 rear-escape가 없으므로 "
      "escape_after_cycles:=0으로 기록한 덤프를 사용할 것.\n",
      h.params.escape_after_cycles);
    return 2;
  }

  CoreState ref_state;
  mgm_init(ref_state, h.params);

  GeneratedMgmAdapter gen(h.params);   // 파라미터 검증 실패 시 예외 → main 밖으로

  int64_t tick = 0;
  int64_t n_state = 0, n_source = 0, n_stop = 0, n_vref = 0, n_np = 0, n_pts = 0;
  int64_t first_bad = -1;
  std::string first_bad_field;
  std::vector<Transition> ref_tl, gen_tl;
  uint8_t ref_prev = 0xFF, gen_prev = 0xFF;

  CoreSnapshot s{};
  while (in.read(reinterpret_cast<char *>(&s), sizeof(s))) {
    const CoreOutput r = mgm_step(s, ref_state);
    const CoreOutput g = gen.step(s);

    if (r.state != ref_prev) {ref_tl.push_back({tick, r.state}); ref_prev = r.state;}
    if (g.state != gen_prev) {gen_tl.push_back({tick, g.state}); gen_prev = g.state;}

    auto note = [&](const char * field, const std::string & a, const std::string & b) {
        if (first_bad < 0) {first_bad = tick; first_bad_field = field;}
        if (want_csv) {
          diff << tick << ',' << tick * 0.01 << ',' << field << ',' << a << ',' << b << '\n';
        }
      };

    if (r.state != g.state) {
      ++n_state; note("state", stateName(r.state), stateName(g.state));
    }
    if (r.path_source != g.path_source) {
      ++n_source; note(
        "path_source",
        std::to_string(r.path_source), std::to_string(g.path_source));
    }
    if (r.immediate_stop != g.immediate_stop) {
      ++n_stop; note(
        "immediate_stop",
        std::to_string(r.immediate_stop), std::to_string(g.immediate_stop));
    }
    if (!near(r.v_ref, g.v_ref)) {
      ++n_vref; note("v_ref", std::to_string(r.v_ref), std::to_string(g.v_ref));
    }
    if (r.n_points != g.n_points) {
      ++n_np; note("n_points", std::to_string(r.n_points), std::to_string(g.n_points));
    }
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      const CorePoint & a = r.ref_points[i];
      const CorePoint & b = g.ref_points[i];
      if (!near(a.x, b.x) || !near(a.y, b.y) ||
        !near(a.yaw, b.yaw) || !near(a.curvature, b.curvature))
      {
        ++n_pts;
        note(
          ("ref_points[" + std::to_string(i) + "]").c_str(),
          std::to_string(a.x) + " " + std::to_string(a.y),
          std::to_string(b.x) + " " + std::to_string(b.y));
        break;              // 한 틱에 점 하나만 보고 — CSV 가 20배로 부풀지 않게
      }
    }
    ++tick;
  }

  std::printf("\n═══ back-to-back 재생 (CLAUDE.md §5.5) ═══\n");
  std::printf("덤프      : %s\n", argv[1]);
  std::printf(
    "재생      : %lld 틱 (%.1f s)\n",
    static_cast<long long>(tick), tick * 0.01);
  std::printf(
    "비교 대상 : %lld 틱 (4상태·지정 구간 포함, 제외 틱 없음)\n",
    static_cast<long long>(tick));

  std::printf("\n스테이트 전이\n");
  printTimeline("레퍼런스", ref_tl);
  printTimeline("생성    ", gen_tl);
  bool tl_same = ref_tl.size() == gen_tl.size();
  if (tl_same) {
    for (size_t i = 0; i < ref_tl.size(); ++i) {
      if (ref_tl[i].state != gen_tl[i].state || ref_tl[i].tick != gen_tl[i].tick) {
        tl_same = false;
        break;
      }
    }
  }
  std::printf(
    "  → %s (전이 %zu회 / %zu회)\n",
    tl_same ? "틱 단위까지 일치" : "★ 다름", ref_tl.size(), gen_tl.size());

  const int64_t total = n_state + n_source + n_stop + n_vref + n_np + n_pts;
  std::printf("\n필드별 불일치 (전체 틱, 허용오차 3e-5)\n");
  std::printf("  state           %lld\n", static_cast<long long>(n_state));
  std::printf("  path_source     %lld\n", static_cast<long long>(n_source));
  std::printf("  immediate_stop  %lld\n", static_cast<long long>(n_stop));
  std::printf("  v_ref           %lld\n", static_cast<long long>(n_vref));
  std::printf("  n_points        %lld\n", static_cast<long long>(n_np));
  std::printf("  ref_points      %lld\n", static_cast<long long>(n_pts));
  if (first_bad >= 0) {
    std::printf(
      "\n첫 불일치: 틱 %lld (%.2f s) — %s\n",
      static_cast<long long>(first_bad), first_bad * 0.01, first_bad_field.c_str());
  }
  if (want_csv) {
    std::printf("불일치 CSV: %s\n", argv[2]);
  }

  // 판정 — 비교 가능한 틱이 있어야 의미가 있다.
  if (tick == 0) {
    std::printf("\n판정: 비교 불가 — 덤프에 스냅샷이 없다\n");
    return 2;
  }
  const bool ok = total == 0 && tl_same;
  std::printf("\n판정: %s\n", ok ? "완전 일치" : "★ 차이 있음 — 위 표 확인");
  return ok ? 0 : 1;
}
