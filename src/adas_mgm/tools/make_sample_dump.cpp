// tools/make_sample_dump — 합성 시나리오 스냅샷 덤프 생성 (ROS 무관, 결정론)
//
// 용도 (CLAUDE.md §5.5 back-to-back):
//  - 실차·rosbag 없이 core_replay 파이프라인 검증
//  - 김재민 MBD 착수 킷의 "이 입력에 이 출력이 나오면 합격" 기준 입력 (docs/MBD_KIT.md)
//
// 시나리오 (틱=10ms, 총 1100틱=11초): 모든 스테이트·전이·우선권 경로를 1회 이상 통과
//   0~ 99  lane 주행 (conf 0.9, 기본 속도)
// 100~199  가속구간
// 200~299  신호등 정지 요구 (감속 정지 → 해제 후 재가속)
// 300~449  차선 신뢰도 0.2 → 20주기 연속 후 waypoint 전이
// 450~549  장애물 감지+회피 가능 → avoid (v_suggest 0.4, 일부 구간 narrow_gap)
// 550      기동 완료 → waypoint 복귀
// 560~659  차선 신뢰도 0.9 복귀 → lane 복귀
// 660~679  avoid 진입 후 TTC<임계 → 즉시 정지 (안전 바닥)
// 680      기동 완료 → lane 복귀
// 700~799  GPS 주차구간+주차공간 인식 → parking (v_suggest 0.3, 일부 경로 침범 정지)
// 900      주차 완료 → lane
// 950~999  긴급 정지 (estop)
// 1000~1099 정상 lane 주행 복귀
//
// 사용법: make_sample_dump <out.bin>
#include <cmath>
#include <cstdio>
#include <fstream>

#include "core/mgm_types.hpp"
#include "tools/dump_format.hpp"

using namespace adas_mgm;

namespace
{

// 직선 경로 (y 오프셋·곡률 지정) — vehicle frame, 0.1m 간격
CorePath makePath(float y_offset, float curvature)
{
  CorePath p{};
  p.n = MGM_NUM_POINTS;
  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    p.pts[i].x = 0.1f * static_cast<float>(i + 1);
    p.pts[i].y = y_offset;
    p.pts[i].yaw = 0.0f;
    p.pts[i].curvature = curvature;
  }
  return p;
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::fprintf(stderr, "usage: make_sample_dump <out.bin>\n");
    return 1;
  }

  // params.yaml 기본값과 동일
  CoreParams p{};
  p.lane_conf_exit = 0.4f;
  p.lane_conf_return = 0.6f;
  p.n_cycles = 20;
  p.v_base = 0.5f;
  p.v_accel_zone = 1.0f;
  p.v_narrow = 0.2f;
  p.ttc_stop = 0.8f;
  p.blend_cycles = 10;
  p.a_up = 0.5f;
  p.a_down = 1.5f;

  std::ofstream out(argv[1], std::ios::binary | std::ios::trunc);
  if (!out) {
    std::fprintf(stderr, "cannot open: %s\n", argv[1]);
    return 1;
  }
  DumpHeader h{kDumpMagic, kDumpVersion,
    static_cast<uint32_t>(sizeof(CoreSnapshot)),
    static_cast<uint32_t>(sizeof(CoreParams)), p};
  out.write(reinterpret_cast<const char *>(&h), sizeof(h));

  for (int t = 0; t < 1100; ++t) {
    CoreSnapshot s{};
    // 기본 배경: 인지 4종 경로 상시 공급 (실차에서 각 스택이 계속 내보내는 것과 동일)
    s.lane_confidence = 0.9f;
    s.lane_path = makePath(0.0f, 0.0f);
    s.gps_path = makePath(0.05f, 0.0f);
    s.avoid_path = makePath(0.5f, 0.1f);
    s.avoid_ttc = 1e9f;
    s.avoid_v_suggest = 0.4f;
    s.parking_path = makePath(-0.3f, -0.2f);
    s.parking_v_suggest = 0.3f;

    if (t >= 100 && t < 200) {
      s.gps_accel_zone = true;
    }
    if (t >= 200 && t < 300) {
      s.traffic_stop_required = true;
    }
    if (t >= 300 && t < 450) {
      s.lane_confidence = 0.2f;  // 20주기 후 waypoint 전이
    }
    if (t >= 450 && t < 550) {
      s.lane_confidence = 0.2f;
      s.avoid_obstacle_detected = true;
      s.avoid_avoidable = true;
      s.avoid_ttc = 2.0f;
      s.avoid_narrow_gap = (t >= 500);  // 후반 여유 폭 좁음 → v_narrow 상한
      s.avoid_maneuver_done = (t == 549);
    }
    if (t >= 550 && t < 560) {
      s.lane_confidence = 0.2f;  // waypoint 복귀 확인 구간
    }
    // t>=560: conf 0.9 → 20주기 후 lane 복귀
    if (t >= 660 && t < 680) {
      s.avoid_obstacle_detected = true;
      s.avoid_avoidable = true;
      s.avoid_ttc = 0.5f;  // TTC < 임계 → 즉시 정지 (안전 바닥)
      s.avoid_maneuver_done = (t == 679);
    }
    if (t >= 700 && t < 900) {
      s.gps_parking_zone = true;
      s.parking_space_found = true;
      s.parking_path_blocked = (t >= 780 && t < 820);  // 경로 침범 → 정지 후 재개
      s.parking_done = (t == 899);
    }
    if (t >= 950 && t < 1000) {
      s.estop = true;
    }

    out.write(reinterpret_cast<const char *>(&s), sizeof(s));
  }

  std::fprintf(stderr, "wrote 1100 ticks → %s\n", argv[1]);
  return 0;
}
