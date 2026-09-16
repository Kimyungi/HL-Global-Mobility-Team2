// tools/core_replay — 스냅샷 덤프를 mgm_step에 오프라인 재생 → CoreOutput CSV (ROS 무관)
//
// back-to-back 검증 (CLAUDE.md §5.5):
//   같은 덤프를 ① 레퍼런스 코어 ② Simulink 생성 코드에 각각 재생 → CSV diff.
//   레퍼런스 코어를 두 번 재생하면 diff가 0이어야 한다 (결정론).
//
// 사용법: core_replay <dump.bin> <out.csv> [key=value ...]
//   파라미터 오버라이드로 같은 run을 다른 임계에 재생해 튜닝을 비교할 수 있다:
//     core_replay run/mgm_snapshots.bin new.csv lane_conf_return=0.7 n_cycles=50
//   기록기와 동일한 v34/ABI/파라미터 크기만 재생한다. 과거 로그는 당시 도구를 사용한다.
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <cstdlib>
#include <string>

#include "core/mgm_step.hpp"
#include "tools/dump_reader.hpp"

using namespace adas_mgm;

int main(int argc, char ** argv)
{
  if (argc < 3) {
    std::fprintf(stderr,
      "usage: core_replay <dump.bin> <out.csv> [key=value ...]\n"
      "  오버라이드 가능: lane_conf_exit lane_conf_return n_cycles "
      "avoid_return_hold_cycles v_base v_avoid stop_zone_hold_cycles "
      "avoid_zone_only\n");
    return 1;
  }

  std::ifstream in(argv[1], std::ios::binary);
  if (!in) {
    std::fprintf(stderr, "cannot open dump: %s\n", argv[1]);
    return 1;
  }
  DumpHeader h{};
  if (!read_dump_header(in, h)) {return 1;}

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
    } else if (key == "v_base") {h.params.v_base = static_cast<float>(val);} else if (
      key == "v_avoid") {h.params.v_avoid = static_cast<float>(val);} else if (
      key == "stop_zone_hold_cycles") {
      h.params.stop_zone_hold_cycles = static_cast<int32_t>(val);
    } else if (key == "avoid_zone_only") {
      h.params.avoid_zone_only = static_cast<int32_t>(val);
    } else if (key == "avoidance_enabled") {
      h.params.avoidance_enabled = static_cast<int32_t>(val);
    } else {
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
  if (h.params.base_state_machine_enabled) {
    out << ",top,navigation,avoidance,signal,safety,mission,mission_type,speed_owner,reference_available,ref_valid,ref_fresh,ref_age_s,ref_generation,stop_reasons,parking_calibration,zone_calibration,zone_generation,recovery_configured,rear_sensor_valid,rear_corridor_state,recovery_eligible,recovery_block_reason,recovery_attempts,reverse_command_time_s,reverse_measured_distance_m,reverse_distance_complete,last_recovery_reason,traffic_remaining_m,traffic_stop_success";
  }
  if (h.params.base_state_machine_enabled) {
    out << ",route_phase,route_index,route_count,route_sequence_id,route_instance_id,route_request_id,route_requested_index,route_end_reached,route_completion,route_connecting,route_requested_connecting,mission_failed,mission_cancel_reason,parking_search_zone_only,parking_zone_entry_active";
  }
  out << ",sensor_alive_mask,reference_motion_blocked,dump_version,revised_v2,estop_active,estop_request_id\n";

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
    if (h.params.base_state_machine_enabled) {
      out << ',' << static_cast<int>(o.top) << ',' << static_cast<int>(o.nav)
          << ',' << static_cast<int>(o.avoid) << ',' << static_cast<int>(o.signal)
          << ',' << static_cast<int>(o.safety) << ',' << static_cast<int>(o.mission)
          << ',' << static_cast<int>(o.mission_type) << ',' << static_cast<int>(o.speed_owner)
          << ',' << o.reference_available << ',' << o.selected_reference.valid
          << ',' << o.selected_reference.fresh << ',' << o.selected_reference.age_s
          << ',' << o.selected_reference.generation << ',' << o.safe_stop_reasons
          << ',' << static_cast<int>(o.parking_calibration) << ',' << static_cast<int>(o.zones.calibration)
          << ',' << o.zones.last_generation << ',' << o.recovery.configured << ',' << o.recovery.rear_sensor_valid
          << ',' << static_cast<int>(o.recovery.rear_corridor_state) << ',' << o.recovery.eligible
          << ',' << static_cast<int>(o.recovery.block_reason) << ',' << o.recovery.attempt_count
          << ',' << o.recovery.command_time_s << ',' << o.recovery.measured_distance_m
          << ',' << o.recovery.measured_distance_complete << ',' << static_cast<int>(o.recovery.last_reason)
          << ',' << o.traffic_remaining_m << ',' << o.traffic_stop_in_success_region;
      out << ',' << static_cast<int>(o.route.phase) << ',' << o.route.index << ',' << o.route.count
          << ',' << o.route.sequence_id << ',' << o.route.instance_id << ',' << o.route.request_id
          << ',' << o.route.requested_index << ',' << o.route.end_reached << ',' << static_cast<int>(o.route.completion) << ',' << o.route.connecting << ',' << o.route.requested_connecting
          << ',' << o.active_mission_failed << ',' << static_cast<int>(o.mission_request.cancel_reason)
          << ',' << h.params.parking_search_zone_only << ',' << h.params.parking_zone_entry_active;
    }
    out << ',' << +s.sensor_alive_mask << ',' << o.reference_motion_blocked << ',' << h.version << ',' << s.revised_v2
        << ',' << o.estop_active << ',' << o.estop_request_id << '\n';
    ++tick;
  }

  std::fprintf(stderr, "replayed %lld ticks (%.1f s) → %s\n",
    static_cast<long long>(tick), tick * 0.01, argv[2]);
  return 0;
}
