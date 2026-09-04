#include "src/transition_log.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace adas_mgm
{

namespace
{

std::string f(const char * key, float v, int prec = 3)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%s=%.*f", key, prec, static_cast<double>(v));
  return buf;
}

std::string i(const char * key, long long v)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%s=%lld", key, v);
  return buf;
}

std::string b(const char * key, bool v)
{
  return std::string(key) + (v ? "=1" : "=0");
}

}  // namespace

const char * stateName(uint8_t state)
{
  switch (state) {
    case MGM_STATE_LANE: return "LANE";
    case MGM_STATE_WAYPOINT: return "WAYPOINT";
    case MGM_STATE_AVOID: return "AVOID";
    case MGM_STATE_PARKING: return "PARKING";
    case MGM_STATE_TRAFFIC: return "TRAFFIC";
    default: return "?";
  }
}

const char * transitionCsvHeader()
{
  return "tick,t_s,from,to,rule,spec_match,"
         "lane_conf,lane_low_cnt,lane_high_cnt,lane_n,"
         "cross_track_m,gps_n,avoid_ticks,return_hold_left,"
         "stop_zone,at_end,gps_only_zone,heading_valid,"
         "estop,traffic_stop,traffic_red,traffic_green,stopline_detected,"
         "stopline_distance_m,vehicle_speed_mps,vehicle_speed_valid,"
         "obstacle,avoidable,ttc,narrow,maneuver_done,avoid_zone,"
         "parking_zone,parking_found,parking_done,v_ref\n";
}

TransitionRecord explainTransition(
  uint8_t from, uint8_t to, const CoreSnapshot & s, const CoreParams & p,
  int32_t lane_low_cnt_before, int32_t lane_high_cnt_before,
  int32_t avoid_ticks_before, int32_t return_hold_left_before,
  float v_ref, int64_t tick)
{
  TransitionRecord r;
  r.tick = tick;
  r.from = from;
  r.to = to;

  // §4 전이 조건표를 그대로 옮겨 "이 전이가 성립하려면 무엇이 참이어야 했나"를 센다.
  // 카운터는 step 직전 값이므로 이 틱에 +1 되어 임계에 닿았을 것 — 그래서 >= n-1.
  const int32_t n = p.n_cycles;

  if (from == MGM_STATE_LANE && to == MGM_STATE_WAYPOINT) {
    if (s.gps_parking_zone && s.gps_path.n > 0) {
      r.rule = "lane→waypoint: GPS 주차구간 탐색 진입 (웨이포인트 추종)";
      r.spec_match = true;
    } else if (s.gps_gps_only_zone && s.gps_path.n > 0) {
      r.rule = "lane→waypoint: GPS 전용 구간 진입 (즉시, 히스테리시스 없음)";
      r.spec_match = true;
    } else {
      r.rule = "lane→waypoint: 차선 신뢰도 < lane_conf_exit 가 n_cycles 연속";
      r.spec_match = s.lane_confidence < p.lane_conf_exit &&
        lane_low_cnt_before >= n - 1;
    }
  } else if (from == MGM_STATE_WAYPOINT && to == MGM_STATE_LANE) {
    r.rule = "waypoint→lane: 신뢰도 > lane_conf_return 가 n_cycles 연속 "
      "+ 트랙 재합류(cross ≤ lane_entry_max_cross) + 강제 waypoint 구간 밖";
    const bool cross_ok = p.lane_entry_max_cross <= 0.0f ||
      s.gps_cross_track <= p.lane_entry_max_cross;
    r.spec_match = s.lane_confidence > p.lane_conf_return &&
      lane_high_cnt_before >= n - 1 && cross_ok && !s.gps_gps_only_zone &&
      !s.gps_parking_zone &&
      return_hold_left_before <= 1;
  } else if (to == MGM_STATE_TRAFFIC) {
    r.rule = "lane/waypoint→traffic: 확정 적색";
    r.spec_match = s.traffic_red_active;
  } else if (from == MGM_STATE_TRAFFIC && to == MGM_STATE_LANE) {
    r.rule = "traffic→lane: 확정 초록";
    r.spec_match = s.traffic_green_active;
  } else if (to == MGM_STATE_AVOID) {
    r.rule = "→avoid: 장애물 감지 + 회피 가능 (+ 회피 허용 구간)";
    r.spec_match = s.avoid_obstacle_detected && s.avoid_avoidable &&
      (p.avoid_zone_only == 0 || s.gps_avoid_zone);
  } else if (from == MGM_STATE_AVOID) {
    r.rule = "avoid→waypoint: 기동 완료 또는 avoid_max_cycles 초과";
    r.spec_match = s.avoid_maneuver_done ||
      (p.avoid_max_cycles > 0 && avoid_ticks_before >= p.avoid_max_cycles);
  } else if (to == MGM_STATE_PARKING) {
    r.rule = "lane→parking: GPS 주차구간 + 주차공간 인식";
    r.spec_match = s.gps_parking_zone && s.parking_space_found;
  } else if (from == MGM_STATE_PARKING) {
    r.rule = "parking→lane: 주차 완료";
    r.spec_match = s.parking_done;
  } else {
    r.rule = "정의되지 않은 전이 — §4 표에 없음";
    r.spec_match = false;
  }

  // 사람이 읽는 줄: 그 전이의 결정 변수를 앞에, 배경을 뒤에.
  std::string d;
  d += f("lane_conf", s.lane_confidence) + " ";
  d += i("lane_low_cnt", lane_low_cnt_before) + "/" + std::to_string(n) + " ";
  d += i("lane_high_cnt", lane_high_cnt_before) + "/" + std::to_string(n) + " ";
  d += i("avoid_ticks", avoid_ticks_before) + " ";
  d += i("return_hold_left", return_hold_left_before) + " ";
  d += f("cross_track", s.gps_cross_track) + " ";
  d += i("gps_n", s.gps_path.n) + " ";
  d += b("gps_only_zone", s.gps_gps_only_zone) + " ";
  d += b("at_end", s.gps_at_end) + " ";
  d += b("estop", s.estop) + " ";
  d += b("traffic_stop", s.traffic_stop_required) + " ";
  d += b("traffic_red", s.traffic_red_active) + " ";
  d += b("traffic_green", s.traffic_green_active) + " ";
  d += b("stopline", s.traffic_stopline_detected) + " ";
  d += f("stopline_m", s.traffic_stop_distance, 2) + " ";
  d += f("v_actual", s.vehicle_speed, 2) + " ";
  d += b("obstacle", s.avoid_obstacle_detected) + " ";
  d += b("avoidable", s.avoid_avoidable) + " ";
  d += f("ttc", s.avoid_ttc, 2) + " ";
  d += b("maneuver_done", s.avoid_maneuver_done) + " ";
  d += f("v_ref", v_ref, 2);
  r.detail = d;

  char csv[768];
  std::snprintf(
    csv, sizeof(csv),
    "%lld,%.2f,%s,%s,\"%s\",%d,"
    "%.4f,%d,%d,%d,"
    "%.4f,%d,%d,%d,%u,%d,%d,%d,"
    "%d,%d,%d,%d,%d,%.3f,%.3f,%d,%d,%d,%.3f,%d,%d,%d,"
    "%d,%d,%d,%.4f\n",
    static_cast<long long>(tick), tick * 0.01,
    stateName(from), stateName(to), r.rule.c_str(), r.spec_match ? 1 : 0,
    static_cast<double>(s.lane_confidence),
    lane_low_cnt_before, lane_high_cnt_before, s.lane_path.n,
    static_cast<double>(s.gps_cross_track), s.gps_path.n,
    avoid_ticks_before, return_hold_left_before, s.gps_stop_zone,
    s.gps_at_end ? 1 : 0, s.gps_gps_only_zone ? 1 : 0, s.gps_heading_valid ? 1 : 0,
    s.estop ? 1 : 0, s.traffic_stop_required ? 1 : 0,
    s.traffic_red_active ? 1 : 0, s.traffic_green_active ? 1 : 0,
    s.traffic_stopline_detected ? 1 : 0,
    static_cast<double>(s.traffic_stop_distance),
    static_cast<double>(s.vehicle_speed), s.vehicle_speed_valid ? 1 : 0,
    s.avoid_obstacle_detected ? 1 : 0, s.avoid_avoidable ? 1 : 0,
    static_cast<double>(s.avoid_ttc), s.avoid_narrow_gap ? 1 : 0,
    s.avoid_maneuver_done ? 1 : 0, s.gps_avoid_zone ? 1 : 0,
    s.gps_parking_zone ? 1 : 0, s.parking_space_found ? 1 : 0,
    s.parking_done ? 1 : 0, static_cast<double>(v_ref));
  r.csv = csv;

  return r;
}

}  // namespace adas_mgm
