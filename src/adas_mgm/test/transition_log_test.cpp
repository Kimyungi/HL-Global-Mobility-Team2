#include <cstdio>
#include <string>

#include "src/transition_log.hpp"

using adas_mgm::CoreParams;
using adas_mgm::CoreSnapshot;
using adas_mgm::MGM_STATE_AVOID;
using adas_mgm::MGM_STATE_LANE;
using adas_mgm::MGM_STATE_TRAFFIC;
using adas_mgm::MGM_STATE_WAYPOINT;
using adas_mgm::explainTransition;
using adas_mgm::transitionCsvHeader;

namespace
{

int failures = 0;

void check(bool condition, const char * message)
{
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", message);
  }
}

CoreParams params()
{
  CoreParams p{};
  p.lane_conf_exit = 0.35f;
  p.lane_conf_return = 0.70f;
  p.n_cycles = 50;
  p.lane_entry_max_cross = 0.5f;
  p.avoid_max_cycles = 100;
  p.avoid_zone_only = 1;
  return p;
}

}  // namespace

int main()
{
  const CoreParams p = params();
  CoreSnapshot s{};
  s.gps_path.n = 1;

  s.lane_confidence = 0.9f;
  s.gps_cross_track = 0.1f;
  auto record = explainTransition(
    MGM_STATE_WAYPOINT, MGM_STATE_LANE, s, p,
    0, 49, 0, 2, 0.5f, 100);
  check(!record.spec_match, "return hold must block WAYPOINT to LANE spec match");
  record = explainTransition(
    MGM_STATE_WAYPOINT, MGM_STATE_LANE, s, p,
    0, 49, 0, 1, 0.5f, 101);
  check(
    record.spec_match,
    "return hold value one must permit the transition after the pre-step decrement");

  s = CoreSnapshot{};
  s.lane_confidence = 0.1f;
  s.gps_gps_only_zone = true;
  record = explainTransition(
    MGM_STATE_LANE, MGM_STATE_WAYPOINT, s, p,
    49, 0, 0, 0, 0.5f, 102);
  check(
    record.spec_match && record.rule.find("차선 신뢰도") != std::string::npos,
    "GPS-only flag without a GPS path must be logged as a confidence transition");

  s = CoreSnapshot{};
  record = explainTransition(
    MGM_STATE_AVOID, MGM_STATE_WAYPOINT, s, p,
    0, 0, 99, 0, 0.5f, 200);
  check(!record.spec_match, "avoid timeout must not match one tick early");
  record = explainTransition(
    MGM_STATE_AVOID, MGM_STATE_WAYPOINT, s, p,
    0, 0, 100, 0, 0.5f, 201);
  check(record.spec_match, "avoid timeout must match at the exact boundary");
  s.avoid_maneuver_done = true;
  record = explainTransition(
    MGM_STATE_AVOID, MGM_STATE_WAYPOINT, s, p,
    0, 0, 1, 0, 0.5f, 202);
  check(record.spec_match, "maneuver_done must explain AVOID to WAYPOINT");

  s = CoreSnapshot{};
  s.avoid_obstacle_detected = true;
  s.avoid_avoidable = true;
  record = explainTransition(
    MGM_STATE_LANE, MGM_STATE_AVOID, s, p,
    0, 0, 0, 0, 0.4f, 300);
  check(!record.spec_match, "zone-only AVOID must require gps_avoid_zone");
  s.gps_avoid_zone = true;
  record = explainTransition(
    MGM_STATE_LANE, MGM_STATE_AVOID, s, p,
    0, 0, 0, 0, 0.4f, 301);
  check(record.spec_match, "AVOID must match inside the configured zone");

  s = CoreSnapshot{};
  s.traffic_red_active = true;
  s.traffic_stopline_detected = true;
  record = explainTransition(
    MGM_STATE_LANE, MGM_STATE_TRAFFIC, s, p,
    0, 0, 0, 0, 0.5f, 400);
  check(record.spec_match, "확정 적색과 정지선 검출은 TRAFFIC 진입을 설명해야 한다");
  s.traffic_red_active = false;
  s.traffic_green_active = true;
  record = explainTransition(
    MGM_STATE_TRAFFIC, MGM_STATE_LANE, s, p,
    0, 0, 0, 0, 0.0f, 401);
  check(record.spec_match, "확정 초록은 TRAFFIC 탈출을 설명해야 한다");
  record = explainTransition(
    MGM_STATE_TRAFFIC, MGM_STATE_WAYPOINT, s, p,
    0, 0, 0, 0, 0.0f, 402);
  check(
    record.spec_match,
    "확정 초록은 진입 상태에 따른 TRAFFIC→WAYPOINT 복귀도 설명해야 한다");
  check(
    record.csv.find("TRAFFIC,WAYPOINT") != std::string::npos,
    "CSV row must contain the TRAFFIC to WAYPOINT transition");

  const std::string header = transitionCsvHeader();
  check(
    header.find("avoid_ticks,return_hold_left") != std::string::npos,
    "CSV must preserve the counters needed to audit transition timing");

  std::printf("transition log: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
