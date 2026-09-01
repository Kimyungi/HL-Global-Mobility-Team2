// test/traffic_state_test.cpp — 신호등 정지 MGM_STATE_TRAFFIC (§4) 단위 시험
//
// 2026-09-01 개정: 최초 한 번만 거리를 래치하고 그 값·속도로 고정한 운동학적
// 제동곡선(v=sqrt(2·decel·remaining))을 따라가던 설계를, 정지선 인식이
// 간헐적으로만 성공하는 조건(YOLO/HSV가 후보조차 못 찾는 완전 실패 포함,
// 2026-09-01 해질녘 실측)에서는 그 최초 한 번의 관측에 전체 제동을 고정하는
// 게 위험하다는 판단으로 단순 비율 방식으로 되돌렸다. 이 파일은 그 최종
// 설계를 고정한다 — 검증 대상은 코어 스테이트 머신 하나다 (ROS 무관).
#include <cmath>
#include <cstdio>

#include "core/mgm_step.hpp"

using namespace adas_mgm;

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
  p.lane_conf_exit = 0.4f;
  p.lane_conf_return = 0.6f;
  p.n_cycles = 20;
  p.v_base = 0.5f;
  p.v_accel_zone = 1.0f;
  p.v_narrow = 0.2f;
  p.ttc_stop = 0.8f;
  p.a_up = 100.0f;   // rate limit을 사실상 끄고 ramp 계산 자체만 본다
  p.a_down = 100.0f;
  p.wrongway_cycles = 20;
  p.traffic_state_enabled = 1;
  p.traffic_stop_offset = 0.0f;
  p.traffic_ramp_distance_m = 1.5f;
  return p;
}

CoreSnapshot input()
{
  CoreSnapshot s{};
  s.lane_confidence = 0.9f;
  s.lane_path.n = 1;
  s.lane_path.pts[0] = CorePoint{1.5f, 0.0f, 0.0f, 0.0f};
  s.avoid_ttc = 100.0f;
  s.vehicle_speed = 0.5f;
  s.vehicle_speed_valid = true;
  return s;
}

void testRedEntersTraffic()
{
  CoreState state{};
  mgm_init(state, params());

  CoreSnapshot s = input();
  for (int tick = 0; tick < 60; ++tick) {
    mgm_step(s, state);
  }
  s.traffic_red_active = true;
  CoreOutput out = mgm_step(s, state);
  check(out.state == MGM_STATE_TRAFFIC, "확정 적색이면 즉시 TRAFFIC에 진입해야 한다");
  check(out.path_source == MGM_SRC_LANE, "TRAFFIC은 차선 경로를 유지해야 한다");
  check(!state.traffic_distance_latched, "정지선 관측 전에는 거리를 래치하면 안 된다");
}

// 핵심 회귀 시험: 정지선을 한 번도 못 봤으면(=거리를 모르면) v_base로 계속
// 달리게 두지 말고 즉시 정지해야 한다. 구 설계는 이 구간에서 v_base를
// 유지했는데, 카메라 정지선 인식이 아예 실패하는 조건이 실제로 있어
// "TRAFFIC(적색 확정)인데 한 번도 감속하지 않고 신호를 지나치는" 위험이 있었다.
void testNeverDetectedFallsBackToImmediateStop()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot s = input();
  s.traffic_red_active = true;

  for (int tick = 0; tick < 300; ++tick) {
    CoreOutput out = mgm_step(s, state);
    check(out.state == MGM_STATE_TRAFFIC, "정지선을 못 봐도 TRAFFIC은 유지돼야 한다");
    check(
      out.v_ref == 0.0f,
      "정지선을 한 번도 못 봤으면(dist 모름) 즉시 0으로 정지해야 한다 "
      "(v_base로 계속 달리면 신호를 지나칠 위험)");
    check(!state.traffic_distance_latched, "관측이 없었으니 래치도 없어야 한다");
  }
}

// 안정 검출되는 매 틱마다 거리를 새로 신뢰해야 한다 — 최초 값에 고정하지 않는다.
void testFreshReadingsAreTrustedContinuously()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot s = input();
  s.traffic_red_active = true;
  mgm_step(s, state);

  s.traffic_stopline_detected = true;
  s.traffic_stop_distance = 2.0f;
  s.vehicle_speed = 0.0f;   // 감쇠와 분리해서 "신선한 값 신뢰"만 본다
  mgm_step(s, state);
  check(state.traffic_distance_latched, "최초 유효 관측에서 래치돼야 한다");
  check(
    std::fabs(state.traffic_stopline_distance - 2.0f) < 1.0e-5f,
    "첫 관측값을 그대로 신뢰해야 한다");

  // 뒤이은 관측이 첫 값과 달라도(더 가까운 재측정) 그대로 덮어써야 한다 —
  // "최초 한 번만 믿고 이후 영상값은 무시"였던 구 설계와 반대되는 요구사항.
  s.traffic_stop_distance = 1.2f;
  mgm_step(s, state);
  check(
    std::fabs(state.traffic_stopline_distance - 1.2f) < 1.0e-5f,
    "안정 검출되는 후속 관측도 dead-reckoning 대신 그대로 신뢰해야 한다");
}

// 정지선이 안 보이는 동안은 실차속도로 dead-reckoning 감쇠하고, 0 아래로는
// 내려가지 않으며, remaining이 0 이하면 v_ref도 정확히 0이어야 한다.
void testDeadReckoningDecaysAndFloorsAtZero()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot s = input();
  s.traffic_red_active = true;
  mgm_step(s, state);

  s.traffic_stopline_detected = true;
  s.traffic_stop_distance = 1.5f;
  mgm_step(s, state);
  const float captured_v_ref = mgm_step(s, state).v_ref;
  check(captured_v_ref > 0.0f, "거리를 아는 상태에서는 v_ref가 0보다 커야 한다");

  s.traffic_stopline_detected = false;
  s.traffic_stop_distance = -1.0f;
  s.vehicle_speed = 5.0f;   // 빠르게 소진해서 바닥을 확인한다

  bool saw_zero = false;
  float previous = captured_v_ref;
  for (int tick = 0; tick < 100; ++tick) {
    CoreOutput out = mgm_step(s, state);
    check(out.v_ref <= previous + 1.0e-4f, "안 보이는 동안 v_ref가 늘어나면 안 된다");
    check(state.traffic_stopline_distance >= 0.0f, "적분 거리는 0 아래로 내려가면 안 된다");
    if (state.traffic_stopline_distance <= 0.0f) {
      check(out.v_ref == 0.0f, "남은 거리가 0이면 v_ref도 정확히 0이어야 한다");
      saw_zero = true;
    }
    previous = out.v_ref;
  }
  check(saw_zero, "충분히 진행하면 거리가 0에 도달해야 한다");
}

void testGreenExitsAndResetsLatch()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot s = input();
  s.traffic_red_active = true;
  mgm_step(s, state);
  s.traffic_stopline_detected = true;
  s.traffic_stop_distance = 1.0f;
  mgm_step(s, state);
  check(state.traffic_distance_latched, "사전 조건: 이 시점엔 래치돼 있어야 한다");

  s.traffic_red_active = false;
  s.traffic_green_active = true;
  CoreOutput out = mgm_step(s, state);
  check(out.state == MGM_STATE_LANE, "확정 초록이면 즉시 LANE으로 복귀해야 한다");
  check(!state.traffic_distance_latched, "TRAFFIC 탈출 시 거리 래치를 폐기해야 한다");
}

void testFailSafeOnMissingVehicleSpeed()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot s = input();
  s.traffic_red_active = true;
  s.vehicle_speed_valid = false;
  CoreOutput out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_TRAFFIC && out.immediate_stop && out.v_ref == 0.0f,
    "TRAFFIC에서 실차속도 피드백이 없으면 fail-safe 정지해야 한다");
}

void testEntryFromWaypointAndExitToLane()
{
  CoreState state{};
  mgm_init(state, params());
  state.state = MGM_STATE_WAYPOINT;
  CoreSnapshot s = input();
  s.traffic_red_active = true;
  CoreOutput out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_TRAFFIC,
    "WAYPOINT에서도 확정 적색이면 TRAFFIC으로 진입해야 한다");
  s.traffic_red_active = false;
  s.traffic_green_active = true;
  out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_LANE,
    "진입 전 상태와 관계없이 확정 초록은 LANE으로 복귀해야 한다");
}

}  // namespace

int main()
{
  testRedEntersTraffic();
  testNeverDetectedFallsBackToImmediateStop();
  testFreshReadingsAreTrustedContinuously();
  testDeadReckoningDecaysAndFloorsAtZero();
  testGreenExitsAndResetsLatch();
  testFailSafeOnMissingVehicleSpeed();
  testEntryFromWaypointAndExitToLane();

  if (failures != 0) {
    std::fprintf(stderr, "traffic_state_test: %d 개 실패\n", failures);
    return 1;
  }
  std::printf("traffic_state_test: 전부 통과\n");
  return 0;
}
