// test/traffic_state_test.cpp — 신호등 정지 MGM_STATE_TRAFFIC (§4) 단위 시험
//
// 거리 추적은 2026-09-02 개정(사용자 지정)으로 "정지선 검출이 화면에서
// 사라지는 edge(true→false) = 시드 거리(기본 1.5m)"로 확정됐다. 그 전
// 두 시도 — ① 진입 시점 거리·속도로 고정한 운동학적 제동곡선(latch-once),
// ② 안정 검출되는 매 틱 새 값을 신뢰(continuous trust) — 는 모두 이
// edge 기반 방식으로 되돌아갔다. 검증 대상은 코어 스테이트 머신 하나다
// (ROS 무관).
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
  p.traffic_ramp_distance_m = 1.5f;  // 소실 edge에서의 시드 거리
  p.traffic_stop_offset = 1.0f;      // 가드 문턱 겸 완전 정지 문턱
  return p;
}

CoreSnapshot input()
{
  CoreSnapshot s{};
  s.lane_confidence = 0.9f;
  s.lane_path.n = 1;
  s.lane_path.pts[0] = CorePoint{1.5f, 0.0f, 0.0f, 0.0f};
  s.avoid_ttc = 100.0f;
  s.vehicle_speed = 0.0f;
  s.vehicle_speed_valid = true;
  return s;
}

// 정지선이 한 틱 보였다가(true) 다음 틱에 사라지면(false) 소실 edge가
// 발생해 거리가 시드(1.5m)로 리셋된다 — 원시 카메라 거리값은 쓰지 않는다.
void triggerStoplineLostEdge(CoreState & st, CoreSnapshot & s)
{
  s.traffic_stopline_detected = true;
  mgm_step(s, st);
  s.traffic_stopline_detected = false;
  mgm_step(s, st);
}

void testRedAndStoplineEnterTraffic()
{
  CoreState state{};
  mgm_init(state, params());

  CoreSnapshot s = input();
  for (int tick = 0; tick < 60; ++tick) {
    mgm_step(s, state);
  }
  s.traffic_red_active = true;
  s.traffic_stopline_detected = true;
  CoreOutput out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_TRAFFIC,
    "확정 적색과 정지선 검출이 동시에 참이면 TRAFFIC에 진입해야 한다");
  check(out.path_source == MGM_SRC_LANE, "TRAFFIC은 차선 경로를 유지해야 한다");
  check(!state.traffic_distance_latched, "소실 edge 전에는 거리를 래치하면 안 된다");
}

void testRedWithoutStoplineDoesNotEnterTraffic()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot s = input();
  s.traffic_red_active = true;

  CoreOutput out = mgm_step(s, state);
  check(out.state == MGM_STATE_LANE, "확정 적색만으로 TRAFFIC에 진입하면 안 된다");
}

void testStoplineWithoutRedDoesNotEnterTraffic()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot s = input();
  s.traffic_stopline_detected = true;

  CoreOutput out = mgm_step(s, state);
  check(out.state == MGM_STATE_LANE, "정지선 검출만으로 TRAFFIC에 진입하면 안 된다");
}

// 소실 edge가 거리를 시드(1.5m)로 세팅하고, 실측 차속으로 dead-reckoning
// 감쇠하며, stop_offset(1.0m) 이하에서 v_ref가 정확히 0이 됨을 확인한다.
void testEdgeSeedsThenDecaysAndFloorsAtZero()
{
  CoreParams p = params();
  CoreState state{};
  mgm_init(state, p);
  CoreSnapshot s = input();
  s.traffic_red_active = true;

  triggerStoplineLostEdge(state, s);
  check(
    std::fabs(state.traffic_stopline_distance - p.traffic_ramp_distance_m) < 1.0e-5f,
    "소실 edge 직후 거리는 시드값이어야 한다");
  check(state.traffic_distance_latched, "소실 edge 이후에는 래치돼야 한다");

  s.vehicle_speed = 5.0f;   // 빠르게 소진해서 바닥을 확인한다
  bool saw_zero = false;
  float previous = state.traffic_stopline_distance;
  for (int tick = 0; tick < 100; ++tick) {
    CoreOutput out = mgm_step(s, state);
    check(
      state.traffic_stopline_distance <= previous + 1.0e-4f,
      "실차속도가 양수인 동안 거리가 늘어나면 안 된다");
    check(state.traffic_stopline_distance >= 0.0f, "적분 거리는 0 아래로 내려가면 안 된다");
    if (state.traffic_stopline_distance <= p.traffic_stop_offset) {
      check(out.v_ref == 0.0f, "거리가 stop_offset 이하이면 v_ref는 정확히 0이어야 한다");
      saw_zero = true;
    }
    previous = state.traffic_stopline_distance;
  }
  check(saw_zero, "충분히 진행하면 거리가 stop_offset 이하로 내려가야 한다");
}

// 핵심 요구사항(사용자 지정, 2026-09-02): 빨간불이 아직 확정 안 된 채 거리가
// stop_offset 이하로 떨어지면 시드로 되돌리고 붙잡아둔다 — 무관한 정지선을
// 스쳐 지나가며 쌓인 낡은 값이 나중에 빨간불이 뜨는 순간 그대로 급정지로
// 이어지는 것을 막는다.
void testStaleDistanceBeforeRedIsHeldAtSeed()
{
  CoreParams p = params();
  CoreState state{};
  mgm_init(state, p);
  CoreSnapshot s = input();
  // 아직 적색 미확정 — LANE 유지
  triggerStoplineLostEdge(state, s);   // 거리 = 1.5m, 소실 edge만 있었음
  s.vehicle_speed = 5.0f;

  for (int tick = 0; tick < 200; ++tick) {
    mgm_step(s, state);
    check(
      state.traffic_stopline_distance > p.traffic_stop_offset - 1.0e-4f,
      "빨간불 확정 전에는 거리가 stop_offset 밑으로 못 내려간다(가드가 시드로 되돌림)");
  }

  // 이제서야 실제로 빨간불이 확정돼도 현재 정지선이 없으면 TRAFFIC에
  // 진입하지 않는다. 과거 정지선 거리만으로 늦게 진입하거나 급정지하면 안 된다.
  s.traffic_red_active = true;
  s.vehicle_speed = 0.0f;
  CoreOutput out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_LANE,
    "과거 정지선 이력과 현재 적색만으로 TRAFFIC에 진입하면 안 된다");
  check(
    out.v_ref > 0.0f,
    "빨간불 확정 시 낡은 감쇠값 때문에 급정지(v_ref=0)하면 안 된다");

  // 현재 정지선을 다시 검출한 시점에는 두 조건이 동시에 만족되므로 진입한다.
  s.traffic_stopline_detected = true;
  out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_TRAFFIC,
    "현재 적색과 새 정지선 검출이 함께 있으면 TRAFFIC에 진입해야 한다");
}

void testGreenExitsAndResetsLatch()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot s = input();
  s.traffic_red_active = true;
  triggerStoplineLostEdge(state, s);
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
  s.traffic_stopline_detected = true;
  s.vehicle_speed_valid = false;
  CoreOutput out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_TRAFFIC && out.immediate_stop && out.v_ref == 0.0f,
    "TRAFFIC에서 실차속도 피드백이 없으면 fail-safe 정지해야 한다");
}

void testWaypointTrafficKeepsGpsPathAndReturnsToWaypoint()
{
  CoreState state{};
  mgm_init(state, params());
  state.state = MGM_STATE_WAYPOINT;
  CoreSnapshot s = input();
  s.traffic_red_active = true;
  s.traffic_stopline_detected = true;
  CoreOutput out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_TRAFFIC,
    "WAYPOINT에서도 확정 적색과 정지선이 함께 있으면 TRAFFIC으로 진입해야 한다");
  check(
    out.path_source == MGM_SRC_GPS,
    "WAYPOINT에서 진입한 TRAFFIC은 정지 중에도 GPS 경로를 유지해야 한다");
  s.traffic_red_active = false;
  s.traffic_green_active = true;
  out = mgm_step(s, state);
  check(
    out.state == MGM_STATE_WAYPOINT,
    "WAYPOINT에서 진입한 TRAFFIC은 확정 초록 뒤 WAYPOINT로 복귀해야 한다");
  check(
    out.path_source == MGM_SRC_GPS,
    "TRAFFIC 해제 틱에도 GPS 경로가 끊기면 안 된다");
}

}  // namespace

int main()
{
  testRedAndStoplineEnterTraffic();
  testRedWithoutStoplineDoesNotEnterTraffic();
  testStoplineWithoutRedDoesNotEnterTraffic();
  testEdgeSeedsThenDecaysAndFloorsAtZero();
  testStaleDistanceBeforeRedIsHeldAtSeed();
  testGreenExitsAndResetsLatch();
  testFailSafeOnMissingVehicleSpeed();
  testWaypointTrafficKeepsGpsPathAndReturnsToWaypoint();

  if (failures != 0) {
    std::fprintf(stderr, "traffic_state_test: %d 개 실패\n", failures);
    return 1;
  }
  std::printf("traffic_state_test: 전부 통과\n");
  return 0;
}
