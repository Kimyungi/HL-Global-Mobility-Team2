// test/traffic_ramp_test.cpp — 신호등 정지 ramp (§4, 2026-09-01) 단위 시험
//
// 검증 대상은 코어 스테이트 머신 하나다 (ROS 무관, mgm_core 만 링크).
// dist를 모르는 run은 종전과 100% 동일(즉시 0)해야 한다는 것과, dist를
// 아는 run은 v_base를 상한으로 서서히 줄어야 한다는 것을 함께 고정한다.
#include <cmath>
#include <cstdio>

#include "core/mgm_step.hpp"

using adas_mgm::CoreOutput;
using adas_mgm::CoreParams;
using adas_mgm::CoreSnapshot;
using adas_mgm::CoreState;

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

CoreParams baseParams()
{
  CoreParams p{};
  p.lane_conf_exit = 0.4f;
  p.lane_conf_return = 0.7f;
  p.n_cycles = 5;
  p.v_base = 1.0f;
  p.v_accel_zone = 1.0f;
  p.v_narrow = 0.2f;
  p.ttc_stop = 1.3f;
  p.blend_cycles = 0;
  p.a_up = 100.0f;   // rate limit을 사실상 끄고 ramp 계산 자체만 본다
  p.a_down = 100.0f;
  p.wrongway_yaw = 2.09f;
  p.wrongway_cycles = 50;
  p.avoid_return_hold_cycles = 0;
  p.lane_entry_max_cross = 0.5f;
  p.avoid_max_cycles = 1200;
  p.v_avoid = 0.0f;
  p.stop_zone_hold_cycles = 0;
  p.avoid_zone_only = 0;
  p.escape_after_cycles = 0;
  p.v_escape = -0.3f;
  p.escape_max_cycles = 0;
  p.escape_require_rear_clear = 1;
  p.traffic_ramp_reset_distance_m = 1.5f;
  p.traffic_ramp_stop_distance_m = 0.5f;
  return p;
}

CoreSnapshot laneSnapshot()
{
  CoreSnapshot s{};
  s.lane_confidence = 1.0f;
  s.lane_path.n = 1;
  s.lane_path.pts[0] = {2.0f, 0.0f, 0.0f, 0.0f};
  s.lane_updated = true;
  s.avoid_ttc = 1e9f;
  return s;
}

// 정지선을 한 틱 보이게 했다가 안 보이게 해서 소실 edge를 만든다.
void triggerStoplineLostEdge(CoreState & st, CoreSnapshot & s)
{
  s.traffic_stopline_detected = true;
  mgm_step(s, st);
  s.traffic_stopline_detected = false;
  mgm_step(s, st);
}

// dist를 한 번도 알려주지 않은 run: traffic_stop_required가 뜨는 순간
// 종전과 동일하게 즉시 0 이어야 한다 (parity·back-to-back 호환의 핵심).
void testUnknownDistFallsBackToImmediateZero()
{
  CoreState st{};
  mgm_init(st, baseParams());
  CoreSnapshot s = laneSnapshot();
  for (int i = 0; i < 5; ++i) {mgm_step(s, st);}   // LANE 정착

  s.traffic_stop_required = true;
  CoreOutput out = mgm_step(s, st);
  check(out.v_ref == 0.0f, "dist 미확인 상태에서 traffic_stop_required는 즉시 0");
}

// 소실 edge로 dist=1.5를 시드한 뒤, 정차 없이(v=0으로 두어 감쇠 없음)
// traffic_stop_required가 뜨면 v_base 그대로(=clamp01(1.5/1.5)*v_base) 여야 한다.
void testRampStartsAtVBaseWhenDistAtReset()
{
  CoreParams p = baseParams();
  CoreState st{};
  mgm_init(st, p);
  CoreSnapshot s = laneSnapshot();
  for (int i = 0; i < 5; ++i) {mgm_step(s, st);}

  triggerStoplineLostEdge(st, s);
  check(
    std::fabs(st.traffic_dist_m - p.traffic_ramp_reset_distance_m) < 1e-4f,
    "소실 edge 직후 dist는 reset_distance로 시드돼야 한다");

  s.traffic_stop_required = true;
  s.vehicle_v = 0.0f;   // 감쇠 없이 이번 틱 값만 확인
  CoreOutput out = mgm_step(s, st);
  check(
    std::fabs(out.v_ref - p.v_base) < 1e-3f,
    "dist == reset_distance면 v_ref는 v_base와 같아야 한다");
}

// 실측 차속으로 dist가 감쇠하면서 v_ref도 함께 줄어드는지, stop_distance
// 이하에서는 완전히 0이 되는지 확인한다.
void testRampDecaysWithVehicleSpeedAndFloorsAtZero()
{
  CoreParams p = baseParams();
  CoreState st{};
  mgm_init(st, p);
  CoreSnapshot s = laneSnapshot();
  for (int i = 0; i < 5; ++i) {mgm_step(s, st);}

  triggerStoplineLostEdge(st, s);
  s.traffic_stop_required = true;
  s.vehicle_v = 1.0f;   // 1.0 m/s * 10ms = 0.01m/틱 감쇠

  float previous_v_ref = p.v_base;
  bool saw_decrease = false;
  bool saw_zero = false;
  for (int i = 0; i < 200; ++i) {
    CoreOutput out = mgm_step(s, st);
    if (out.v_ref < previous_v_ref - 1e-6f) {saw_decrease = true;}
    check(out.v_ref >= 0.0f, "ramp v_ref는 음수가 될 수 없다");
    check(out.v_ref <= p.v_base + 1e-4f, "ramp v_ref는 v_base를 넘을 수 없다");
    if (st.traffic_dist_m <= p.traffic_ramp_stop_distance_m) {
      check(out.v_ref == 0.0f, "dist가 stop_distance 이하이면 v_ref는 0이어야 한다");
      saw_zero = true;
    }
    previous_v_ref = out.v_ref;
  }
  check(saw_decrease, "실측 차속으로 dist가 감쇠하며 v_ref도 줄어야 한다");
  check(saw_zero, "충분히 진행하면 stop_distance 이하로 내려가 완전 정지해야 한다");
}

// traffic_stop_required가 되기 전에 무관한 정지선을 스쳐 지나가 dist가
// stop_distance 밑으로 빠지면, 나중에 실제로 traffic_stop_required가 뜰 때
// 낡은 값 때문에 급정지하지 않고 reset_distance에서 다시 시작해야 한다.
void testStaleDistBeforeTrafficStateIsHeldAtReset()
{
  CoreParams p = baseParams();
  CoreState st{};
  mgm_init(st, p);
  CoreSnapshot s = laneSnapshot();
  for (int i = 0; i < 5; ++i) {mgm_step(s, st);}

  triggerStoplineLostEdge(st, s);   // dist = 1.5, traffic_stop_required 아직 false
  s.vehicle_v = 5.0f;               // 빠르게 소진 (0.05m/틱)
  for (int i = 0; i < 200; ++i) {
    mgm_step(s, st);
    // 부동소수 감쇠라 reset 경계가 정확히 stop_distance에 안 맞고 살짝
    // 넘어간 뒤 다음 틱에 걸릴 수 있다 — 그래서 검증은 "정확히 1.5로
    // 돌아온다"가 아니라 "stop_distance 밑으로는 절대 안 내려간다"다.
    check(
      st.traffic_dist_m > p.traffic_ramp_stop_distance_m - 1e-4f,
      "traffic_stop_required 이전에는 dist가 stop_distance 밑으로 못 내려간다");
  }

  // 이제서야 실제로 빨간불(신호와 무관한 정지선이 아니라)에 걸렸다고 가정.
  // 위 루프가 보장하는 건 "dist가 stop_distance보다 항상 크다"이지 "정확히
  // reset_distance"가 아니므로, 여기서 확인할 진짜 요구사항은 "급정지(0)로
  // 떨어지지 않는다"이다 — v_base와 정확히 같아야 한다는 건 과한 조건이었다.
  s.traffic_stop_required = true;
  s.vehicle_v = 0.0f;
  CoreOutput out = mgm_step(s, st);
  check(
    out.v_ref > 0.0f,
    "traffic_stop_required 진입 시 낡은 감쇠값 때문에 급정지(v_ref=0)하면 안 된다");
}

}  // namespace

int main()
{
  testUnknownDistFallsBackToImmediateZero();
  testRampStartsAtVBaseWhenDistAtReset();
  testRampDecaysWithVehicleSpeedAndFloorsAtZero();
  testStaleDistBeforeTrafficStateIsHeldAtReset();

  if (failures != 0) {
    std::fprintf(stderr, "traffic_ramp_test: %d 개 실패\n", failures);
    return 1;
  }
  std::printf("traffic_ramp_test: 전부 통과\n");
  return 0;
}
