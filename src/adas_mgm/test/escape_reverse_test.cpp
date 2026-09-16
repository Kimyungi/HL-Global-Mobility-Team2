#include <cstdlib>
#include <cmath>
#include <cstdio>
#include "core/mgm_step.hpp"
using namespace adas_mgm;

void check(bool condition) {
  if (!condition) {
    std::fputs("estop_reverse_test failed\n", stderr);
    std::exit(1);
  }
}

int main() {
  CoreParams p{};
  p.n_cycles = 5;
  p.v_base = 1.0f;
  p.a_up = p.a_down = 2.0f;
  p.escape_after_cycles = 1000;
  p.escape_max_cycles = 500;
  p.v_escape = -0.3f;
  p.escape_require_rear_clear = 1;
  CoreState st{};
  mgm_init(st, p);
  CoreSnapshot s{};
  s.lane_confidence = 1.0f;
  s.lane_path.n = 1;
  s.lane_path.pts[0] = {2, 0, 0, 0};
  s.vehicle_speed_valid = true;
  for (int i = 0; i < 30; ++i) mgm_step(s, st);
  s.estop = s.estop_latch_release = s.estop_rear_clear = true;
  s.vehicle_speed = 0;
  for (int i = 0; i < 999; ++i) {
    const auto o = mgm_step(s, st);
    check(o.state == MGM_STATE_ESTOP && o.v_ref == 0 && o.immediate_stop);
  }
  auto o = mgm_step(s, st);
  check(o.state == MGM_STATE_ESTOP && o.v_ref < 0);
  check(o.n_points == 1 && o.ref_points[0].x == -1.0f);
  check(o.ref_points[0].y == 0 && o.ref_points[0].yaw == 0 &&
    o.ref_points[0].curvature == 0);
  s.vehicle_speed = -0.3f;
  for (int i = 0; i < 333; ++i) {
    o = mgm_step(s, st);
    check(o.state == MGM_STATE_ESTOP && o.v_ref < 0);
  }
  o = mgm_step(s, st);
  check(o.state == MGM_STATE_LANE && o.v_ref == 0 && o.immediate_stop);
  for (int i = 0; i < 1000; ++i) {
    o = mgm_step(s, st);
    check(o.state == MGM_STATE_LANE && o.v_ref == 0);
  }
  s.estop = s.estop_latch_release = false;
  o = mgm_step(s, st);
  check(o.state == MGM_STATE_LANE && o.v_ref > 0);
  // 장애물이 대기 중 사라지면 후진 없이 원래 상태로 복귀한다.
  CoreState clear_st{};
  mgm_init(clear_st, p);
  for (int i = 0; i < 30; ++i) mgm_step(s, clear_st);
  s.estop = s.estop_latch_release = true;
  s.vehicle_speed = 0;
  for (int i = 0; i < 500; ++i) mgm_step(s, clear_st);
  s.estop = s.estop_latch_release = false;
  o = mgm_step(s, clear_st);
  check(o.state == MGM_STATE_LANE && o.v_ref >= 0);

  // 후방 미확인 시 10초가 지나도 후진하지 않는다.
  CoreState blocked_st{};
  mgm_init(blocked_st, p);
  for (int i = 0; i < 30; ++i) mgm_step(s, blocked_st);
  s.estop = s.estop_latch_release = true;
  s.estop_rear_clear = false;
  for (int i = 0; i < 1200; ++i) {
    o = mgm_step(s, blocked_st);
    check(o.state == MGM_STATE_ESTOP && o.v_ref == 0);
  }
  // 후진 중 후방 차단, 실제 estop 입력 소실, 차속 피드백 소실은 즉시 정지한다.
  for (int fault = 0; fault < 3; ++fault) {
    CoreState fault_st{};
    mgm_init(fault_st, p);
    s.estop = s.estop_latch_release = false;
    s.estop_rear_clear = true;
    s.vehicle_speed_valid = true;
    for (int i = 0; i < 30; ++i) mgm_step(s, fault_st);
    s.estop = s.estop_latch_release = true;
    for (int i = 0; i < 1000; ++i) o = mgm_step(s, fault_st);
    check(o.v_ref < 0);
    if (fault == 0) s.estop_rear_clear = false;
    if (fault == 1) s.estop_latch_release = false;
    if (fault == 2) s.vehicle_speed_valid = false;
    o = mgm_step(s, fault_st);
    check(o.state == MGM_STATE_ESTOP && o.v_ref == 0 && o.immediate_stop);
  }

  // 차가 한 번도 주행하지 않았다면 자동 후진하지 않는다.
  CoreState unarmed_st{};
  mgm_init(unarmed_st, p);
  s.estop = s.estop_latch_release = s.estop_rear_clear = true;
  s.vehicle_speed_valid = true;
  for (int i = 0; i < 1200; ++i) o = mgm_step(s, unarmed_st);
  check(o.state == MGM_STATE_ESTOP && o.v_ref == 0);

  // 실제 이동이 없는 경우 시간 상한에서 멈추고 재후진하지 않는다.
  CoreState timeout_st{};
  mgm_init(timeout_st, p);
  s.estop = s.estop_latch_release = false;
  for (int i = 0; i < 30; ++i) mgm_step(s, timeout_st);
  s.estop = s.estop_latch_release = true;
  s.vehicle_speed = 0;
  for (int i = 0; i < 1600; ++i) o = mgm_step(s, timeout_st);
  check(o.state == MGM_STATE_ESTOP && o.v_ref == 0);
  std::puts("estop_reverse_test passed");
}
