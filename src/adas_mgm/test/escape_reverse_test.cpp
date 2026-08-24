// test/escape_reverse_test.cpp — 후진 탈출 (§4, 2026-08-24) 단위 시험
//
// 검증 대상은 코어 스테이트 머신 하나다 (ROS 무관, mgm_core 만 링크).
// 이 기능은 "estop 이 참인 채로 차를 움직이는" 유일한 경로라, 여기 시험의
// 절반은 **후진하지 않아야 하는 경우**를 고정하는 데 쓴다.
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

// 후진이 실제로 성립할 수 있는 최소 설정. escape_after_cycles 를 짧게 잡아
// 시험이 10초를 기다리지 않게 한다 (동작은 틱 수에만 의존한다).
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
  p.blend_cycles = 0;          // 블렌드 없음 — ref 를 있는 그대로 보기 위해
  p.a_up = 1.5f;
  p.a_down = 1.5f;
  p.wrongway_yaw = 2.09f;
  p.wrongway_cycles = 50;
  p.avoid_return_hold_cycles = 0;
  p.lane_entry_max_cross = 0.5f;
  p.avoid_max_cycles = 1200;
  p.v_avoid = 0.0f;
  p.stop_zone_hold_cycles = 0;
  p.avoid_zone_only = 0;
  p.escape_after_cycles = 20;
  p.v_escape = -0.3f;
  p.escape_max_cycles = 30;
  p.escape_require_rear_clear = 1;
  return p;
}

// 차선 주행 중인 평범한 스냅샷 (estop 없음, 유효 경로 있음)
CoreSnapshot drivingSnapshot()
{
  CoreSnapshot s{};
  s.lane_confidence = 1.0f;
  s.lane_path.n = 1;
  s.lane_path.pts[0] = {2.0f, 0.0f, 0.0f, 0.0f};
  s.lane_updated = true;
  s.gps_path.n = 1;
  s.gps_path.pts[0] = {2.0f, 0.0f, 0.0f, 0.0f};
  s.gps_updated = true;
  s.avoid_ttc = 1e9f;
  return s;
}

void setEstop(CoreSnapshot & s, bool on, bool rear_clear)
{
  s.estop = on;
  s.estop_latch_release = on;   // "실제 EstopRequest 인가" — 코어의 후진 카운터 입력
  s.estop_rear_clear = rear_clear;
}

// 차를 실제로 굴려 escape_armed 를 세운다 (무장 조건 ③).
void driveUntilArmed(CoreState & st, int ticks = 30)
{
  CoreSnapshot s = drivingSnapshot();
  for (int i = 0; i < ticks; ++i) {
    mgm_step(s, st);
  }
}

// estop 을 걸고 지정 틱수만큼 돌린 뒤 마지막 출력을 돌려준다.
CoreOutput holdEstop(CoreState & st, int ticks, bool rear_clear)
{
  CoreSnapshot s = drivingSnapshot();
  setEstop(s, true, rear_clear);
  CoreOutput out{};
  for (int i = 0; i < ticks; ++i) {
    out = mgm_step(s, st);
  }
  return out;
}

// ── ① 정상 경로: 갇힌 차가 물러난 뒤 AVOID 에 남는다
void testReverseEngages()
{
  CoreState st{};
  mgm_init(st, baseParams());
  driveUntilArmed(st);

  // 문턱 직전까지는 평범한 긴급 정지여야 한다
  CoreOutput out = holdEstop(st, 19, true);
  check(out.v_ref == 0.0f, "문턱 전에는 v_ref 0 이어야 한다");
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "문턱 전에는 후진 페이즈가 아니어야 한다");

  // 문턱을 넘기는 그 틱에 후진 개시 — AVOID 스테이트 + 탈출 소스 + 음수 속도
  out = holdEstop(st, 1, true);
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_REVERSING, "문턱 후 후진 페이즈여야 한다");
  check(out.state == adas_mgm::MGM_STATE_AVOID, "후진은 AVOID 스테이트 안에서 돈다");
  check(out.path_source == adas_mgm::MGM_SRC_ESCAPE, "후진 중 경로 소스는 ESCAPE");
  check(out.v_ref < 0.0f, "후진 중 v_ref 는 음수여야 한다");
  check(!out.immediate_stop, "후진은 rate limit 을 타야 한다 (급후진 금지)");

  // ref 는 곧게 앞으로 — y·yaw·κ 가 전부 0 이어야 조향이 중립이다
  check(out.n_points == adas_mgm::MGM_NUM_POINTS, "탈출 ref 는 20점");
  bool straight = true;
  for (int i = 0; i < out.n_points; ++i) {
    if (out.ref_points[i].y != 0.0f || out.ref_points[i].yaw != 0.0f ||
      out.ref_points[i].curvature != 0.0f || out.ref_points[i].x <= 0.0f)
    {
      straight = false;
    }
  }
  check(straight, "탈출 ref 는 y=0·yaw=0·κ=0 인 전방 직선이어야 한다");

  // rate limit 을 타고 v_escape 까지 내려간다 (a_down 1.5 → 0 에서 -0.3 까지 20틱)
  out = holdEstop(st, 25, true);              // escape_ticks 1..25
  check(out.v_ref <= -0.29f, "후진 속도가 v_escape 에 도달해야 한다");

  // 시간 상한(escape_max_cycles=30)에 닿는 틱에 페이즈가 닫히고 AVOID 에 남는다.
  // 여기서 더 오래 돌리면 재무장 간격(escape_after_cycles)까지 채워져 **다시**
  // 후진에 들어가므로, 닫히는 순간을 정확히 그 틱에서 본다.
  out = holdEstop(st, 5, true);               // escape_ticks 26..30 → 30 에서 닫힘
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "상한 초과 시 후진 페이즈가 닫혀야 한다");
  check(out.state == adas_mgm::MGM_STATE_AVOID, "후진 후에도 AVOID 에 남아 회피를 시도한다");
  check(out.v_ref == 0.0f, "estop 이 남아 있으면 후진 종료 후 다시 정지");
}

// ── ② 무장 전 후진 금지: 벽을 보고 launch 한 차는 스스로 물러나지 않는다
void testNotArmedBeforeMoving()
{
  CoreState st{};
  mgm_init(st, baseParams());
  // driveUntilArmed 없이 곧바로 estop — 첫 틱부터 갇힌 상태
  CoreOutput out = holdEstop(st, 200, true);
  check(!st.escape_armed, "굴러간 적이 없으면 무장되지 않아야 한다");
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "무장 전에는 후진하지 않아야 한다");
  check(out.v_ref == 0.0f, "무장 전에는 정지를 유지해야 한다");
}

// ── ③ 후방을 모르면 후진하지 않는다 (센서 미탑재 = rear_clear false)
void testRearBlockedNeverReverses()
{
  CoreState st{};
  mgm_init(st, baseParams());
  driveUntilArmed(st);
  CoreOutput out = holdEstop(st, 200, false);   // 뒤가 막혔거나 모른다
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "후방 미확인이면 후진 금지");
  check(out.v_ref == 0.0f, "후방 미확인이면 정지 유지");
}

// ── ④ 후진 중 뒤가 막히면 그 자리에서 멈춘다
void testRearBlockedDuringReverseAborts()
{
  CoreState st{};
  mgm_init(st, baseParams());
  driveUntilArmed(st);
  holdEstop(st, 25, true);
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_REVERSING, "먼저 후진에 들어가 있어야 한다");

  CoreOutput out = holdEstop(st, 1, false);     // 뒤에 뭔가 들어옴
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "후진 중 후방 막힘 → 즉시 중단");
  check(out.v_ref >= -0.3f, "중단 후 속도는 0 으로 되돌아가야 한다");
  out = holdEstop(st, 30, false);
  check(out.v_ref == 0.0f, "중단 후 정지");
}

// ── ⑤ watchdog 보정 estop 으로는 후진하지 않는다
// wait_go 대기·인지 노드 사망은 "갇힘"이 아니라 안전 장치다. 그때 차가 스스로
// 물러나면 출발 인가 전에 움직이는 꼴이 된다 (안전 불변식 ④).
void testWatchdogEstopDoesNotCount()
{
  CoreState st{};
  mgm_init(st, baseParams());
  driveUntilArmed(st);

  CoreSnapshot s = drivingSnapshot();
  s.estop = true;                 // wrapper 가 보정한 estop
  s.estop_latch_release = false;  // 실제 EstopRequest 인가는 아니다
  s.estop_rear_clear = true;
  CoreOutput out{};
  for (int i = 0; i < 200; ++i) {
    out = mgm_step(s, st);
  }
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "보정 estop 으로는 후진하지 않아야 한다");
  check(out.v_ref == 0.0f, "보정 estop 은 그대로 정지");
}

// ── ⑥ 기능 끔이 기본값이고, v_escape 가 음수가 아니면 구조적으로 잠긴다
void testDisabledByDefaultAndBySign()
{
  {
    CoreParams p = baseParams();
    p.escape_after_cycles = 0;              // 기본 = 끔
    CoreState st{};
    mgm_init(st, p);
    driveUntilArmed(st);
    holdEstop(st, 300, true);
    check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "escape_after_cycles 0 이면 끔");
  }
  {
    CoreParams p = baseParams();
    p.v_escape = 0.3f;                      // 전진 값 — 안전 불변식 ①
    CoreState st{};
    mgm_init(st, p);
    driveUntilArmed(st);
    CoreOutput out = holdEstop(st, 300, true);
    check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "v_escape 가 양수면 기능이 잠겨야 한다");
    check(out.v_ref == 0.0f, "v_escape 가 양수여도 전진 명령이 나가면 안 된다");
  }
  {
    CoreParams p = baseParams();
    p.escape_max_cycles = 0;                // 상한 없음 = 금지 (안전 불변식 ②)
    CoreState st{};
    mgm_init(st, p);
    driveUntilArmed(st);
    holdEstop(st, 300, true);
    check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "상한이 없으면 후진하지 않아야 한다");
  }
}

// ── ⑦ estop 이 풀리면 후진을 중단하고 평시로 돌아간다
void testEstopClearAborts()
{
  CoreState st{};
  mgm_init(st, baseParams());
  driveUntilArmed(st);
  holdEstop(st, 25, true);
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_REVERSING, "먼저 후진에 들어가 있어야 한다");

  CoreSnapshot s = drivingSnapshot();       // 장애물이 치워짐
  CoreOutput out{};
  for (int i = 0; i < 5; ++i) {
    out = mgm_step(s, st);
  }
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "estop 해제 → 후진 중단");
  check(out.v_ref > -0.3f, "해제 후 후진 속도에서 벗어나야 한다");
}

// ── ⑧ 재무장 간격: 한 번 물러난 뒤 곧바로 또 물러나지 않는다
void testReArmRequiresFullInterval()
{
  CoreState st{};
  mgm_init(st, baseParams());
  driveUntilArmed(st);
  holdEstop(st, 20, true);                  // 1회차 후진 개시 (hold 20 = 문턱)
  holdEstop(st, 30, true);                  // escape_ticks 1..30 → 상한으로 종료
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "1회차가 끝나 있어야 한다");

  // 종료 시 estop 카운터가 0 으로 리셋되므로 곧바로 재개시되면 안 된다.
  // 이 리셋이 "연속 후진으로 트랙에서 무한히 멀어지는 것"을 시간으로 막는 장치다.
  holdEstop(st, 19, true);
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "종료 직후 즉시 재후진 금지");
  // escape_after_cycles 를 새로 채우면 다시 물러난다
  holdEstop(st, 1, true);
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_REVERSING, "간격을 채우면 재후진 가능");
}

// ── ⑨ PARKING 중에는 관여하지 않는다 (주차 후진은 stack_parking 소관)
void testParkingExcluded()
{
  CoreState st{};
  mgm_init(st, baseParams());
  driveUntilArmed(st);
  st.state = adas_mgm::MGM_STATE_PARKING;

  CoreSnapshot s = drivingSnapshot();
  s.parking_path.n = 1;
  s.parking_path.pts[0] = {1.0f, 0.0f, 0.0f, 0.0f};
  setEstop(s, true, true);
  CoreOutput out{};
  for (int i = 0; i < 300; ++i) {
    out = mgm_step(s, st);
  }
  check(st.escape_phase == adas_mgm::MGM_ESCAPE_NONE, "PARKING 에서는 후진 탈출 금지");
  check(out.state == adas_mgm::MGM_STATE_PARKING, "PARKING 스테이트를 유지해야 한다");
}

}  // namespace

int main()
{
  testReverseEngages();
  testNotArmedBeforeMoving();
  testRearBlockedNeverReverses();
  testRearBlockedDuringReverseAborts();
  testWatchdogEstopDoesNotCount();
  testDisabledByDefaultAndBySign();
  testEstopClearAborts();
  testReArmRequiresFullInterval();
  testParkingExcluded();

  if (failures != 0) {
    std::fprintf(stderr, "escape_reverse_test: %d 개 실패\n", failures);
    return 1;
  }
  std::printf("escape_reverse_test: 전부 통과\n");
  return 0;
}
