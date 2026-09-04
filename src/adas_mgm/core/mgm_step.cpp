// core/mgm_step.cpp — MGM 로직 코어 구현 (ROS 헤더 include 금지, 동적 할당 금지)
//
// 구조는 CLAUDE.md 그대로 세 단계:
//   판단(스테이트 머신, §4 — 시스템에서 유일한 곳)
//   → 실행 1: ref 조립 (§5.1/§5.6 — 포맷 변환·전환 연속 처리만)
//   → 실행 2: 종방향 병합 (§5.6 — rate limit만, immediate_stop은 우회)
#include "mgm_step.hpp"

#include <cmath>

namespace adas_mgm
{

namespace
{

float lerp(float a, float b, float t)
{
  return a + (b - a) * t;
}

float min_f(float a, float b)
{
  return (a < b) ? a : b;
}

float max_f(float a, float b)
{
  return (a > b) ? a : b;
}

float clamp01(float x)
{
  return max_f(0.0f, min_f(1.0f, x));
}

// "실제로 멈췄다"로 보는 명령 속도 [m/s] — 지정 지점 정차 카운트다운 시작 조건.
// 실제 차속 피드백(`/vehicle/vector`)이 실측 0건이라 직전 틱 **명령** 속도를 쓴다.
// merge()의 rate limit은 0에 정확히 도달하므로(감속 하한이 0을 넘으면 0으로 고정)
// 문턱은 부동소수 여유분이면 충분하다.
constexpr float kStoppedSpeed = 1e-3f;

// 부동소수 그대로 통과된 값이라 실질적으로는 완전 동일 비교지만, 혹시 모를
// 미세 오차에 대비해 아주 작은 허용치를 둔다.
bool points_equal(const CorePoint & a, const CorePoint & b)
{
  constexpr float eps = 1e-6f;
  auto close = [](float x, float y) {return (x > y ? x - y : y - x) <= eps;};
  return close(a.x, b.x) && close(a.y, b.y) && close(a.yaw, b.yaw) &&
    close(a.curvature, b.curvature);
}

bool paths_equal(const CorePoint * a, const CorePoint * b, int32_t n)
{
  for (int32_t i = 0; i < n; ++i) {
    if (!points_equal(a[i], b[i])) {return false;}
  }
  return true;
}

// ── 판단: 스테이트 전이 (§4 전이 조건표)
void transition(const CoreSnapshot & s, CoreState & st)
{
  // 히스테리시스 카운터 — 이탈/복귀 임계 분리, N주기 연속
  st.lane_low_cnt = (s.lane_confidence < st.params.lane_conf_exit) ? st.lane_low_cnt + 1 : 0;
  st.lane_high_cnt = (s.lane_confidence > st.params.lane_conf_return) ? st.lane_high_cnt + 1 : 0;

  // 역방향 판정 (§4 waypoint) — GPS 경로 첫 점의 상대 yaw가 임계를 넘으면
  // 차가 경로를 등진 것 (유턴 후 트랙 역추종, 2026-08-03 2회 재현).
  //
  // ★ **헤딩을 믿을 수 있을 때만 센다** (2026-08-16). 접선 폴백은 "최근접 트랙
  //   접선 = 차량 헤딩" 가정이라 ref[0].yaw 가 항상 0 부근으로 나온다 — 차가
  //   트랙을 등지고 있어도 "정렬됨"으로 보인다. 그래서 종전 구현은 다음 왕복에
  //   빠졌다(run_0816_184505): 역방향 감지 → 정지 → 정지하니 COG 무효(속도 문턱)
  //   → 접선 폴백이 "오차 0°" → 가드 해제 → 엉뚱한 방향으로 재출발 → 속도 붙자
  //   COG 복귀 → 다시 역방향 → 정지. 횡오차가 2.7→5.1m 로 벌어졌다.
  //   신뢰 불가 구간에서는 두 카운터 모두 **정지**시켜 래치 상태를 보존한다.
  const bool head_ok = s.gps_heading_valid && s.gps_path.n > 0;
  const float y0 = (s.gps_path.n > 0) ? s.gps_path.pts[0].yaw : 0.0f;
  const bool wrongway = (y0 > st.params.wrongway_yaw) || (y0 < -st.params.wrongway_yaw);
  if (head_ok) {
    st.wrongway_cnt = wrongway ? st.wrongway_cnt + 1 : 0;
    st.wrongway_ok_cnt = wrongway ? 0 : st.wrongway_ok_cnt + 1;
  }
  // 역방향 **래치** — 한 번 걸리면 "신뢰 가능한 헤딩으로 정렬됐음"이 확인될 때까지
  // 유지한다. 해제 조건을 신뢰 가능한 헤딩으로 못박는 것이 위 왕복을 끊는 핵심이다.
  // at_end 래치와 마찬가지로 실제 EstopRequest 인가로도 해제된다(run 종료·재준비).
  if (s.estop_latch_release) {
    st.wrongway_latched = false;
  } else if (st.wrongway_cnt >= st.params.wrongway_cycles) {
    st.wrongway_latched = true;
  } else if (head_ok && st.wrongway_ok_cnt >= st.params.wrongway_cycles) {
    st.wrongway_latched = false;
  }

  // 종점 래치 (§4) — 정지 후 미세하게 밀려 최근접점이 뒤로 바뀌면 at_end가
  // 풀려 재출발·유턴하던 것 방지 (2026-08-03 직선 run 실사례). 해제는 **실제
  // EstopRequest 인가**(= run 종료/새 run 준비)로만 — s.estop은 wrapper의
  // staleness 보정이 섞여 있어, gps 단절→복구 같은 일시 장애로 래치가 풀려
  // 재출발하는 구멍이 있었다 (2026-08-11, CLAUDE.md §4 래치).
  //
  // 2026-08-15: 래치 조건에서 "waypoint 스테이트" 제약을 뺐다. GPS 트랙의 종점은
  // 코스의 끝이지 특정 스테이트의 사정이 아닌데, lane에서 종점을 지나면 아무도
  // 세우지 않았다 — run_0815_163614에서 at_end 후 **7.41m 초과 주행**(13.3초),
  // 그 사이 횡오차 0.41→6.37m + 트랙 밖 AVOID 진입.
  //   · `s.gps_path.n > 0` 게이트: lane 스테이트에서는 §5.7 ②의 gps 신선도
  //     watchdog이 동작하지 않는다(사용 중인 소스만 감시). 낡거나 무효인 gps의
  //     at_end로 래치가 걸리는 것을 막는다 — fix 상실 시 stack_gps는 빈 경로를
  //     계속 발행하므로 n==0으로 걸러진다.
  //   · parking 제외: 주차 구간에서 트랙 종점은 의미가 없다.
  if (s.estop_latch_release) {
    st.at_end_latched = false;
  } else if (st.state != MGM_STATE_PARKING && s.gps_at_end && s.gps_path.n > 0) {
    st.at_end_latched = true;
  }

  // ── 지정 지점 정지 (§4, 2026-08-18) — 트랙 위 지정 지점에 도달하면 정지하고
  // stop_zone_hold_cycles 만큼 머문 뒤 **스스로** 재출발한다 (언덕 정차 시험 등).
  // 어디서 서는지는 인지(stack_gps)가 위경도로 알고, "서고 다시 간다"는 판단은
  // 여기 스테이트 머신에만 있다 (§5.1).
  //
  // 설계 세 가지:
  //  ① 지점 **번호**로 소진 관리 — 진입 즉시 done_id에 찍는다. 언덕에서 정차 중
  //     차가 밀려 구간 밖으로 나갔다 다시 들어와도 같은 번호면 재정지하지 않는다
  //     (bool이면 밀림 → 재진입 → 재정지의 무한 루프가 된다).
  //  ② 기동 시점에 이미 지점 안이면 **그 지점만 임시로 억제**한다 — 지점 안에서
  //     launch 했을 때(그 자리에 멈춰 있는 상태) 출발도 하기 전에 정차를 소비하는
  //     것을 막는다. 단 억제는 **구간을 벗어나면 풀린다**: 차를 출발점으로 옮겨
  //     다시 지나가면 정상적으로 선다. ①의 소진(done_id)과 구분하는 것이 핵심이다
  //     — 하나로 합치면 "언덕에 선 채로 launch → 출발점으로 옮김 → 그 지점을
  //     영영 안 섬"이 된다(현장에서 알아채기 어려운 함정).
  //  ③ 카운트다운은 **실제로 멈춘 뒤**(직전 틱 명령 속도 0) 시작한다. 진입
  //     시점부터 세면 감속에 쓴 0.4s만큼 정차가 짧아진다.
  if (!st.stop_zone_init && s.gps_path.n > 0) {
    st.stop_zone_init = true;
    st.stop_zone_boot_id = s.gps_stop_zone;   // 기동 지점이 정지 구간이면 임시 억제
  }
  if (st.stop_zone_boot_id != 0 && s.gps_stop_zone != st.stop_zone_boot_id) {
    st.stop_zone_boot_id = 0;                 // 그 구간을 벗어남 → 억제 해제
  }
  if (s.estop_latch_release) {   // 새 run 준비 — at_end/역방향 래치와 같은 규약
    st.stop_zone_done_id = 0;
    st.stop_zone_holding = false;
    st.stop_hold_left = 0;
  }
  if (st.params.stop_zone_hold_cycles > 0 && st.state != MGM_STATE_PARKING) {
    if (!st.stop_zone_holding) {
      if (s.gps_stop_zone != 0 && s.gps_stop_zone != st.stop_zone_done_id &&
        s.gps_stop_zone != st.stop_zone_boot_id && s.gps_path.n > 0)
      {
        st.stop_zone_holding = true;
        st.stop_hold_left = st.params.stop_zone_hold_cycles;
        st.stop_zone_done_id = s.gps_stop_zone;   // ① 진입 시점에 소진 확정
      }
    } else if (st.v <= kStoppedSpeed) {
      if (--st.stop_hold_left <= 0) {
        st.stop_hold_left = 0;
        st.stop_zone_holding = false;            // 재출발 (다음 틱부터 v_base 램프)
      }
    }
  }

  // ── 후진 탈출 (§4, 2026-08-24) — 회피 불가 장애물 앞에서 estop 이 무한히
  // 유지되는 교착을 끊는다. estop 은 레벨 신호라 장애물이 치워져야 풀리는데,
  // 시험 코스에서는 치워질 일이 없는 장애물이 있다. 그러면 v_ref 0 으로 영원히
  // 서 있게 된다 — 그래서 충분히 오래 갇혀 있었으면 곧게 조금 물러나 회피가
  // 성립하는 거리를 만들고 AVOID 로 넘어간다.
  //
  // 스테이트로 승격하지 않은 이유는 mgm_types.hpp 의 MGM_ESCAPE_* 주석 참조.
  //
  // 안전 불변식 5개 — 이 중 하나라도 무너지면 후진하지 않는다:
  //  ① v_escape < 0        — 탈출 페이즈는 구조적으로 전진할 수 없다
  //  ② escape_max_cycles>0 — 상한 없는 후진은 만들지 않는다
  //  ③ escape_armed        — 한 번이라도 굴러간 뒤에만 "갇혔다"고 판정한다
  //                          (벽을 보고 launch → 출발 인가 전 자동 후진 차단)
  //  ④ estop_latch_release — **실제** EstopRequest 만 센다. §5.7 watchdog 보정이나
  //                          wait_go 대기로 걸린 estop 은 교착이 아니라 안전 장치다
  //  ⑤ 후방 여유           — escape_require_rear_clear 가 켜져 있으면 rear_clear 필수
  const bool escape_usable =
    st.params.escape_after_cycles > 0 &&
    st.params.escape_max_cycles > 0 &&
    st.params.v_escape < 0.0f;

  // 주행 무장 — 직전 틱의 명령 속도가 0을 넘은 적이 있는가(transition 시점의
  // st.v 는 아직 이전 틱 값이다). 벽을 마주 보고 launch 하면 첫 틱부터 실제
  // estop 이 참이라 v 가 0 에서 벗어나지 못하고, 그래서 영원히 무장되지 않는다.
  if (st.v > kStoppedSpeed) {
    st.escape_armed = true;
  }

  // 실제 estop 연속 틱. wrapper 보정이 섞인 s.estop 이 아니라 s.estop_latch_release
  // 를 쓰는 것이 ④의 핵심이다 — 이 필드는 "신선한 실제 EstopRequest 의 estop 값"이다.
  if (s.estop_latch_release) {
    ++st.estop_hold_cnt;
  } else {
    st.estop_hold_cnt = 0;
  }

  // 후방 여유 게이트 — 진입뿐 아니라 **후진 중에도 매 틱 다시 본다**.
  // 후진하는 동안 뒤에 뭔가 들어오면 그 자리에서 멈춰야 한다.
  const bool rear_ok =
    (st.params.escape_require_rear_clear == 0) || s.estop_rear_clear;

  if (st.escape_phase == MGM_ESCAPE_REVERSING) {
    ++st.escape_ticks;
    // 종료 조건 4개: 시간 상한 · 후방 막힘 · estop 해제(장애물이 사라짐) ·
    // 기능이 런타임에 꺼짐. 어느 쪽이든 페이즈를 닫고 카운터를 리셋한다 —
    // 리셋 덕분에 다시 갇히면 escape_after_cycles 를 새로 채워야 후진한다
    // (연속 후진으로 트랙에서 무한히 멀어지는 것을 시간으로 막는다).
    if (!escape_usable || !rear_ok || !s.estop_latch_release ||
      st.escape_ticks >= st.params.escape_max_cycles)
    {
      st.escape_phase = MGM_ESCAPE_NONE;
      st.escape_ticks = 0;
      st.estop_hold_cnt = 0;
    }
  }

  // 진입 판정 — 위 5개 불변식 + 연속 유지 시간. PARKING 은 제외한다(주차는
  // parking_v_suggest 로 자체 후진을 하며, 그 판단은 stack_parking 소관이다).
  const bool escape_entry =
    escape_usable && st.escape_armed && rear_ok &&
    st.escape_phase == MGM_ESCAPE_NONE &&
    st.state != MGM_STATE_PARKING &&
    st.state != MGM_STATE_TRAFFIC &&
    st.estop_hold_cnt >= st.params.escape_after_cycles;

  // avoid 복귀 보류 카운터 — waypoint에서 GPS 트랙에 재합류할 시간을 벌어준다
  if (st.return_hold_left > 0) {
    --st.return_hold_left;
  }

  // 트랙 재합류 판정 (waypoint→lane 게이트). gps_path가 유효할 때만 의미가
  // 있으므로 점이 없으면 미합류로 본다 — 신선도/유효성 보정은 wrapper가
  // 이미 끝낸 뒤다(§5.7 ②). 임계 0 이하면 게이트 끔(구동작).
  const bool rejoined =
    (st.params.lane_entry_max_cross <= 0.0f) ||
    (s.gps_path.n > 0 && s.gps_cross_track <= st.params.lane_entry_max_cross);

  // AVOID 진입 게이트 (2026-08-18) — avoid_zone_only 를 켜면 **회피 허용 구간
  // 안에서만** 전이한다. 구간 밖 장애물은 회피 대신 stack_estop 정지로 대응한다.
  // 이미 AVOID 중이면 관여하지 않는다 — 기동 중에 구간을 벗어났다고 회피를
  // 중도 포기하면 장애물 옆에서 트랙으로 되꺾는 꼴이 된다. 이탈 상한은
  // avoid_max_cycles 가 따로 지킨다.
  const bool avoid_entry =
    s.avoid_obstacle_detected && s.avoid_avoidable &&
    (st.params.avoid_zone_only == 0 || s.gps_avoid_zone);

  // GPS 전용 구간 (2026-08-18) — 이 구간 안에서는 차선으로 넘어가지 않는다.
  // 판정에 gps_path 유효성을 함께 요구한다: 플래그는 fix 가 있어야 계산되고,
  // 무효한 gps 로 WAYPOINT 를 강제하면 §5.7 ②의 gps watchdog 이 estop 을 걸어
  // 되레 멈춰 세운다. 래치하지 않는다 — 위치로 정해지는 값이라 구간을 벗어나면
  // 평소 히스테리시스(신뢰도 N주기 + 트랙 재합류)로 자연 복귀한다.
  const bool gps_only_zone = s.gps_gps_only_zone && s.gps_path.n > 0;
  // 신호등 정지 상태는 확정 적색과 현재의 안정적인 정지선 검출이 동시에
  // 성립할 때만 진입한다. 적색만 보인 교차로나 정지선만 보인 일반 노면에서
  // TRAFFIC으로 잘못 전이하지 않는다. 진입 후에는 정지선이 카메라 아래로
  // 사라져도 거리 적분을 계속해야 하므로, 해제 조건은 기존처럼 확정 초록이다.
  const bool traffic_entry = st.params.traffic_state_enabled != 0 &&
    s.traffic_red_active && s.traffic_stopline_detected;
  // ★ 구간 안에서는 LANE 복귀 카운터를 **세지 않는다**(0으로 묶는다).
  //   안 그러면 구간을 지나는 내내 쌓인 값으로 **벗어나는 순간 한 틱 만에** LANE이
  //   된다 — 차선을 못 믿겠다고 지정한 구간을 막 빠져나온 참에, 그 구간 동안의
  //   신뢰도로 곧장 카메라를 믿는 꼴이다. 2026-08-14 avoid 복귀에서 겪은 것과
  //   같은 유형의 버그다(CLAUDE.md §4 히스테리시스 규약: "N주기 연속"은 실제로
  //   전이가 가능한 동안 세어야 한다). 구간을 벗어난 뒤 새로 n_cycles(0.5s) 동안
  //   신뢰도가 유지되고 트랙에도 붙어 있어야 LANE으로 간다.
  if (gps_only_zone) {
    st.lane_high_cnt = 0;
  }

  const uint8_t prev_state = st.state;

  // 후진 탈출 개시 — 어느 주행 스테이트에 있든 AVOID 로 들어가 후진 페이즈를 연다.
  // 아래 스테이트별 전이표보다 **먼저** 본다: 교착을 끊는 동작이라, 그 교착을
  // 만든 스테이트의 평시 전이 조건에 종속시키면 의미가 없다.
  //
  // AVOID 로 들어가는 이유는 두 가지다. ① 후진의 목적 자체가 "회피가 성립하는
  // 거리를 만드는 것"이라 후진은 회피 기동의 첫 단계다 — 물러난 뒤 그대로 AVOID
  // 안에 있으므로 별도 인수인계가 필요 없다. ② 스테이트가 CAN flags 로 나가므로
  // 후진이 로그·dSPACE 양쪽에서 관측된다.
  //
  // ⚠ avoid_zone_only 게이트는 **적용하지 않는다.** 그 스위치는 "평시 회피를
  //   지정 구간에만 쓴다"는 운용 선택이지, 구간 밖에서 갇힌 차를 갇힌 채로 두라는
  //   뜻이 아니다. 그렇게 하면 이 기능이 풀려는 교착을 스위치가 다시 만든다.
  if (escape_entry) {
    st.state = MGM_STATE_AVOID;
    st.escape_phase = MGM_ESCAPE_REVERSING;
    st.escape_ticks = 0;
  }

  switch (st.state) {
    case MGM_STATE_LANE:
      if (traffic_entry) {
        st.state = MGM_STATE_TRAFFIC;
      } else if (s.gps_parking_zone && s.parking_space_found) {
        st.state = MGM_STATE_PARKING;
      } else if (avoid_entry) {
        st.state = MGM_STATE_AVOID;
      } else if (gps_only_zone || st.lane_low_cnt >= st.params.n_cycles) {
        // 구간 진입은 **즉시** 내려간다(히스테리시스 없음) — 차선을 못 믿는
        // 구간이라고 사람이 지정한 곳이므로, 신뢰도가 높게 나오는 동안 기다릴
        // 이유가 없다. 나가는 쪽은 평소 조건(신뢰도 + 재합류)을 그대로 지킨다.
        st.state = MGM_STATE_WAYPOINT;
      }
      break;

    case MGM_STATE_WAYPOINT:
      if (traffic_entry) {
        st.state = MGM_STATE_TRAFFIC;
      } else if (s.gps_parking_zone && s.parking_space_found) {
        st.state = MGM_STATE_PARKING;
      } else if (avoid_entry) {
        st.state = MGM_STATE_AVOID;
      } else if (!gps_only_zone && st.return_hold_left == 0 &&
        st.lane_high_cnt >= st.params.n_cycles && rejoined)
      {
        // 전이 조건 3개: ① 복귀 보류 시간 경과 ② 차선 신뢰도 히스테리시스
        // ③ **트랙에 실제로 재합류** — ③이 없으면 회피로 이탈한 채 카메라로
        //    넘어가, 트랙 복귀는 영영 못 하고 차선만 보고 간다 (§4).
        st.state = MGM_STATE_LANE;
      }
      break;

    case MGM_STATE_AVOID:
      // 기동 완료 → waypoint로 복귀 (§4, 2026-08-12 개정 — 회피 직후 차선 검출은
      // 신뢰 불가(차로 이탈 상태). GPS 트랙 재합류 후 lane은 신뢰도 히스테리시스로
      // 자연 재전이. 구 복귀처 변수(진입 스테이트 기억)는 폐기.
      // 상한(avoid_max_cycles) 초과 시에도 복귀 — AVOID는 직진 유지라 무한히
      // 지속되면 트랙에서 무한히 멀어진다 (mgm_types.hpp 주석의 실측 근거).
      //
      // 후진 페이즈 중에는 나가지 않는다 (2026-08-24). 두 출구 모두 후진에는
      // 뜻이 맞지 않는다: maneuver_done 은 후진 직전 값이 그대로 남아 있을 수
      // 있어 개시한 틱에 곧바로 튕겨 나가고, avoid_max_cycles 는 회피 기동의
      // 이탈 상한이지 후진의 상한이 아니다. 후진의 상한은 escape_max_cycles 가
      // 따로 지키며, 그 페이즈가 닫힌 뒤 아래 조건이 평소대로 적용된다.
      if (st.escape_phase == MGM_ESCAPE_NONE &&
        (s.avoid_maneuver_done ||
        (st.params.avoid_max_cycles > 0 && st.avoid_ticks >= st.params.avoid_max_cycles)))
      {
        st.state = MGM_STATE_WAYPOINT;
        st.return_hold_left = st.params.avoid_return_hold_cycles;
      }
      break;

    case MGM_STATE_PARKING:
      // parking→avoid 전이 없음 = 주차 중 회피 금지가 구조적으로 보장 (§4)
      if (s.parking_done) {
        st.state = (
          st.parking_entry_state == MGM_STATE_WAYPOINT ?
          MGM_STATE_WAYPOINT : MGM_STATE_LANE);
      }
      break;

    case MGM_STATE_TRAFFIC:
      // 적색/미검출은 해제 근거가 아니다. 확정 초록에서 진입 전 주행 상태로
      // 복귀한다. WAYPOINT에서 들어왔는데 LANE으로 고정 복귀하면 GPS 전용
      // 구간에서도 한 틱 동안 신뢰하지 않는 차선 경로가 출력되고 다음 틱 다시
      // WAYPOINT로 튀는 문제가 생긴다. 저장값은 TRAFFIC의 유일한 진입원인
      // LANE/WAYPOINT만 허용하고, 손상된 값은 안전한 기존 기본인 LANE으로 둔다.
      // estop/fail-safe가 동시에 참이어도 상태 전이는 수행하고 복귀 상태의
      // 우선권에서 계속 정지한다. 즉 "초록이면 상태 탈출"과 "고장 중 출발 금지"가
      // 양립한다.
      if (s.traffic_green_active && !s.traffic_red_active) {
        st.state = st.traffic_entry_state == MGM_STATE_WAYPOINT ?
          MGM_STATE_WAYPOINT : MGM_STATE_LANE;
      }
      break;

    default:
      st.state = MGM_STATE_LANE;
      break;
  }

  // TRAFFIC 거리 메모리 (2026-09-02 개정, 사용자 지정 — 정지선이 화면에서
  // 사라지는 edge 기준). 스테이트와 무관하게 **항상** 추적한다 — 빨간불이
  // 뜨기 전에도 정지선을 스쳐 지나갈 수 있고, 그때 쌓인 낡은 값을 아래 가드가
  // 걸러야 하기 때문이다.
  //   ① 정지선 검출이 true→false로 떨어지는 순간(=화면에서 사라짐) 거리를
  //      seed(traffic_ramp_distance_m, 기본 1.5m)로 리셋한다. 카메라
  //      optical-Z 거리는 검출이 불안정하면 즉시 무효가 되어 그 자체로는 못
  //      쓴다 — 정지선이 사라지는 지점은 카메라 장착 기준 대략 고정된
  //      거리라는 사실을 시드로 쓴다(bridge_dspace/tools/
  //      camera_traffic_ref_test.py로 벤치에서 검증).
  //   ② 그 뒤로는 실측 차속(vehicle_speed)으로 dead-reckoning 감쇠한다.
  //   ③ 빨간불이 아직 확정 안 된 채(=이번 정지 판정 전) 감쇠값이
  //      traffic_stop_offset(기본 0.5m) 이하로 떨어지면, 무관한 정지선을
  //      스쳐 지나가며 쌓인 낡은 값이 나중에 빨간불이 뜨는 순간 그대로
  //      급정지로 이어지는 것을 막기 위해 seed로 되돌리고 그 상태를
  //      붙잡아둔다 — 실질적으로 "seed에서 1m(=0.5m 남을 때까지) 이상 진행한
  //      상태로 빨간불이 확정돼야만" 실제 정지가 성립한다.
  if (st.traffic_prev_stopline_detected && !s.traffic_stopline_detected) {
    st.traffic_stopline_distance = st.params.traffic_ramp_distance_m;
    st.traffic_distance_latched = true;
  }
  st.traffic_prev_stopline_detected = s.traffic_stopline_detected;
  if (st.traffic_distance_latched && s.vehicle_speed_valid) {
    st.traffic_stopline_distance =
      max_f(0.0f, st.traffic_stopline_distance - s.vehicle_speed * MGM_PERIOD_S);
  }
  if (!s.traffic_red_active && st.traffic_distance_latched &&
    st.traffic_stopline_distance <= st.params.traffic_stop_offset)
  {
    st.traffic_stopline_distance = st.params.traffic_ramp_distance_m;
  }
  if (st.state == MGM_STATE_TRAFFIC && prev_state != MGM_STATE_TRAFFIC) {
    st.traffic_entry_state = prev_state;
  } else if (st.state != MGM_STATE_TRAFFIC && prev_state == MGM_STATE_TRAFFIC) {
    // 초록으로 빠져나가면 다음 신호를 위해 완전히 새로 시작한다.
    st.traffic_distance_latched = false;
    st.traffic_stopline_distance = 0.0f;
    st.traffic_prev_stopline_detected = false;
  }

  // 스테이트가 바뀌면 히스테리시스 카운터를 리셋한다 — "N주기 연속"은 **새
  // 스테이트 안에서** 세어야 의미가 있다. 리셋이 없으면 이전 스테이트에 있는
  // 동안 쌓인 값으로 진입 즉시 되튄다 (2026-08-14 run_0814_184624: AVOID 5~9초
  // 기동 중 lane_high_cnt가 500~900까지 누적 → waypoint 복귀 한 틱 만에 lane 전이,
  // 110초에 전이 22회·횡오차 8m 발산).
  if (st.state != prev_state) {
    if (st.state == MGM_STATE_PARKING) {
      st.parking_entry_state = prev_state;
    }
    st.lane_low_cnt = 0;
    st.lane_high_cnt = 0;
    st.wrongway_cnt = 0;
    st.wrongway_ok_cnt = 0;   // 래치(wrongway_latched)는 리셋하지 않는다 — 스테이트가
                              // 바뀐다고 차가 트랙 방향으로 돌아선 것은 아니다.
  }
  // AVOID 지속 틱 — 상한 판정용. AVOID를 벗어나면 0으로 (재진입 시 다시 셈).
  // 후진 페이즈 동안에는 세지 않는다 — avoid_max_cycles 는 회피 기동이 트랙에서
  // 얼마나 멀어져도 되는지의 상한이라, 물러나는 데 쓴 시간을 거기서 빼면 정작
  // 회피에 쓸 창이 줄어든다. 후진이 끝난 시점부터 온전한 창으로 다시 센다.
  st.avoid_ticks =
    (st.state == MGM_STATE_AVOID && st.escape_phase == MGM_ESCAPE_NONE) ?
    st.avoid_ticks + 1 : 0;
}

// ── 판단: 스테이트 내부 우선권 (§4 우선권 표 — 전역 min/max 금지)
void prioritize(const CoreSnapshot & s, const CoreState & st, CoreOutput & out)
{
  out.state = st.state;
  out.immediate_stop = false;

  switch (st.state) {
    case MGM_STATE_LANE:
    case MGM_STATE_WAYPOINT:
      out.path_source = (st.state == MGM_STATE_LANE) ? MGM_SRC_LANE : MGM_SRC_GPS;
      // 종방향 우선권: 긴급 정지 > 신호등 정지 > 트랙 종점(waypoint만) > 가속구간 > 기본 속도
      if (s.estop) {
        out.v_ref = 0.0f;
        out.immediate_stop = true;
      } else if (s.traffic_stop_required) {
        // traffic_state_enabled=false(생성 v1.88 등 MGM_STATE_TRAFFIC 미지원
        // 백엔드)일 때의 안전망 — 그 경우엔 traffic_entry가 절대 서지 않아
        // LANE/WAYPOINT에 계속 머무르므로, stack_traffic 자체의 적색+근접
        // 래치만으로 즉시 정지한다. TRAFFIC 스테이트가 켜져 있으면 traffic_entry가
        // 이보다 먼저 상태를 옮겨 이 분기는 사실상 안 쓰인다(§4 우선권 표).
        out.v_ref = 0.0f;  // 일반 감속 정지 (rate limit 적용)
      } else if (st.at_end_latched) {
        // 트랙 종점(래치) — 통과·밀림 시 유턴 방지 (§4, 2026-08-03).
        // **lane·waypoint 공통** (2026-08-15). 래치는 transition()이 이 틱에서
        // 이미 갱신했으므로 여기서 s.gps_at_end를 또 볼 필요가 없다 — 유효성
        // 게이트(gps_path.n>0)·parking 제외도 거기 한 곳에만 둔다.
        out.v_ref = 0.0f;
      } else if (st.state == MGM_STATE_WAYPOINT && st.wrongway_latched) {
        out.v_ref = 0.0f;  // 역방향 — 경로를 등진 채 주행 금지 (§4, 2026-08-03)
      } else if (st.stop_zone_holding) {
        // 지정 지점 정지 (2026-08-18) — 신호등 정지와 같은 **일반 감속 정지**다
        // (immediate_stop 아님 → a_down rate limit 적용). 판정·타이머는
        // transition()에 있고 여기서는 그 결정을 속도로 옮기기만 한다.
        out.v_ref = 0.0f;
      } else if (s.gps_accel_zone) {
        out.v_ref = st.params.v_accel_zone;
      } else {
        out.v_ref = st.params.v_base;
      }
      break;

    case MGM_STATE_AVOID:
      // ── 후진 탈출 페이즈 (2026-08-24) — AVOID 우선권 표의 **최상위**.
      //
      // 여기가 이 기능에서 가장 조심해야 할 지점이다: 후진 중에는 estop 이
      // 참인 채로 차를 움직인다. estop 을 무시하는 유일한 자리이므로, 그
      // 무시가 안전한 이유를 조건이 아니라 **구조**로 보장한다.
      //   · v_escape 는 음수임이 transition() 의 escape_usable 에서 이미
      //     확인됐다 → 이 분기는 전진 명령을 낼 수 없다. estop 을 건 장애물은
      //     차 앞에 있고, 우리는 그 반대로만 간다.
      //   · 경로는 인지가 준 것이 아니라 조립 블록이 만드는 직선이다
      //     (MGM_SRC_ESCAPE) → 후진 중 조향은 중립이다.
      //   · 시간 상한·후방 여유·estop 해제는 transition() 이 매 틱 다시 보고
      //     페이즈를 닫는다 → 이 분기는 그 판정 결과를 속도로 옮기기만 한다.
      //
      // immediate_stop 을 세우지 않는 이유: rate limit 을 그대로 태워 0 →
      // v_escape 로 완만히 물러나기 위해서다. 급후진은 그 자체가 위험하다.
      if (st.escape_phase == MGM_ESCAPE_REVERSING) {
        out.path_source = MGM_SRC_ESCAPE;
        out.v_ref = st.params.v_escape;
        break;
      }
      out.path_source = MGM_SRC_AVOID;
      // 기동 완료 우선 — 신호등 정지 요구는 기동 이탈 후 적용 (여기서 참조하지 않음).
      // 안전 바닥: TTC < 임계 또는 긴급 정지 → 즉시 정지 (우선권 표 최상위).
      if (s.estop || s.avoid_ttc < st.params.ttc_stop) {
        out.v_ref = 0.0f;
        out.immediate_stop = true;
      } else {
        // 종방향은 회피 기하가 결정, 여유 폭 좁으면 감속
        out.v_ref = s.avoid_narrow_gap ?
          min_f(s.avoid_v_suggest, st.params.v_narrow) : s.avoid_v_suggest;
        // AVOID 전용 속도 상한 (§4 스테이트별 속도). 0 이하면 상한 없음 = 구동작.
        // stack_avoid의 target_speed_mps를 내리지 않는 이유는 mgm_types.hpp의
        // v_avoid 주석 참조 — 그 값은 TTC 자차속도도 겸해서 내리면 TTC가 부풀어 오른다.
        if (st.params.v_avoid > 0.0f) {
          out.v_ref = min_f(out.v_ref, st.params.v_avoid);
        }
      }
      break;

    case MGM_STATE_PARKING:
      out.path_source = MGM_SRC_PARKING;
      // 경로 침범 정지 > 주차 진행. 신호등·가속구간 요구 비활성.
      if (s.estop) {
        out.v_ref = 0.0f;
        out.immediate_stop = true;
      } else if (s.parking_path_blocked) {
        out.v_ref = 0.0f;
      } else {
        out.v_ref = s.parking_v_suggest;
      }
      break;

    case MGM_STATE_TRAFFIC: {
      // 종방향만 정지선 거리가 맡고, 횡방향은 TRAFFIC 진입 전 주행 소스를
      // 그대로 유지한다. GPS 전용 구간의 WAYPOINT에서 들어온 경우 카메라
      // 차선으로 바꾸면 안 된다. 잘못된 저장값은 LANE으로 fail-closed한다.
      out.path_source = st.traffic_entry_state == MGM_STATE_WAYPOINT ?
        MGM_SRC_GPS : MGM_SRC_LANE;
      if (s.estop || s.traffic_fail_safe_stop || !s.vehicle_speed_valid) {
        out.v_ref = 0.0f;
        out.immediate_stop = true;
      } else if (!st.traffic_distance_latched) {
        // 정지선을 아직 한 번도 안정 검출하지 못했다 — 거리를 모른다.
        // 2026-09-02 확정(사용자 지정): 정지선을 못 봤으면(=미인지 상태) 감속
        // 없이 v_base로 그냥 통과한다. 한때 안전 쪽 폴백으로 즉시 정지를
        // 넣었었는데(정지선 인식이 아예 실패하는 조건이 실제로 있어 신호를
        // 지나칠 위험을 우려), 검증 단계인 지금은 정지선 인지 자체가 아직
        // 미덥지 않은 채라 "못 봤으면 무조건 정지"가 오히려 관련 없는 곳에서
        // 잦은 오정지를 만든다는 판단으로 되돌렸다.
        out.v_ref = st.params.v_base;
      } else if (
        st.traffic_stopline_distance <= st.params.traffic_stop_offset ||
        st.params.traffic_ramp_distance_m <= 0.0f)
      {
        out.v_ref = 0.0f;
      } else {
        out.v_ref = clamp01(
          st.traffic_stopline_distance / st.params.traffic_ramp_distance_m) *
          st.params.v_base;
      }
      break;
    }

    default:
      out.path_source = MGM_SRC_LANE;
      out.v_ref = 0.0f;
      break;
  }
}

const CorePath * select_path(uint8_t src, const CoreSnapshot & s)
{
  switch (src) {
    case MGM_SRC_LANE: return &s.lane_path;
    case MGM_SRC_GPS: return &s.gps_path;
    case MGM_SRC_AVOID: return &s.avoid_path;
    case MGM_SRC_PARKING: return &s.parking_path;
    default: return nullptr;
  }
}

// 후진 탈출 ref — 차 앞으로 곧게 뻗은 등간격 20점 (y=0, yaw=0, κ=0).
// 인지가 준 경로가 아니라 조립 블록이 만드는 고정 기하이며, 판단이 아니라
// 스테이트가 고른 소스를 포맷으로 옮기는 일이다 (§5.1 허용 업무).
//
// **왜 전진용 ref 를 그대로 두고 v_ref 만 뒤집지 않는가:** 후진 중에도 조향은
// ref 를 따라간다. 차 앞 왼쪽에 있는 목표점을 그대로 둔 채 뒤로 가면 차는
// 그 목표에서 멀어지는 쪽으로 꺾인다 — 물러나면서 엉뚱한 방향으로 돌아버린다.
// 곧게 빼는 것이 후진 탈출에서 유일하게 예측 가능한 기하다.
constexpr float kEscapeRefSpanM = 1.5f;

void build_escape_ref(CorePoint * out)
{
  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    const float t = static_cast<float>(i + 1) / static_cast<float>(MGM_NUM_POINTS);
    out[i].x = kEscapeRefSpanM * t;
    out[i].y = 0.0f;
    out[i].yaw = 0.0f;
    out[i].curvature = 0.0f;
  }
}

// ── 실행 1: ref 조립 — 스테이트가 고른 경로를 채택(유효 n_out개), 전환 시 블렌드 (§5.6)
void assemble(const CoreSnapshot & s, uint8_t src, CoreState & st)
{
  const CorePath * path = select_path(src, s);
  if (src != MGM_SRC_ESCAPE && (path == nullptr || path->n <= 0)) {
    return;  // 선택 소스 미도착 → 직전 출력(ref_out) 유지 (판단 아님 — 데이터 hold)
  }

  // 내부 배열은 20 고정(블렌드 계산용) — 부족분은 마지막 점 복제.
  // 출력 유효분은 n_out개 (와이어에는 유효 점만 실린다 — PROTOCOL.md)
  CorePoint target[MGM_NUM_POINTS];
  const bool escape = (src == MGM_SRC_ESCAPE);
  if (escape) {
    build_escape_ref(target);
  }
  const int32_t n = escape ? MGM_NUM_POINTS :
    (path->n < MGM_NUM_POINTS ? path->n : MGM_NUM_POINTS);
  const int32_t last = n - 1;
  if (!escape) {
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      target[i] = path->pts[i < last ? i : last];
    }
  }

  // 단일 목표점 소스(avoid)는 원점→목표 직선 보간으로 20점 경로화 (§5.1 포맷 변환).
  // 근거 (2026-08-12 실차, run_0812_234253): 같은 run·같은 속도(v_ref 0.44)에서
  // 20점(gps)=조향 정상 / 1점(avoid)=str 무반응으로 콘에 직진 → estop. 2026-08-08
  // "1점=str 무반응" 실측의 재확인이며, "원인은 저속"이라는 2026-08-10 재해석을 반증.
  // dSPACE 수정 없이 PC 조립에서 해결 — 와이어에는 항상 다점이 실린다.
  int32_t n_wire = n;
  if (n == 1) {
    const CorePoint tgt = path->pts[0];
    const float yaw = atan2f(tgt.y, tgt.x);
    // ★ 등간격(첫 점 = 목표/20 ≈ 7.5cm)은 **의도적으로 유지한다.** 첫 점이
    //   원점에 가까우면 dSPACE가 보는 도달 곡률 κ=2y/L² 이 실제 경로 곡률의
    //   10~20배로 부풀려지는데, 이 부풀림이 **회피 기동을 성립시키는 일을 하고
    //   있다** — dSPACE가 명령 곡률의 10~55%만 실현하기 때문이다(CLAUDE.md §3 ③).
    //   2026-08-15에 이걸 "수치 결함"으로 보고 첫 점을 1.2m로 옮겼다가 같은
    //   크기의 회피 목표(|y|≈0.29m)에서 조향이 반토막 났다:
    //     첫 점 0.075m (run_0815_144142) 헤딩 +13.0° · |str| 0.089 · 횡변위 0.26m → 통과
    //     첫 점 1.2m   (run_0815_153633) 헤딩  +4.3° · |str| 0.047 · 횡변위 0.08m → estop
    //   방위는 어느 쪽이든 보존되므로 차이는 순수하게 곡률 크기다.
    //   **근본 해결은 dSPACE MPC 쪽**(지평 200ms×0.5m/s=0.1m)이며, 그게 잡히기
    //   전에는 여기를 "정직한 기하"로 되돌리지 말 것.
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      const float t = static_cast<float>(i + 1) / static_cast<float>(MGM_NUM_POINTS);
      target[i].x = tgt.x * t;
      target[i].y = tgt.y * t;
      target[i].yaw = yaw;
      target[i].curvature = 0.0f;
    }
    n_wire = MGM_NUM_POINTS;
  }

  // 새 인지/GNSS 표본이 아직 오지 않아 wrapper가 같은 스냅샷을 다시 준 틱을 판정한다.
  // 10 ms CAN 송신 주기는 유지하되 ref는 다음 표본까지 그대로 hold한다. 새 표본 여부는
  // s.*_updated가 정본이고, 값 비교는 갱신 플래그가 없는 옛 스냅샷의 호환 경로다.
  // MGM_SRC_ESCAPE는 인지 입력이 아니라 이 함수가 매 틱 만드는 경로이므로 항상 최신이다.
  const bool src_updated =
    (src == MGM_SRC_LANE) ? s.lane_updated :
    (src == MGM_SRC_GPS) ? s.gps_updated :
    (src == MGM_SRC_AVOID) ? s.avoid_updated :
    (src == MGM_SRC_ESCAPE) ? true : false;
  const bool is_stale_repeat = !src_updated && st.has_raw_target && st.raw_n == n &&
    paths_equal(target, st.last_raw_target, n);

  st.n_out = n_wire;

  if (src != st.last_src) {  // 스테이트 전환 → ref 불연속 방지 블렌드 시작
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      st.blend_from[i] = st.ref_out[i];
    }
    st.blend_left = st.params.blend_cycles;
    st.last_src = src;
  }

  if (st.blend_left > 0) {
    const float a = 1.0f -
      static_cast<float>(st.blend_left) / static_cast<float>(st.params.blend_cycles + 1);
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      st.ref_out[i].x = lerp(st.blend_from[i].x, target[i].x, a);
      st.ref_out[i].y = lerp(st.blend_from[i].y, target[i].y, a);
      st.ref_out[i].yaw = lerp(st.blend_from[i].yaw, target[i].yaw, a);
      st.ref_out[i].curvature = lerp(st.blend_from[i].curvature, target[i].curvature, a);
    }
    --st.blend_left;
  } else if (is_stale_repeat) {
    // Hold the last ref until the next perception/GNSS sample. The previous
    // v_cmd-based 10 ms extrapolation was not a measured pose correction.
  } else {
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      st.ref_out[i] = target[i];
    }
  }

  if (!is_stale_repeat) {
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      st.last_raw_target[i] = target[i];
    }
    st.raw_n = n;
    st.has_raw_target = true;
  }
}

// ── 실행 2: 종방향 병합 — rate limit만. immediate_stop은 스테이트의 결정으로 우회.
float merge(const CoreOutput & d, CoreState & st)
{
  if (d.immediate_stop) {
    st.v = 0.0f;  // 긴급 정지·TTC 바닥은 램프 없이 즉시 (스테이트 머신이 결정)
    return st.v;
  }
  const float lo = st.v - st.params.a_down * MGM_PERIOD_S;
  const float hi = st.v + st.params.a_up * MGM_PERIOD_S;
  float v = d.v_ref;
  if (v < lo) {v = lo;}
  if (v > hi) {v = hi;}
  st.v = v;
  return st.v;
}

}  // namespace

void mgm_init(CoreState & st, const CoreParams & params)
{
  st = CoreState{};
  st.params = params;
  st.state = MGM_STATE_LANE;
  st.traffic_entry_state = MGM_STATE_LANE;
  st.last_src = MGM_SRC_LANE;
  st.n_out = 1;
  // ref_out은 전부 (0,0,0,0) — 인지 도착 전: 제자리 점 1개 (v_ref가 어차피 속도를 지배)
}

CoreOutput mgm_step(const CoreSnapshot & in, CoreState & st)
{
  CoreOutput out{};

  transition(in, st);        // 판단: 전이
  prioritize(in, st, out);   // 판단: 우선권 → v_ref 요구·경로 소스·immediate_stop
  assemble(in, out.path_source, st);  // 실행: 조립
  out.v_ref = merge(out, st);         // 실행: 병합 (rate limit)

  out.n_points = st.n_out;
  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    out.ref_points[i] = st.ref_out[i];
  }
  return out;
}

}  // namespace adas_mgm
