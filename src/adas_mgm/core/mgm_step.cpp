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

  // 역방향 카운터 (§4 waypoint) — GPS 경로 첫 점의 상대 yaw가 임계를 넘으면
  // 차가 경로를 등진 것 (유턴 후 트랙 역추종, 2026-08-03 2회 재현)
  const float y0 = (s.gps_path.n > 0) ? s.gps_path.pts[0].yaw : 0.0f;
  const bool wrongway = (y0 > st.params.wrongway_yaw) || (y0 < -st.params.wrongway_yaw);
  st.wrongway_cnt = wrongway ? st.wrongway_cnt + 1 : 0;

  // 종점 래치 (§4) — 정지 후 미세하게 밀려 최근접점이 뒤로 바뀌면 at_end가
  // 풀려 재출발·유턴하던 것 방지 (2026-08-03 직선 run 실사례). 해제는 **실제
  // EstopRequest 인가**(= run 종료/새 run 준비)로만 — s.estop은 wrapper의
  // staleness 보정이 섞여 있어, gps 단절→복구 같은 일시 장애로 래치가 풀려
  // 재출발하는 구멍이 있었다 (2026-08-11, CLAUDE.md §4 래치).
  if (s.estop_latch_release) {
    st.at_end_latched = false;
  } else if (st.state == MGM_STATE_WAYPOINT && s.gps_at_end) {
    st.at_end_latched = true;
  }

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

  const uint8_t prev_state = st.state;

  switch (st.state) {
    case MGM_STATE_LANE:
      if (s.gps_parking_zone && s.parking_space_found) {
        st.state = MGM_STATE_PARKING;
      } else if (s.avoid_obstacle_detected && s.avoid_avoidable) {
        st.state = MGM_STATE_AVOID;
      } else if (st.lane_low_cnt >= st.params.n_cycles) {
        st.state = MGM_STATE_WAYPOINT;
      }
      break;

    case MGM_STATE_WAYPOINT:
      if (s.avoid_obstacle_detected && s.avoid_avoidable) {
        st.state = MGM_STATE_AVOID;
      } else if (st.return_hold_left == 0 &&
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
      if (s.avoid_maneuver_done) {
        st.state = MGM_STATE_WAYPOINT;
        st.return_hold_left = st.params.avoid_return_hold_cycles;
      }
      break;

    case MGM_STATE_PARKING:
      // parking→avoid 전이 없음 = 주차 중 회피 금지가 구조적으로 보장 (§4)
      if (s.parking_done) {
        st.state = MGM_STATE_LANE;
      }
      break;

    default:
      st.state = MGM_STATE_LANE;
      break;
  }

  // 스테이트가 바뀌면 히스테리시스 카운터를 리셋한다 — "N주기 연속"은 **새
  // 스테이트 안에서** 세어야 의미가 있다. 리셋이 없으면 이전 스테이트에 있는
  // 동안 쌓인 값으로 진입 즉시 되튄다 (2026-08-14 run_0814_184624: AVOID 5~9초
  // 기동 중 lane_high_cnt가 500~900까지 누적 → waypoint 복귀 한 틱 만에 lane 전이,
  // 110초에 전이 22회·횡오차 8m 발산).
  if (st.state != prev_state) {
    st.lane_low_cnt = 0;
    st.lane_high_cnt = 0;
    st.wrongway_cnt = 0;
  }
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
        out.v_ref = 0.0f;  // 일반 감속 정지 (rate limit 적용)
      } else if (st.state == MGM_STATE_WAYPOINT &&
        (s.gps_at_end || st.at_end_latched))
      {
        out.v_ref = 0.0f;  // 트랙 종점(래치) — 통과·밀림 시 유턴 방지 (§4, 2026-08-03)
      } else if (st.state == MGM_STATE_WAYPOINT &&
        st.wrongway_cnt >= st.params.wrongway_cycles)
      {
        out.v_ref = 0.0f;  // 역방향 — 경로를 등진 채 주행 금지 (§4, 2026-08-03)
      } else if (s.gps_accel_zone) {
        out.v_ref = st.params.v_accel_zone;
      } else {
        out.v_ref = st.params.v_base;
      }
      break;

    case MGM_STATE_AVOID:
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

// ── 실행 1: ref 조립 — 스테이트가 고른 경로를 채택(유효 n_out개), 전환 시 블렌드 (§5.6)
void assemble(const CoreSnapshot & s, uint8_t src, CoreState & st)
{
  const CorePath * path = select_path(src, s);
  if (path == nullptr || path->n <= 0) {
    return;  // 선택 소스 미도착 → 직전 출력(ref_out) 유지 (판단 아님 — 데이터 hold)
  }

  // 내부 배열은 20 고정(블렌드 계산용) — 부족분은 마지막 점 복제.
  // 출력 유효분은 n_out개 (와이어에는 유효 점만 실린다 — PROTOCOL.md)
  CorePoint target[MGM_NUM_POINTS];
  const int32_t n = path->n < MGM_NUM_POINTS ? path->n : MGM_NUM_POINTS;
  const int32_t last = n - 1;
  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    target[i] = path->pts[i < last ? i : last];
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
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      const float t = static_cast<float>(i + 1) / static_cast<float>(MGM_NUM_POINTS);
      target[i].x = tgt.x * t;
      target[i].y = tgt.y * t;
      target[i].yaw = yaw;
      target[i].curvature = 0.0f;
    }
    n_wire = MGM_NUM_POINTS;
  }

  // 인지 소스가 이전 틱과 완전히 같은 값을 냈는지(= 아직 새 추론이 안 나와
  // wrapper가 같은 스냅샷을 또 읽어준 것) 판정. 2026-08-08 조향 미반영 진단:
  // dSPACE가 완전 동일한 CAN 페이로드 반복 수신 시 이를 무시하는 것으로 실측
  // 확인됨(실카메라 로그에서 78.7%가 직전 틱과 동일값이었고 그 구간 str 무반응,
  // 명시적으로 매틱 값이 바뀌게 한 진단 스크립트는 45도 반응). stack_lane 추론이
  // ~21Hz로 CAN 주기(100Hz)보다 느려 구조적으로 발생.
  // "새 추론 미도착" 판정은 **메시지 도착 여부**로 한다 (s.*_updated).
  // 값 동일성으로 판정하던 것을 2026-08-14에 교체: 인지가 의도적으로 상수를 내는
  // 경우(회피 통과 유지점 (1.5,0))를 영원히 낡은 값으로 오판해 x를 무한 감쇠시켰다.
  // wrapper가 소스를 못 알려주는 경우(_updated 전부 false인 옛 스냅샷)를 대비해
  // 값 동일성을 보조 조건으로 남긴다.
  const bool src_updated =
    (src == MGM_SRC_LANE) ? s.lane_updated :
    (src == MGM_SRC_GPS) ? s.gps_updated :
    (src == MGM_SRC_AVOID) ? s.avoid_updated : false;
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
    // 새 인지 값이 아직 안 옴 → 직전 출력을 그대로 반복 송신하지 않고, 그동안
    // 차량이 이동했을 거리만큼 x(전방 거리)를 깎아서 내보낸다. 실제 속도 피드백
    // (dSPACE 0x202 vehicle_vector.v) 배선 없이, 우리가 직전 틱에 명령한 st.v를
    // 등속 근사로 사용 — 10ms 구간·저속 주행에서는 근사 오차가 무시할 수준.
    // y/yaw/curvature는 보정하지 않음(차로 진행방향 유지 가정) — 이 근사가 틀리는
    // 급커브 등은 어차피 다음 실제 인지값(21Hz)이 금방 덮어써서 누적되지 않음.
    const float dx = st.v * MGM_PERIOD_S;
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      // 하한: 첫 점이 차 뒤로 가면 전진밖에 못 하는 차에게 도달 불가능한 목표가
      // 된다. 감쇠는 "인지가 잠깐 늦은 동안의 보정"이지 목표를 뒤로 보내는
      // 수단이 아니다 — 안전 불변식으로 고정한다 (2026-08-14).
      st.ref_out[i].x = (st.ref_out[i].x - dx > MGM_MIN_REF_X)
        ? st.ref_out[i].x - dx : MGM_MIN_REF_X;
    }
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
