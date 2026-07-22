// core/mgm_step.cpp — MGM 로직 코어 구현 (ROS 헤더 include 금지, 동적 할당 금지)
//
// 구조는 CLAUDE.md 그대로 세 단계:
//   판단(스테이트 머신, §4 — 시스템에서 유일한 곳)
//   → 실행 1: ref 조립 (§5.1/§5.6 — 포맷 변환·전환 연속 처리만)
//   → 실행 2: 종방향 병합 (§5.6 — rate limit만, immediate_stop은 우회)
#include "mgm_step.hpp"

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

// ── 판단: 스테이트 전이 (§4 전이 조건표)
void transition(const CoreSnapshot & s, CoreState & st)
{
  // 히스테리시스 카운터 — 이탈/복귀 임계 분리, N주기 연속
  st.lane_low_cnt = (s.lane_confidence < st.params.lane_conf_exit) ? st.lane_low_cnt + 1 : 0;
  st.lane_high_cnt = (s.lane_confidence > st.params.lane_conf_return) ? st.lane_high_cnt + 1 : 0;

  switch (st.state) {
    case MGM_STATE_LANE:
      if (s.gps_parking_zone && s.parking_space_found) {
        st.state = MGM_STATE_PARKING;
      } else if (s.avoid_obstacle_detected && s.avoid_avoidable) {
        st.avoid_return = MGM_STATE_LANE;
        st.state = MGM_STATE_AVOID;
      } else if (st.lane_low_cnt >= st.params.n_cycles) {
        st.state = MGM_STATE_WAYPOINT;
      }
      break;

    case MGM_STATE_WAYPOINT:
      if (s.avoid_obstacle_detected && s.avoid_avoidable) {
        st.avoid_return = MGM_STATE_WAYPOINT;
        st.state = MGM_STATE_AVOID;
      } else if (st.lane_high_cnt >= st.params.n_cycles) {
        st.state = MGM_STATE_LANE;
      }
      break;

    case MGM_STATE_AVOID:
      // 기동 완료 → 진입했던 스테이트로 복귀
      if (s.avoid_maneuver_done) {
        st.state = st.avoid_return;
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
      // 종방향 우선권: 긴급 정지 > 신호등 정지 > 가속구간 > 기본 속도
      if (s.estop) {
        out.v_ref = 0.0f;
        out.immediate_stop = true;
      } else if (s.traffic_stop_required) {
        out.v_ref = 0.0f;  // 일반 감속 정지 (rate limit 적용)
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

// ── 실행 1: ref 조립 — 스테이트가 고른 경로를 N=20 고정, 전환 시 블렌드 (§5.6)
void assemble(const CoreSnapshot & s, uint8_t src, CoreState & st)
{
  const CorePath * path = select_path(src, s);
  if (path == nullptr || path->n <= 0) {
    return;  // 선택 소스 미도착 → 직전 출력(ref_out) 유지 (판단 아님 — 데이터 hold)
  }

  // 부족분은 마지막 점 복제해 N=20 정규화
  CorePoint target[MGM_NUM_POINTS];
  const int32_t last = (path->n < MGM_NUM_POINTS ? path->n : MGM_NUM_POINTS) - 1;
  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    target[i] = path->pts[i < last ? i : last];
  }

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
  } else {
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      st.ref_out[i] = target[i];
    }
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
  st.avoid_return = MGM_STATE_LANE;
  st.last_src = MGM_SRC_LANE;
  // ref_out은 전부 (0,0,0,0) — 인지 도착 전: 제자리 (v_ref가 어차피 속도를 지배)
}

CoreOutput mgm_step(const CoreSnapshot & in, CoreState & st)
{
  CoreOutput out{};

  transition(in, st);        // 판단: 전이
  prioritize(in, st, out);   // 판단: 우선권 → v_ref 요구·경로 소스·immediate_stop
  assemble(in, out.path_source, st);  // 실행: 조립
  out.v_ref = merge(out, st);         // 실행: 병합 (rate limit)

  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    out.ref_points[i] = st.ref_out[i];
  }
  return out;
}

}  // namespace adas_mgm
