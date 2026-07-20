#include "state_machine.hpp"

#include <algorithm>

namespace adas_mgm
{

StateMachine::StateMachine(const Params & params)
: p_(params)
{
}

Decision StateMachine::decide(const Snapshot & s)
{
  transition(s);
  return prioritize(s);
}

void StateMachine::transition(const Snapshot & s)
{
  // 히스테리시스 카운터 — 이탈/복귀 임계 분리, N주기 연속 (CLAUDE.md §4)
  lane_low_cnt_ = (s.lane.confidence < p_.lane_conf_exit) ? lane_low_cnt_ + 1 : 0;
  lane_high_cnt_ = (s.lane.confidence > p_.lane_conf_return) ? lane_high_cnt_ + 1 : 0;

  switch (state_) {
    case State::Lane:
      if (s.gps.parking_zone && s.parking.space_found) {
        state_ = State::Parking;
      } else if (s.avoid.obstacle_detected && s.avoid.avoidable) {
        avoid_return_ = State::Lane;
        state_ = State::Avoid;
      } else if (lane_low_cnt_ >= p_.n_cycles) {
        state_ = State::Waypoint;
      }
      break;

    case State::Waypoint:
      if (s.avoid.obstacle_detected && s.avoid.avoidable) {
        avoid_return_ = State::Waypoint;
        state_ = State::Avoid;
      } else if (lane_high_cnt_ >= p_.n_cycles) {
        state_ = State::Lane;
      }
      break;

    case State::Avoid:
      // 기동 완료 → 직전하던 스테이트로 복귀
      if (s.avoid.maneuver_done) {
        state_ = avoid_return_;
      }
      break;

    case State::Parking:
      // parking→avoid 전이 없음 = 주차 중 회피 금지가 구조적으로 보장 (CLAUDE.md §4)
      if (s.parking.done) {
        state_ = State::Lane;
      }
      break;
  }
}

Decision StateMachine::prioritize(const Snapshot & s) const
{
  Decision d{};
  d.state = state_;
  d.immediate_stop = false;

  switch (state_) {
    case State::Lane:
    case State::Waypoint:
      d.path_source = (state_ == State::Lane) ? PathSource::Lane : PathSource::Gps;
      // 종방향 우선권: 긴급 정지 > 신호등 정지 > 가속구간 > 기본 속도
      if (s.estop.estop) {
        d.v_ref = 0.0;
        d.immediate_stop = true;
      } else if (s.traffic.stop_required) {
        d.v_ref = 0.0;  // 일반 감속 정지 (rate limit 적용)
      } else if (s.gps.accel_zone) {
        d.v_ref = p_.v_accel_zone;
      } else {
        d.v_ref = p_.v_base;
      }
      break;

    case State::Avoid:
      d.path_source = PathSource::Avoid;
      // 기동 완료 우선 — 신호등 정지 요구는 기동 이탈 후 적용 (여기서 참조하지 않음).
      // 안전 바닥: TTC < 임계 또는 긴급 정지 → 즉시 정지 (우선권 표 최상위).
      if (s.estop.estop || s.avoid.ttc < p_.ttc_stop) {
        d.v_ref = 0.0;
        d.immediate_stop = true;
      } else {
        // 종방향은 회피 기하가 결정, 여유 폭 좁으면 감속
        d.v_ref = s.avoid.narrow_gap ?
          std::min<double>(s.avoid.v_suggest, p_.v_narrow) : s.avoid.v_suggest;
      }
      break;

    case State::Parking:
      d.path_source = PathSource::Parking;
      // 경로 침범 정지 > 주차 진행. 신호등·가속구간 요구 비활성.
      if (s.estop.estop) {
        d.v_ref = 0.0;
        d.immediate_stop = true;
      } else if (s.parking.path_blocked) {
        d.v_ref = 0.0;
      } else {
        d.v_ref = s.parking.v_suggest;
      }
      break;
  }
  return d;
}

}  // namespace adas_mgm
