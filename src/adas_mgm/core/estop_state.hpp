#ifndef ADAS_MGM_CORE_ESTOP_STATE_HPP
#define ADAS_MGM_CORE_ESTOP_STATE_HPP
#include "mgm_types.hpp"
#include "reference_safety.hpp"
#include <algorithm>
#include <cmath>

namespace adas_mgm {
// Transition contract only. A future executor must echo this episode ID and
// publish fresh geometry/completion. No historical timed reverse is substituted.
inline bool estop_transition(const CoreSnapshot & s, CoreState & st) {
  auto & m = st.managers;
  bool trigger = false;
  const float limits[] = {.25f, .15f, .15f};
  for (int i = 0; i < 3; ++i) {
    const auto & scan = s.estop_scans[i];
    if (!scan.generation || !std::isfinite(scan.age_s) || scan.age_s < 0 || scan.age_s > .35f ||
      scan.generation <= m.estop_generation[i] || std::isnan(s.estop_clearance_m[i])) {continue;}
    m.estop_generation[i] = scan.generation;
    const bool hit = s.estop_clearance_m[i] >= 0 && s.estop_clearance_m[i] <= limits[i];
    if (!hit) {
      m.estop_count[i] = 0;
      m.estop_rearm_blocked[i] = false;
    } else if (!m.estop_rearm_blocked[i]) {
      m.estop_count[i] = std::min(3, m.estop_count[i] + 1);
      trigger |= m.estop_count[i] >= 3;
    }
  }
  const auto & ref = s.recovery_reference;
  const bool matching = s.recovery_request_id == m.estop_request_id && ref.generation >= m.estop_request_id &&
    ref.generation != 0 && std::isfinite(ref.age_s) && ref.age_s >= 0 &&
    ref.timeout_s > 0 && ref.age_s <= ref.timeout_s;
  if (m.estop_active) {
    m.safety = SafetyState::ESTOP;
    // Operator/CAN stop must never consume an executor's done while suspended.
    if (matching && s.recovery_done && !s.external_stop && m.top == TopState::AUTONOMOUS_DRIVE) {
      m.estop_active = false;
      m.nav = m.estop_return_nav;
      m.avoid = m.estop_return_avoid;
      m.mission = m.estop_return_mission;
      m.signal = m.estop_return_signal;
      m.safety = SafetyState::NORMAL;
      for (int i = 0; i < 3; ++i) {
        // Block only continuing hazards; a clear sample rearms that sensor.
        m.estop_rearm_blocked[i] = m.estop_count[i] > 0;
        m.estop_count[i] = 0;
      }
    }
    return true;  // exact prior state restored; no reselect on the completion tick
  }
  if (trigger && m.top == TopState::AUTONOMOUS_DRIVE && !s.external_stop) {
    m.estop_active = true;
    m.estop_request_id = std::max(m.estop_request_id + 1,
      static_cast<uint64_t>(std::max<int64_t>(1, s.event_time_ns)));
    m.estop_return_nav = m.nav; m.estop_return_avoid = m.avoid;
    m.estop_return_mission = m.mission; m.estop_return_signal = m.signal;
    m.safety = SafetyState::ESTOP;
    st.escape_phase = MGM_ESCAPE_NONE;
    return true;
  }
  return false;
}
inline void estop_decision(const CoreSnapshot & s, const CoreState & st, CoreOutput & out) {
  const auto & m = st.managers;
  const auto & sample = s.recovery_reference;
  auto & ref = out.references[MGM_SRC_ESCAPE];
  ref.source = MGM_SRC_ESCAPE;
  ref.available = s.recovery_path.n > 0;
  ref.generation = sample.generation;
  ref.age_s = sample.age_s;
  ref.fresh = sample.generation >= m.estop_request_id && sample.generation != 0 &&
    std::isfinite(sample.age_s) && sample.age_s >= 0 && sample.timeout_s > 0 &&
    sample.age_s <= sample.timeout_s;
  ref.valid = ref.fresh && s.recovery_request_id == m.estop_request_id &&
    s.recovery_path.n == MGM_CONTROL_POINTS &&
    reference_geometry_valid(s.recovery_path.pts, s.recovery_path.n) && std::isfinite(s.recovery_speed);
  out.path_source = MGM_SRC_ESCAPE;
  out.safety = SafetyState::ESTOP;
  out.speed_owner = SpeedOwner::SAFETY;
  const bool stopped = !ref.valid || s.external_stop || m.top != TopState::AUTONOMOUS_DRIVE ||
    m.route.phase == RoutePhase::FAULT;
  out.v_ref = stopped ? 0.0f : s.recovery_speed;
  out.immediate_stop = stopped;
}
}
#endif
