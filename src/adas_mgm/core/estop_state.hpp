#ifndef ADAS_MGM_CORE_ESTOP_STATE_HPP
#define ADAS_MGM_CORE_ESTOP_STATE_HPP
#include "mgm_types.hpp"
#include <algorithm>
#include <cmath>

namespace adas_mgm {
// CSV zone [6] defines the current-position station. -1 selects GPS-derived
// ESTOP memberships; 0 explicitly disables; a positive ID supports bench wiring.
inline bool estop_transition(const CoreSnapshot & s, CoreState & st) {
  auto & m = st.managers;
  int id = st.params.estop_station_zone_id;
  if (id == -1) {
    id = 0;
    for (const auto & candidate : m.zones.contexts) {
      if (candidate.zone_type == ZoneType::ESTOP_ZONE && candidate.zone_valid && candidate.in_zone) {
        id = candidate.zone_id; break;
      }
    }
  }
  const bool configured = id > 0 && id < MGM_ZONE_CAPACITY;
  const auto & zone = m.zones.contexts[configured ? id : 0];
  const bool inside = configured && zone.zone_valid && zone.in_zone && zone.raw_in_zone;
  // Lost GPS cannot certify exit of the station that actually stopped us.
  const auto & previous = m.zones.contexts[m.estop_station_id];
  if (m.estop_station_id && previous.zone_valid && !previous.in_zone) {
    m.estop_station_completed = false;
  }
  m.estop_detection_enabled = inside && !m.estop_station_completed &&
    s.autonomous_enabled && !s.external_stop && m.top == TopState::AUTONOMOUS_DRIVE;
  if (!m.estop_active && !m.estop_detection_enabled) {
    m.estop_count[0] = 0;
    m.estop_stopped_since_ns = m.estop_stopped_last_ns = 0;
    m.estop_generation[0] = s.estop_scans[0].generation;
    return false;
  }
  const auto & scan = s.estop_scans[0];
  const bool fresh = scan.generation && scan.generation > m.estop_generation[0] &&
    std::isfinite(scan.age_s) && scan.age_s >= 0 && scan.age_s <= .35f;
  if (fresh) {
    m.estop_generation[0] = scan.generation;
    if (!m.estop_active) {
      m.estop_count[0] = s.estop_front_obstacle_points >= 5 ?
        std::min(3, m.estop_count[0] + 1) : 0;
    }
  }
  if (m.estop_active) {
    m.safety = SafetyState::ESTOP;
    // Actual stationary speed for seven continuous seconds, independent of
    // obstacle clearance. Invalid speed, motion, stop authority or clock gaps
    // restart the hold; old recovery done never releases it.
    const bool stopped = s.vehicle_speed_valid && std::isfinite(s.vehicle_speed) &&
      std::fabs(s.vehicle_speed) <= 1e-3f && s.autonomous_enabled &&
      !s.external_stop && m.top == TopState::AUTONOMOUS_DRIVE;
    const auto now = s.monotonic_ns;
    if (!stopped || now <= 0) {
      m.estop_stopped_since_ns = m.estop_stopped_last_ns = 0;
    } else {
      if (!m.estop_stopped_since_ns || now < m.estop_stopped_last_ns ||
        now - m.estop_stopped_last_ns > 350'000'000LL) {
        m.estop_stopped_since_ns = now;
      }
      m.estop_stopped_last_ns = now;
    }
    if (stopped && m.estop_stopped_since_ns > 0 &&
      now - m.estop_stopped_since_ns >= 7'000'000'000LL) {
      m.estop_active = false;
      m.estop_station_completed = true;
      m.nav = m.estop_return_nav; m.avoid = m.estop_return_avoid;
      m.mission = m.estop_return_mission; m.signal = m.estop_return_signal;
      m.safety = SafetyState::NORMAL;
      m.estop_count[0] = 0; m.estop_stopped_since_ns = m.estop_stopped_last_ns = 0;
    }
    return true;
  }
  if (fresh && m.estop_count[0] >= 3 && m.estop_detection_enabled) {
    m.estop_active = true;
    m.estop_station_id = static_cast<uint8_t>(id);
    m.estop_request_id = std::max(m.estop_request_id + 1,
      static_cast<uint64_t>(std::max<int64_t>(1, s.event_time_ns)));
    m.estop_return_nav = m.nav; m.estop_return_avoid = m.avoid;
    m.estop_return_mission = m.mission; m.estop_return_signal = m.signal;
    m.safety = SafetyState::ESTOP;
    m.estop_stopped_since_ns = m.estop_stopped_last_ns = 0;
    st.escape_phase = MGM_ESCAPE_NONE;
    return true;
  }
  return false;
}
inline void estop_decision(const CoreSnapshot &, const CoreState &, CoreOutput & out) {
  // Stop-only contract: even an old recovery executor cannot request reverse.
  out.references[MGM_SRC_ESCAPE] = {};
  out.state = MGM_STATE_ESTOP;
  out.path_source = MGM_SRC_ESCAPE;
  out.safety = SafetyState::ESTOP;
  out.speed_owner = SpeedOwner::SAFETY;
  out.v_ref = 0.0f;
  out.immediate_stop = true;
}
}
#endif
