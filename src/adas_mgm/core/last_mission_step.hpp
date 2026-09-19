#ifndef ADAS_MGM_CORE_LAST_MISSION_STEP_HPP
#define ADAS_MGM_CORE_LAST_MISSION_STEP_HPP
#include "mgm_types.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace adas_mgm {
inline bool last_mission_active(const LastMissionControl & last) {
  return last.phase != LastMissionPhase::IDLE && last.phase != LastMissionPhase::DONE;
}
inline void last_mission_suspend(LastMissionControl & last) {
  if (last.phase == LastMissionPhase::JUDGING) {
    last.phase = LastMissionPhase::STOPPING;
    last.left_votes = last.right_votes = 0;
    last.started_ns = last.started_event_ns = 0;
  }
}
// Called after upper safety arbitration and before ordinary mission/navigation.
// GPS owns zone measurements; only MGM consumes them as a state transition.
inline bool last_mission_step(const CoreSnapshot & s, CoreState & st) {
  auto & m = st.managers;
  auto & last = m.last_mission;
  auto & route = m.route;
  if (!s.revised_v2) {return false;}
  bool prior_missions_complete = !m.request.active;
  for (int i=0; i<MGM_MISSION_CAPACITY; ++i) {
    prior_missions_complete &= !route.required_missions[i] || m.mission_completed[i] || m.mission_failed[i];
  }
  if (last.phase == LastMissionPhase::IDLE && route.last_mission_enabled &&
    (route.phase == RoutePhase::RUNNING || route.phase == RoutePhase::WAIT_MISSION) && !route.changed &&
    m.top == TopState::AUTONOMOUS_DRIVE && !s.external_stop &&
    prior_missions_complete && m.mission == MissionState::MISSION_IDLE && m.avoid == AvoidState::INACTIVE &&
    m.signal == SignalState::SIGNAL_IDLE && s.gps_position_valid)
  {
    for (const auto & zone : m.zones.contexts) {
      if (zone.zone_type != ZoneType::LAST_MISSION_ZONE || !zone.zone_valid || !zone.in_zone) {continue;}
      last.phase = LastMissionPhase::APPROACH;
      last.source_zone_id = zone.zone_id;
      break;
    }
  }
  if (last.phase == LastMissionPhase::APPROACH) {
    // Zone [2] arms perception while navigation continues. Only the current
    // waypoint station reaching CSV state=3 requests a stop (never preview).
    if (!s.gps_position_valid || !s.gps_exit_stop_reached || !route.last_mission_enabled) {
      return false;
    }
    last.phase = LastMissionPhase::STOPPING;
  }
  if (!last_mission_active(last)) {return false;}
  if (last.phase == LastMissionPhase::WAIT_ROUTE && route.changed &&
    route.phase == RoutePhase::RUNNING && route.index == last.selected_index)
  {
    last.phase = LastMissionPhase::DONE;
    m.nav = m.gps_only_context ? NavState::GPS_ONLY_NAV : NavState::GPS_BACKUP;
    st.lane_high_cnt = st.lane_low_cnt = 0;
    return true;  // acknowledge while stopped; ordinary navigation resumes next tick
  }
  const bool authorized = m.top == TopState::AUTONOMOUS_DRIVE && !s.external_stop &&
    !m.estop_active && route.phase != RoutePhase::FAULT;
  const bool stopped = s.vehicle_speed_valid && std::isfinite(s.vehicle_speed) &&
    std::fabs(s.vehicle_speed) <= 0.001f;
  if (!authorized || !stopped) {last_mission_suspend(last); return true;}
  if (last.phase == LastMissionPhase::STOPPING) {
    if (last.request_id == std::numeric_limits<uint64_t>::max()) {
      route.phase = RoutePhase::FAULT; return true;
    }
    last.request_id = std::max(last.request_id + 1,
      static_cast<uint64_t>(std::max<int64_t>(1, s.event_time_ns)));
    last.phase = LastMissionPhase::JUDGING;
    last.started_ns = s.monotonic_ns;
    last.started_event_ns = s.event_time_ns;
    last.left_votes = last.right_votes = 0;
    last.last_frame = 0;
  }
  if (last.phase != LastMissionPhase::JUDGING) {return true;}
  if (s.monotonic_ns < last.started_ns) {last_mission_suspend(last); return true;}
  if (s.monotonic_ns - last.started_ns >= 3'000'000'000LL) {
    last.fallback = last.left_votes == last.right_votes;
    const bool right = last.right_votes > last.left_votes;
    last.route_id = right ? 7 : 6;
    last.selected_index = right ? route.right_index : route.left_index;
    last.phase = LastMissionPhase::SELECTED;
    return true;
  }
  const auto & frame = s.exit_reference;
  if (s.exit_request_id != last.request_id || !frame.generation ||
    frame.generation <= last.last_frame ||
    frame.generation < static_cast<uint64_t>(std::max<int64_t>(1, last.started_event_ns)) ||
    !std::isfinite(frame.age_s) || frame.age_s < 0 || frame.age_s > 0.5f) {return true;}
  last.last_frame = frame.generation;
  if (!std::isfinite(s.exit_confidence) || s.exit_confidence < 0.5f || s.exit_confidence > 1.f) {return true;}
  if (s.exit_class_id == 0 && last.right_votes < std::numeric_limits<uint32_t>::max()) {++last.right_votes;}
  if (s.exit_class_id == 1 && last.left_votes < std::numeric_limits<uint32_t>::max()) {++last.left_votes;}
  return true;
}
}
#endif
