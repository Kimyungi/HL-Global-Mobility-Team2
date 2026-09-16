#include "mission_step.hpp"
#include "reference_safety.hpp"
#include <algorithm>
#include <cmath>

namespace adas_mgm
{
namespace
{
MissionObservation observe(const CoreSnapshot & s, const ManagerState & m)
{
  return MissionObservation{true, s.event_time_ns, s.gps_position_valid,
    s.gps_x, s.gps_y, s.gps_position_valid ? s.gps_track_index : -1,
    s.vehicle_speed_valid, s.vehicle_speed, m.zones.selected.zone_id,
    m.request.travel_distance};
}
}
CalibrationState parking_calibration(const CoreParams & p)
{
  if (p.parking_zone_entry_active || p.parking_search_zone_only) {return CalibrationState::NOT_REQUIRED;}
  const double values[] = {p.parking_search_timeout, p.max_parking_search_distance};
  bool unset = false;
  for (double value : values) {
    if (!std::isfinite(value) || (value <= 0 && value != -1.0)) {return CalibrationState::INVALID_CONFIG;}
    unset = unset || value == -1.0;
  }
  return unset ? CalibrationState::UNCALIBRATED : CalibrationState::CALIBRATED;
}

bool mission_search_zone_known(const CoreState & st)
{
  const auto & m = st.managers;
  const auto & zone = m.zones.contexts[m.request.source_zone_id];
  return zone.zone_valid && zone.zone_id == m.request.source_zone_id &&
    zone.zone_type == ZoneType::MISSION_ZONE && zone.mission_id == m.request.mission_id &&
    zone.mission_type == m.request.mission_type;
}

bool mission_searches_along_gps(const CoreState & st)
{
  const auto & m = st.managers;
  return st.params.parking_zone_entry_active && m.mission == MissionState::MISSION_ACTIVE &&
    m.request.active && !m.request.preparation_ready;
}

bool mission_reference_authority(const CoreState & st)
{
  return st.managers.mission == MissionState::MISSION_ACTIVE && !mission_searches_along_gps(st);
}

void cancel_mission(CoreState & st, MissionCancelReason reason)
{
  auto & m = st.managers;
  if (!m.request.active) {return;}
  m.request.active = false;
  m.request.cancel_reason = reason;
  m.request.preparation_ready = false;
  m.mission = MissionState::MISSION_IDLE;
  m.mission_type = MissionType::NONE;
  m.mission_feedback_seen = false;
  m.mission_cancel = true;
  m.mission_prepare = false;
  m.mission_events |= MISSION_EVENT_CANCEL;
}

namespace
{
bool current_route_ended(const CoreSnapshot & s, const CoreState & st)
{
  if (!provider_reference(s, MGM_SRC_GPS).valid) {return false;}
  if (!st.params.route_sequence_enabled) {return s.gps_at_end;}
  const auto & route = st.managers.route;
  if (route.connecting || (route.phase != RoutePhase::RUNNING &&
    route.phase != RoutePhase::WAIT_MISSION)) {return false;}
  return route.end_reached || (route.seen_nonterminal && s.gps_at_end &&
    s.references[MGM_SRC_GPS].generation > route.last_generation);
}

bool active_parking_step(const CoreSnapshot & s, CoreState & st, bool matching)
{
  auto & m = st.managers;
  auto & r = m.request;
  // These are telemetry only; loss of motion input must not return authority to Nav.
  if (s.monotonic_ns >= r.last_update_ns) {
    if (s.vehicle_speed_valid && std::isfinite(s.vehicle_speed)) {
      r.travel_distance += std::fabs(s.vehicle_speed) * ((s.monotonic_ns - r.last_update_ns) * 1e-9);
    }
    r.last_update_ns = s.monotonic_ns;
  }
  // A fresh done from an acknowledged execution wins over a simultaneous endpoint.
  if (matching && r.preparation_ready && m.mission_feedback_seen && s.parking_done) {
    m.mission_completed[r.mission_id] = true;
    r.active = false;
    r.preparation_ready = false;
    m.mission = MissionState::MISSION_IDLE;
    m.mission_type = MissionType::NONE;
    m.mission_feedback_seen = false;
    m.mission_events |= MISSION_EVENT_DONE;
    return true;
  }
  // T parking uses this endpoint as its reverse-entry/forward-return junction.
  // Retain the request even with lost module feedback: only acknowledged done
  // AFTER forward exit or an explicit cancellation may release route 04.
  if (current_route_ended(s, st) && r.mission_type != MissionType::T_PARKING) {
    m.mission_failed[r.mission_id] = true;
    cancel_mission(st, MissionCancelReason::ROUTE_END);
    return true;
  }
  if (!matching) {return false;}
  if (s.parking_search_active) {
    if (!r.search_acknowledged) {
      r.search_acknowledged = true;
      r.search_start = observe(s, m);
      m.mission_events |= MISSION_EVENT_SEARCH_START;
    }
    r.space_found = s.parking_search_space_found;
    if (r.space_found && !r.space.recorded) {
      r.space = observe(s, m); m.mission_events |= MISSION_EVENT_SPACE_FOUND;
    }
    const auto & ref = s.parking_preparation_reference;
    if (!r.preparation_ready && s.parking_wall_acquisition_complete && s.parking_preparation_ready &&
      ref.generation >= static_cast<uint64_t>(std::max<int64_t>(1, r.zone_entry.time_ns)) &&
      std::isfinite(ref.age_s) && ref.age_s >= 0 && std::isfinite(ref.timeout_s) &&
      ref.timeout_s > 0 && ref.age_s <= ref.timeout_s)
    {
      r.preparation_ready = true;
      r.ready = observe(s, m); m.mission_events |= MISSION_EVENT_READY;
      m.mission_start = true;
      m.mission_feedback_seen = false;
      r.handoff = observe(s, m); m.mission_events |= MISSION_EVENT_HANDOFF;
      // Activation ack must arrive on a subsequent status, after ACTIVATE is sent.
      return false;
    }
  }
  if (r.preparation_ready && s.parking_mission_active && !s.parking_done) {
    m.mission_feedback_seen = true;
  } else if (!s.parking_mission_active && !s.parking_done) {
    m.mission_feedback_seen = false;
  }
  // Before readiness, PARKING follows GPS while the module searches. After the
  // handoff, lost execution feedback/reference holds zero; never resume GPS mid-maneuver.
  return false;
}
}

bool mission_step(const CoreSnapshot & s, CoreState & st)
{
  auto & m = st.managers;
  auto & r = m.request;
  // An explicit cancel never starts a different request on the same tick.
  if (s.mission_cancel_requested) {
    const bool ended = r.active;
    cancel_mission(st, MissionCancelReason::EXPLICIT);
    return ended;
  }
  if (r.active) {
    r.elapsed_s = std::max(0.0, (s.monotonic_ns - r.start_time_ns) * 1e-9);
    const bool matching = s.parking_valid && s.parking_updated &&
      s.parking_request_id == r.request_id &&
      s.parking_mission_mode == static_cast<uint8_t>(r.mission_type);
    if (st.params.parking_zone_entry_active) {return active_parking_step(s, st, matching);}
    if (m.mission == MissionState::MISSION_PREPARE) {
      const bool zone_only = st.params.parking_search_zone_only != 0;
      // The latched source Zone, not the display-priority Zone or another overlap.
      // A confirmed exit wins over readiness received on the same control tick.
      if (zone_only && mission_search_zone_known(st) && !m.zones.contexts[r.source_zone_id].in_zone) {
        m.mission_failed[r.mission_id] = true;
        cancel_mission(st, MissionCancelReason::ZONE_EXIT); return true;
      }
      if (!zone_only && parking_calibration(st.params) != CalibrationState::CALIBRATED) {
        cancel_mission(st, MissionCancelReason::CALIBRATION_REQUIRED); return true;
      }
      if (s.monotonic_ns < r.last_update_ns || !s.vehicle_speed_valid ||
        !std::isfinite(s.vehicle_speed))
      {
        cancel_mission(st, MissionCancelReason::MOTION_UNAVAILABLE);
        return true;
      }
      // Actual vehicle feedback, including reverse distance. Never v_ref.
      r.travel_distance += std::fabs(s.vehicle_speed) *
        ((s.monotonic_ns - r.last_update_ns) * 1e-9);
      r.last_update_ns = s.monotonic_ns;
      if (!zone_only && r.elapsed_s >= st.params.parking_search_timeout) {
        cancel_mission(st, MissionCancelReason::SEARCH_TIMEOUT); return true;
      }
      if (!zone_only && r.travel_distance >= st.params.max_parking_search_distance) {
        cancel_mission(st, MissionCancelReason::TRAVEL_DISTANCE); return true;
      }
      if (zone_only && !mission_search_zone_known(st)) {return false;}
      if (matching && s.parking_search_active) {
        if (!r.search_acknowledged) {
          r.search_acknowledged = true;
          r.search_start = observe(s, m);
          m.mission_events |= MISSION_EVENT_SEARCH_START;
        }
        r.space_found = s.parking_search_space_found;
        const auto & ref = s.parking_preparation_reference;
        r.preparation_ready = s.parking_preparation_ready &&
          ref.generation >= static_cast<uint64_t>(std::max<int64_t>(1, r.zone_entry.time_ns)) &&
          std::isfinite(ref.age_s) && ref.age_s >= 0 && std::isfinite(ref.timeout_s) &&
          ref.timeout_s > 0 && ref.age_s <= ref.timeout_s;
        if (r.space_found && !r.space.recorded) {
          r.space = observe(s, m); m.mission_events |= MISSION_EVENT_SPACE_FOUND;
        }
        if (r.preparation_ready) {
          r.ready = observe(s, m); m.mission_events |= MISSION_EVENT_READY;
          m.mission = MissionState::MISSION_ACTIVE;
          m.mission_start = true;
          m.mission_feedback_seen = false;  // wait for ACTIVATE acknowledgement
          r.handoff = observe(s, m); m.mission_events |= MISSION_EVENT_HANDOFF;
        }
      } else if (matching && r.search_acknowledged) {
        cancel_mission(st, MissionCancelReason::MODULE_ABORT); return true;
      }
    } else if (m.mission == MissionState::MISSION_ACTIVE && matching) {
      if (s.parking_mission_active && !s.parking_done) {m.mission_feedback_seen = true;}
      if (m.mission_feedback_seen && s.parking_done) {
        m.mission_completed[r.mission_id] = true;
        r.active = false;
        r.preparation_ready = false;
        m.mission = MissionState::MISSION_IDLE;
        m.mission_type = MissionType::NONE;
        m.mission_events |= MISSION_EVENT_DONE;
        return true;
      }
      if (r.search_acknowledged && !s.parking_search_active && !s.parking_done) {
        cancel_mission(st, MissionCancelReason::MODULE_ABORT); return true;
      }
    }
    return false;  // occupied request: no queue/overwrite, even on zone entry
  }
  if (m.top != TopState::AUTONOMOUS_DRIVE || s.new_session) {return false;}
  if (st.params.route_sequence_enabled && (m.route.connecting || m.route.phase == RoutePhase::WAIT_ACK ||
    m.route.phase == RoutePhase::FAULT || m.route.phase == RoutePhase::DISABLED)) {return false;}
  if (st.params.parking_zone_entry_active && current_route_ended(s, st)) {return false;}
  for (int id = 1; id < MGM_ZONE_CAPACITY; ++id) {
    const auto & zone = m.zones.contexts[id];
    if (!zone.zone_valid || !zone.in_zone || !zone.zone_entered || zone.mission_entry_suppressed ||
      zone.zone_type != ZoneType::MISSION_ZONE || m.mission_completed[zone.mission_id] || m.mission_failed[zone.mission_id] ||
      (zone.mission_type != MissionType::T_PARKING &&
       zone.mission_type != MissionType::PARALLEL_PARKING)) {continue;}
    r = MissionRequest{};
    r.active = true;
    r.request_id = std::max(m.last_request_id + 1,
      static_cast<uint64_t>(std::max<int64_t>(1, s.event_time_ns)));
    m.last_request_id = r.request_id;
    r.mission_id = zone.mission_id;
    r.mission_type = zone.mission_type;
    r.source_zone_id = zone.zone_id;
    r.start_time_ns = r.last_update_ns = s.monotonic_ns;
    r.zone_entry = observe(s, m);
    m.mission_events |= MISSION_EVENT_ZONE_ENTRY;
    m.active_mission = zone.mission_id;
    m.mission_type = zone.mission_type;
    m.mission = MissionState::MISSION_PREPARE;
    m.mission_feedback_seen = false;
    m.mission_prepare = true;
    if (st.params.parking_zone_entry_active) {
      m.mission = MissionState::MISSION_ACTIVE;
      // State entry starts preparation; exclusive Parking control starts at ready.
    } else if (!st.params.parking_search_zone_only && parking_calibration(st.params) != CalibrationState::CALIBRATED) {
      cancel_mission(st, MissionCancelReason::CALIBRATION_REQUIRED);
    } else if (!s.vehicle_speed_valid || !std::isfinite(s.vehicle_speed)) {
      cancel_mission(st, MissionCancelReason::MOTION_UNAVAILABLE);
    }
    break;
  }
  return false;
}
}
