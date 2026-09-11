#include "mission_step.hpp"
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
  const double values[] = {p.parking_search_timeout, p.max_parking_search_distance};
  bool unset = false;
  for (double value : values) {
    if (!std::isfinite(value) || (value <= 0 && value != -1.0)) {return CalibrationState::INVALID_CONFIG;}
    unset = unset || value == -1.0;
  }
  return unset ? CalibrationState::UNCALIBRATED : CalibrationState::CALIBRATED;
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
    if (m.mission == MissionState::MISSION_PREPARE) {
      if (parking_calibration(st.params) != CalibrationState::CALIBRATED) {
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
      if (r.elapsed_s >= st.params.parking_search_timeout) {
        cancel_mission(st, MissionCancelReason::SEARCH_TIMEOUT); return true;
      }
      if (r.travel_distance >= st.params.max_parking_search_distance) {
        cancel_mission(st, MissionCancelReason::TRAVEL_DISTANCE); return true;
      }
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
  for (int id = 1; id < MGM_ZONE_CAPACITY; ++id) {
    const auto & zone = m.zones.contexts[id];
    if (!zone.zone_valid || !zone.in_zone || !zone.zone_entered || zone.mission_entry_suppressed ||
      zone.zone_type != ZoneType::MISSION_ZONE || m.mission_completed[zone.mission_id] ||
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
    if (parking_calibration(st.params) != CalibrationState::CALIBRATED) {
      cancel_mission(st, MissionCancelReason::CALIBRATION_REQUIRED);
    } else if (!s.vehicle_speed_valid || !std::isfinite(s.vehicle_speed)) {
      cancel_mission(st, MissionCancelReason::MOTION_UNAVAILABLE);
    }
    break;
  }
  return false;
}
}
