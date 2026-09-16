// ROS-free parallel state machine. Existing perception/control algorithms stay in
// their modules; existing_source_request/assemble/merge retain output geometry.
#include "manager_step.hpp"
#include "route_step.hpp"
#include "mgm_step.hpp"
#include "zone_step.hpp"
#include "reference_safety.hpp"
#include "mission_step.hpp"
#include "estop_state.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace adas_mgm
{
namespace
{
constexpr float kStoppedSpeed = 1e-3f;  // same existing MGM stopped tolerance
float fixed_motion_speed(float request, float speed)
{
  // Keep provider stops and invalid-input evidence; only normalize motion.
  if (request == 0.0f || !std::isfinite(request)) {return request;}
  if (!std::isfinite(speed) || speed <= 0.0f) {
    return std::numeric_limits<float>::quiet_NaN();  // final reference gate stops
  }
  return std::copysign(speed, request);
}
bool recovery_rear_allowed(const CoreSnapshot & s, const CoreParams & p)
{
  // The unoccupied RC test profile can explicitly opt out of rear certification.
  // Preserve the actual rear diagnostics; do not fabricate CLEAR input.
  return !p.escape_require_rear_clear || (s.estop_rear_clear && s.rear_sensor_valid &&
    s.rear_corridor_state == RearCorridorState::CLEAR);
}
bool line_valid(const CoreSnapshot & s)
{
  return provider_reference(s, MGM_SRC_LANE).valid;
}
bool gps_valid(const CoreSnapshot & s) {return provider_reference(s, MGM_SRC_GPS).valid;}
bool zone_gps_valid(const CoreSnapshot & s) {
  return s.revised_v2 ? s.gps_position_valid : gps_valid(s);
}
bool gps_return_aligned(const CoreSnapshot & s)
{
  constexpr float kYawLimit = 20.0f * 3.14159265358979323846f / 180.0f;
  return gps_valid(s) && s.gps_heading_valid && s.gps_station_error_valid &&
    std::isfinite(s.gps_cross_track) && std::isfinite(s.gps_station_yaw_error) &&
    std::fabs(s.gps_cross_track) <= 0.1f && std::fabs(s.gps_station_yaw_error) <= kYawLimit;
}
void update_avoid_zone(const CoreSnapshot & s, CoreState & st)
{
  auto & m = st.managers;
  if (st.params.avoid_zone_only && m.route.changed) {
    // The new CSV has its own start marker and the producer resets its frame.
    m.avoid = AvoidState::INACTIVE;
    m.avoid_fallback_only = false;
    m.clear_count = st.avoid_ticks = st.return_hold_left = 0;
  }
  if (!st.params.avoid_zone_only || m.route.changed) {
    m.avoid_zone_inside = m.avoid_zone_maneuver_seen = m.avoid_zone_completed = false;
    m.avoid_zone_enter_count = m.avoid_zone_exit_count = 0;
    m.avoid_zone_generation = 0;
  }
  if (!st.params.avoid_zone_only) {return;}
  const bool valid = zone_gps_valid(s) && s.zones.zone_valid && s.zones.generation != 0 &&
    st.params.zone_enter_confirm_samples > 0 && st.params.zone_exit_confirm_samples > 0;
  if (!valid || s.zones.generation < m.avoid_zone_generation) {
    // Lost GPS cannot certify exit or cancel a confirmed zone's ownership.
    m.avoid_zone_enter_count = m.avoid_zone_exit_count = 0;
    return;
  }
  if (s.zones.generation == m.avoid_zone_generation) {return;}
  m.avoid_zone_generation = s.zones.generation;
  if (s.gps_avoid_zone) {
    m.avoid_zone_exit_count = 0;
    if (!m.avoid_zone_inside && ++m.avoid_zone_enter_count >= st.params.zone_enter_confirm_samples) {
      m.avoid_zone_inside = true;
    }
  } else {
    m.avoid_zone_enter_count = 0;
    if (m.avoid_zone_inside && ++m.avoid_zone_exit_count >= st.params.zone_exit_confirm_samples) {
      m.avoid_zone_inside = false;
      m.avoid_zone_completed = false;
    }
  }
}
bool line_return_ready(const CoreSnapshot & s, const CoreState & st)
{
  return line_valid(s) && st.lane_high_cnt >= st.params.n_cycles &&
         !st.managers.gps_only_context && (s.revised_v2 || !st.managers.route.connecting) && (st.return_hold_left == 0 || !gps_valid(s));
}
void nav_reselect(const CoreSnapshot & s, CoreState & st)
{
  if (mission_searches_along_gps(st) || st.managers.avoid == AvoidState::GPS_RETURN) {
    st.managers.nav = st.managers.gps_only_context ? NavState::GPS_ONLY_NAV : NavState::GPS_BACKUP;
  } else if (!s.revised_v2 && st.managers.route.enabled && st.managers.route.connecting) {
    st.managers.nav = NavState::GPS_BACKUP;
  } else if (st.managers.gps_only_context) {
    st.managers.nav = NavState::GPS_ONLY_NAV;
  } else if (line_return_ready(s, st) || (!gps_valid(s) && line_valid(s) && (!s.revised_v2 || !st.managers.lane_recovery_required))) {
    st.managers.nav = NavState::LINE;
  } else if (gps_valid(s)) {
    st.managers.nav = NavState::GPS_BACKUP;
  }
  // Neither source: avoidance/fail-safe arbitration handles availability. There
  // is no artificial persistent NAV_RESELECT or invented straight reference.
}
bool nav_available(const CoreSnapshot & s, const CoreState & st)
{
  return st.managers.nav == NavState::LINE ?
    line_valid(s) && st.lane_low_cnt < st.params.n_cycles : gps_valid(s);
}
void update_existing_guards(const CoreSnapshot & s, CoreState & st)
{
  // Existing wrong-way threshold/latch, now evaluated with explicit GPS validity.
  if (gps_valid(s) && s.gps_heading_valid) {
    const bool wrong = std::fabs(s.gps_path.pts[0].yaw) > st.params.wrongway_yaw;
    st.wrongway_cnt = wrong ? std::min(st.wrongway_cnt + 1, st.params.wrongway_cycles) : 0;
    st.wrongway_ok_cnt = wrong ? 0 : std::min(st.wrongway_ok_cnt + 1, st.params.wrongway_cycles);
    if (st.params.wrongway_cycles > 0 && st.wrongway_cnt >= st.params.wrongway_cycles) {
      st.wrongway_latched = true;
    } else if (st.params.wrongway_cycles > 0 && st.wrongway_ok_cnt >= st.params.wrongway_cycles) {
      st.wrongway_latched = false;
    }
  }
  // Existing configured stop-zone dwell; session reset owns memory reset.
  if (gps_valid(s)) {
    if (!st.stop_zone_init) {
      st.stop_zone_init = true;
      st.stop_zone_boot_id = s.gps_stop_zone;
    }
    if (s.gps_stop_zone != st.stop_zone_boot_id) {st.stop_zone_boot_id = 0;}
    if (st.params.stop_zone_hold_cycles > 0 && !st.stop_zone_holding &&
      !mission_reference_authority(st) && s.gps_stop_zone != 0 &&
      s.gps_stop_zone != st.stop_zone_done_id && s.gps_stop_zone != st.stop_zone_boot_id)
    {
      st.stop_zone_holding = true;
      st.stop_hold_left = st.params.stop_zone_hold_cycles;
      st.stop_zone_done_id = s.gps_stop_zone;
    }
  }
  // A zero command is immediate in v2; dwell starts only once the car stops.
  const bool stopped = s.vehicle_speed_valid && std::isfinite(s.vehicle_speed) &&
    std::fabs(s.vehicle_speed) <= kStoppedSpeed;
  if (st.stop_zone_holding && stopped && --st.stop_hold_left <= 0) {
    st.stop_zone_holding = false;
    st.stop_hold_left = 0;
  }
}
uint32_t base_stop_reasons(const CoreSnapshot & s, const CoreState & st)
{
  if (s.revised_v2) {
    // Route FAULT is an independent hard stop, even in sensor-only policy.
    uint32_t reasons = st.managers.route.phase == RoutePhase::FAULT ? SAFE_STOP_ROUTE_SEQUENCE : 0u;
    if (!st.managers.estop_active && (s.sensor_alive_mask & 0x77) == 0) {
      reasons |= SAFE_STOP_ALL_SENSORS_LOST;  // rear (bit 3) excluded
    }
    return reasons;
  }
  if (st.params.safe_stop_all_sensors_only) {
    return (s.sensor_alive_mask & 0x7f) == 0 ? static_cast<uint32_t>(SAFE_STOP_ALL_SENSORS_LOST) : 0u;
  }
  const auto & m = st.managers;
  const bool mission = mission_reference_authority(st);
  const bool gps = gps_valid(s);
  const bool signal_stop = m.signal == SignalState::APPROACH_STOP_LINE ||
    m.signal == SignalState::STOPPED_WAIT;
  // Reasons are recomputed independently; clearing one never clears another.
  uint32_t reasons = 0;
  if (route_stop(s, st)) {reasons |= SAFE_STOP_ROUTE_SEQUENCE;}
  if (st.params.parking_search_zone_only && m.mission == MissionState::MISSION_PREPARE &&
    !mission_search_zone_known(st)) {reasons |= SAFE_STOP_MISSION_ZONE_UNKNOWN;}
  if (!mission && m.gps_only_context && !gps) {
    reasons |= SAFE_STOP_GPS_ONLY_GPS_LOSS;
  }
  if (!mission && !s.camera_line_valid && !s.gps_valid && !s.lidar_valid) {
    reasons |= SAFE_STOP_ALL_SENSORS_LOST;
  }
  if (!mission && m.zones.definitions_seen && m.zones.calibration != CalibrationState::CALIBRATED) {
    reasons |= SAFE_STOP_ZONE_CONTEXT_UNAVAILABLE;
  }
  if (s.start_gate_enabled && !s.lidar_valid) {reasons |= SAFE_STOP_LIDAR_INPUT;}
  if (s.external_stop) {reasons |= SAFE_STOP_EXTERNAL;}
  if (mission && !s.parking_valid) {reasons |= SAFE_STOP_MISSION_FEEDBACK;}
  if (!mission && s.traffic_fail_safe_stop) {reasons |= SAFE_STOP_TRAFFIC_INPUT;}
  if (!mission && signal_stop && !s.vehicle_speed_valid) {
    reasons |= SAFE_STOP_VEHICLE_SPEED;
  }
  return reasons;
}
}  // namespace

void manager_transition(const CoreSnapshot & s, CoreState & st)
{
  const auto previous_request = st.managers.request;
  const auto last_request_id = st.managers.last_request_id;
  const auto previous_route = st.managers.route;
  if (s.new_session) {
    const CoreParams params = st.params;
    mgm_init(st, params);
    st.managers.last_request_id = last_request_id;
    route_reset(previous_route, s, st);
  }
  if (!s.revised_v2 || !st.managers.estop_active) {route_observe(s, st);}
  auto & m = st.managers;
  const bool clock_valid = !m.previous_tick_known || s.monotonic_ns >= m.previous_tick_ns;
  if (!clock_valid && m.route.enabled) {m.route.phase = RoutePhase::FAULT;}
  const double dt = m.previous_tick_known && clock_valid ?
    (s.monotonic_ns - m.previous_tick_ns) * 1e-9 : 0.0;
  if (m.previous_reverse_command) {
    if (clock_valid) {m.recovery.command_time_s += dt;}
    if (clock_valid && s.vehicle_speed_valid && std::isfinite(s.vehicle_speed)) {
      m.recovery.measured_distance_m += std::max(0.0f, -s.vehicle_speed) * dt;
    } else {m.recovery.measured_distance_complete = false;}
  }
  m.previous_tick_ns = s.monotonic_ns; m.previous_tick_known = true;
  m.recovery.configured = st.params.escape_after_cycles > 0 && st.params.escape_max_cycles > 0 &&
    std::isfinite(st.params.v_escape) && st.params.v_escape < 0;
  m.recovery.rear_sensor_valid = s.rear_sensor_valid;
  m.recovery.rear_corridor_state = s.rear_corridor_state;
  m.mission_start = m.mission_prepare = m.mission_cancel = false;
  m.mission_events = 0;
  if (s.new_session && previous_request.active) {
    m.request = previous_request;
    cancel_mission(st, MissionCancelReason::SESSION_RESET);
  }
  const bool was_zone = m.gps_only_context;
  const bool zone_source_confirmed = !s.revised_v2 || m.route.phase != RoutePhase::WAIT_ACK ||
    (s.route.index == m.route.index && s.route.connecting == m.route.connecting);
  zone_step(s.zones, zone_source_confirmed && zone_gps_valid(s), m.zones,
    st.params.zone_enter_confirm_samples, st.params.zone_exit_confirm_samples);
  if (zone_source_confirmed) {update_avoid_zone(s, st);}
  m.gps_only_context = m.zones.in_gps_only_zone;
  if (s.new_session || (m.route.enabled && m.route.session_reset_pending &&
    m.route.phase == RoutePhase::RUNNING)) {
    for (auto & zone : m.zones.contexts) {
      if (zone.raw_in_zone && zone.zone_type == ZoneType::MISSION_ZONE) {zone.mission_entry_suppressed = true;}
    }
    if (m.route.phase == RoutePhase::RUNNING && m.zones.selected.zone_valid) {m.route.session_reset_pending = false;}
  }
  // FINISH can only be cleared by the explicit new-session event above.
  if (m.top == TopState::FINISH) {
    m.recovery.eligible = false; m.recovery.block_reason = RecoveryBlockReason::NOT_DRIVING;
    return;
  }
  const bool starting_ready = !s.start_gate_enabled || ((s.revised_v2 ? s.start_lidar_ready : s.lidar_valid) && (s.camera_available || s.gps_fixed_ready));
  const bool already_driving = m.top == TopState::AUTONOMOUS_DRIVE;
  m.top = s.autonomous_enabled && (already_driving || starting_ready) ?
    TopState::AUTONOMOUS_DRIVE : TopState::AUTONOMOUS_ENABLE;

  if (s.revised_v2 && estop_transition(s, st)) {return;}
  const bool line = line_valid(s);
  const bool gps = gps_valid(s);
  st.lane_low_cnt = line && s.lane_confidence < st.params.lane_conf_exit ?
    std::min(st.lane_low_cnt + 1, st.params.n_cycles) : 0;
  st.lane_high_cnt = line && s.lane_confidence >= st.params.lane_conf_return ?
    std::min(st.lane_high_cnt + 1, st.params.n_cycles) : 0;
  if (s.revised_v2) {
    if (st.lane_low_cnt >= st.params.n_cycles && !gps) {
      m.lane_recovery_required = true;
    }
    if (gps || st.lane_high_cnt >= st.params.n_cycles) {m.lane_recovery_required = false;}
  }
  if (st.return_hold_left > 0) {--st.return_hold_left;}

  if (!s.revised_v2 && m.route.enabled && m.route.connecting) {
    m.nav = NavState::GPS_BACKUP;
  } else if (m.gps_only_context) {
    m.nav = NavState::GPS_ONLY_NAV;
  } else if (was_zone) {
    nav_reselect(s, st);
  } else if (m.nav == NavState::LINE) {
    if ((!line || st.lane_low_cnt >= st.params.n_cycles) && (gps || s.revised_v2)) {
      m.nav = NavState::GPS_BACKUP;
    }
  } else if (line_return_ready(s, st) || (!gps && line && (!s.revised_v2 || !m.lane_recovery_required))) {
    m.nav = NavState::LINE;
  }

  if (!st.params.route_sequence_enabled && m.top == TopState::AUTONOMOUS_DRIVE && gps && s.gps_at_end) {
    if (st.params.parking_zone_entry_active && m.request.active) {mission_step(s, st);}
    m.top = TopState::FINISH;
    st.at_end_latched = true;
    if (st.escape_phase == MGM_ESCAPE_REVERSING) {m.recovery.last_reason = RecoveryReason::AUTHORITY_LOST;}
    m.recovery.eligible = false; m.recovery.block_reason = RecoveryBlockReason::NOT_DRIVING;
    st.escape_phase = MGM_ESCAPE_NONE;
    cancel_mission(st, MissionCancelReason::FINISH);
    return;
  }

  const bool mission_ended = mission_step(s, st);
  if (mission_ended) {nav_reselect(s, st);}

  const bool mission = m.mission == MissionState::MISSION_ACTIVE;
  const bool maneuver = mission_reference_authority(st);
  const bool fallback = !m.gps_only_context && !nav_available(s, st) && !gps;
  const bool avoid_allowed = st.params.avoidance_enabled && !mission && s.lidar_valid &&
    !(m.gps_only_context && !gps) && m.top == TopState::AUTONOMOUS_DRIVE;
  // Detection claims obstacle handling even when the planner has no target.
  // Reference validity gates motion separately; a missing path must not leave
  // a healthy LINE/GPS provider in control in front of a detected obstacle.
  const bool avoid_entry = (s.avoid_obstacle_detected &&
    (st.params.avoid_zone_only == 0 || s.gps_avoid_zone)) || fallback;
  const bool was_avoiding = m.avoid != AvoidState::INACTIVE;
  if (!st.params.avoidance_enabled || mission || m.top != TopState::AUTONOMOUS_DRIVE) {
    m.avoid = AvoidState::INACTIVE;
    m.clear_count = 0;
    m.avoid_fallback_only = false;
    if (!st.params.avoidance_enabled) {
      st.return_hold_left = 0;
      if (was_avoiding) {nav_reselect(s, st);}
    }
  } else if (st.params.avoid_zone_only) {
    // Zone entry itself claims AVOID, even before an obstacle is detected.
    // Empty/stale references stop through the final gate without falling back
    // to LINE. The CSV marker starts an episode; only a completed waypoint
    // return ends it. The consumed marker cannot repeatedly start that episode.
    if (m.avoid_zone_inside && !m.avoid_zone_completed) {
      if (m.avoid == AvoidState::INACTIVE) {m.avoid_zone_maneuver_seen = false;}
      m.avoid = AvoidState::AVOID_ACTIVE;
    }
    if (m.avoid == AvoidState::AVOID_ACTIVE) {
      if (s.avoid_obstacle_detected) {m.avoid_zone_maneuver_seen = true;}
      if (s.lidar_valid && !s.avoid_obstacle_detected &&
        st.escape_phase == MGM_ESCAPE_NONE &&
        m.avoid_zone_maneuver_seen && s.avoid_maneuver_done)
      {
        m.avoid = AvoidState::GPS_RETURN;
        m.avoid_zone_completed = true;
        nav_reselect(s, st);
      }
    } else if (m.avoid == AvoidState::GPS_RETURN && gps_return_aligned(s) && !s.auto_estop) {
      m.avoid = AvoidState::INACTIVE;
      m.avoid_zone_maneuver_seen = false;
      st.return_hold_left = 0;
      nav_reselect(s, st);
    }
    m.clear_count = 0;
    m.avoid_fallback_only = false;
  } else if (st.escape_phase == MGM_ESCAPE_REVERSING) {
    // Reverse and its following maneuver are one avoidance episode.
    m.avoid = AvoidState::AVOID_ACTIVE;
    m.avoid_fallback_only = false;
  } else if (m.avoid == AvoidState::GPS_RETURN) {
    // Preserve ownership through lost/stale GPS. Its reference gate stops output;
    // a camera confidence increase or elapsed timer cannot finish the return.
    if (avoid_allowed && s.avoid_obstacle_detected &&
      (st.params.avoid_zone_only == 0 || s.gps_avoid_zone)) {
      m.avoid = AvoidState::AVOID_ACTIVE;
      st.avoid_ticks = 0;
    } else if (gps_return_aligned(s) && !s.auto_estop) {
      m.avoid = AvoidState::INACTIVE;
      st.return_hold_left = 0;
      nav_reselect(s, st);
    }
  } else if (!avoid_allowed) {
    m.avoid = AvoidState::INACTIVE;
    m.clear_count = 0;
    m.avoid_fallback_only = false;
  } else if (m.avoid == AvoidState::INACTIVE) {
    if (avoid_entry) {
      m.avoid = AvoidState::AVOID_ACTIVE;
      m.avoid_fallback_only = fallback && !s.avoid_obstacle_detected;
    }
  } else if (m.avoid_fallback_only && !fallback && !s.avoid_obstacle_detected) {
    // No obstacle episode occurred: a recovered navigation source replaces the
    // LiDAR-only request without fabricating an obstacle-clear timer.
    m.avoid = AvoidState::INACTIVE;
    m.avoid_fallback_only = false;
    nav_reselect(s, st);
  } else if (!s.avoid_obstacle_detected && gps &&
    !provider_reference(s, MGM_SRC_AVOID).valid && st.escape_phase == MGM_ESCAPE_NONE)
  {
    // A cleared obstacle with no usable avoidance target returns directly to
    // the live GPS reference, even without a signal-exit or maneuver_done.
    m.avoid = AvoidState::INACTIVE;
    m.avoid_fallback_only = false;
    m.clear_count = st.avoid_ticks = st.return_hold_left = 0;
    st.lane_high_cnt = st.lane_low_cnt = 0;
    m.nav = m.gps_only_context ? NavState::GPS_ONLY_NAV : NavState::GPS_BACKUP;
  } else if (!m.avoid_fallback_only && st.escape_phase == MGM_ESCAPE_NONE &&
    !s.avoid_obstacle_detected && (s.avoid_maneuver_done ||
    (provider_reference(s, MGM_SRC_AVOID).valid && st.params.avoid_max_cycles > 0 &&
     st.avoid_ticks >= st.params.avoid_max_cycles)))
  {
    // Completion/episode limit ends only obstacle steering. Remain in AVOID
    // while the actual GPS station errors converge; no elapsed-time exit.
    m.avoid = AvoidState::GPS_RETURN;
    m.clear_count = 0;
    st.return_hold_left = 0;
    m.nav = m.gps_only_context ? NavState::GPS_ONLY_NAV : NavState::GPS_BACKUP;
  } else {
    m.avoid = AvoidState::AVOID_ACTIVE;
    m.clear_count = 0;
    if (s.avoid_obstacle_detected) {m.avoid_fallback_only = false;}
  }
  if (m.avoid == AvoidState::GPS_RETURN) {nav_reselect(s, st);}
  // Preserve any hold established by other existing transitions.
  if (!m.gps_only_context && st.return_hold_left > 0 && gps) {
    m.nav = NavState::GPS_BACKUP;
  }
  if (mission_searches_along_gps(st)) {nav_reselect(s, st);}
  st.avoid_ticks = m.avoid != AvoidState::INACTIVE && was_avoiding ? st.avoid_ticks + 1 : 0;
  if (m.avoid == AvoidState::INACTIVE || !was_avoiding) {
    m.avoid_episode_reference_seen = false;
  }
  if (m.avoid != AvoidState::INACTIVE && provider_reference(s, MGM_SRC_AVOID).valid) {
    m.avoid_episode_reference_seen = true;
  }

  if (s.revised_v2 && s.vehicle_speed_valid && std::isfinite(s.vehicle_speed)) {
    m.actual_speed_seen = true;
    m.last_actual_speed = s.vehicle_speed;
  }
  const bool traffic_zone = !s.revised_v2 || m.gps_only_context;
  if (s.revised_v2 && traffic_zone != m.traffic_zone_active) {
    m.traffic_zone_active = traffic_zone;
    m.traffic_zone_enter_ns = s.event_time_ns;
    m.signal = SignalState::SIGNAL_IDLE;
    st.traffic_distance_latched = st.traffic_prev_stopline_detected = false;
    st.traffic_stopline_distance = 0;
  }
  const bool traffic_fresh = !s.revised_v2 || (s.traffic_status_fresh &&
    s.traffic_status_stamp_ns >= m.traffic_zone_enter_ns);
  if (st.params.traffic_state_enabled && traffic_zone) {
    // (!red) || (!red && green) reduces to !red. Apply the GPS handoff
    // only on a signal exit, not on every ordinary no-signal driving tick.
    const bool release = traffic_fresh && !s.traffic_red_active;
    const bool signal_exit = release && m.signal != SignalState::SIGNAL_IDLE;
    if (release) {
      m.signal = SignalState::SIGNAL_IDLE;
      st.traffic_distance_latched = false;
      st.traffic_stopline_distance = 0.0f;
      st.traffic_prev_stopline_detected = false;
      if (signal_exit && !mission && m.top == TopState::AUTONOMOUS_DRIVE) {
        m.nav = m.gps_only_context ? NavState::GPS_ONLY_NAV : NavState::GPS_BACKUP;
        st.lane_high_cnt = st.lane_low_cnt = 0;
        st.return_hold_left = 0;
        // A vanished obstacle from signal waiting must not keep an empty
        // avoidance episode in charge. A current obstacle/reverse maneuver
        // still owns its reference and all independent stop gates remain.
        if (!st.params.avoid_zone_only && s.lidar_valid && !s.avoid_obstacle_detected &&
          st.escape_phase == MGM_ESCAPE_NONE)
        {
          m.avoid = AvoidState::INACTIVE;
          m.clear_count = st.avoid_ticks = 0;
          m.avoid_fallback_only = false;
          m.avoid_episode_reference_seen = false;
        }
      }
    } else {
      if (traffic_fresh && m.signal == SignalState::SIGNAL_IDLE && s.traffic_red_active) {
        m.signal = SignalState::RED_DETECTED;
      }
      if (traffic_fresh && m.signal == SignalState::RED_DETECTED && s.traffic_red_active &&
        s.traffic_stopline_detected)
      {
        m.signal = SignalState::APPROACH_STOP_LINE;
      }
      const bool previously_stopped = m.signal == SignalState::STOPPED_WAIT;
      if (m.signal == SignalState::APPROACH_STOP_LINE) {
        // Revised v2 reseeds on every fresh detected->lost edge while approaching.
        // Historical profiles retain their once-per-episode latch.
        const bool seed_now = traffic_fresh && (s.revised_v2 || !st.traffic_distance_latched) &&
          st.traffic_prev_stopline_detected && !s.traffic_stopline_detected;
        if (seed_now) {
          st.traffic_stopline_distance = st.params.traffic_ramp_distance_m;
          st.traffic_distance_latched = true;
        } else if (st.traffic_distance_latched) {
          const float speed = s.vehicle_speed_valid && std::isfinite(s.vehicle_speed) ? s.vehicle_speed :
            s.revised_v2 ? (m.actual_speed_seen ? m.last_actual_speed : st.v) : 0.0f;
          st.traffic_stopline_distance -= static_cast<float>(std::fabs(speed) * dt);
        }
        if (st.traffic_distance_latched && s.vehicle_speed_valid &&
          st.traffic_stopline_distance <= st.params.traffic_stop_offset &&
          std::fabs(s.vehicle_speed) <= kStoppedSpeed)
        {
          m.signal = SignalState::STOPPED_WAIT;
        }
      }
      if (previously_stopped && st.traffic_distance_latched &&
        s.vehicle_speed_valid && std::isfinite(s.vehicle_speed)) {
        // Moving after a stopped sample invalidates the stationary success observation.
        // The transition tick above has zero speed, so no double integration occurs.
        st.traffic_stopline_distance -= static_cast<float>(std::fabs(s.vehicle_speed) * dt);
      }
      if (traffic_fresh) {st.traffic_prev_stopline_detected = s.traffic_stopline_detected;}
    }
  }

  update_existing_guards(s, st);
  if (!s.new_session && clock_valid) {route_step(s, st);}
  const bool signal_stop = m.signal == SignalState::APPROACH_STOP_LINE ||
    m.signal == SignalState::STOPPED_WAIT;
  m.safe_stop_reasons = base_stop_reasons(s, st);
  if (!st.params.safe_stop_all_sensors_only && !maneuver && signal_stop &&
    (!clock_valid || !std::isfinite(s.vehicle_speed))) {
    m.safe_stop_reasons |= SAFE_STOP_VEHICLE_SPEED;
  }
  const bool sensor_stop = !st.params.safe_stop_all_sensors_only && !maneuver && (((m.gps_only_context || mission_searches_along_gps(st)) && !gps) ||
    (!nav_available(s, st) && !gps && (!s.lidar_valid || !st.params.avoidance_enabled)));
  const bool fault_stop = m.safe_stop_reasons != 0;
  // Revised v2 has already evaluated the upper ESTOP state, including PARKING.
  // Independent E-stop / timed reverse below are historical-profile behavior only.
  if (s.revised_v2) {
    st.escape_phase = MGM_ESCAPE_NONE;
    m.recovery_waiting_reference = false;
    m.safety = fault_stop ? SafetyState::SAFE_STOP : SafetyState::NORMAL;
    return;
  }
  const bool lidar_stop = !mission && s.auto_estop;
  const bool was_reversing = st.escape_phase == MGM_ESCAPE_REVERSING;
  CoreSnapshot recovery = s;
  recovery.estop_latch_release = s.auto_estop;
  const bool rear_clear = s.estop_rear_clear && s.rear_sensor_valid &&
    s.rear_corridor_state == RearCorridorState::CLEAR;
  recovery.estop_rear_clear = rear_clear;
  const bool rear_required = st.params.escape_require_rear_clear != 0;
  const bool rear_allowed = recovery_rear_allowed(s, st.params);
  auto & diag = m.recovery;
  diag.block_reason = !diag.configured ? RecoveryBlockReason::CONFIG_DISABLED :
    rear_required && s.rear_corridor_state == RearCorridorState::UNKNOWN ? RecoveryBlockReason::REAR_UNKNOWN :
    rear_required && s.rear_corridor_state == RearCorridorState::BLOCKED ? RecoveryBlockReason::REAR_BLOCKED :
    !rear_allowed ? RecoveryBlockReason::REAR_INVALID :
    m.top != TopState::AUTONOMOUS_DRIVE ? RecoveryBlockReason::NOT_DRIVING :
    mission ? RecoveryBlockReason::MISSION_ACTIVE :
    s.external_stop || sensor_stop || fault_stop || st.stop_zone_holding ? RecoveryBlockReason::FORCED_STOP :
    signal_stop ? RecoveryBlockReason::SIGNAL_STOP :
    !st.escape_armed && st.v <= kStoppedSpeed ? RecoveryBlockReason::NOT_ARMED :
    !s.auto_estop ? RecoveryBlockReason::NO_DANGER : RecoveryBlockReason::NONE;
  const bool enter_recovery = update_escape(recovery, st,
    !mission && !s.external_stop && !sensor_stop && !fault_stop && !signal_stop && !st.stop_zone_holding &&
    m.top == TopState::AUTONOMOUS_DRIVE &&
    (st.escape_phase != MGM_ESCAPE_REVERSING || rear_allowed));
  diag.eligible = diag.block_reason == RecoveryBlockReason::NONE && (enter_recovery ||
    st.escape_phase == MGM_ESCAPE_REVERSING);
  if (diag.block_reason == RecoveryBlockReason::NONE && !diag.eligible) {
    diag.block_reason = RecoveryBlockReason::WAIT_DELAY;
  }
  if (was_reversing && st.escape_phase == MGM_ESCAPE_NONE) {
    diag.last_reason = !diag.configured ? RecoveryReason::CONFIG_DISABLED :
      !rear_allowed ? RecoveryReason::REAR_LOST : !s.auto_estop ? RecoveryReason::DANGER_CLEARED :
      (!mission && !sensor_stop && !fault_stop && !signal_stop && !st.stop_zone_holding &&
       m.top == TopState::AUTONOMOUS_DRIVE) ? RecoveryReason::TIME_LIMIT : RecoveryReason::AUTHORITY_LOST;
  }
  if (enter_recovery && rear_allowed && diag.configured) {
    ++diag.attempt_count;
    diag.last_reason = RecoveryReason::ENTERED;
    st.escape_phase = MGM_ESCAPE_REVERSING;
    st.escape_ticks = 0;
    m.recovery_waiting_reference = false;
    if (st.params.avoidance_enabled && (!st.params.avoid_zone_only ||
      (m.avoid_zone_inside && !m.avoid_zone_completed) || m.avoid == AvoidState::AVOID_ACTIVE)) {
      m.avoid = AvoidState::AVOID_ACTIVE;
      m.avoid_fallback_only = false;
      m.clear_count = st.avoid_ticks = st.return_hold_left = 0;
      m.avoid_episode_reference_seen = false;
    }
  }
  if (!mission && was_reversing && st.escape_phase == MGM_ESCAPE_NONE) {
    m.recovery_waiting_reference = true;
    st.avoid_ticks = 0;  // Reverse duration does not consume the following maneuver budget.
    nav_reselect(s, st);
  }
  if (mission) {m.recovery_waiting_reference = false;}
  if (m.recovery_waiting_reference) {
    nav_reselect(s, st);
    const bool reference = m.avoid != AvoidState::INACTIVE && m.avoid != AvoidState::GPS_RETURN ?
      provider_reference(s, MGM_SRC_AVOID).valid : nav_available(s, st);
    if (reference) {m.recovery_waiting_reference = false;}
  }
  const auto previous_safety = m.safety;
  if (sensor_stop || fault_stop) {
    m.safety = SafetyState::SAFE_STOP;
  } else if (st.escape_phase == MGM_ESCAPE_REVERSING || m.recovery_waiting_reference) {
    m.safety = SafetyState::REVERSE_RECOVERY;
  } else if (lidar_stop) {
    m.safety = SafetyState::AUTO_ESTOP;
  } else {
    m.safety = SafetyState::NORMAL;
    if (previous_safety == SafetyState::SAFE_STOP && !mission_ended) {nav_reselect(s, st);}
  }
}

uint8_t legacy_state_projection(const CoreState & st)
{
  const auto & m = st.managers;
  if (m.mission == MissionState::MISSION_ACTIVE) {return MGM_STATE_PARKING;}
  if (st.escape_phase == MGM_ESCAPE_REVERSING ||
    (st.params.avoidance_enabled && m.avoid != AvoidState::INACTIVE)) {return MGM_STATE_AVOID;}
  if (st.params.traffic_state_enabled && (m.signal == SignalState::APPROACH_STOP_LINE ||
    m.signal == SignalState::STOPPED_WAIT)) {return MGM_STATE_TRAFFIC;}
  return m.nav == NavState::LINE ? MGM_STATE_LANE : MGM_STATE_WAYPOINT;
}

CoreOutput manager_decision(const CoreSnapshot & s, const CoreState & st)
{
  const auto & m = st.managers;
  const bool mission = mission_reference_authority(st);
  const bool avoid = st.params.avoidance_enabled && m.avoid != AvoidState::INACTIVE;
  const bool gps_return = avoid && m.avoid == AvoidState::GPS_RETURN;
  const uint8_t source_state = mission ? MGM_STATE_PARKING : mission_searches_along_gps(st) ? MGM_STATE_WAYPOINT :
    gps_return ? MGM_STATE_WAYPOINT : avoid ? MGM_STATE_AVOID :
    m.nav == NavState::LINE ? MGM_STATE_LANE : MGM_STATE_WAYPOINT;
  CoreSnapshot request = s;
  request.estop = false;  // independent safety arbitration below
  request.traffic_stop_required = false;
  if (s.revised_v2) {
    request.traffic_fail_safe_stop = false;
    request.vehicle_speed_valid = true;  // distance uses fallback; actual stopped proof remains separate
  }
  CoreOutput out = existing_source_request(request, st, source_state);
  if (source_state == MGM_STATE_PARKING && m.mission_type == MissionType::T_PARKING) {
    if (std::isfinite(out.v_ref)) {
      out.v_ref = std::copysign(std::min(std::fabs(out.v_ref), std::fabs(st.params.v_base)), out.v_ref);
    }
  } else if (source_state != MGM_STATE_AVOID) {
    out.v_ref = fixed_motion_speed(out.v_ref, st.params.v_base);
  }
  if (gps_return && !mission && st.params.avoid_zone_only && st.params.v_avoid > 0.0f) {
    // The zone episode owns its reduced speed until GPS alignment releases it.
    // Reference source changes to GPS before that episode is complete.
    out.v_ref = std::min(out.v_ref, st.params.v_avoid);
  }
  out.top = m.top; out.nav = m.nav; out.avoid = m.avoid; out.signal = m.signal;
  out.parking_calibration = parking_calibration(st.params);
  out.recovery = m.recovery;
  out.route = m.route;
  out.estop_active = m.estop_active;
  out.estop_request_id = m.estop_request_id;
  out.traffic_distance_known = st.traffic_distance_latched;
  out.traffic_remaining_m = st.traffic_stopline_distance;
  out.traffic_stop_in_success_region = st.traffic_distance_latched && s.vehicle_speed_valid &&
    std::isfinite(s.vehicle_speed) && std::fabs(s.vehicle_speed) <= kStoppedSpeed &&
    st.traffic_stopline_distance > 0 && st.traffic_stopline_distance <= (s.revised_v2 ? st.params.traffic_stop_offset : 1.0f);
  out.safety = m.safety; out.mission = m.mission; out.mission_type = m.mission_type;
  out.mission_start = m.mission_start;
  out.mission_cancel = m.mission_cancel;
  out.mission_prepare = m.mission_prepare;
  out.mission_request = m.request;
  out.mission_events = m.mission_events;
  out.active_mission_completed = m.mission_completed[m.request.mission_id];
  out.active_mission_failed = m.mission_failed[m.request.mission_id];
  out.zones = m.zones;
  out.active_mission_id = m.active_mission;
  out.speed_owner = mission ? SpeedOwner::MISSION : avoid ? SpeedOwner::AVOIDANCE : SpeedOwner::NAVIGATION;
  out.safe_stop_reasons = base_stop_reasons(s, st);
  out.avoid_episode_reference_seen = m.avoid_episode_reference_seen;
  for (uint8_t source = 0; source < MGM_REFERENCE_PROVIDERS; ++source) {
    out.references[source] = provider_reference(s, source);
  }
  // Preserve the existing 50-cycle low-confidence hysteresis; after it expires
  // sensor availability does not make the LINE path drivable.
  out.references[MGM_SRC_LANE].valid = out.references[MGM_SRC_LANE].valid &&
    st.lane_low_cnt < st.params.n_cycles && (!s.revised_v2 || !m.lane_recovery_required);
  out.references[MGM_SRC_PARKING].valid = out.references[MGM_SRC_PARKING].valid &&
    mission && m.mission_feedback_seen && s.parking_mission_active &&
    (!st.params.parking_zone_entry_active || m.request.preparation_ready) &&
    s.parking_request_id == m.request.request_id &&
    s.references[MGM_SRC_PARKING].generation >= static_cast<uint64_t>(
      std::max<int64_t>(1, m.request.zone_entry.time_ns)) &&
    s.parking_mission_mode == static_cast<uint8_t>(m.mission_type);
  if (mission && !m.mission_feedback_seen) {out.v_ref = 0.0f; out.immediate_stop = true;}

  if (!mission && st.params.traffic_state_enabled &&
    (m.signal == SignalState::APPROACH_STOP_LINE || m.signal == SignalState::STOPPED_WAIT))
  {
    // Existing stop-line profile, independently capped against the selected
    // navigation/avoidance speed. Never changes lateral ownership.
    const CoreOutput traffic = existing_source_request(request, st, MGM_STATE_TRAFFIC);
    out.v_ref = m.signal == SignalState::STOPPED_WAIT ? 0.0f :
      std::min(out.v_ref, traffic.v_ref);
    out.immediate_stop = out.immediate_stop || traffic.immediate_stop;
    out.speed_owner = SpeedOwner::TRAFFIC;

  }
  if (!mission && !st.params.traffic_state_enabled && s.traffic_stop_required) {
    out.v_ref = 0.0f;
    out.speed_owner = SpeedOwner::TRAFFIC;
  }
  if (st.stop_zone_holding && !mission) {out.v_ref = 0.0f;}
  // A negative parking/recovery ramp cannot carry over to a forward owner.
  // This uses the existing immediate-stop output for one ownership handoff tick.
  if (!mission && st.escape_phase == MGM_ESCAPE_NONE && st.v < 0.0f) {
    out.v_ref = 0.0f;
    out.immediate_stop = true;
    out.speed_owner = SpeedOwner::SAFETY;
  }
  if (m.safety == SafetyState::REVERSE_RECOVERY) {
    out.speed_owner = SpeedOwner::SAFETY;
    if (st.escape_phase == MGM_ESCAPE_REVERSING) {
      out.path_source = MGM_SRC_ESCAPE;

      out.v_ref = st.params.v_escape;  // Recovery has its own requested speed magnitude.
      out.immediate_stop = false;
      auto & escape = out.references[MGM_SRC_ESCAPE];
      escape.source = MGM_SRC_ESCAPE;
      escape.available = true;  // existing deterministic generator, checked after assemble
      escape.fresh = true; escape.age_s = 0.0f;
      const bool rear_allowed = recovery_rear_allowed(s, st.params);
      escape.valid = rear_allowed && st.params.escape_after_cycles > 0 &&
        st.params.escape_max_cycles > 0 && std::isfinite(st.params.v_escape) && st.params.v_escape < 0;
      if (!rear_allowed && !st.params.safe_stop_all_sensors_only) {
        out.safe_stop_reasons |= SAFE_STOP_REAR_UNAVAILABLE;
      }
    } else {
      out.v_ref = 0.0f;
      out.immediate_stop = true;  // existing recovery ended, waiting for real ref
    }
  }
  if (m.safety == SafetyState::SAFE_STOP || m.safety == SafetyState::AUTO_ESTOP ||
    s.external_stop || m.top != TopState::AUTONOMOUS_DRIVE)
  {
    out.v_ref = 0.0f;
    out.immediate_stop = true;
    out.speed_owner = m.top == TopState::FINISH ? SpeedOwner::FINISH : SpeedOwner::SAFETY;
  }
  // Entry braking precedes the CAN-speed-gated five-frame wall acquisition.
  // Only fresh completion for this request releases normal GPS search speed.
  const bool t_search_endpoint = m.mission_type == MissionType::T_PARKING &&
    (m.route.enabled ? m.route.end_reached : s.gps_at_end);
  if (mission_searches_along_gps(st) && (t_search_endpoint || !(s.parking_valid &&
    s.parking_request_id == m.request.request_id &&
    s.parking_mission_mode == static_cast<uint8_t>(m.mission_type) &&
    s.parking_wall_acquisition_complete)))
  {
    out.v_ref = 0.0f;
    out.immediate_stop = true;
    out.speed_owner = SpeedOwner::MISSION;
  }
  if (s.revised_v2 && m.estop_active) {estop_decision(s, st, out);}
  // A final route waits for real stationary feedback; middle transitions do not.
  if (s.revised_v2 && !m.estop_active && (m.route.phase == RoutePhase::WAIT_STOP ||
    m.route.phase == RoutePhase::WAIT_MISSION)) {
    out.v_ref = 0; out.immediate_stop = true;
  }
  out.selected_reference = out.references[out.path_source];
  out.reference_available = out.selected_reference.available;
  if (!out.selected_reference.valid && !st.params.safe_stop_all_sensors_only && !s.revised_v2) {
    out.safe_stop_reasons |= SAFE_STOP_REFERENCE_INVALID;
  }
  if (out.safe_stop_reasons != 0) {
    out.safety = SafetyState::SAFE_STOP;
    out.v_ref = 0.0f;
    out.immediate_stop = true;
    out.speed_owner = m.top == TopState::FINISH ? SpeedOwner::FINISH : SpeedOwner::SAFETY;
  }
  out.state = s.revised_v2 && m.estop_active ? MGM_STATE_ESTOP : legacy_state_projection(st);
  return out;
}
}  // namespace adas_mgm
