// ROS-free parallel state machine. Existing perception/control algorithms stay in
// their modules; existing_source_request/assemble/merge retain output geometry.
#include "manager_step.hpp"
#include "route_step.hpp"
#include "mgm_step.hpp"
#include "zone_step.hpp"
#include "reference_safety.hpp"
#include "mission_step.hpp"
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
bool line_return_ready(const CoreSnapshot & s, const CoreState & st)
{
  return line_valid(s) && st.lane_high_cnt >= st.params.n_cycles &&
         !st.managers.gps_only_context && !st.managers.route.connecting && (st.return_hold_left == 0 || !gps_valid(s));
}
void nav_reselect(const CoreSnapshot & s, CoreState & st)
{
  if (mission_searches_along_gps(st)) {
    st.managers.nav = st.managers.gps_only_context ? NavState::GPS_ONLY_NAV : NavState::GPS_BACKUP;
  } else if (st.managers.route.enabled && st.managers.route.connecting) {
    st.managers.nav = NavState::GPS_BACKUP;
  } else if (st.managers.gps_only_context) {
    st.managers.nav = NavState::GPS_ONLY_NAV;
  } else if (line_return_ready(s, st)) {
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
  if (st.stop_zone_holding && st.v <= kStoppedSpeed && --st.stop_hold_left <= 0) {
    st.stop_zone_holding = false;
    st.stop_hold_left = 0;
  }
}
uint32_t base_stop_reasons(const CoreSnapshot & s, const CoreState & st)
{
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
  route_observe(s, st);
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
  zone_step(s.zones, gps_valid(s), m.zones,
    st.params.zone_enter_confirm_samples, st.params.zone_exit_confirm_samples);
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
  m.top = s.autonomous_enabled ? TopState::AUTONOMOUS_DRIVE : TopState::AUTONOMOUS_ENABLE;

  const bool line = line_valid(s);
  const bool gps = gps_valid(s);
  st.lane_low_cnt = line && s.lane_confidence < st.params.lane_conf_exit ?
    std::min(st.lane_low_cnt + 1, st.params.n_cycles) : 0;
  st.lane_high_cnt = line && s.lane_confidence >= st.params.lane_conf_return ?
    std::min(st.lane_high_cnt + 1, st.params.n_cycles) : 0;
  if (st.return_hold_left > 0) {--st.return_hold_left;}

  if (m.route.enabled && m.route.connecting) {
    m.nav = NavState::GPS_BACKUP;
  } else if (m.gps_only_context) {
    m.nav = NavState::GPS_ONLY_NAV;
  } else if (was_zone) {
    nav_reselect(s, st);
  } else if (m.nav == NavState::LINE) {
    if ((!line || st.lane_low_cnt >= st.params.n_cycles) && gps) {
      m.nav = NavState::GPS_BACKUP;
    }
  } else if (line_return_ready(s, st)) {
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
  const bool avoid_entry = (s.avoid_obstacle_detected && s.avoid_avoidable &&
    (st.params.avoid_zone_only == 0 || s.gps_avoid_zone)) || fallback;
  const bool was_avoiding = m.avoid != AvoidState::INACTIVE;
  if (!avoid_allowed) {
    m.avoid = AvoidState::INACTIVE;
    m.clear_count = 0;
    m.avoid_fallback_only = false;
    if (!st.params.avoidance_enabled) {
      st.return_hold_left = 0;
      if (was_avoiding) {nav_reselect(s, st);}
    }
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
  } else if (!m.avoid_fallback_only && st.escape_phase == MGM_ESCAPE_NONE &&
    (s.avoid_maneuver_done ||
    (st.params.avoid_max_cycles > 0 && st.avoid_ticks >= st.params.avoid_max_cycles)))
  {
    // main: disappearing from the forward corridor is not passing the object.
    // Wait for the producer's maneuver completion (or the configured episode
    // limit), then start the full GPS return hold. Never finish reverse recovery.
    m.avoid = AvoidState::INACTIVE;
    m.clear_count = 0;
    st.return_hold_left = st.params.avoid_return_hold_cycles;
    m.nav = m.gps_only_context ? NavState::GPS_ONLY_NAV : NavState::GPS_BACKUP;
  } else {
    m.avoid = AvoidState::AVOID_ACTIVE;
    m.clear_count = 0;
    if (s.avoid_obstacle_detected) {m.avoid_fallback_only = false;}
  }
  // The GPS hold begins on actual avoidance exit, as it did in main.
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

  if (st.params.traffic_state_enabled) {
    const bool release = s.traffic_green_active && !s.traffic_red_active;
    if (release) {
      m.signal = SignalState::SIGNAL_IDLE;
      st.traffic_distance_latched = false;
      st.traffic_stopline_distance = 0.0f;
      st.traffic_prev_stopline_detected = false;
    } else {
      if (m.signal == SignalState::SIGNAL_IDLE && s.traffic_red_active) {
        m.signal = SignalState::RED_DETECTED;
      }
      if (m.signal == SignalState::RED_DETECTED && s.traffic_red_active &&
        s.traffic_stopline_detected)
      {
        m.signal = SignalState::APPROACH_STOP_LINE;
      }
      const bool previously_stopped = m.signal == SignalState::STOPPED_WAIT;
      if (m.signal == SignalState::APPROACH_STOP_LINE) {
        // Latch once per stop episode. Detector flicker cannot reseed distance;
        // losing red without green cannot reset it either.
        const bool seed_now = !st.traffic_distance_latched &&
          st.traffic_prev_stopline_detected && !s.traffic_stopline_detected;
        if (seed_now) {
          st.traffic_stopline_distance = st.params.traffic_ramp_distance_m;
          st.traffic_distance_latched = true;
        } else if (st.traffic_distance_latched && s.vehicle_speed_valid && std::isfinite(s.vehicle_speed)) {
          st.traffic_stopline_distance -= static_cast<float>(std::fabs(s.vehicle_speed) * dt);
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
      st.traffic_prev_stopline_detected = s.traffic_stopline_detected;
    }
  }

  update_existing_guards(s, st);
  if (!s.new_session && clock_valid) {route_step(s, st);}
  const bool signal_stop = m.signal == SignalState::APPROACH_STOP_LINE ||
    m.signal == SignalState::STOPPED_WAIT;
  m.safe_stop_reasons = base_stop_reasons(s, st);
  if (!maneuver && signal_stop && (!clock_valid || !std::isfinite(s.vehicle_speed))) {
    m.safe_stop_reasons |= SAFE_STOP_VEHICLE_SPEED;
  }
  const bool sensor_stop = !maneuver && (((m.gps_only_context || mission_searches_along_gps(st)) && !gps) ||
    (!nav_available(s, st) && !gps && (!s.lidar_valid || !st.params.avoidance_enabled)));
  const bool fault_stop = m.safe_stop_reasons != 0;
  // PARKING includes GPS-guided search before readiness. Suppress ordinary
  // LiDAR E-stop throughout that state, just like ordinary avoidance above.
  const bool auto_stop = !mission && (s.auto_estop ||
    (m.avoid != AvoidState::INACTIVE && s.lidar_valid && s.avoid_ttc < st.params.ttc_stop));
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
    sensor_stop || fault_stop || st.stop_zone_holding ? RecoveryBlockReason::FORCED_STOP :
    signal_stop ? RecoveryBlockReason::SIGNAL_STOP :
    !st.escape_armed && st.v <= kStoppedSpeed ? RecoveryBlockReason::NOT_ARMED :
    !s.auto_estop ? RecoveryBlockReason::NO_DANGER : RecoveryBlockReason::NONE;
  const bool enter_recovery = update_escape(recovery, st,
    !mission && !sensor_stop && !fault_stop && !signal_stop && !st.stop_zone_holding &&
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
  }
  if (!mission && was_reversing && st.escape_phase == MGM_ESCAPE_NONE) {
    m.recovery_waiting_reference = true;
    nav_reselect(s, st);
  }
  if (mission) {m.recovery_waiting_reference = false;}
  if (m.recovery_waiting_reference) {
    nav_reselect(s, st);
    const bool reference = m.avoid != AvoidState::INACTIVE ?
      provider_reference(s, MGM_SRC_AVOID).valid : nav_available(s, st);
    if (reference) {m.recovery_waiting_reference = false;}
  }
  const auto previous_safety = m.safety;
  if (sensor_stop || fault_stop) {
    m.safety = SafetyState::SAFE_STOP;
  } else if (st.escape_phase == MGM_ESCAPE_REVERSING || m.recovery_waiting_reference) {
    m.safety = SafetyState::REVERSE_RECOVERY;
  } else if (auto_stop) {
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
  const uint8_t source_state = mission ? MGM_STATE_PARKING : mission_searches_along_gps(st) ? MGM_STATE_WAYPOINT :
    avoid ? MGM_STATE_AVOID :
    m.nav == NavState::LINE ? MGM_STATE_LANE : MGM_STATE_WAYPOINT;
  CoreSnapshot request = s;
  request.estop = false;  // independent safety arbitration below
  request.traffic_stop_required = false;
  CoreOutput out = existing_source_request(request, st, source_state);
  if (source_state != MGM_STATE_AVOID) {
    out.v_ref = fixed_motion_speed(out.v_ref, st.params.v_base);
  }
  out.top = m.top; out.nav = m.nav; out.avoid = m.avoid; out.signal = m.signal;
  out.parking_calibration = parking_calibration(st.params);
  out.recovery = m.recovery;
  out.route = m.route;
  out.traffic_distance_known = st.traffic_distance_latched;
  out.traffic_remaining_m = st.traffic_stopline_distance;
  out.traffic_stop_in_success_region = st.traffic_distance_latched && s.vehicle_speed_valid &&
    std::isfinite(s.vehicle_speed) && std::fabs(s.vehicle_speed) <= kStoppedSpeed &&
    st.traffic_stopline_distance > 0 && st.traffic_stopline_distance <= 1.0f;
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
    st.lane_low_cnt < st.params.n_cycles;
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
      if (!rear_allowed) {out.safe_stop_reasons |= SAFE_STOP_REAR_UNAVAILABLE;}
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
  out.selected_reference = out.references[out.path_source];
  out.reference_available = out.selected_reference.available;
  if (!out.selected_reference.valid) {out.safe_stop_reasons |= SAFE_STOP_REFERENCE_INVALID;}
  if (out.safe_stop_reasons != 0) {
    out.safety = SafetyState::SAFE_STOP;
    out.v_ref = 0.0f;
    out.immediate_stop = true;
    out.speed_owner = m.top == TopState::FINISH ? SpeedOwner::FINISH : SpeedOwner::SAFETY;
  }
  out.state = legacy_state_projection(st);
  return out;
}
}  // namespace adas_mgm
