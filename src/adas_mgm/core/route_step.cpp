#include "route_step.hpp"
#include "reference_safety.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
namespace adas_mgm {
namespace {
bool gps(const CoreSnapshot & s) {return provider_reference(s, MGM_SRC_GPS).valid;}
bool metadata(const RouteFeedback & f) {
  if (!f.enabled || !f.sequence_id || !f.instance_id || f.count < 1 || f.count > 256 ||
    f.index < 0 || f.index >= f.count || f.required_count < 0 || f.required_count > 256) {return false;}
  if (f.completion != RouteCompletion::ENDPOINT_AND_MISSIONS &&
    f.completion != RouteCompletion::MISSIONS_COMPLETE) {return false;}
  if (f.completion == RouteCompletion::MISSIONS_COMPLETE && f.required_count == 0) {return false;}
  if ((f.index == 0 && f.connecting) || (f.index + 1 == f.count && f.next_connecting)) {return false;}
  bool seen[256]{};
  for (int i=0; i<f.required_count; ++i) {
    if (seen[f.required_missions[i]]) {return false;}
    seen[f.required_missions[i]] = true;
  }
  return true;
}
void requirements(const RouteFeedback & f, RouteControl & r) {
  r.completion = f.completion;
  r.connecting = f.connecting; r.next_connecting = f.next_connecting;
  for (auto & required : r.required_missions) {required = false;}
  for (int i=0; i<f.required_count; ++i) {r.required_missions[f.required_missions[i]] = true;}
}
void request(const CoreSnapshot & s, RouteControl & r, int index, bool connecting=false) {
  if (r.request_id == std::numeric_limits<uint64_t>::max()) {r.phase = RoutePhase::FAULT; return;}
  r.request_id = std::max(r.request_id + 1, static_cast<uint64_t>(std::max<int64_t>(0, s.event_time_ns)));
  r.requested_index = index; r.requested_connecting = connecting;
  r.request_generation = s.references[MGM_SRC_GPS].generation;
  r.phase = RoutePhase::WAIT_ACK;
}
}
void route_reset(const RouteControl & previous, const CoreSnapshot & s, CoreState & st) {
  if (!st.params.route_sequence_enabled) {return;}
  auto & r = st.managers.route;
  r = previous; r.enabled = true; r.changed = false; r.session_reset_pending = true;
  if (gps(s) && metadata(s.route)) {
    r.sequence_id = s.route.sequence_id; r.instance_id = s.route.instance_id;
    r.count = s.route.count; r.index = s.route.index;
    r.request_id = std::max(r.request_id, s.route.acknowledged_request);
  }
  r.seen_nonterminal = r.end_reached = false;
  if (r.instance_id) {request(s, r, 0);} else {r.phase = RoutePhase::DISABLED;}
}
void route_observe(const CoreSnapshot & s, CoreState & st) {
  auto & r = st.managers.route;
  const auto & f = s.route;
  r.changed = false;
  if (!st.params.route_sequence_enabled) {
    if (f.enabled) {r.enabled = true; r.phase = RoutePhase::FAULT;}
    return;
  }
  r.enabled = true;
  if ((!gps(s) && !s.route_metadata_fresh) || r.phase == RoutePhase::FAULT || r.phase == RoutePhase::FINISHED) {return;}
  if (!metadata(f)) {r.phase = RoutePhase::FAULT; return;}
  if (r.phase == RoutePhase::DISABLED) {
    r.sequence_id = f.sequence_id; r.instance_id = f.instance_id; r.count = f.count; r.index = f.index;
    if (f.index != 0 || f.acknowledged_request != 0) {r.phase = RoutePhase::FAULT; return;}
    requirements(f, r); r.phase = RoutePhase::RUNNING;
  }
  if (r.sequence_id != f.sequence_id || r.instance_id != f.instance_id || r.count != f.count) {
    r.phase = RoutePhase::FAULT; return;
  }
  if (r.phase == RoutePhase::WAIT_ACK) {
    if (!gps(s)) {return;}  // CSV handoff still requires a localized new generation.
    if (f.index == r.requested_index && f.connecting == r.requested_connecting && f.acknowledged_request == r.request_id) {
      if (s.references[MGM_SRC_GPS].generation <= r.request_generation) {return;}
      r.index = f.index; r.seen_nonterminal = r.end_reached = false;
      r.last_generation = 0; r.phase = RoutePhase::RUNNING; r.changed = true;
      requirements(f, r);
      // A route boundary is not a new mission session. Only spatial history and
      // the navigation reference handoff are renewed; completion memory survives.
      st.managers.zones = ZoneState{};
      st.managers.gps_only_context = false;
      st.managers.nav = NavState::GPS_BACKUP;
      st.lane_high_cnt = st.lane_low_cnt = 0;
    } else if (f.index != r.index || f.connecting != r.connecting || f.acknowledged_request >= r.request_id) {
      r.phase = RoutePhase::FAULT;
    }
    return;
  }
  if (f.index != r.index || f.acknowledged_request != r.request_id) {r.phase = RoutePhase::FAULT; return;}
  if (r.completion != f.completion || r.connecting != f.connecting || r.next_connecting != f.next_connecting) {r.phase = RoutePhase::FAULT; return;}
  bool expected[256]{};
  for (int i=0; i<f.required_count; ++i) {expected[f.required_missions[i]] = true;}
  for (int i=0; i<256; ++i) {
    if (expected[i] != r.required_missions[i]) {r.phase = RoutePhase::FAULT; return;}
  }
}
void route_step(const CoreSnapshot & s, CoreState & st) {
  auto & m = st.managers; auto & r = m.route;
  if (!r.enabled || r.phase == RoutePhase::FAULT || r.phase == RoutePhase::WAIT_ACK ||
    r.phase == RoutePhase::DISABLED || r.phase == RoutePhase::FINISHED || r.changed ||
    m.top != TopState::AUTONOMOUS_DRIVE || !gps(s)) {return;}
  const auto generation = s.references[MGM_SRC_GPS].generation;
  if (generation > r.last_generation) {
    r.last_generation = generation;
    if (!s.gps_at_end) {r.seen_nonterminal = true;}
    else if (r.seen_nonterminal) {r.end_reached = true;}
  }
  // Immediate-entry Parking is ended by mission_step on a valid current endpoint.
  // Historical PREPARE policy retains its ACTIVE maneuver across the boundary.
  if (m.mission == MissionState::MISSION_ACTIVE) {return;}
  bool complete = !m.request.active;
  if (!r.connecting) {
    for (int i=0; i<256; ++i) {complete &= !r.required_missions[i] || m.mission_completed[i] || m.mission_failed[i];}
  }
  const bool boundary = !r.connecting && r.completion == RouteCompletion::MISSIONS_COMPLETE ? complete : r.end_reached;
  if (!boundary) {return;}
  r.phase = complete ? RoutePhase::WAIT_STOP : RoutePhase::WAIT_MISSION;
  if (!complete || s.external_stop || s.auto_estop || s.traffic_fail_safe_stop ||
    m.signal == SignalState::APPROACH_STOP_LINE || m.signal == SignalState::STOPPED_WAIT ||
    st.stop_zone_holding || st.wrongway_latched || st.escape_phase != MGM_ESCAPE_NONE ||
    (m.zones.definitions_seen && m.zones.calibration != CalibrationState::CALIBRATED) ||
    !s.vehicle_speed_valid || !std::isfinite(s.vehicle_speed) || std::fabs(s.vehicle_speed) > 1e-3f) {return;}
  if (r.connecting) {request(s, r, r.index, false);}
  else if (r.index + 1 == r.count) {
    r.phase = RoutePhase::FINISHED; m.top = TopState::FINISH; st.at_end_latched = true;
  } else {request(s, r, r.index + 1, r.next_connecting);}
}
bool route_stop(const CoreSnapshot & s, const CoreState & st) {
  const auto & r = st.managers.route;
  if (!r.enabled && !st.params.route_sequence_enabled) {return false;}
  if (r.phase == RoutePhase::FAULT) {return true;}
  if (r.phase == RoutePhase::FINISHED) {return false;}
  if (st.managers.mission == MissionState::MISSION_ACTIVE) {return false;}
  // During an ordinary CSV leg, camera navigation may continue without GPS.
  // Route/mission advancement still requires new GPS evidence in route_step.
  const bool camera_navigation = !r.connecting &&
    st.managers.nav == NavState::LINE && provider_reference(s, MGM_SRC_LANE).valid;
  return r.changed || (!gps(s) && !camera_navigation) || r.phase != RoutePhase::RUNNING ||
    (s.gps_at_end && !r.seen_nonterminal);
}
}
