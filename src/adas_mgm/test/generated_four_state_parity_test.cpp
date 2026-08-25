#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>

#include "core/mgm_step.hpp"
#include "src/generated_adapter.hpp"

using adas_mgm::CoreOutput;
using adas_mgm::CoreParams;
using adas_mgm::CorePath;
using adas_mgm::CorePoint;
using adas_mgm::CoreSnapshot;
using adas_mgm::CoreState;
using adas_mgm::GeneratedMgmAdapter;
using adas_mgm::MGM_NUM_POINTS;
using adas_mgm::MGM_SRC_AVOID;
using adas_mgm::MGM_SRC_GPS;
using adas_mgm::MGM_SRC_LANE;
using adas_mgm::MGM_SRC_PARKING;
using adas_mgm::MGM_STATE_AVOID;
using adas_mgm::MGM_STATE_LANE;
using adas_mgm::MGM_STATE_PARKING;
using adas_mgm::MGM_STATE_WAYPOINT;
using adas_mgm::mgm_init;
using adas_mgm::mgm_step;

namespace
{

constexpr float kTolerance = 3e-5f;

bool near(float left, float right)
{
  return std::fabs(left - right) <= kTolerance;
}

void makePath(CorePath & path, float x, float y, float yaw, int32_t count = MGM_NUM_POINTS)
{
  path.n = count;
  for (int32_t i = 0; i < count; ++i) {
    const float index = static_cast<float>(i);
    path.pts[i] = CorePoint{
      x + 0.07f * index,
      y + 0.003f * index,
      yaw + 0.001f * index,
      0.006f + 0.0004f * index};
  }
}

// All 18 parameters implemented by ADAS_MGR2 v1.88 deliberately differ from
// the constants compiled into the generated C. Every one is exercised below,
// so an omitted adapter assignment creates an observable parity failure rather
// than passing because the C default happened to match the C++ reference.
CoreParams makeParams()
{
  CoreParams params{};
  params.lane_conf_exit = 0.31f;
  params.lane_conf_return = 0.74f;
  params.n_cycles = 4;
  params.v_base = 0.53f;
  params.v_accel_zone = 0.91f;
  params.v_narrow = 0.17f;
  params.ttc_stop = 0.67f;
  params.blend_cycles = 3;
  params.a_up = 4.3f;
  params.a_down = 3.1f;
  params.wrongway_yaw = 1.83f;
  params.wrongway_cycles = 3;
  params.avoid_return_hold_cycles = 5;
  params.lane_entry_max_cross = 0.42f;
  params.avoid_max_cycles = 40;
  params.v_avoid = 0.37f;
  params.stop_zone_hold_cycles = 4;
  params.avoid_zone_only = 1;

  // v1.88 predates the rear-clear/escape extension. Keep that extension
  // disabled; an escape-enabled dump is rejected explicitly by parity_replay.
  params.escape_after_cycles = 0;
  params.v_escape = -0.3f;
  params.escape_max_cycles = 200;
  params.escape_require_rear_clear = 1;
  return params;
}

CoreSnapshot nominalSnapshot()
{
  CoreSnapshot input{};
  input.lane_confidence = 0.9f;
  input.gps_cross_track = 0.1f;
  input.gps_heading_valid = true;
  input.lane_updated = true;
  input.gps_updated = true;
  input.avoid_updated = true;
  input.avoid_ttc = 100.0f;
  input.avoid_v_suggest = 0.81f;
  input.parking_v_suggest = -0.29f;
  makePath(input.lane_path, 1.00f, 0.00f, 0.01f);
  makePath(input.gps_path, 1.20f, 0.12f, 0.03f);
  // A one-point avoid target also exercises the generated model's 20-point
  // wire expansion while the full-output comparison checks every point.
  makePath(input.avoid_path, 1.45f, 0.52f, 0.19f, 1);
  makePath(input.parking_path, 0.75f, -0.34f, -0.16f);
  return input;
}

bool equal(const CoreOutput & reference, const CoreOutput & generated)
{
  if (reference.state != generated.state ||
    reference.path_source != generated.path_source ||
    reference.immediate_stop != generated.immediate_stop ||
    !near(reference.v_ref, generated.v_ref) ||
    reference.n_points != generated.n_points)
  {
    return false;
  }
  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    if (!near(reference.ref_points[i].x, generated.ref_points[i].x) ||
      !near(reference.ref_points[i].y, generated.ref_points[i].y) ||
      !near(reference.ref_points[i].yaw, generated.ref_points[i].yaw) ||
      !near(reference.ref_points[i].curvature, generated.ref_points[i].curvature))
    {
      return false;
    }
  }
  return true;
}

class ParityHarness
{
public:
  explicit ParityHarness(const CoreParams & params)
  : params_(params), generated_(params)
  {
    mgm_init(reference_state_, params_);
  }

  CoreOutput run(const CoreSnapshot & input, const char * phase)
  {
    const CoreOutput reference = mgm_step(input, reference_state_);
    const CoreOutput generated = generated_.step(input);
    ++ticks_;
    if (!equal(reference, generated)) {
      if (mismatches_ < 24) {
        std::fprintf(
          stderr,
          "tick=%d phase=%s C++(state=%u src=%u stop=%d v=%.6f n=%d) "
          "ERT(state=%u src=%u stop=%d v=%.6f n=%d)\n",
          ticks_ - 1, phase,
          reference.state, reference.path_source, reference.immediate_stop,
          reference.v_ref, reference.n_points,
          generated.state, generated.path_source, generated.immediate_stop,
          generated.v_ref, generated.n_points);
      }
      ++mismatches_;
    }
    seen_state_[reference.state < 4U ? reference.state : 0U] = true;
    seen_source_[reference.path_source < 4U ? reference.path_source : 0U] = true;
    return reference;
  }

  void expect(bool condition, const char * message)
  {
    if (!condition) {
      ++assertion_failures_;
      std::fprintf(stderr, "coverage/assertion failed: %s\n", message);
    }
  }

  int finish()
  {
    for (int state = 0; state < 4; ++state) {
      if (!seen_state_[state] || !seen_source_[state]) {
        ++assertion_failures_;
        std::fprintf(
          stderr, "coverage missing: state[%d]=%d source[%d]=%d\n",
          state, seen_state_[state], state, seen_source_[state]);
      }
    }
    std::printf(
      "four-state generated parity: ticks=%d mismatches=%d assertions=%d\n",
      ticks_, mismatches_, assertion_failures_);
    return mismatches_ == 0 && assertion_failures_ == 0 ? 0 : 1;
  }

private:
  CoreParams params_{};
  CoreState reference_state_{};
  GeneratedMgmAdapter generated_;
  int ticks_{0};
  int mismatches_{0};
  int assertion_failures_{0};
  bool seen_state_[4]{};
  bool seen_source_[4]{};
};

}  // namespace

int main()
{
  const CoreParams params = makeParams();
  ParityHarness harness(params);

  CoreOutput output{};

  // Initialization, a_up, v_base, v_accel_zone, and normal/emergency stop
  // ordering. The first ramp value discriminates a_up from the ERT default.
  output = harness.run(nominalSnapshot(), "initial lane ramp");
  harness.expect(
    output.state == MGM_STATE_LANE && output.path_source == MGM_SRC_LANE,
    "initial state/source must be LANE");
  harness.expect(near(output.v_ref, params.a_up * 0.01f), "custom a_up must drive first ramp tick");
  for (int i = 0; i < 20; ++i) {
    output = harness.run(nominalSnapshot(), "lane base speed");
  }
  harness.expect(near(output.v_ref, params.v_base), "LANE must reach custom v_base");

  for (int i = 0; i < 12; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_accel_zone = true;
    output = harness.run(input, "lane acceleration zone");
  }
  harness.expect(
    near(
      output.v_ref,
      params.v_accel_zone), "acceleration zone must reach custom target");

  for (int i = 0; i < 35; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_accel_zone = true;
    input.traffic_stop_required = true;
    output = harness.run(input, "traffic over acceleration priority");
  }
  harness.expect(
    !output.immediate_stop && near(output.v_ref, 0.0f),
    "traffic stop must override acceleration through a_down ramp");
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_accel_zone = true;
    input.traffic_stop_required = true;
    input.estop = true;
    output = harness.run(input, "estop over traffic priority");
  }
  harness.expect(
    output.immediate_stop && near(output.v_ref, 0.0f),
    "E-stop must override traffic stop immediately");
  for (int i = 0; i < 20; ++i) {
    output = harness.run(nominalSnapshot(), "recover before stop zone");
  }

  // Numbered stop zone: stop beats acceleration, hold begins only once the
  // rate-limited command reaches zero, then exactly the configured short hold
  // is consumed and the same zone ID cannot retrigger.
  bool stop_zone_reached_zero = false;
  bool stop_zone_restarted = false;
  for (int i = 0; i < 60; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_stop_zone = 7U;
    input.gps_accel_zone = true;
    output = harness.run(input, "numbered stop zone");
    stop_zone_reached_zero = stop_zone_reached_zero || near(output.v_ref, 0.0f);
    stop_zone_restarted = stop_zone_restarted ||
      (stop_zone_reached_zero && output.v_ref > 0.0f);
    if (stop_zone_restarted && near(output.v_ref, params.v_accel_zone)) {
      break;
    }
  }
  harness.expect(stop_zone_reached_zero, "stop zone must command a complete stop");
  harness.expect(stop_zone_restarted, "stop zone must release after custom hold count");
  for (int i = 0; i < 8; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_stop_zone = 7U;
    output = harness.run(input, "consumed stop zone ID");
  }
  harness.expect(output.v_ref > 0.0f, "consumed stop zone ID must not retrigger");

  // GPS-only zone must force WAYPOINT immediately and reset the lane-return
  // counter; leaving it requires a fresh n_cycles run.
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_gps_only_zone = true;
    output = harness.run(input, "enter gps-only zone");
  }
  harness.expect(
    output.state == MGM_STATE_WAYPOINT && output.path_source == MGM_SRC_GPS,
    "GPS-only zone must immediately select WAYPOINT");
  for (int i = 0; i < 7; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_gps_only_zone = true;
    output = harness.run(input, "hold gps-only zone");
  }
  harness.expect(output.state == MGM_STATE_WAYPOINT, "GPS-only zone must suppress LANE return");
  int gps_only_return_ticks = 0;
  do {
    output = harness.run(nominalSnapshot(), "leave gps-only zone");
    ++gps_only_return_ticks;
  } while (output.state != MGM_STATE_LANE && gps_only_return_ticks < 10);
  harness.expect(
    gps_only_return_ticks == params.n_cycles,
    "GPS-only exit must collect a fresh custom n_cycles window");

  // Exercise both confidence thresholds and the cross-track re-entry gate
  // with values deliberately between custom and generated defaults.
  for (int i = 0; i < params.n_cycles + 2; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.lane_confidence = 0.33f;  // > custom exit, < generated default
    output = harness.run(input, "custom lane exit threshold hold");
  }
  harness.expect(
    output.state == MGM_STATE_LANE,
    "confidence above custom exit threshold must remain LANE");
  for (int i = 0; i < params.n_cycles; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.lane_confidence = 0.20f;
    output = harness.run(input, "lane to waypoint");
  }
  harness.expect(output.state == MGM_STATE_WAYPOINT, "low confidence must enter WAYPOINT");
  for (int i = 0; i < params.n_cycles + 2; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.lane_confidence = 0.72f;  // < custom return, > generated default
    output = harness.run(input, "custom lane return threshold hold");
  }
  harness.expect(
    output.state == MGM_STATE_WAYPOINT,
    "confidence below custom return threshold must remain WAYPOINT");
  for (int i = 0; i < params.n_cycles + 2; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_cross_track = 0.45f;  // > custom gate, < generated default
    output = harness.run(input, "cross-track return gate");
  }
  harness.expect(
    output.state == MGM_STATE_WAYPOINT,
    "custom cross-track gate must block LANE return");
  output = harness.run(nominalSnapshot(), "cross-track gate release");
  harness.expect(output.state == MGM_STATE_LANE, "rejoined track must permit LANE return");

  // avoid_zone_only, v_avoid, v_narrow, TTC priority, explicit completion,
  // return hold, and timeout are all covered in two AVOID passes.
  for (int i = 0; i < params.n_cycles + 1; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.avoid_obstacle_detected = true;
    input.avoid_avoidable = true;
    input.gps_avoid_zone = false;
    output = harness.run(input, "avoid denied outside zone");
  }
  harness.expect(
    output.state == MGM_STATE_LANE,
    "avoid_zone_only must deny AVOID outside designated zone");
  {
    CoreSnapshot input = nominalSnapshot();
    input.avoid_obstacle_detected = true;
    input.avoid_avoidable = true;
    input.gps_avoid_zone = true;
    output = harness.run(input, "enter avoid zone");
  }
  harness.expect(
    output.state == MGM_STATE_AVOID && output.path_source == MGM_SRC_AVOID,
    "designated avoid zone must permit AVOID");
  for (int i = 0; i < 12; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_avoid_zone = true;
    output = harness.run(input, "avoid speed cap");
  }
  harness.expect(near(output.v_ref, params.v_avoid), "AVOID must honor custom v_avoid cap");
  for (int i = 0; i < 10; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_avoid_zone = true;
    input.avoid_narrow_gap = true;
    output = harness.run(input, "narrow-gap speed cap");
  }
  harness.expect(near(output.v_ref, params.v_narrow), "narrow gap must honor custom v_narrow cap");
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_avoid_zone = true;
    input.avoid_ttc = 0.70f;  // above custom threshold, below generated default
    output = harness.run(input, "custom TTC threshold clear");
  }
  harness.expect(
    !output.immediate_stop && output.v_ref > 0.0f,
    "TTC above custom threshold must not immediate-stop");
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_avoid_zone = true;
    input.avoid_ttc = 0.50f;
    output = harness.run(input, "TTC immediate stop");
  }
  harness.expect(
    output.immediate_stop && near(output.v_ref, 0.0f),
    "TTC below custom threshold must immediate-stop");
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_avoid_zone = true;
    input.avoid_maneuver_done = true;
    output = harness.run(input, "avoid maneuver complete");
  }
  harness.expect(
    output.state == MGM_STATE_WAYPOINT,
    "completed AVOID must return to WAYPOINT");
  int avoid_return_ticks = 0;
  do {
    output = harness.run(nominalSnapshot(), "avoid return hold");
    ++avoid_return_ticks;
  } while (output.state != MGM_STATE_LANE && avoid_return_ticks < 12);
  harness.expect(
    avoid_return_ticks == params.avoid_return_hold_cycles,
    "AVOID return must honor custom hold count before LANE");

  {
    CoreSnapshot input = nominalSnapshot();
    input.avoid_obstacle_detected = true;
    input.avoid_avoidable = true;
    input.gps_avoid_zone = true;
    output = harness.run(input, "enter avoid for timeout");
  }
  harness.expect(output.state == MGM_STATE_AVOID, "second AVOID entry must succeed");
  int avoid_timeout_ticks = 0;
  do {
    output = harness.run(nominalSnapshot(), "avoid timeout");
    ++avoid_timeout_ticks;
  } while (output.state == MGM_STATE_AVOID && avoid_timeout_ticks < 60);
  harness.expect(
    output.state == MGM_STATE_WAYPOINT && avoid_timeout_ticks <= params.avoid_max_cycles + 1,
    "AVOID must leave at the custom maximum duration");
  for (int i = 0; i < params.avoid_return_hold_cycles; ++i) {
    output = harness.run(nominalSnapshot(), "return after avoid timeout");
  }
  harness.expect(output.state == MGM_STATE_LANE, "timeout return must eventually reach LANE");

  // LANE transition priority is PARKING before AVOID. In PARKING, avoidance,
  // traffic, and acceleration are ignored; signed parking velocity is valid.
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_parking_zone = true;
    input.parking_space_found = true;
    input.avoid_obstacle_detected = true;
    input.avoid_avoidable = true;
    input.gps_avoid_zone = true;
    output = harness.run(input, "parking over avoid transition priority");
  }
  harness.expect(
    output.state == MGM_STATE_PARKING && output.path_source == MGM_SRC_PARKING,
    "PARKING transition must outrank simultaneous AVOID in LANE");
  for (int i = 0; i < 35; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_parking_zone = true;
    input.parking_space_found = true;
    input.avoid_obstacle_detected = true;
    input.avoid_avoidable = true;
    input.gps_avoid_zone = true;
    input.traffic_stop_required = true;
    input.gps_accel_zone = true;
    output = harness.run(input, "signed parking velocity");
  }
  harness.expect(
    near(output.v_ref, -0.29f),
    "PARKING must preserve its negative velocity command");
  for (int i = 0; i < 12; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_parking_zone = true;
    input.parking_space_found = true;
    input.parking_path_blocked = true;
    output = harness.run(input, "parking path blocked");
  }
  harness.expect(
    !output.immediate_stop && near(output.v_ref, 0.0f),
    "parking blockage must rate-limit to zero");
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_parking_zone = true;
    input.parking_space_found = true;
    input.parking_path_blocked = true;
    input.estop = true;
    output = harness.run(input, "parking estop priority");
  }
  harness.expect(
    output.immediate_stop && near(output.v_ref, 0.0f),
    "PARKING E-stop must be immediate");
  {
    CoreSnapshot input = nominalSnapshot();
    input.parking_done = true;
    output = harness.run(input, "parking complete");
  }
  harness.expect(output.state == MGM_STATE_LANE, "parking completion must return to LANE");

  // at_end is a latched, non-immediate stop shared by LANE/WAYPOINT. It must
  // beat acceleration and persist after the raw flag clears until a real
  // EstopRequest release event resets it.
  for (int i = 0; i < 20; ++i) {
    output = harness.run(nominalSnapshot(), "recover before at-end");
  }
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_at_end = true;
    input.gps_accel_zone = true;
    output = harness.run(input, "at-end latch");
  }
  for (int i = 0; i < 25; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_accel_zone = true;
    output = harness.run(input, "at-end remains latched");
  }
  harness.expect(
    !output.immediate_stop && near(output.v_ref, 0.0f),
    "at_end latch must beat acceleration after gps_at_end clears");
  {
    CoreSnapshot input = nominalSnapshot();
    input.estop = true;
    input.estop_latch_release = true;
    output = harness.run(input, "at-end release event");
  }
  harness.expect(output.immediate_stop, "real E-stop release event tick still honors E-stop");
  output = harness.run(nominalSnapshot(), "drive after at-end release");
  harness.expect(output.v_ref > 0.0f, "released at_end latch must allow driving again");

  // Wrong-way latch: yaw is deliberately between the custom and generated
  // default thresholds. Untrusted heading must preserve the latch; only a
  // trusted aligned run of wrongway_cycles clears it.
  {
    CoreSnapshot input = nominalSnapshot();
    input.gps_gps_only_zone = true;
    output = harness.run(input, "waypoint for wrong-way test");
  }
  harness.expect(output.state == MGM_STATE_WAYPOINT, "wrong-way test must begin in WAYPOINT");
  for (int i = 0; i < params.wrongway_cycles; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_cross_track = 0.8f;
    input.gps_path.pts[0].yaw = 2.0f;
    output = harness.run(input, "wrong-way latch set");
  }
  for (int i = 0; i < 20; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_cross_track = 0.8f;
    input.gps_heading_valid = false;
    output = harness.run(input, "wrong-way latch untrusted hold");
  }
  harness.expect(
    output.state == MGM_STATE_WAYPOINT && near(output.v_ref, 0.0f),
    "untrusted heading must not release wrong-way stop");
  for (int i = 0; i < params.wrongway_cycles; ++i) {
    CoreSnapshot input = nominalSnapshot();
    input.gps_cross_track = 0.8f;
    input.gps_heading_valid = true;
    output = harness.run(input, "wrong-way trusted clear");
  }
  harness.expect(output.v_ref > 0.0f, "trusted aligned heading must release wrong-way latch");
  output = harness.run(nominalSnapshot(), "wrong-way rejoin");
  harness.expect(
    output.state == MGM_STATE_LANE,
    "rejoined track must return from WAYPOINT to LANE");

  return harness.finish();
}
