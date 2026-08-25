#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

#include "core/mgm_step.hpp"
#include "src/decision_backend.hpp"

using adas_mgm::CoreOutput;
using adas_mgm::CoreParams;
using adas_mgm::CorePath;
using adas_mgm::CorePoint;
using adas_mgm::CoreSnapshot;
using adas_mgm::CoreState;
using adas_mgm::DecisionBackend;
using adas_mgm::MGM_NUM_POINTS;
using adas_mgm::MGM_SRC_AVOID;
using adas_mgm::MGM_SRC_PARKING;
using adas_mgm::MGM_STATE_AVOID;
using adas_mgm::MGM_STATE_LANE;
using adas_mgm::MGM_STATE_PARKING;
using adas_mgm::MGM_STATE_WAYPOINT;
using adas_mgm::generatedInputWithinLaneWaypointScope;
using adas_mgm::mgm_init;
using adas_mgm::mgm_step;
using adas_mgm::validateGeneratedOutput;

namespace
{

int failures = 0;

void expect(bool condition, const char * message)
{
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", message);
  }
}

CoreParams makeParams()
{
  CoreParams params{};
  params.lane_conf_exit = 0.35f;
  params.lane_conf_return = 0.70f;
  params.n_cycles = 3;
  params.v_base = 0.6f;
  params.v_accel_zone = 1.0f;
  params.v_narrow = 0.2f;
  params.ttc_stop = 0.8f;
  params.blend_cycles = 2;
  params.a_up = 0.5f;
  params.a_down = 1.5f;
  params.wrongway_yaw = 2.1f;
  params.wrongway_cycles = 3;
  params.avoid_return_hold_cycles = 300;
  params.lane_entry_max_cross = 0.5f;
  params.avoid_max_cycles = 1200;
  params.v_avoid = 0.5f;
  params.stop_zone_hold_cycles = 3;
  params.avoid_zone_only = 0;
  params.escape_after_cycles = 0;
  return params;
}

void makePath(CorePath & path, float x)
{
  path.n = 1;
  path.pts[0] = CorePoint{x, 0.0f, 0.0f, 0.0f};
}

CoreSnapshot makeInput()
{
  CoreSnapshot input{};
  input.lane_confidence = 0.9f;
  input.gps_cross_track = 0.1f;
  input.gps_heading_valid = true;
  input.lane_updated = true;
  input.gps_updated = true;
  input.avoid_ttc = 1.0e9f;
  makePath(input.lane_path, 1.0f);
  makePath(input.gps_path, 1.2f);
  return input;
}

bool equal(const CoreOutput & left, const CoreOutput & right)
{
  if (left.state != right.state || left.path_source != right.path_source ||
    left.immediate_stop != right.immediate_stop || left.v_ref != right.v_ref ||
    left.n_points != right.n_points)
  {
    return false;
  }
  for (int32_t i = 0; i < left.n_points; ++i) {
    if (left.ref_points[i].x != right.ref_points[i].x ||
      left.ref_points[i].y != right.ref_points[i].y ||
      left.ref_points[i].yaw != right.ref_points[i].yaw ||
      left.ref_points[i].curvature != right.ref_points[i].curvature)
    {
      return false;
    }
  }
  return true;
}

void testSelectorAndCoreDefault()
{
  const CoreParams params = makeParams();
  const CoreSnapshot input = makeInput();
  CoreState reference_state{};
  mgm_init(reference_state, params);

  DecisionBackend backend("core", false, params);
  expect(backend.name() == "core", "core must be the selectable default backend");
  expect(
    equal(backend.step(input), mgm_step(input, reference_state)),
    "core dispatch must remain byte-for-byte behaviorally equivalent");

  bool unknown_failed = false;
  try {
    DecisionBackend unknown("typo", false, params);
  } catch (const std::invalid_argument &) {
    unknown_failed = true;
  }
  expect(unknown_failed, "unknown backend must fail instead of falling back to core");

  bool acknowledgement_failed = false;
  try {
    DecisionBackend generated("generated", false, params);
  } catch (const std::invalid_argument &) {
    acknowledgement_failed = true;
  }
  expect(acknowledgement_failed, "generated backend must require explicit scope acknowledgement");

  CoreParams invalid_guard = params;
  invalid_guard.ttc_stop = std::numeric_limits<float>::quiet_NaN();
  bool invalid_guard_failed = false;
  try {
    DecisionBackend generated("generated", true, invalid_guard);
  } catch (const std::invalid_argument &) {
    invalid_guard_failed = true;
  }
  expect(invalid_guard_failed, "generated safety-guard parameters must fail closed");

  invalid_guard = params;
  invalid_guard.wrongway_yaw = 3.2f;
  invalid_guard_failed = false;
  try {
    DecisionBackend generated("generated", true, invalid_guard);
  } catch (const std::invalid_argument &) {
    invalid_guard_failed = true;
  }
  expect(invalid_guard_failed, "wrong-way threshold above pi must fail closed");

  invalid_guard = params;
  invalid_guard.escape_after_cycles = 1;
  invalid_guard_failed = false;
  try {
    DecisionBackend generated("generated", true, invalid_guard);
  } catch (const std::invalid_argument &) {
    invalid_guard_failed = true;
  }
  expect(
    invalid_guard_failed,
    "generated backend must reject the unsupported rear-escape extension at startup");
}

void testGeneratedDispatchAndActiveState()
{
  const CoreParams params = makeParams();
  DecisionBackend backend("generated", true, params);
  CoreSnapshot startup{};
  startup.avoid_ttc = 1.0e9f;
  startup.estop = true;
  const CoreOutput startup_stop = backend.step(startup);
  expect(!backend.faulted(), "missing startup paths under E-stop must remain recoverable");
  expect(
    startup_stop.immediate_stop && startup_stop.v_ref == 0.0f &&
    startup_stop.n_points == 1,
    "missing startup paths must publish a nonempty fail-stop reference");

  CoreSnapshot input = makeInput();
  CoreOutput output = backend.step(input);
  expect(!backend.faulted(), "fresh valid input must recover after startup hold");
  expect(output.state == MGM_STATE_LANE, "generated backend must start in LANE");

  input.lane_confidence = 0.1f;
  for (int i = 0; i < params.n_cycles; ++i) {
    output = backend.step(input);
  }
  expect(output.state == MGM_STATE_WAYPOINT, "generated backend must enter WAYPOINT");
  expect(
    backend.activeState() == MGM_STATE_WAYPOINT,
    "watchdogs must observe the generated backend's validated state");
}

void testRuntimeFaultIsLatchedAndNonempty()
{
  const CoreParams params = makeParams();
  DecisionBackend backend("generated", true, params);
  CoreSnapshot input = makeInput();
  const CoreOutput valid = backend.step(input);
  expect(valid.n_points > 0, "generated pre-fault output must contain a path");

  input.avoid_ttc = std::numeric_limits<float>::quiet_NaN();
  const CoreOutput stopped = backend.step(input);
  expect(backend.faulted(), "invalid generated input must latch a backend fault");
  expect(
    stopped.immediate_stop && stopped.v_ref == 0.0f && stopped.n_points > 0,
    "fault output must be an immediate nonempty zero-speed reference");

  input.avoid_ttc = 1.0e9f;
  const CoreOutput still_stopped = backend.step(input);
  expect(
    still_stopped.immediate_stop && still_stopped.v_ref == 0.0f &&
    still_stopped.n_points > 0,
    "runtime fault must remain latched after the triggering input clears");
}

void testFourStateDispatchAndObservers()
{
  CoreParams params = makeParams();
  params.stop_zone_hold_cycles = 2;
  {
    DecisionBackend backend("generated", true, params);
    CoreSnapshot input = makeInput();
    makePath(input.avoid_path, 1.5f);
    input.avoid_v_suggest = 0.4f;
    input.avoid_obstacle_detected = true;
    input.avoid_avoidable = true;
    CoreOutput output = backend.step(input);
    expect(!backend.faulted(), "AVOID input must be supported by ADAS_MGR2 v1.88");
    expect(output.state == MGM_STATE_AVOID, "generated backend must enter AVOID");
    expect(output.path_source == MGM_SRC_AVOID, "AVOID must select the avoid path");
    expect(backend.avoidTicks() > 0, "generated AVOID tick observer must expose DWork");

    input.avoid_obstacle_detected = false;
    input.avoid_avoidable = false;
    input.avoid_maneuver_done = true;
    output = backend.step(input);
    expect(output.state == MGM_STATE_WAYPOINT, "AVOID completion must return to WAYPOINT");
    expect(
      backend.returnHoldLeft() == params.avoid_return_hold_cycles,
      "generated return-hold observer must expose DWork");
  }

  {
    DecisionBackend backend("generated", true, params);
    CoreSnapshot input = makeInput();
    makePath(input.parking_path, 0.8f);
    input.parking_v_suggest = -1.5f;
    input.gps_parking_zone = true;
    input.parking_space_found = true;
    CoreOutput output = backend.step(input);
    expect(output.state == MGM_STATE_PARKING, "generated backend must enter PARKING");
    expect(output.path_source == MGM_SRC_PARKING, "PARKING must select the parking path");
    for (int i = 0; i < 200 && output.v_ref > -1.49f; ++i) {
      output = backend.step(input);
    }
    expect(output.v_ref < -1.49f, "PARKING must accept the full negative speed request");
    expect(!backend.faulted(), "valid PARKING reverse speed must not fault the backend");

    input.parking_done = true;
    input.parking_v_suggest = 0.0f;
    output = backend.step(input);
    expect(output.state == MGM_STATE_LANE, "PARKING completion must return to LANE");
    expect(
      output.v_ref < 0.0f && !backend.faulted(),
      "PARKING-origin reverse ramp must remain valid while returning to LANE");
    float previous = output.v_ref;
    for (int i = 0; i < 400 && output.v_ref < 0.0f; ++i) {
      output = backend.step(input);
      expect(
        output.v_ref + 1.0e-4f >= previous,
        "PARKING exit ramp must recover monotonically toward zero");
      expect(!backend.faulted(), "a valid PARKING exit ramp must not latch a fault");
      previous = output.v_ref;
    }
    expect(output.v_ref >= 0.0f, "PARKING exit ramp must eventually clear reverse speed");
  }

  {
    DecisionBackend backend("generated", true, params);
    CoreSnapshot input = makeInput();
    backend.step(input);  // initialize stop-zone boot suppression outside a zone
    input.gps_stop_zone = 1;
    backend.step(input);
    expect(backend.stopZoneHolding(), "generated stop-zone observer must expose DWork");
    expect(backend.stopHoldLeft() > 0, "generated stop-hold counter must expose DWork");
  }
}

void testPureFailClosedValidation()
{
  const CoreParams params = makeParams();
  CoreSnapshot input = makeInput();
  CoreOutput output{};
  output.state = MGM_STATE_LANE;
  output.path_source = 0;
  output.n_points = 1;
  output.ref_points[0] = CorePoint{1.0f, 0.0f, 0.0f, 0.0f};
  output.v_ref = 0.2f;
  std::string reason;
  expect(
    validateGeneratedOutput(output, input, params, reason),
    "valid generated output must pass");

  output.n_points = 0;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "empty output path must fail closed");
  output.n_points = MGM_NUM_POINTS + 1;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "oversized output path must fail closed");
  output.n_points = 1;
  output.state = MGM_STATE_AVOID;
  output.path_source = 0;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "state/source mismatch must fail closed");
  output.state = MGM_STATE_LANE;
  output.path_source = 0;
  output.v_ref = std::numeric_limits<float>::quiet_NaN();
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "NaN v_ref must fail closed");
  output.v_ref = params.v_accel_zone + 0.1f;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "speed above the configured maximum must fail closed");
  output.v_ref = 0.2f;
  output.ref_points[0].x = std::numeric_limits<float>::infinity();
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "non-finite path must fail closed");
  output.ref_points[0].x = 1.0f;
  output.immediate_stop = true;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "immediate-stop with nonzero speed must fail closed");
  output.immediate_stop = false;
  input.estop = true;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "ignored E-stop must fail closed");

  input = makeInput();
  input.avoid_ttc = 0.3f;
  output.state = MGM_STATE_AVOID;
  output.path_source = MGM_SRC_AVOID;
  output.v_ref = 0.2f;
  output.immediate_stop = false;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "AVOID output that ignores the TTC threshold must fail closed");
  output.v_ref = 0.0f;
  output.immediate_stop = true;
  expect(
    validateGeneratedOutput(output, input, params, reason),
    "AVOID immediate stop must satisfy the TTC threshold contract");

  input = makeInput();
  input.avoid_obstacle_detected = true;
  expect(
    generatedInputWithinLaneWaypointScope(input, params, reason),
    "AVOID input must be in scope for ADAS_MGR2 v1.88");
  input = makeInput();
  input.parking_space_found = true;
  expect(
    generatedInputWithinLaneWaypointScope(input, params, reason),
    "PARKING input must be in scope for ADAS_MGR2 v1.88");
  input = makeInput();
  input.avoid_ttc = 0.3f;
  expect(
    generatedInputWithinLaneWaypointScope(input, params, reason),
    "TTC immediate-stop input must be in scope for ADAS_MGR2 v1.88");
  input = makeInput();
  input.gps_path.pts[0].yaw = 2.2f;
  expect(
    generatedInputWithinLaneWaypointScope(input, params, reason),
    "wrong-way input must be in scope for ADAS_MGR2 v1.88");

  input = makeInput();
  input.avoid_ttc = std::numeric_limits<float>::quiet_NaN();
  expect(
    !generatedInputWithinLaneWaypointScope(input, params, reason),
    "non-finite TTC input must fail closed before entering generated code");
  input = makeInput();
  input.avoid_v_suggest = -0.1f;
  expect(
    !generatedInputWithinLaneWaypointScope(input, params, reason),
    "negative AVOID speed suggestion must fail closed");
  input = makeInput();
  input.lane_path.n = MGM_NUM_POINTS + 1;
  expect(
    !generatedInputWithinLaneWaypointScope(input, params, reason),
    "out-of-range input path length must fail closed");

  output = {};
  output.state = MGM_STATE_PARKING;
  output.path_source = MGM_SRC_PARKING;
  output.n_points = 1;
  output.ref_points[0] = CorePoint{1.0f, 0.0f, 0.0f, 0.0f};
  output.v_ref = -0.2f;
  input = makeInput();
  input.parking_v_suggest = -0.2f;
  expect(
    validateGeneratedOutput(output, input, params, reason),
    "negative PARKING v_ref must be accepted");
  output.state = MGM_STATE_LANE;
  output.path_source = 0;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "negative non-PARKING v_ref without a PARKING ramp must fail closed");
  expect(
    validateGeneratedOutput(output, input, params, reason, true, -0.3f),
    "a monotonic negative PARKING exit ramp must be accepted");
  output.v_ref = -0.4f;
  expect(
    !validateGeneratedOutput(output, input, params, reason, true, -0.3f),
    "a PARKING exit ramp that becomes more negative must fail closed");
  output.v_ref = params.v_accel_zone + 0.1f;
  expect(
    !validateGeneratedOutput(output, input, params, reason, true, -1.5f),
    "a previous PARKING speed must not raise the positive speed limit");

  CoreParams escape = params;
  escape.escape_after_cycles = 100;
  expect(
    !generatedInputWithinLaneWaypointScope(input, escape, reason),
    "rear escape must remain explicitly out of generated scope");
}

}  // namespace

int main()
{
  testSelectorAndCoreDefault();
  testGeneratedDispatchAndActiveState();
  testRuntimeFaultIsLatchedAndNonempty();
  testFourStateDispatchAndObservers();
  testPureFailClosedValidation();
  std::printf("decision backend tests: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
