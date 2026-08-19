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
using adas_mgm::MGM_STATE_LANE;
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

void testScopeFaultIsLatchedAndNonempty()
{
  const CoreParams params = makeParams();
  DecisionBackend backend("generated", true, params);
  CoreSnapshot input = makeInput();
  const CoreOutput valid = backend.step(input);
  expect(valid.n_points > 0, "generated pre-fault output must contain a path");

  input.gps_at_end = true;
  const CoreOutput stopped = backend.step(input);
  expect(backend.faulted(), "unsupported input must latch a backend fault");
  expect(
    stopped.immediate_stop && stopped.v_ref == 0.0f && stopped.n_points > 0,
    "fault output must be an immediate nonempty zero-speed reference");

  input.gps_at_end = false;
  const CoreOutput still_stopped = backend.step(input);
  expect(
    still_stopped.immediate_stop && still_stopped.v_ref == 0.0f,
    "scope fault must remain latched after the triggering input clears");
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
  output.state = 2;
  expect(
    !validateGeneratedOutput(output, input, params, reason),
    "out-of-scope state must fail closed");
  output.state = MGM_STATE_LANE;
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
  input.avoid_obstacle_detected = true;
  expect(
    !generatedInputWithinLaneWaypointScope(input, params, reason),
    "AVOID request must be rejected by the two-state experiment");
  input = makeInput();
  input.parking_space_found = true;
  expect(
    !generatedInputWithinLaneWaypointScope(input, params, reason),
    "PARKING request must be rejected by the two-state experiment");
  input = makeInput();
  input.avoid_ttc = 0.3f;
  expect(
    !generatedInputWithinLaneWaypointScope(input, params, reason),
    "TTC immediate-stop request must be rejected when the generated policy is unavailable");
  input = makeInput();
  input.gps_path.pts[0].yaw = 2.2f;
  expect(
    !generatedInputWithinLaneWaypointScope(input, params, reason),
    "wrong-way input must be rejected when the generated latch is unavailable");
}

}  // namespace

int main()
{
  testSelectorAndCoreDefault();
  testGeneratedDispatchAndActiveState();
  testScopeFaultIsLatchedAndNonempty();
  testPureFailClosedValidation();
  std::printf("decision backend tests: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
