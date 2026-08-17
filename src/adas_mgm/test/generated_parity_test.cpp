#include <cmath>
#include <cstdio>

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
using adas_mgm::mgm_init;
using adas_mgm::mgm_step;

namespace
{

void makePath(CorePath & path, float x, float y, int32_t count = MGM_NUM_POINTS)
{
  path.n = count;
  for (int32_t i = 0; i < count; ++i) {
    const float index = static_cast<float>(i);
    path.pts[i] = CorePoint{
      x + 0.05f * index,
      y + 0.002f * index,
      0.01f + 0.001f * index,
      0.005f + 0.0005f * index};
  }
}

CoreParams makeParams()
{
  CoreParams params{};
  // Deliberately differ from every compiled ERT default so a missing adapter
  // assignment cannot pass by coincidence.
  params.lane_conf_exit = 0.31f;
  params.lane_conf_return = 0.74f;
  params.n_cycles = 17;
  params.v_base = 0.53f;
  params.v_accel_zone = 0.91f;
  params.v_narrow = 0.17f;
  params.ttc_stop = 0.67f;
  params.blend_cycles = 7;
  params.a_up = 0.43f;
  params.a_down = 1.21f;
  params.wrongway_yaw = 1.9f;
  params.wrongway_cycles = 13;
  params.avoid_return_hold_cycles = 61;
  params.lane_entry_max_cross = 0.42f;
  params.avoid_max_cycles = 37;
  return params;
}

CoreSnapshot makeSnapshot(int tick)
{
  CoreSnapshot input{};
  input.lane_confidence = 0.9f;
  input.gps_cross_track = 0.1f;
  input.gps_heading_valid = true;
  input.lane_updated = true;
  input.gps_updated = true;
  input.avoid_updated = true;
  input.avoid_ttc = 1e9f;
  input.avoid_v_suggest = 0.4f;
  input.parking_v_suggest = -0.25f;
  makePath(input.lane_path, 1.0f, 0.0f);
  makePath(input.gps_path, 1.2f, 0.05f);
  makePath(input.avoid_path, 1.5f, 0.35f, 1);
  makePath(input.parking_path, 0.8f, -0.2f);

  if (tick >= 20 && tick < 70) {
    input.gps_accel_zone = true;
  }
  if (tick >= 100 && tick < 170) {
    input.traffic_stop_required = true;
  }
  if (tick >= 200 && tick < 230) {
    // Between the custom and generated-default lane-exit thresholds.
    input.lane_confidence = 0.33f;
  }
  if (tick >= 250 && tick < 400) {
    input.lane_confidence = 0.1f;
  }
  if (tick >= 300 && tick < 330) {
    // Between the generated-default and custom lane-return thresholds.
    input.lane_confidence = 0.72f;
  }
  if (tick >= 410 && tick < 416) {
    input.avoid_obstacle_detected = true;
  }
  if (tick >= 416 && tick < 420) {
    input.avoid_avoidable = true;
  }
  if (tick >= 420 && tick < 500) {
    input.avoid_obstacle_detected = true;
    input.avoid_avoidable = true;
  }
  if (tick >= 440 && tick < 460) {
    input.avoid_narrow_gap = true;
  }
  if (tick >= 470 && tick < 475) {
    // Between the custom and generated-default TTC stop thresholds.
    input.avoid_ttc = 0.73f;
  }
  if (tick >= 475 && tick < 480) {
    input.avoid_ttc = 0.3f;
  }
  if (tick == 500) {
    input.avoid_maneuver_done = true;
  }
  if (tick >= 500 && tick < 580) {
    input.gps_cross_track = 0.9f;
  }
  if (tick >= 580 && tick < 620) {
    // Between the custom and generated-default lane-entry cross-track gates.
    input.gps_cross_track = 0.46f;
  }
  if (tick >= 900 && tick < 980) {
    input.lane_confidence = 0.1f;
  }
  if (tick >= 900 && tick < 960) {
    // Between the custom and generated-default wrong-way thresholds.
    input.gps_path.pts[0].yaw = 2.0f;
  }
  if (tick >= 930 && tick < 950) {
    input.gps_heading_valid = false;
    input.gps_path.pts[0].yaw = 2.0f;
  }
  if (tick >= 1200 && tick < 1220) {
    input.gps_at_end = true;
  }
  if (tick >= 1250 && tick < 1260) {
    // Watchdog-derived E-stop must not release the at-end latch.
    input.estop = true;
  }
  if (tick >= 1300 && tick < 1310) {
    input.estop = true;
    input.estop_latch_release = true;
  }
  if (tick >= 1380 && tick < 1390) {
    input.gps_parking_zone = true;
  }
  if (tick >= 1390 && tick < 1400) {
    input.parking_space_found = true;
  }
  if (tick >= 1400 && tick < 1450) {
    input.gps_parking_zone = true;
    input.parking_space_found = true;
  }
  if (tick >= 1420 && tick < 1430) {
    input.parking_path_blocked = true;
  }
  if (tick == 1450) {
    input.parking_done = true;
  }
  if (tick >= 1500 && tick < 1550) {
    input.estop = true;
  }
  input.lane_updated = tick % 2 == 0;
  input.gps_updated = tick % 3 == 0;
  input.avoid_updated = tick % 5 == 0;
  return input;
}

bool close(float left, float right)
{
  return std::fabs(left - right) <= 3e-5f;
}

bool equal(const CoreOutput & reference, const CoreOutput & generated)
{
  if (reference.state != generated.state ||
    reference.path_source != generated.path_source ||
    reference.immediate_stop != generated.immediate_stop ||
    !close(reference.v_ref, generated.v_ref) ||
    reference.n_points != generated.n_points)
  {
    return false;
  }
  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    if (!close(reference.ref_points[i].x, generated.ref_points[i].x) ||
      !close(reference.ref_points[i].y, generated.ref_points[i].y) ||
      !close(reference.ref_points[i].yaw, generated.ref_points[i].yaw) ||
      !close(reference.ref_points[i].curvature, generated.ref_points[i].curvature))
    {
      return false;
    }
  }
  return true;
}

}  // namespace

int main()
{
  const CoreParams params = makeParams();
  CoreState reference_state{};
  mgm_init(reference_state, params);
  GeneratedMgmAdapter generated(params);

  int mismatches = 0;
  bool seen_state[4]{};
  bool seen_source[4]{};
  bool seen_immediate_stop = false;
  bool seen_forward = false;
  bool seen_reverse = false;
  constexpr int kTicks = 2400;
  for (int tick = 0; tick < kTicks; ++tick) {
    const CoreSnapshot input = makeSnapshot(tick);
    const CoreOutput reference = mgm_step(input, reference_state);
    const CoreOutput actual = generated.step(input);
    if (reference.state < 4) {
      seen_state[reference.state] = true;
    }
    if (reference.path_source < 4) {
      seen_source[reference.path_source] = true;
    }
    seen_immediate_stop = seen_immediate_stop || reference.immediate_stop;
    seen_forward = seen_forward || reference.v_ref > 0.0f;
    seen_reverse = seen_reverse || reference.v_ref < 0.0f;
    if (!equal(reference, actual)) {
      if (mismatches < 10) {
        std::fprintf(
          stderr,
          "tick=%d C++(state=%u src=%u v=%.6f n=%d) ERT(state=%u src=%u v=%.6f n=%d)\n",
          tick, reference.state, reference.path_source, reference.v_ref, reference.n_points,
          actual.state, actual.path_source, actual.v_ref, actual.n_points);
      }
      ++mismatches;
    }
  }

  // Initialization must also be repeatable in one process.
  generated.reset(params);
  mgm_init(reference_state, params);
  const CoreSnapshot first = makeSnapshot(0);
  if (!equal(mgm_step(first, reference_state), generated.step(first))) {
    ++mismatches;
    std::fputs("reset parity mismatch\n", stderr);
  }

  for (int i = 0; i < 4; ++i) {
    if (!seen_state[i] || !seen_source[i]) {
      ++mismatches;
      std::fprintf(
        stderr, "coverage missing: state[%d]=%d source[%d]=%d\n",
        i, seen_state[i], i, seen_source[i]);
    }
  }
  if (!seen_immediate_stop || !seen_forward || !seen_reverse) {
    ++mismatches;
    std::fprintf(
      stderr, "coverage missing: immediate=%d forward=%d reverse=%d\n",
      seen_immediate_stop, seen_forward, seen_reverse);
  }

  std::printf("generated parity: ticks=%d mismatches=%d\n", kTicks, mismatches);
  return mismatches == 0 ? 0 : 1;
}
