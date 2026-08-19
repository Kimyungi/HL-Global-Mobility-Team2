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
  // Every parameter exported by the two-state ERT differs from its compiled
  // default so an omitted adapter assignment cannot pass by coincidence.
  params.lane_conf_exit = 0.31f;
  params.lane_conf_return = 0.74f;
  params.n_cycles = 17;
  params.v_base = 0.53f;
  params.v_accel_zone = 0.91f;
  params.blend_cycles = 7;
  params.a_up = 0.43f;
  params.a_down = 1.21f;
  params.lane_entry_max_cross = 0.42f;

  // These belong to the production four-state CoreParams ABI but are outside
  // this LANE/WAYPOINT experiment. Keep them neutral in the C++ reference.
  params.v_narrow = 0.2f;
  params.ttc_stop = 0.8f;
  params.wrongway_yaw = 3.2f;
  params.wrongway_cycles = 1000;
  return params;
}

CoreSnapshot makeSnapshot(int tick)
{
  CoreSnapshot input{};
  input.lane_confidence = 0.9f;
  input.gps_cross_track = 0.1f;
  input.gps_heading_valid = false;
  input.lane_updated = true;
  input.gps_updated = true;
  input.avoid_ttc = 1e9f;
  makePath(input.lane_path, 1.0f, 0.0f);
  makePath(input.gps_path, 1.2f, 0.05f);

  // Exercise the actual one-point input contract in both shared states. The
  // first tick is a fresh observation; the following ticks are a consecutive
  // stale run that must stay expanded to 20 output points with full parity.
  if (tick >= 30 && tick < 50) {
    makePath(input.lane_path, 1.0f, 0.0f, 1);
  }
  if (tick >= 530 && tick < 545) {
    makePath(input.gps_path, 1.2f, 0.05f, 1);
  }

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
  if (tick >= 360 && tick < 420) {
    // Between the custom and generated-default lane-entry cross-track gates.
    input.gps_cross_track = 0.46f;
  }
  if (tick >= 500 && tick < 760) {
    input.lane_confidence = 0.1f;
  }
  if (tick >= 530 && tick < 650) {
    // Pure WAYPOINT acceleration: no traffic stop or E-stop overlaps.
    input.gps_accel_zone = true;
  }
  if (tick >= 650 && tick < 735) {
    input.traffic_stop_required = true;
  }
  if (tick >= 740 && tick < 745) {
    // Isolated WAYPOINT E-stop, after the normal-stop interval ends.
    input.estop = true;
  }
  if (tick >= 820 && tick < 830) {
    // LANE E-stop coverage.
    input.estop = true;
  }
  input.lane_updated = tick % 2 == 0;
  input.gps_updated = tick % 3 == 0;
  if (tick >= 30 && tick < 50) {
    input.lane_updated = tick == 30;
  }
  if (tick >= 530 && tick < 545) {
    input.gps_updated = tick == 530;
  }
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
  bool seen_state[2]{};
  bool seen_source[2]{};
  bool seen_accel[2]{};
  bool seen_estop[2]{};
  bool seen_normal_stop[2]{};
  bool seen_single_point[2]{};
  bool seen_single_point_stale_parity[2]{};
  bool seen_accel_target = false;
  bool seen_forward = false;
  bool seen_lane_exit = false;
  bool seen_lane_return = false;
  bool seen_cross_track_gate_hold = false;
  int consecutive_single_point_stale = 0;
  uint8_t previous_state = 0U;
  constexpr int kTicks = 900;
  for (int tick = 0; tick < kTicks; ++tick) {
    const CoreSnapshot input = makeSnapshot(tick);
    const CoreOutput reference = mgm_step(input, reference_state);
    const CoreOutput actual = generated.step(input);
    if (reference.state < 2) {
      seen_state[reference.state] = true;
    } else {
      ++mismatches;
      std::fprintf(stderr, "out-of-scope reference state at tick=%d: %u\n", tick, reference.state);
    }
    if (reference.path_source < 2) {
      seen_source[reference.path_source] = true;
    } else {
      ++mismatches;
      std::fprintf(
        stderr, "out-of-scope reference source at tick=%d: %u\n", tick, reference.path_source);
    }
    if (reference.state < 2) {
      const auto state = reference.state;
      seen_accel[state] = seen_accel[state] ||
        (input.gps_accel_zone && !input.traffic_stop_required && !input.estop &&
        !reference.immediate_stop && reference.v_ref > 0.0f);
      seen_accel_target = seen_accel_target ||
        (input.gps_accel_zone && !input.traffic_stop_required && !input.estop &&
        close(reference.v_ref, params.v_accel_zone));
      seen_estop[state] = seen_estop[state] ||
        (input.estop && reference.immediate_stop && close(reference.v_ref, 0.0f));
      seen_normal_stop[state] = seen_normal_stop[state] ||
        (input.traffic_stop_required && !input.estop && !reference.immediate_stop &&
        close(reference.v_ref, 0.0f));
    }
    seen_forward = seen_forward || reference.v_ref > 0.0f;
    seen_lane_exit = seen_lane_exit || (previous_state == 0U && reference.state == 1U);
    seen_lane_return = seen_lane_return || (previous_state == 1U && reference.state == 0U);
    seen_cross_track_gate_hold = seen_cross_track_gate_hold ||
      (reference.state == 1U &&
      input.lane_confidence > params.lane_conf_return &&
      reference_state.lane_high_cnt >= params.n_cycles &&
      input.gps_cross_track > params.lane_entry_max_cross);
    previous_state = reference.state;

    const bool outputs_match = equal(reference, actual);
    const bool selected_single_point =
      (reference.path_source == 0U && input.lane_path.n == 1) ||
      (reference.path_source == 1U && input.gps_path.n == 1);
    const bool selected_source_updated =
      reference.path_source == 0U ? input.lane_updated : input.gps_updated;
    bool single_point_contract_ok = true;
    if (selected_single_point) {
      if (reference.n_points != MGM_NUM_POINTS || actual.n_points != MGM_NUM_POINTS) {
        single_point_contract_ok = false;
        if (mismatches < 10) {
          std::fprintf(
            stderr,
            "tick=%d single-point expansion failed: C++ n=%d ERT n=%d expected=%d\n",
            tick, reference.n_points, actual.n_points, MGM_NUM_POINTS);
        }
      } else if (reference.state < 2) {
        seen_single_point[reference.state] = true;
      }

      if (!selected_source_updated) {
        ++consecutive_single_point_stale;
        if (consecutive_single_point_stale >= 3 && outputs_match &&
          single_point_contract_ok && reference.state < 2)
        {
          // equal() compares state, velocity, n_points, and all 20 point fields.
          seen_single_point_stale_parity[reference.state] = true;
        }
      } else {
        consecutive_single_point_stale = 0;
      }
    } else {
      consecutive_single_point_stale = 0;
    }

    if (!outputs_match || !single_point_contract_ok) {
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

  for (int i = 0; i < 2; ++i) {
    if (!seen_state[i] || !seen_source[i]) {
      ++mismatches;
      std::fprintf(
        stderr, "coverage missing: state[%d]=%d source[%d]=%d\n",
        i, seen_state[i], i, seen_source[i]);
    }
    if (!seen_accel[i] || !seen_estop[i] || !seen_normal_stop[i] ||
      !seen_single_point[i] || !seen_single_point_stale_parity[i])
    {
      ++mismatches;
      std::fprintf(
        stderr,
        "state[%d] coverage missing: accel=%d estop=%d normal_stop=%d "
        "single=%d single_stale_parity=%d\n",
        i, seen_accel[i], seen_estop[i], seen_normal_stop[i],
        seen_single_point[i], seen_single_point_stale_parity[i]);
    }
  }
  if (!seen_forward || !seen_accel_target || !seen_lane_exit || !seen_lane_return ||
    !seen_cross_track_gate_hold)
  {
    ++mismatches;
    std::fprintf(
      stderr,
      "coverage missing: forward=%d accel_target=%d exit=%d return=%d cross_ready_hold=%d\n",
      seen_forward, seen_accel_target, seen_lane_exit, seen_lane_return,
      seen_cross_track_gate_hold);
  }

  std::printf("lane-waypoint generated parity: ticks=%d mismatches=%d\n", kTicks, mismatches);
  return mismatches == 0 ? 0 : 1;
}
