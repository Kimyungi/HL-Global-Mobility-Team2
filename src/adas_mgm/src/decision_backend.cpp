#include "src/decision_backend.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "core/mgm_step.hpp"

#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
#include "src/generated_adapter.hpp"
#endif

namespace adas_mgm
{

namespace
{

constexpr float kPi = 3.14159265358979323846f;

bool finitePoint(const CorePoint & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.yaw) && std::isfinite(point.curvature);
}

bool nonzeroOrInvalid(float value)
{
  return !std::isfinite(value) || std::fabs(value) > 1.0e-6f;
}

}  // namespace

bool validateGeneratedOutput(
  const CoreOutput & output, const CoreSnapshot & input,
  const CoreParams & params, std::string & reason)
{
  if (output.state > MGM_STATE_WAYPOINT) {
    reason = "generated state is outside LANE/WAYPOINT";
    return false;
  }
  if (output.path_source > MGM_SRC_GPS || output.path_source != output.state) {
    reason = "generated path source does not match the two-state output";
    return false;
  }
  if (output.n_points < 1 || output.n_points > MGM_NUM_POINTS) {
    reason = "generated output path must contain 1..20 points";
    return false;
  }
  if (!std::isfinite(output.v_ref) || output.v_ref < 0.0f) {
    reason = "generated v_ref is non-finite or negative";
    return false;
  }
  const float maximum_v_ref = std::max(params.v_base, params.v_accel_zone);
  if (output.v_ref > maximum_v_ref + 1.0e-4f) {
    reason = "generated v_ref exceeds the configured maximum";
    return false;
  }
  for (int32_t i = 0; i < output.n_points; ++i) {
    if (!finitePoint(output.ref_points[i])) {
      reason = "generated output contains a non-finite reference point";
      return false;
    }
  }
  if (output.immediate_stop && output.v_ref != 0.0f) {
    reason = "generated immediate-stop output has nonzero v_ref";
    return false;
  }
  if (input.estop && (!output.immediate_stop || output.v_ref != 0.0f)) {
    reason = "generated output did not honor E-stop";
    return false;
  }
  return true;
}

bool generatedInputWithinLaneWaypointScope(
  const CoreSnapshot & input, const CoreParams & params,
  std::string & reason)
{
  if (input.avoid_obstacle_detected || input.avoid_avoidable ||
    input.avoid_narrow_gap || input.avoid_maneuver_done ||
    input.avoid_path.n != 0 || nonzeroOrInvalid(input.avoid_v_suggest) ||
    !std::isfinite(input.avoid_ttc) || !std::isfinite(params.ttc_stop) ||
    input.avoid_ttc <= params.ttc_stop)
  {
    reason = "AVOID input is unsupported by ADAS_MGR2 v1.68";
    return false;
  }
  if (input.gps_parking_zone || input.parking_space_found ||
    input.parking_path_blocked || input.parking_done ||
    input.parking_path.n != 0 || nonzeroOrInvalid(input.parking_v_suggest))
  {
    reason = "PARKING input is unsupported by ADAS_MGR2 v1.68";
    return false;
  }
  if (input.gps_at_end) {
    reason = "the production gps_at_end latch is unsupported by ADAS_MGR2 v1.68";
    return false;
  }
  if (input.gps_heading_valid && input.gps_path.n > 0 &&
    std::isfinite(input.gps_path.pts[0].yaw) &&
    std::fabs(input.gps_path.pts[0].yaw) > params.wrongway_yaw)
  {
    reason = "the production wrong-way latch is unsupported by ADAS_MGR2 v1.68";
    return false;
  }
  return true;
}

DecisionBackend::DecisionBackend(
  const std::string & requested, bool generated_scope_acknowledged,
  const CoreParams & params)
: params_(params)
{
  if (requested == "core") {
    mgm_init(core_state_, params_);
    return;
  }
  if (requested != "generated") {
    throw std::invalid_argument(
            "backend must be exactly 'core' or 'generated'; got '" + requested + "'");
  }
  if (!generated_scope_acknowledged) {
    throw std::invalid_argument(
            "backend=generated requires generated_backend_acknowledge_limited_scope=true");
  }
  if (!std::isfinite(params_.ttc_stop) || params_.ttc_stop < 0.0f) {
    throw std::invalid_argument("backend=generated requires a finite non-negative ttc_stop");
  }
  if (!std::isfinite(params_.wrongway_yaw) || params_.wrongway_yaw < 0.0f ||
    params_.wrongway_yaw > kPi)
  {
    throw std::invalid_argument(
            "backend=generated requires wrongway_yaw in [0, pi]");
  }
#ifndef ADAS_MGM_HAS_GENERATED_BACKEND
  throw std::runtime_error(
          "backend=generated is unavailable: rebuild adas_mgm with "
          "-DADAS_MGM_ENABLE_GENERATED_BACKEND=ON");
#else
  kind_ = Kind::kGenerated;
  name_ = "generated";
  generated_ = std::make_unique<GeneratedMgmAdapter>(params_);
#endif
}

DecisionBackend::~DecisionBackend() = default;

CoreOutput DecisionBackend::step(const CoreSnapshot & input)
{
  if (kind_ == Kind::kCore) {
    return mgm_step(input, core_state_);
  }
  return stepGenerated(input);
}

CoreOutput DecisionBackend::stepGenerated(const CoreSnapshot & input)
{
  if (faulted_) {
    return failStopOutput();
  }

  std::string reason;
  if (!generatedInputWithinLaneWaypointScope(input, params_, reason)) {
    latchFault(reason);
    return failStopOutput();
  }

#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  try {
    CoreOutput output = generated_->step(input);
    // Before the first perception message, the node watchdog conditions the
    // snapshot to E-stop and the generated model legitimately has no path to
    // copy. Publish a nonempty zero-speed fallback for that transient startup
    // state, but do not permanently latch a model fault: a fresh path must be
    // allowed to recover the bench run.
    if (input.estop && output.n_points == 0) {
      return failStopOutput();
    }
    if (!validateGeneratedOutput(output, input, params_, reason)) {
      latchFault(reason);
      return failStopOutput();
    }
    active_state_ = output.state;
    last_valid_output_ = output;
    has_last_valid_output_ = true;
    return output;
  } catch (const std::exception & error) {
    latchFault(std::string("generated backend exception: ") + error.what());
    return failStopOutput();
  } catch (...) {
    latchFault("generated backend raised an unknown exception");
    return failStopOutput();
  }
#else
  latchFault("generated backend is not compiled");
  return failStopOutput();
#endif
}

bool DecisionBackend::stopZoneHolding() const
{
  return kind_ == Kind::kCore ? core_state_.stop_zone_holding : false;
}

int32_t DecisionBackend::stopHoldLeft() const
{
  return kind_ == Kind::kCore ? core_state_.stop_hold_left : 0;
}

int32_t DecisionBackend::laneLowCnt() const
{
#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  if (kind_ == Kind::kGenerated) {
    return generated_->laneLowCnt();
  }
#endif
  return core_state_.lane_low_cnt;
}

int32_t DecisionBackend::laneHighCnt() const
{
#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  if (kind_ == Kind::kGenerated) {
    return generated_->laneHighCnt();
  }
#endif
  return core_state_.lane_high_cnt;
}

uint8_t DecisionBackend::activeState() const
{
  return kind_ == Kind::kCore ? core_state_.state : active_state_;
}

const std::string & DecisionBackend::name() const
{
  return name_;
}

bool DecisionBackend::faulted() const
{
  return faulted_;
}

const std::string & DecisionBackend::faultReason() const
{
  return fault_reason_;
}

void DecisionBackend::latchFault(const std::string & reason)
{
  if (!faulted_) {
    faulted_ = true;
    fault_reason_ = reason;
  }
}

CoreOutput DecisionBackend::failStopOutput() const
{
  CoreOutput output{};
  if (has_last_valid_output_) {
    output = last_valid_output_;
  } else {
    output.state = active_state_;
    output.path_source = active_state_ == MGM_STATE_WAYPOINT ? MGM_SRC_GPS : MGM_SRC_LANE;
    output.n_points = 1;
    output.ref_points[0] = CorePoint{MGM_MIN_REF_X, 0.0f, 0.0f, 0.0f};
  }
  output.v_ref = 0.0f;
  output.immediate_stop = true;
  return output;
}

}  // namespace adas_mgm
