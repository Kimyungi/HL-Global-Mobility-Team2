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

bool validInputPath(const CorePath & path)
{
  if (path.n < 0 || path.n > MGM_NUM_POINTS) {
    return false;
  }
  for (int32_t index = 0; index < path.n; ++index) {
    if (!finitePoint(path.pts[index])) {
      return false;
    }
  }
  return true;
}

uint8_t sourceForState(uint8_t state)
{
  switch (state) {
    case MGM_STATE_WAYPOINT: return MGM_SRC_GPS;
    case MGM_STATE_AVOID: return MGM_SRC_AVOID;
    case MGM_STATE_PARKING: return MGM_SRC_PARKING;
    default: return MGM_SRC_LANE;
  }
}

}  // namespace

bool validateGeneratedOutput(
  const CoreOutput & output, const CoreSnapshot & input,
  const CoreParams & params, std::string & reason,
  bool allow_parking_reverse_ramp, float previous_v_ref)
{
  if (output.state > MGM_STATE_PARKING) {
    reason = "generated state is outside LANE/WAYPOINT/AVOID/PARKING";
    return false;
  }
  if (output.path_source != sourceForState(output.state)) {
    reason = "generated path source does not match the four-state output";
    return false;
  }
  if (output.n_points < 1 || output.n_points > MGM_NUM_POINTS) {
    reason = "generated output path must contain 1..20 points";
    return false;
  }
  if (!std::isfinite(output.v_ref)) {
    reason = "generated v_ref is non-finite";
    return false;
  }
  if (output.v_ref < 0.0f && output.state != MGM_STATE_PARKING) {
    if (!allow_parking_reverse_ramp || !std::isfinite(previous_v_ref) ||
      previous_v_ref >= 0.0f || output.v_ref + 1.0e-4f < previous_v_ref)
    {
      reason = "generated negative v_ref is not a monotonic PARKING exit ramp";
      return false;
    }
  }
  float maximum_abs_v_ref = std::max(std::fabs(params.v_base), std::fabs(params.v_accel_zone));
  maximum_abs_v_ref = std::max(maximum_abs_v_ref, std::fabs(params.v_narrow));
  maximum_abs_v_ref = std::max(maximum_abs_v_ref, std::fabs(params.v_avoid));
  if (std::isfinite(input.avoid_v_suggest)) {
    maximum_abs_v_ref = std::max(maximum_abs_v_ref, std::fabs(input.avoid_v_suggest));
  }
  if (std::isfinite(input.parking_v_suggest)) {
    maximum_abs_v_ref = std::max(maximum_abs_v_ref, std::fabs(input.parking_v_suggest));
  }
  if (output.v_ref < 0.0f && allow_parking_reverse_ramp &&
    std::isfinite(previous_v_ref))
  {
    maximum_abs_v_ref = std::max(maximum_abs_v_ref, std::fabs(previous_v_ref));
  }
  if (std::fabs(output.v_ref) > maximum_abs_v_ref + 1.0e-4f) {
    reason = "generated |v_ref| exceeds all configured and requested speeds";
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
  if (output.state == MGM_STATE_AVOID && input.avoid_ttc < params.ttc_stop &&
    (!output.immediate_stop || output.v_ref != 0.0f))
  {
    reason = "generated AVOID output did not honor the TTC stop threshold";
    return false;
  }
  return true;
}

bool generatedInputWithinLaneWaypointScope(
  const CoreSnapshot & input, const CoreParams & params,
  std::string & reason)
{
  if (params.escape_after_cycles != 0) {
    reason = "rear escape is unsupported by ADAS_MGR2 v1.88";
    return false;
  }
  if (!std::isfinite(input.lane_confidence) ||
    !std::isfinite(input.gps_cross_track) ||
    !std::isfinite(input.avoid_ttc) ||
    !std::isfinite(input.avoid_v_suggest) ||
    !std::isfinite(input.parking_v_suggest))
  {
    reason = "generated backend input contains a non-finite decision value";
    return false;
  }
  if (input.avoid_v_suggest < 0.0f) {
    reason = "generated backend requires a non-negative AVOID speed suggestion";
    return false;
  }
  if (!validInputPath(input.lane_path) || !validInputPath(input.gps_path) ||
    !validInputPath(input.avoid_path) || !validInputPath(input.parking_path))
  {
    reason = "generated backend input contains an invalid path";
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
  if (params_.escape_after_cycles != 0) {
    throw std::invalid_argument(
            "backend=generated requires escape_after_cycles=0 because "
            "ADAS_MGR2 v1.88 has no rear-escape input or state");
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
    const bool allow_parking_reverse =
      output.state == MGM_STATE_PARKING || active_state_ == MGM_STATE_PARKING ||
      parking_reverse_ramp_active_;
    const float previous_v_ref =
      has_last_valid_output_ ? last_valid_output_.v_ref : 0.0f;
    if (!validateGeneratedOutput(
        output, input, params_, reason, allow_parking_reverse, previous_v_ref))
    {
      latchFault(reason);
      return failStopOutput();
    }
    parking_reverse_ramp_active_ = allow_parking_reverse && output.v_ref < 0.0f;
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
#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  if (kind_ == Kind::kGenerated) {
    return generated_->stopZoneHolding();
  }
#endif
  return core_state_.stop_zone_holding;
}

int32_t DecisionBackend::stopHoldLeft() const
{
#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  if (kind_ == Kind::kGenerated) {
    return generated_->stopHoldLeft();
  }
#endif
  return core_state_.stop_hold_left;
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

int32_t DecisionBackend::avoidTicks() const
{
#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  if (kind_ == Kind::kGenerated) {
    return generated_->avoidTicks();
  }
#endif
  return core_state_.avoid_ticks;
}

int32_t DecisionBackend::returnHoldLeft() const
{
#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  if (kind_ == Kind::kGenerated) {
    return generated_->returnHoldLeft();
  }
#endif
  return core_state_.return_hold_left;
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
    output.path_source = sourceForState(active_state_);
    output.n_points = 1;
    output.ref_points[0] = CorePoint{MGM_MIN_REF_X, 0.0f, 0.0f, 0.0f};
  }
  output.v_ref = 0.0f;
  output.immediate_stop = true;
  return output;
}

}  // namespace adas_mgm
