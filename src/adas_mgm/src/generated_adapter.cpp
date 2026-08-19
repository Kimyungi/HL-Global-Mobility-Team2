#include "src/generated_adapter.hpp"

#include <atomic>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string>

extern "C"
{
#include "ADAS_MGR2.h"
}

namespace adas_mgm
{

namespace
{

std::atomic<bool> generated_adapter_owned{false};

void validateParams(const CoreParams & params)
{
  auto require_finite = [](float value, const char * name) {
      if (!std::isfinite(value)) {
        throw std::invalid_argument(
                std::string("GeneratedMgmAdapter: ") + name +
                " must be finite");
      }
    };

  require_finite(params.lane_conf_exit, "lane_conf_exit");
  require_finite(params.lane_conf_return, "lane_conf_return");
  require_finite(params.v_base, "v_base");
  require_finite(params.v_accel_zone, "v_accel_zone");
  require_finite(params.a_up, "a_up");
  require_finite(params.a_down, "a_down");
  require_finite(params.lane_entry_max_cross, "lane_entry_max_cross");

  const bool gps_only_sentinel =
    params.lane_conf_exit == 2.0f && params.lane_conf_return == 2.0f;
  if (!gps_only_sentinel) {
    if (params.lane_conf_exit < 0.0f || params.lane_conf_exit > 1.0f) {
      throw std::invalid_argument(
              "GeneratedMgmAdapter: lane_conf_exit must be in [0, 1] "
              "or both confidence thresholds must be the GPS-only sentinel 2.0");
    }
    if (params.lane_conf_return < 0.0f || params.lane_conf_return > 1.0f) {
      throw std::invalid_argument(
              "GeneratedMgmAdapter: lane_conf_return must be in [0, 1] "
              "or both confidence thresholds must be the GPS-only sentinel 2.0");
    }
    if (params.lane_conf_exit >= params.lane_conf_return) {
      throw std::invalid_argument(
              "GeneratedMgmAdapter: lane_conf_exit must be less than lane_conf_return");
    }
  }
  if (params.n_cycles <= 0) {
    throw std::invalid_argument("GeneratedMgmAdapter: n_cycles must be greater than zero");
  }
  if (params.v_base < 0.0f) {
    throw std::invalid_argument("GeneratedMgmAdapter: v_base must be non-negative");
  }
  if (params.v_accel_zone < params.v_base) {
    throw std::invalid_argument(
            "GeneratedMgmAdapter: v_accel_zone must be at least v_base");
  }
  if (params.blend_cycles < 0) {
    throw std::invalid_argument("GeneratedMgmAdapter: blend_cycles must be non-negative");
  }
  if (params.a_up <= 0.0f) {
    throw std::invalid_argument("GeneratedMgmAdapter: a_up must be greater than zero");
  }
  if (params.a_down <= 0.0f) {
    throw std::invalid_argument("GeneratedMgmAdapter: a_down must be greater than zero");
  }
  if (params.lane_entry_max_cross < 0.0f) {
    throw std::invalid_argument(
            "GeneratedMgmAdapter: lane_entry_max_cross must be non-negative");
  }
}

void throwIfModelError(const char * operation)
{
  const char * const error = rtmGetErrorStatus(ADAS_MGR2_M);
  if (error != nullptr) {
    throw std::runtime_error(
            std::string("GeneratedMgmAdapter: ADAS_MGR2 ") + operation +
            " failed: " + error);
  }
}

void copyPoint(const CorePoint & source, CorePointBus & target)
{
  target.x = source.x;
  target.y = source.y;
  target.yaw = source.yaw;
  target.curvature = source.curvature;
}

void copyPath(const CorePath & source, CorePathBus & target)
{
  target.n = source.n;
  const int32_t count =
    source.n < 0 ? 0 : (source.n < MGM_NUM_POINTS ? source.n : MGM_NUM_POINTS);
  for (int32_t i = 0; i < count; ++i) {
    copyPoint(source.pts[i], target.pts[i]);
  }
}

void copySnapshot(const CoreSnapshot & source, CoreSnapshotBus & target)
{
  std::memset(&target, 0, sizeof(target));
  target.lane_confidence = source.lane_confidence;
  copyPath(source.lane_path, target.lane_path);
  copyPath(source.gps_path, target.gps_path);
  target.gps_accel_zone = source.gps_accel_zone;
  target.gps_parking_zone = source.gps_parking_zone;
  target.gps_at_end = source.gps_at_end;
  target.gps_cross_track = source.gps_cross_track;
  target.gps_heading_valid = source.gps_heading_valid;
  target.lane_updated = source.lane_updated;
  target.gps_updated = source.gps_updated;
  target.avoid_updated = source.avoid_updated;
  target.avoid_obstacle_detected = source.avoid_obstacle_detected;
  target.avoid_avoidable = source.avoid_avoidable;
  target.avoid_ttc = source.avoid_ttc;
  target.avoid_narrow_gap = source.avoid_narrow_gap;
  target.avoid_maneuver_done = source.avoid_maneuver_done;
  copyPath(source.avoid_path, target.avoid_path);
  target.avoid_v_suggest = source.avoid_v_suggest;
  target.parking_space_found = source.parking_space_found;
  target.parking_path_blocked = source.parking_path_blocked;
  target.parking_done = source.parking_done;
  copyPath(source.parking_path, target.parking_path);
  target.parking_v_suggest = source.parking_v_suggest;
  target.traffic_stop_required = source.traffic_stop_required;
  target.estop = source.estop;
  target.estop_latch_release = source.estop_latch_release;
}

}  // namespace

GeneratedMgmAdapter::GeneratedMgmAdapter(const CoreParams & params)
{
  validateParams(params);

  bool expected = false;
  if (!generated_adapter_owned.compare_exchange_strong(
      expected, true, std::memory_order_acquire, std::memory_order_relaxed))
  {
    throw std::logic_error(
            "GeneratedMgmAdapter: only one process-wide instance may exist at a time");
  }
  lease_acquired_ = true;

  try {
    // Construction deliberately does not bind the calling thread. A ROS node
    // may construct the adapter on its executor thread and perform every step
    // later on its dedicated 10 ms loop thread.
    initialize(params);
  } catch (...) {
    ADAS_MGR2_terminate();
    releaseLease();
    throw;
  }
}

GeneratedMgmAdapter::~GeneratedMgmAdapter() noexcept
{
  if (lease_acquired_) {
    ADAS_MGR2_terminate();
    releaseLease();
  }
}

void GeneratedMgmAdapter::initialize(const CoreParams & params)
{
  validateParams(params);

  // ERT uses static/global storage and initialize() assumes zero-initialized
  // process memory. Clear it explicitly so a replay can restart in-process.
  std::memset(&ADAS_MGR2_B, 0, sizeof(ADAS_MGR2_B));
  std::memset(&ADAS_MGR2_DW, 0, sizeof(ADAS_MGR2_DW));
  std::memset(&ADAS_MGR2_U, 0, sizeof(ADAS_MGR2_U));
  std::memset(&ADAS_MGR2_Y, 0, sizeof(ADAS_MGR2_Y));
  rtmSetErrorStatus(ADAS_MGR2_M, nullptr);

  MGM_lane_conf_exit = params.lane_conf_exit;
  MGM_lane_conf_return = params.lane_conf_return;
  MGM_n_cycles = params.n_cycles;
  MGM_v_base = params.v_base;
  MGM_v_accel_zone = params.v_accel_zone;
  MGM_blend_cycles = params.blend_cycles;
  MGM_a_up = params.a_up;
  MGM_a_down = params.a_down;
  MGM_lane_entry_max_cross = params.lane_entry_max_cross;

  ADAS_MGR2_initialize();
  throwIfModelError("initialize");
}

void GeneratedMgmAdapter::requireOwningThread(const char * operation)
{
  const std::thread::id caller = std::this_thread::get_id();
  std::lock_guard<std::mutex> lock(owner_mutex_);
  if (!owner_thread_bound_) {
    owner_thread_ = caller;
    owner_thread_bound_ = true;
    return;
  }
  if (owner_thread_ != caller) {
    throw std::logic_error(
            std::string("GeneratedMgmAdapter::") + operation +
            " called from a non-owning thread");
  }
}

void GeneratedMgmAdapter::releaseLease() noexcept
{
  lease_acquired_ = false;
  generated_adapter_owned.store(false, std::memory_order_release);
}

void GeneratedMgmAdapter::reset(const CoreParams & params)
{
  requireOwningThread("reset");
  initialize(params);
}

int32_t GeneratedMgmAdapter::laneLowCnt() const
{
  return ADAS_MGR2_DW.lane_low_cnt;
}

int32_t GeneratedMgmAdapter::laneHighCnt() const
{
  return ADAS_MGR2_DW.lane_high_cnt;
}

CoreOutput GeneratedMgmAdapter::step(const CoreSnapshot & input)
{
  requireOwningThread("step");
  throwIfModelError("before step");
  copySnapshot(input, ADAS_MGR2_U.core_snapshot);
  ::mgm_step();
  throwIfModelError("step");

  const CoreOutputBus & source = ADAS_MGR2_Y.core_output;
  CoreOutput output{};
  output.state = source.state;
  output.path_source = source.path_source;
  output.immediate_stop = source.immediate_stop;
  output.v_ref = source.v_ref;
  output.n_points = source.n_points;
  for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
    output.ref_points[i] = CorePoint{
      source.ref_points[i].x,
      source.ref_points[i].y,
      source.ref_points[i].yaw,
      source.ref_points[i].curvature};
  }
  return output;
}

}  // namespace adas_mgm
