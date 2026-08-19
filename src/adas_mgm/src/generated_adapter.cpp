#include "src/generated_adapter.hpp"

#include <cstring>

extern "C"
{
#include "ADAS_MGR2.h"
}

namespace adas_mgm
{

namespace
{

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
  reset(params);
}

void GeneratedMgmAdapter::reset(const CoreParams & params)
{
  // ERT uses static/global storage and initialize() assumes zero-initialized
  // process memory. Clear it explicitly so a replay can restart in-process.
  std::memset(&ADAS_MGR2_B, 0, sizeof(ADAS_MGR2_B));
  std::memset(&ADAS_MGR2_DW, 0, sizeof(ADAS_MGR2_DW));
  std::memset(&ADAS_MGR2_U, 0, sizeof(ADAS_MGR2_U));
  std::memset(&ADAS_MGR2_Y, 0, sizeof(ADAS_MGR2_Y));

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
}

CoreOutput GeneratedMgmAdapter::step(const CoreSnapshot & input)
{
  copySnapshot(input, ADAS_MGR2_U.core_snapshot);
  ::mgm_step();

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
