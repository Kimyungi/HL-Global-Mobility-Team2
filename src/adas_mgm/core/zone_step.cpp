#include "zone_step.hpp"
#include <algorithm>
namespace adas_mgm
{
void zone_step(const ZoneSnapshot & input, bool gps_usable, ZoneState & state,
  int32_t enter_samples, int32_t exit_samples)
{
  bool valid = gps_usable && input.zone_valid && input.count >= 0 &&
    input.count < MGM_ZONE_CAPACITY;
  const ZoneObservation * by_id[MGM_ZONE_CAPACITY]{};
  if (valid) {
    for (int i = 0; i < input.count; ++i) {
      const auto & zone = input.observations[i];
      if (zone.zone_id == 0 || by_id[zone.zone_id] != nullptr ||
        zone.zone_type > ZoneType::MISSION_ZONE)
      {
        valid = false;
        break;
      }
      by_id[zone.zone_id] = &zone;
    }
  }
  const bool snapshot_valid = valid;
  state.calibration = enter_samples < 0 || exit_samples < 0 ? CalibrationState::INVALID_CONFIG :
    enter_samples == 0 || exit_samples == 0 ? CalibrationState::UNCALIBRATED : CalibrationState::CALIBRATED;
  if (valid && input.count > 0) {state.definitions_seen = true;}
  const bool configured = state.calibration == CalibrationState::CALIBRATED;
  valid = valid && (configured || !state.definitions_seen) && input.generation != 0 &&
    input.generation >= state.last_generation;
  const bool new_fix = valid && input.generation > state.last_generation;
  if (new_fix) {state.last_generation = input.generation;}
  state.in_gps_only_zone = false;
  ZoneContext selected{};
  selected.zone_valid = valid;
  selected.in_zone = true;  // implicit NORMAL_ZONE if no special membership
  for (int id = 1; id < MGM_ZONE_CAPACITY; ++id) {
    auto & context = state.contexts[id];
    context.zone_valid = valid;
    context.zone_entered = false;
    context.zone_exited = false;
    const auto * raw = snapshot_valid ? by_id[id] : nullptr;
    if (raw) {
      context.raw_in_zone = raw->in_zone;
      context.zone_id = raw->zone_id; context.zone_type = raw->zone_type;
      context.mission_id = raw->mission_id; context.mission_type = raw->mission_type;
      context.boundary_distance_m = raw->boundary_distance_m;
    }
    if (!valid) {context.enter_count = context.exit_count = 0;}
    if (new_fix) {
      const bool was_in = context.in_zone;
      const auto * current = by_id[id];
      if (current) {
        context.zone_id = current->zone_id;
        context.zone_type = current->zone_type;
        context.mission_type = current->mission_type;
        context.mission_id = current->mission_id;
      }
      context.raw_in_zone = current && current->in_zone;
      context.boundary_distance_m = current ? current->boundary_distance_m : -1.0f;
      if (context.raw_in_zone) {
        context.exit_count = 0;
        if (!was_in && context.enter_count < enter_samples) {++context.enter_count;}
        if (!was_in && configured && context.enter_count >= enter_samples) {context.in_zone = true;}
      } else {
        context.enter_count = 0;
        if ((was_in || context.mission_entry_suppressed) && context.exit_count < exit_samples) {++context.exit_count;}
        if (configured && context.exit_count >= exit_samples) {
          context.in_zone = false; context.mission_entry_suppressed = false;
        }
      }
      context.stable_in_zone = context.in_zone;
      context.zone_entered = !was_in && context.in_zone;
      context.zone_exited = was_in && !context.in_zone;
    }
    if (!context.in_zone) {continue;}
    state.in_gps_only_zone = state.in_gps_only_zone || context.zone_type == ZoneType::GPS_ONLY_ZONE;
    // Ascending ID makes equal-priority overlaps deterministic. All contexts
    // remain available to Mission/Safety regardless of this display selection.
    if (selected.zone_id == 0 || context.zone_type > selected.zone_type) {selected = context;}
  }
  auto & normal = state.contexts[0];
  const bool was_normal = normal.in_zone;
  normal.zone_valid = valid;
  normal.zone_entered = false;
  normal.zone_exited = false;
  if (valid) {
    normal.in_zone = selected.zone_id == 0;
    normal.stable_in_zone = normal.in_zone;
    normal.raw_in_zone = true;
    for (int i=0; i<input.count; ++i) {normal.raw_in_zone &= !input.observations[i].in_zone;}
    normal.zone_entered = !was_normal && normal.in_zone;
    normal.zone_exited = was_normal && !normal.in_zone;
  }
  state.selected = selected.zone_id == 0 ? normal : selected;
}
}  // namespace adas_mgm
