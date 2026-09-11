#include "reference_safety.hpp"
#include <cmath>
namespace adas_mgm
{
bool reference_geometry_valid(const CorePoint * points, int32_t count)
{
  if (count <= 0 || count > MGM_NUM_POINTS) {return false;}
  bool nonzero = false;
  for (int i = 0; i < count; ++i) {
    const auto & p = points[i];
    if (!std::isfinite(p.x) || !std::isfinite(p.y) ||
      !std::isfinite(p.yaw) || !std::isfinite(p.curvature)) {return false;}
    nonzero = nonzero || p.x != 0.0f || p.y != 0.0f;
  }
  return nonzero;  // rejects default/zero-filled paths, without a new distance threshold
}
ReferenceStatus provider_reference(const CoreSnapshot & s, uint8_t source)
{
  ReferenceStatus r{};
  r.source = source; r.age_s = -1.0f;
  if (source >= MGM_SRC_ESCAPE) {return r;}  // Escape is certified by its active phase + assembler
  const CorePath * paths[] = {&s.lane_path, &s.gps_path, &s.avoid_path, &s.parking_path};
  const bool usable[] = {
    s.camera_line_valid && std::isfinite(s.lane_confidence) &&
    s.lane_confidence >= 0.0f && s.lane_confidence <= 1.0f,
    s.gps_valid, s.lidar_valid, s.parking_valid};
  const auto & sample = s.references[source];
  r.generation = sample.generation; r.age_s = sample.age_s;
  r.available = paths[source]->n > 0;
  r.fresh = sample.generation != 0 && std::isfinite(sample.age_s) && sample.age_s >= 0 &&
    std::isfinite(sample.timeout_s) && sample.timeout_s > 0 && sample.age_s <= sample.timeout_s;
  r.valid = r.available && r.fresh && usable[source] &&
    reference_geometry_valid(paths[source]->pts, paths[source]->n);
  return r;
}
void final_reference_gate(CoreOutput & out, CoreState & st)
{
  const bool geometry = reference_geometry_valid(out.ref_points, out.n_points);
  if (!geometry || !out.selected_reference.fresh ||
    out.selected_reference.source != out.path_source)
  {
    out.selected_reference.valid = false;
    if (out.path_source < MGM_REFERENCE_PROVIDERS) {out.references[out.path_source].valid = false;}
  }
  if (!out.selected_reference.valid || !std::isfinite(out.v_ref)) {
    out.safe_stop_reasons |= SAFE_STOP_REFERENCE_INVALID;
  }
  if (out.safe_stop_reasons != 0) {
    out.v_ref = st.v = 0.0f;  // includes negative parking/recovery speed
    out.immediate_stop = true;
    out.safety = SafetyState::SAFE_STOP;
    out.speed_owner = out.top == TopState::FINISH ? SpeedOwner::FINISH : SpeedOwner::SAFETY;
  }
  st.managers.safe_stop_reasons = out.safe_stop_reasons;
  st.managers.safety = out.safety;
}
}
