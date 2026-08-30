#include <cmath>
#include <cstdio>

#include "core/mgm_step.hpp"

using namespace adas_mgm;

namespace
{
int failures = 0;

void check(bool ok, const char * message)
{
  if (!ok) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", message);
  }
}

CoreParams params()
{
  CoreParams p{};
  p.n_cycles = 100;
  p.a_up = 100.0F;
  p.a_down = 100.0F;
  p.v_base = 0.8F;
  p.v_avoid = 0.8F;
  p.ttc_stop = 0.5F;
  return p;
}

void setPath(CorePath & path, float x, float y)
{
  path.n = 1;
  path.pts[0] = CorePoint{x, y, 0.1F, 0.2F};
}

void verifyHold(uint8_t state, const char * label)
{
  CoreState memory{};
  mgm_init(memory, params());
  memory.state = state;
  memory.last_src = state;
  memory.v = 0.8F;

  CoreSnapshot input{};
  input.lane_confidence = state == MGM_STATE_WAYPOINT ? 0.0F : 1.0F;
  input.gps_heading_valid = true;
  input.avoid_ttc = 1.0e9F;
  input.lane_updated = input.gps_updated = input.avoid_updated = true;
  setPath(input.lane_path, 1.2F, 0.1F);
  setPath(input.gps_path, 1.3F, 0.2F);
  setPath(input.avoid_path, 1.4F, 0.3F);
  setPath(input.parking_path, 1.5F, 0.4F);

  const CoreOutput first = mgm_step(input, memory);
  input.lane_updated = input.gps_updated = input.avoid_updated = false;
  const CoreOutput repeated = mgm_step(input, memory);
  check(first.n_points == repeated.n_points, label);
  for (int32_t i = 0; i < first.n_points; ++i) {
    check(first.ref_points[i].x == repeated.ref_points[i].x, label);
    check(first.ref_points[i].y == repeated.ref_points[i].y, label);
    check(first.ref_points[i].yaw == repeated.ref_points[i].yaw, label);
    check(first.ref_points[i].curvature == repeated.ref_points[i].curvature, label);
  }
}
}  // namespace

int main()
{
  verifyHold(MGM_STATE_LANE, "LANE stale ref must hold");
  verifyHold(MGM_STATE_WAYPOINT, "WAYPOINT stale ref must hold");
  verifyHold(MGM_STATE_AVOID, "AVOID stale ref must hold");
  verifyHold(MGM_STATE_PARKING, "PARKING stale ref must hold");
  return failures == 0 ? 0 : 1;
}
