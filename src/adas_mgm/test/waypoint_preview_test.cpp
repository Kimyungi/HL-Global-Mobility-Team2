#include <cmath>
#include <cstdio>
#include "core/mgm_step.hpp"
using namespace adas_mgm;

int main()
{
  CoreParams p{};
  p.avoid_fixed_preview = 1;
  p.avoid_max_cycles = 2;
  p.blend_cycles = 10;
  p.n_cycles = 100;
  p.a_up = p.a_down = 100;
  p.v_base = p.v_avoid = 0.6F;
  p.ttc_stop = 1.0F;
  CoreState st{};
  mgm_init(st, p);
  st.state = MGM_STATE_WAYPOINT;
  CoreSnapshot s{};
  s.gps_heading_valid = true;
  s.avoid_obstacle_detected = s.avoid_avoidable = true;
  s.avoid_ttc = 1.0e9F;
  s.avoid_v_suggest = .6F;
  s.avoid_updated = true;
  s.avoid_path.n = 1;
  s.avoid_path.pts[0] = CorePoint{.8F, .6F, .31F, .42F};
  for (int i = 0; i < 20; ++i) {
    const auto out = mgm_step(s, st);
    if (out.state != MGM_STATE_AVOID || out.n_points != 1 ||
        std::fabs(out.ref_points[0].x-.8F) > 1e-6F ||
        std::fabs(out.ref_points[0].y-.6F) > 1e-6F ||
        std::fabs(out.ref_points[0].yaw-.31F) > 1e-6F ||
        std::fabs(out.ref_points[0].curvature-.42F) > 1e-6F) {
      std::fprintf(stderr, "fixed preview was rescaled, blended or timed out\n");
      return 1;
    }
    s.avoid_updated = false;
  }
  s.avoid_ttc = 0;
  auto stopped = mgm_step(s, st);
  if (stopped.v_ref != 0 || !stopped.immediate_stop) {return 2;}
  s.avoid_ttc = 1.0e9F;
  s.avoid_maneuver_done = true;
  if (mgm_step(s, st).state != MGM_STATE_WAYPOINT) {return 3;}
  return 0;
}
