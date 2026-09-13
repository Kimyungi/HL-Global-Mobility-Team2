// Station-preview wire geometry and existing avoidance motion policy; no CAN TX.
#include "manager_test_fixture.hpp"
#include <limits>
using namespace manager_test;

namespace
{
void configure(Run & r)
{
  r.st.params.a_up = .5f;
  r.st.params.a_down = 1.5f;
  r.st.params.blend_cycles = 10;
  r.st.params.avoid_max_cycles = 1200;
  r.s.avoid_v_suggest = 1.f;
  r.s.avoid_path.pts[0] = CorePoint{.96f, .24f, .21f, .12f};
}

void wire_geometry_and_speed()
{
  Run r; configure(r); r.tick();
  const auto from = r.out.ref_points[0];
  r.obstacle();
  check(r.out.path_source == MGM_SRC_AVOID && r.out.n_points == 1,
    "AVOID preserves v2 single wire point");
  check(near(r.out.ref_points[0].x, from.x + (.96f-from.x)/11.f),
    "entry uses main ten-cycle blend, not an immediate endpoint jump");
  check(near(r.out.v_ref, .985f), "entry decelerates with main a_down");
  r.tick(10);
  check(near(r.out.ref_points[0].x, .96f) && near(r.out.ref_points[0].y, .24f),
    "wire point preserves the station preview without scaling");
  check(near(r.out.ref_points[0].yaw, .21f) &&
    near(r.out.ref_points[0].curvature, .12f), "wire heading/curvature preserve path tangent");
  r.tick(16);
  check(near(r.out.v_ref,.6f), "normal avoidance reaches main .6 cap");
  r.s.avoid_narrow_gap = true; r.tick(27);
  check(near(r.out.v_ref,.2f), "narrow avoidance respects main .2 cap");
  r.s.avoid_narrow_gap = false;
  r.s.avoid_path.pts[0] = CorePoint{2.4f,-.8f,1.f,.3f};
  r.tick();
  check(near(r.out.ref_points[0].x,2.4f) && near(r.out.ref_points[0].y,-.8f) &&
    near(r.out.ref_points[0].yaw,1.f) && near(r.out.ref_points[0].curvature,.3f),
    "right-side refreshed target preserves the new provider geometry");
  const auto held = r.out.ref_points[0];
  r.s.avoid_updated = false; r.tick(5);
  check(near(r.out.ref_points[0].x,held.x) && near(r.out.ref_points[0].y,held.y),
    "100 Hz repeat does not divide the held point by twenty again");
}

void stop_and_restart()
{
  Run r; configure(r); r.tick(); r.obstacle(); r.tick(30);
  r.s.auto_estop = true; r.tick();
  check(r.out.v_ref==0 && r.out.immediate_stop && r.out.selected_reference.valid,
    "LiDAR stop is immediate while valid avoidance geometry remains available");
  check(near(r.out.ref_points[0].x,.96f), "stop does not straighten or discard avoidance");
  r.s.auto_estop = false; r.tick();
  check(near(r.out.v_ref,.005f), "restart begins at .005 rather than fixed 1 m/s");
  r.tick(119); check(near(r.out.v_ref,.6f), "restart reaches .6 over main acceleration ramp");
  r.s.avoid_ttc = .5f; r.tick();
  check(r.out.v_ref==0 && r.out.safety==SafetyState::AUTO_ESTOP,"TTC immediate stop survives");
  r.s.avoid_ttc = 100.f; r.tick();
  check(near(r.out.v_ref,.005f), "TTC release also uses main ramp");
  r.s.external_stop=true; r.tick(); check(r.out.v_ref==0,"external stop wins");
  r.s.external_stop=false; r.s.references[MGM_SRC_AVOID].age_s=1.f; r.tick();
  check(r.out.v_ref==0 && (r.out.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),
    "stale reference still stops rather than certifying transformed geometry");
  r.s.references[MGM_SRC_AVOID].age_s=0; r.s.avoid_path.n=2; r.tick();
  check(r.out.v_ref==0 && !r.out.selected_reference.valid,"oversized provider remains invalid");
  r.s.avoid_path.n=1; r.s.avoid_path.pts[0]={0,0,0,0}; r.tick();
  check(r.out.v_ref==0,"zero provider cannot become a valid synthetic goal");
  r.s.avoid_path.pts[0]={3.76f,.66f,0,0};
  r.s.avoid_v_suggest=std::numeric_limits<float>::quiet_NaN(); r.tick();
  check(r.out.v_ref==0 && (r.out.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),
    "speed caps cannot hide nonfinite provider input");
}

void completion_and_return()
{
  Run r; configure(r); r.tick(); r.obstacle(); r.tick(30);
  r.s.avoid_obstacle_detected=false; r.s.vehicle_speed=0; r.s.auto_estop=true;
  r.tick(250);
  check(r.out.avoid==AvoidState::AVOID_ACTIVE && r.st.return_hold_left==0,
    "two seconds of disappearance while stopped does not finish the maneuver");
  r.s.auto_estop=false; r.tick(120);
  check(near(r.out.v_ref,.6f),"avoidance remains active until completion");
  r.s.avoid_maneuver_done=true; r.tick();
  check(r.out.avoid==AvoidState::INACTIVE && r.out.path_source==MGM_SRC_GPS &&
    r.st.return_hold_left==300,"done returns to GPS and starts a full three-second hold");
  check(near(r.out.v_ref,.605f), "GPS return ramps out of avoidance speed");
  r.s.avoid_maneuver_done=false; r.tick(79);
  check(near(r.out.v_ref,1.f), "GPS return reaches navigation target at main ramp rate");
  r.tick(220); check(r.out.nav==NavState::GPS_BACKUP,"GPS hold persists through tick 299");
  r.tick(); check(r.out.nav==NavState::LINE,"LINE may return only after full hold");
  Run limited; configure(limited); limited.st.params.avoid_max_cycles=3;
  limited.tick(); limited.obstacle(); limited.tick(3);
  check(limited.out.avoid==AvoidState::AVOID_ACTIVE,"main episode counter excludes entry tick");
  limited.tick(); check(limited.out.avoid==AvoidState::INACTIVE && limited.st.return_hold_left==300,
    "configured main episode limit exits to GPS");
  Run unlimited; configure(unlimited); unlimited.st.params.avoid_max_cycles=0;
  unlimited.tick(); unlimited.obstacle(); unlimited.s.avoid_obstacle_detected=false; unlimited.tick(1300);
  check(unlimited.out.avoid==AvoidState::AVOID_ACTIVE,"zero limit does not invent a timeout");
}

void other_owners()
{
  Run r; configure(r); r.tick();
  check(near(r.out.v_ref,1.f) && near(r.out.ref_points[0].x,r.s.lane_path.pts[0].x),
    "ordinary LINE retains v2 speed and original one-point geometry");
  r.gps_zone(true); r.tick();
  check(near(r.out.v_ref,1.f) && near(r.out.ref_points[0].x,r.s.gps_path.pts[0].x),
    "ordinary GPS is not divided by twenty");
  Run fallback; configure(fallback);
  fallback.s.camera_line_valid=false; fallback.s.gps_valid=false; fallback.tick();
  check(fallback.st.managers.avoid_fallback_only,"navigation loss enters fallback-only avoidance");
  fallback.s.camera_line_valid=true; fallback.s.gps_valid=true;
  fallback.s.avoid_obstacle_detected=true; fallback.tick();
  check(fallback.out.avoid==AvoidState::AVOID_ACTIVE && !fallback.st.managers.avoid_fallback_only,
    "navigation recovery with obstacle retains avoidance ownership");
  Run disabled; configure(disabled); disabled.st.params.avoidance_enabled=0;
  disabled.tick(); disabled.obstacle();
  check(disabled.out.path_source!=MGM_SRC_AVOID && near(disabled.out.v_ref,1.f),
    "explicit avoidance OFF still blocks entry");
}
}
int main()
{
  wire_geometry_and_speed(); stop_and_restart(); completion_and_return(); other_owners();
  std::printf("avoid_main_compat_test: %d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
