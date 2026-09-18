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
  check(near(r.out.v_ref, .6f), "entry publishes avoidance cap without a second speed ramp");
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
  check(near(r.out.v_ref,.6f), "restart immediately publishes the avoidance target");
  r.tick(119); check(near(r.out.v_ref,.6f), "restart retains the avoidance target");
  r.s.avoid_ttc = .5f; r.tick();
  check(r.out.v_ref>0 && r.out.safety==SafetyState::NORMAL,"TTC alone does not stop v2");
  r.s.avoid_ttc = 100.f; r.tick();
  check(near(r.out.v_ref,.6f), "TTC changes do not reset avoidance speed");
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
  check(r.out.avoid==AvoidState::GPS_RETURN && r.out.path_source==MGM_SRC_GPS &&
    r.out.state==MGM_STATE_AVOID && r.st.return_hold_left==0,
    "done changes steering to GPS while retaining AVOID state");
  check(near(r.out.v_ref,1.f), "GPS return immediately publishes navigation target");
  r.s.avoid_maneuver_done=false; r.s.gps_cross_track=.5f;
  r.s.gps_heading_valid=r.s.gps_station_error_valid=true;
  r.tick(400);
  check(near(r.out.v_ref,1.f) && r.out.avoid==AvoidState::GPS_RETURN,
    "GPS return reaches navigation speed but elapsed time cannot release AVOID");
  r.s.gps_cross_track=.3f; r.s.gps_station_yaw_error=0; r.tick();
  check(r.out.avoid==AvoidState::INACTIVE && r.out.nav==NavState::LINE,
    "aligned station releases avoidance and reselects navigation without another hold");
  Run limited; configure(limited); limited.st.params.avoid_max_cycles=3;
  limited.tick(); limited.obstacle(); limited.s.avoid_obstacle_detected=false; limited.tick(3);
  check(limited.out.avoid==AvoidState::AVOID_ACTIVE,"main episode counter excludes entry tick");
  limited.tick(); check(limited.out.avoid==AvoidState::GPS_RETURN && limited.out.state==MGM_STATE_AVOID,
    "maneuver time limit starts GPS return instead of releasing avoidance");
  Run unlimited; configure(unlimited); unlimited.st.params.avoid_max_cycles=0;
  unlimited.tick(); unlimited.obstacle(); unlimited.s.avoid_obstacle_detected=false; unlimited.tick(1300);
  check(unlimited.out.avoid==AvoidState::AVOID_ACTIVE,"zero limit does not invent a timeout");
}

void gps_return_guards_and_reverse_entry()
{
  Run r; configure(r); r.tick(); r.obstacle();
  r.s.avoid_obstacle_detected=false; r.s.avoid_maneuver_done=true; r.tick();
  r.s.avoid_maneuver_done=false;
  r.s.gps_heading_valid=r.s.gps_station_error_valid=true;
  r.s.gps_cross_track=.3001f; r.s.gps_station_yaw_error=0;
  r.tick(); check(r.out.avoid==AvoidState::GPS_RETURN,"lateral threshold must also pass");
  r.s.gps_cross_track=0; r.s.gps_station_yaw_error=20.001f*3.14159265358979323846f/180.f;
  r.s.gps_path.pts[0].yaw=0;
  r.tick(); check(r.out.avoid==AvoidState::GPS_RETURN,
    "station heading threshold must pass even if preview yaw is zero");
  r.s.gps_station_yaw_error=0; r.s.gps_station_error_valid=false; r.tick();
  check(r.out.avoid==AvoidState::GPS_RETURN,"unknown heading cannot certify return");
  r.s.gps_station_error_valid=true; r.s.gps_heading_valid=false; r.tick();
  check(r.out.avoid==AvoidState::GPS_RETURN,"tangent fallback cannot certify return");
  r.s.gps_heading_valid=true;
  r.s.gps_cross_track=std::numeric_limits<float>::quiet_NaN(); r.tick();
  check(r.out.avoid==AvoidState::GPS_RETURN,"NaN lateral error cannot certify return");
  r.s.gps_cross_track=0; r.s.gps_station_yaw_error=std::numeric_limits<float>::infinity(); r.tick();
  check(r.out.avoid==AvoidState::GPS_RETURN,"infinite yaw error cannot certify return");
  r.s.gps_station_yaw_error=0; r.s.references[MGM_SRC_GPS].age_s=1; r.tick();
  check(r.out.avoid==AvoidState::GPS_RETURN && r.out.path_source==MGM_SRC_GPS && r.out.v_ref==0,
    "stale GPS cannot switch to camera or certify convergence");
  r.s.references[MGM_SRC_GPS].age_s=0; r.s.gps_valid=false; r.tick();
  check(r.out.avoid==AvoidState::GPS_RETURN && r.out.v_ref==0,"invalid fix stops the return");
  r.s.gps_valid=true; r.s.gps_cross_track=.5f; r.s.avoid_path.n=0; r.tick();
  check(r.out.path_source==MGM_SRC_GPS && r.out.selected_reference.valid && r.out.v_ref>0,
    "GPS return does not require an avoidance steering reference");
  r.s.external_stop=true; r.tick();
  check(r.out.avoid==AvoidState::GPS_RETURN && r.out.v_ref==0,"external stop retains unaligned return");
  r.s.external_stop=false;
  r.s.gps_cross_track=-.3f; r.s.gps_station_yaw_error=-20.f*3.14159265358979323846f/180.f;
  r.s.gps_path.pts[0].yaw=1.f; r.tick();
  check(r.out.avoid==AvoidState::INACTIVE,
    "inclusive absolute error boundaries release return independently of preview yaw");

  Run reverse; configure(reverse); reverse.st.params.escape_after_cycles=1;
  reverse.st.params.escape_max_cycles=3; reverse.st.params.escape_require_rear_clear=0;
  reverse.tick(); reverse.s.auto_estop=true; reverse.tick();
  check(reverse.out.avoid==AvoidState::AVOID_ACTIVE && reverse.out.state==MGM_STATE_AVOID &&
    reverse.out.path_source==MGM_SRC_ESCAPE && reverse.out.v_ref<0,
    "reverse entry immediately latches avoidance even without avoidable obstacle");
  reverse.tick(3); reverse.s.auto_estop=false; reverse.tick(2);
  check(reverse.out.avoid==AvoidState::AVOID_ACTIVE && reverse.out.path_source==MGM_SRC_AVOID,
    "reverse completion retains obstacle maneuver ownership");
  reverse.s.avoid_maneuver_done=true; reverse.tick();
  check(reverse.out.avoid==AvoidState::GPS_RETURN && reverse.out.path_source==MGM_SRC_GPS,
    "post-reverse maneuver completion uses the same GPS return exit gate");
  reverse.s.avoid_maneuver_done=false; reverse.s.avoid_obstacle_detected=true;
  reverse.s.avoid_avoidable=true; reverse.tick();
  check(reverse.out.avoid==AvoidState::AVOID_ACTIVE && reverse.out.path_source==MGM_SRC_AVOID,
    "a new avoidable obstacle restarts maneuver steering during GPS return");
}

void obstacle_without_reference()
{
  Run r; configure(r); r.tick();
  r.s.avoid_obstacle_detected=true; r.s.avoid_avoidable=false;
  r.s.avoid_ttc=2.929f; r.s.avoid_path.n=0;
  r.s.references[MGM_SRC_AVOID].generation=0;
  r.tick();
  check(r.out.state==MGM_STATE_AVOID && r.out.avoid==AvoidState::AVOID_ACTIVE &&
    r.out.path_source==MGM_SRC_AVOID && r.out.v_ref==0 &&
    (r.out.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),
    "detected obstacle without target claims AVOID and stops instead of following LINE");
  r.st.params.avoid_max_cycles=3; r.tick(180);
  check(r.out.path_source==MGM_SRC_AVOID && r.out.v_ref==0,
    "waiting for a target never expires into LINE or GPS driving");
  r.s.avoid_obstacle_detected=false; r.tick(10);
  check(r.out.path_source==MGM_SRC_GPS && r.out.avoid==AvoidState::INACTIVE && r.out.v_ref>0,
    "cleared obstacle with empty avoidance returns to valid GPS");
  r.s.avoid_obstacle_detected=true; r.s.avoid_path.n=1;
  r.s.references[MGM_SRC_AVOID]={2, 1.f, .5f}; r.s.avoid_avoidable=true; r.tick();
  check(r.out.v_ref==0, "stale replacement target cannot release stop");
  r.s.references[MGM_SRC_AVOID]={3, 0.f, .5f}; r.tick();
  check(r.out.path_source==MGM_SRC_AVOID && near(r.out.v_ref,.6f),
    "fresh replacement immediately releases the decided avoidance target");
  r.s.avoid_ttc=1.4f; r.s.avoid_avoidable=false; r.tick();
  check(r.out.v_ref>0 && r.out.safety==SafetyState::NORMAL,
    "provider avoidable flag does not create v2 E-stop");

  Run returning; configure(returning); returning.obstacle();
  returning.s.avoid_obstacle_detected=false; returning.s.avoid_maneuver_done=true;
  returning.tick();
  returning.s.avoid_obstacle_detected=true; returning.s.avoid_avoidable=false;
  returning.s.avoid_path.n=0; returning.tick();
  check(returning.out.avoid==AvoidState::AVOID_ACTIVE &&
    returning.out.path_source==MGM_SRC_AVOID && returning.out.v_ref==0,
    "new obstacle interrupts GPS return even with an old done flag and missing target");

  Run idle; configure(idle); idle.s.avoid_path.n=0; idle.s.avoid_avoidable=false; idle.tick();
  check(idle.out.avoid==AvoidState::INACTIVE && idle.out.v_ref>0,
    "no obstacle and no active episode needs no avoidance target");
  idle.st.params.avoid_zone_only=1; idle.s.gps_avoid_zone=false;
  idle.s.avoid_obstacle_detected=true; idle.tick();
  check(idle.out.avoid==AvoidState::INACTIVE, "explicit avoidance zone restriction remains effective");
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
  wire_geometry_and_speed(); stop_and_restart(); completion_and_return(); gps_return_guards_and_reverse_entry(); obstacle_without_reference(); other_owners();
  std::printf("avoid_main_compat_test: %d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
