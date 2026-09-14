#include "manager_test_fixture.hpp"
#include <initializer_list>
#include <limits>
using namespace manager_test;
void configure(Run&r) {
  r.st.params.safe_stop_all_sensors_only=1;r.s.sensor_alive_mask=127;
  r.st.params.v_base=r.st.params.v_avoid=2.f;
  r.s.avoid_v_suggest=2.f;r.s.vehicle_speed=2.f;
  r.tick();
}
int main() {
  for(float ttc: {1.75f,1.495f,1.0f,0.f}) for(bool possible: {false,true}) {
    Run r;configure(r);r.s.avoid_obstacle_detected=true;
    r.s.avoid_avoidable=possible;r.s.avoid_ttc=ttc;r.tick();
    check(r.out.avoid==AvoidState::AVOID_ACTIVE && r.out.path_source==MGM_SRC_AVOID,
      "obstacle keeps avoidance authority regardless of feasibility flag");
    check(r.out.safety==SafetyState::NORMAL && !r.out.immediate_stop && r.out.v_ref>0,
      "TTC/avoidable never create an additional E-stop or zero-speed request");
    r.s.auto_estop=true;r.tick();
    check(r.out.safety==SafetyState::AUTO_ESTOP && r.out.immediate_stop && r.out.v_ref==0,
      "independent LiDAR request stops the same avoidance path");
    r.s.auto_estop=false;r.tick();
    check(r.out.safety==SafetyState::NORMAL && r.out.v_ref>0,
      "independent release works even when short TTC/avoidable=false remain");
  }
  Run missing;configure(missing);missing.s.avoid_obstacle_detected=true;
  missing.s.avoid_avoidable=false;missing.s.avoid_path.n=0;missing.s.avoid_ttc=.1f;missing.tick();
  check(missing.out.safety==SafetyState::NORMAL && missing.out.reference_motion_blocked && missing.out.v_ref==0,
    "no steering reference remains an output hold, not an E-stop");
  missing.s.avoid_path.n=1;missing.tick();
  check(!missing.out.reference_motion_blocked && missing.out.v_ref>0,
    "valid target restores motion despite short TTC and false avoidable");
  for(bool stale: {false,true}) {
    Run gps;configure(gps);gps.s.lane_path.n=0;gps.s.camera_line_valid=false;
    gps.s.avoid_obstacle_detected=true;gps.s.avoid_path.n=0;gps.tick();
    gps.s.avoid_obstacle_detected=false;
    if(stale) {gps.s.avoid_path.n=1;gps.s.references[MGM_SRC_AVOID].age_s=1.f;}
    gps.tick();
    check(gps.out.avoid==AvoidState::INACTIVE && gps.out.nav==NavState::GPS_BACKUP &&
      gps.out.path_source==MGM_SRC_GPS && gps.out.reference_available && gps.out.v_ref>0,
      "no obstacle and no LINE uses GPS when old avoidance is empty/stale");
    gps.tick(10);check(gps.out.path_source==MGM_SRC_GPS && gps.out.v_ref>0,
      "GPS fallback persists without a signal or planner completion event");
    gps.s.auto_estop=true;gps.tick();check(gps.out.v_ref==0,
      "independent lidar stop remains effective after GPS fallback");
  }
  Run absent;configure(absent);absent.s.camera_line_valid=false;absent.s.gps_path.n=0;
  absent.s.avoid_obstacle_detected=true;absent.s.avoid_path.n=0;absent.tick();
  absent.s.avoid_obstacle_detected=false;absent.tick();
  check(absent.out.v_ref==0 && !absent.out.reference_available,
    "missing GPS cannot fabricate a driving reference");
  Run nav;configure(nav);nav.s.auto_estop=true;nav.tick();
  check(nav.out.safety==SafetyState::AUTO_ESTOP && nav.out.v_ref==0,"independent stop covers navigation");
  nav.s.traffic_red_active=true;nav.tick();nav.s.traffic_red_active=false;nav.tick();
  check(nav.out.path_source==MGM_SRC_GPS && nav.out.safety==SafetyState::AUTO_ESTOP && nav.out.v_ref==0,
    "signal-to-GPS transition cannot clear independent stop");
  nav.s.auto_estop=false;nav.s.external_stop=true;nav.tick();
  check(nav.out.v_ref==0,"operator/CAN stop is preserved");
  Run route;configure(route);route.st.params.route_sequence_enabled=1;
  route.s.route.enabled=true;route.s.route.sequence_id=11;route.s.route.instance_id=22;route.s.route.count=2;
  route.tick();route.s.avoid_obstacle_detected=true;route.s.avoid_avoidable=false;
  route.s.avoid_ttc=.1f;route.s.vehicle_speed=0;route.s.gps_at_end=true;
  ++route.s.references[MGM_SRC_GPS].generation;route.s.auto_estop=true;route.tick();
  check(route.out.route.phase==RoutePhase::WAIT_STOP && route.out.route.request_id==0,
    "independent stop still prevents route handoff");
  route.s.auto_estop=false;++route.s.references[MGM_SRC_GPS].generation;route.tick();
  check(route.out.route.phase==RoutePhase::WAIT_ACK && route.out.route.requested_index==1,
    "avoid TTC does not independently block stopped route handoff");
  std::printf("lidar_only_estop_test: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
