#include "manager_test_fixture.hpp"
#include <initializer_list>
using namespace manager_test;

void stopped(Run & r)
{
  r.tick(50);  // Mature LINE confidence must not win the release tick.
  r.redline();
  r.s.traffic_stopline_detected=false; r.tick();
  r.st.traffic_stopline_distance=.9f;
  r.s.vehicle_speed=0; r.tick();
  check(r.out.signal==SignalState::STOPPED_WAIT && r.out.v_ref==0,"fixture: signal stopped");
}

int main()
{
  for (bool red : {false,true}) for (bool green : {false,true}) {
    Run r; stopped(r);
    r.s.traffic_red_active=red; r.s.traffic_green_active=green; r.tick();
    check((r.out.signal==SignalState::SIGNAL_IDLE)==!red,"release truth table equals !red");
    if (!red) {
      check(r.out.nav==NavState::GPS_BACKUP && r.out.state==MGM_STATE_WAYPOINT &&
        r.out.path_source==MGM_SRC_GPS && r.out.v_ref>0,"no red releases to GPS even with mature LINE");
      check(!r.st.traffic_distance_latched && r.st.traffic_stopline_distance==0 &&
        !r.st.traffic_prev_stopline_detected,"release resets stop-line history");
      r.tick(5);check(r.out.path_source==MGM_SRC_GPS,"GPS survives the following ticks");
    } else {check(r.out.v_ref==0,"red wins even with green");}
  }
  for (bool sensor_only : {false,true}) for (bool green : {false,true}) {
    Run r; r.st.params.safe_stop_all_sensors_only=sensor_only; r.s.sensor_alive_mask=0x7f;
    r.gps_zone(true); stopped(r);
    r.s.avoid_path.n=0; r.obstacle();
    check(r.out.avoid==AvoidState::AVOID_ACTIVE && r.out.v_ref==0,"waiting obstacle with empty path");
    r.s.avoid_obstacle_detected=false;r.s.traffic_red_active=false;r.s.traffic_green_active=green;r.tick();
    check(r.out.avoid==AvoidState::INACTIVE && r.out.nav==NavState::GPS_ONLY_NAV &&
      r.out.state==MGM_STATE_WAYPOINT && r.out.path_source==MGM_SRC_GPS &&
      r.out.safety==SafetyState::NORMAL && r.out.v_ref>0,"historical empty avoid episode releases to GPS");
    check(r.st.avoid_ticks==0 && !r.st.managers.avoid_episode_reference_seen,"avoid bookkeeping cleared");
    r.tick(10);check(r.out.avoid==AvoidState::INACTIVE && r.out.v_ref>0,"no reentry from old empty avoid output");
  }
  Run obstacle;stopped(obstacle);obstacle.s.avoid_path.n=0;obstacle.obstacle();
  obstacle.s.traffic_red_active=false;obstacle.s.avoid_avoidable=false;obstacle.tick();
  check(obstacle.out.signal==SignalState::SIGNAL_IDLE && obstacle.out.avoid==AvoidState::AVOID_ACTIVE &&
    obstacle.out.v_ref==0,"current obstacle is not discarded by signal release");
  Run gps;stopped(gps);gps.s.gps_path.n=0;gps.s.traffic_red_active=false;gps.tick();
  check(gps.out.path_source==MGM_SRC_GPS && gps.out.v_ref==0 && !gps.out.reference_available,
    "GPS handoff with missing GPS holds motion");
  for (int stop=0;stop<2;++stop) {
    Run r;stopped(r);r.s.traffic_red_active=false;
    r.s.external_stop=stop==0;r.s.auto_estop=stop==1;r.tick();
    check(r.out.signal==SignalState::SIGNAL_IDLE && r.out.path_source==MGM_SRC_GPS && r.out.v_ref==0,
      "operator/CAN and auto-estop remain independent");
  }
  Run normal;normal.tick(100);
  check(normal.out.nav==NavState::LINE,"no-red without signal episode does not force GPS continuously");
  Run disabled;disabled.st.params.traffic_state_enabled=0;disabled.st.managers.signal=SignalState::STOPPED_WAIT;
  disabled.tick(50);check(disabled.out.nav==NavState::LINE,"disabled signal manager does not hand off");
  Run mission;mission.mission();mission.st.managers.signal=SignalState::STOPPED_WAIT;mission.tick();
  check(mission.out.mission==MissionState::MISSION_ACTIVE && mission.out.path_source==MGM_SRC_PARKING,
    "signal release does not take parking authority");
  std::printf("traffic_gps_release_test: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
