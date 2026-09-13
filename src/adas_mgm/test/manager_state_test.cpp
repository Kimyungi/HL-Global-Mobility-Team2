#include "core/mgm_step.hpp"
#include <cmath>
#include <cstdio>
#include <limits>
using namespace adas_mgm;
#include "manager_test_fixture.hpp"
using namespace manager_test;
namespace
{
void navigation()
{
  Run r;
  r.s.lane_confidence = .34f;
  r.tick(49); check(r.out.nav==NavState::LINE,"1: low 49 remains LINE");
  r.tick(); check(r.out.nav==NavState::GPS_BACKUP,"1: low 50 selects GPS");
  r.s.lane_confidence=.7f;
  r.tick(49); check(r.out.nav==NavState::GPS_BACKUP,"3: high 49 stays GPS");
  r.tick(); check(r.out.nav==NavState::LINE,"3: exactly .7 high 50 selects LINE");
  Run reset;
  reset.s.lane_confidence=.1f; reset.tick(49);
  reset.s.lane_confidence=.35f; reset.tick();
  check(reset.st.lane_low_cnt==0,"2: .35 resets low count");
  reset.st.managers.nav=NavState::GPS_BACKUP;
  reset.s.lane_confidence=.7f; reset.tick(49);
  reset.s.lane_confidence=.69f; reset.tick();
  check(reset.st.lane_high_cnt==0,"4: <.7 resets high count");
  reset.s.lane_confidence=.9f; reset.tick(49);
  reset.s.camera_line_valid=false; reset.tick();
  check(reset.st.lane_high_cnt==0,"4: invalid camera resets high count");
  Run timeout; timeout.s.camera_line_valid=false; timeout.tick();
  check(timeout.out.nav==NavState::GPS_BACKUP,"5: timeout immediately uses GPS");
  Run zone; zone.tick(50); zone.gps_zone(true); zone.tick();
  check(zone.out.nav==NavState::GPS_ONLY_NAV,"6: zone overrides LINE");
  zone.s.gps_path.pts[0].x += 1.f; zone.tick();
  check(zone.out.nav==NavState::GPS_ONLY_NAV,"8: another waypoint is not zone exit");
  zone.s.gps_valid=false; zone.gps_zone(false); zone.tick();
  check(zone.out.nav==NavState::GPS_ONLY_NAV && zone.out.safety==SafetyState::SAFE_STOP &&
    zone.out.v_ref==0,"7: GPS loss retains zone and stops, no LINE/LiDAR replacement");
  zone.s.gps_valid=true; zone.gps_zone(true); zone.tick();
  check(zone.out.nav==NavState::GPS_ONLY_NAV && zone.out.safety==SafetyState::NORMAL,"GPS recovery in zone");
  zone.gps_zone(false); zone.tick();
  check(zone.out.nav==NavState::LINE,"9: valid zone exit reselects immediately");
  Run zonebackup; zonebackup.s.camera_line_valid=false; zonebackup.tick();
  zonebackup.gps_zone(true); zonebackup.tick();
  check(zonebackup.out.nav==NavState::GPS_ONLY_NAV,"6: zone overrides GPS_BACKUP");
  zonebackup.gps_zone(false); zonebackup.tick();
  check(zonebackup.out.nav==NavState::GPS_BACKUP,"9: exit with low LINE selects GPS_BACKUP");
}
void avoidance()
{
  Run r; r.obstacle();
  check(r.out.avoid==AvoidState::AVOID_ACTIVE && r.out.path_source==MGM_SRC_AVOID,"10: obstacle selects avoidance");
  r.s.avoid_obstacle_detected=false;
  r.tick(250);
  check(r.out.avoid==AvoidState::AVOID_ACTIVE && r.out.path_source==MGM_SRC_AVOID,"11: disappearance alone keeps avoidance");
  r.s.avoid_maneuver_done=true; r.tick();
  check(r.out.avoid==AvoidState::GPS_RETURN && r.out.path_source==MGM_SRC_GPS,"11: completion starts GPS return inside avoidance");
  check(r.st.lane_high_cnt==50,"high confidence accumulates throughout avoidance");
  r.s.avoid_maneuver_done=false;
  r.tick(400); check(r.out.avoid==AvoidState::GPS_RETURN,"12: elapsed time cannot release GPS return");
  r.s.gps_heading_valid=r.s.gps_station_error_valid=true; r.tick();
  check(r.out.nav==NavState::LINE && r.out.avoid==AvoidState::INACTIVE,"12: valid station alignment releases GPS return");
  Run redetect; redetect.obstacle(); redetect.s.avoid_obstacle_detected=false; redetect.tick(199);
  redetect.s.avoid_obstacle_detected=true; redetect.tick();
  check(redetect.out.avoid==AvoidState::AVOID_ACTIVE && redetect.st.managers.clear_count==0,"13: redetection keeps episode active");
  redetect.s.avoid_obstacle_detected=false; redetect.tick(200);
  check(redetect.out.avoid==AvoidState::AVOID_ACTIVE && redetect.st.return_hold_left==0,"13: disappearance cannot start return hold");
  Run nogps; nogps.obstacle(); nogps.s.avoid_obstacle_detected=false; nogps.tick(199);
  nogps.s.avoid_maneuver_done=true; nogps.tick();
  nogps.s.gps_valid=false; nogps.tick();
  check(nogps.out.avoid==AvoidState::GPS_RETURN && nogps.out.path_source==MGM_SRC_GPS && nogps.out.v_ref==0,"GPS loss preserves GPS return ownership and stops");
  Run notready; notready.st.managers.nav=NavState::GPS_BACKUP;
  notready.st.return_hold_left=300; notready.s.gps_valid=false;
  notready.tick(49); check(notready.out.nav==NavState::GPS_BACKUP,"GPS loss must not bypass high 49");
  notready.tick(); check(notready.out.nav==NavState::LINE,"GPS loss high 50 can return");
  Run invalid; invalid.obstacle(); invalid.s.avoid_obstacle_detected=false; invalid.tick(100);
  invalid.s.lidar_valid=false; invalid.tick(100);
  check(invalid.st.managers.clear_count==0 && invalid.out.safety==SafetyState::NORMAL,"invalid LiDAR cannot confirm clear, navigation continues");
  Run fallback; fallback.s.camera_line_valid=false; fallback.s.gps_valid=false;
  fallback.s.avoid_path.n=0; fallback.tick();
  check(fallback.out.avoid==AvoidState::AVOID_ACTIVE && !fallback.out.reference_available,"LiDAR only empty path reported as integration constraint");
}
void signal()
{
  Run red; red.s.traffic_red_active=true; red.tick();
  check(red.out.signal==SignalState::RED_DETECTED && red.out.v_ref>0,"14: red alone does not stop");
  Run line; line.s.traffic_stopline_detected=true; line.tick();
  check(line.out.signal==SignalState::SIGNAL_IDLE && line.out.v_ref>0,"15: stop line alone does not stop");
  Run r; r.redline();
  check(r.out.signal==SignalState::APPROACH_STOP_LINE,"16: red + line starts approach");
  r.s.traffic_stopline_detected=false; r.s.vehicle_speed=.5f; r.tick();
  check(near(r.st.traffic_stopline_distance,1.5f),"17: first loss seeds exactly 1.5 once");
  r.tick(10); check(near(r.st.traffic_stopline_distance,1.45f),"17: distance uses actual .5 m/s");
  r.s.traffic_stopline_detected=true; r.tick(); r.s.traffic_stopline_detected=false; r.tick();
  check(near(r.st.traffic_stopline_distance,1.44f),"17: flicker cannot reseed");
  r.s.traffic_red_active=false; r.tick(100); r.s.vehicle_speed=0; r.tick();
  check(r.out.signal==SignalState::STOPPED_WAIT && r.out.v_ref==0,"18: lost red without green remains stopped");
  r.s.traffic_red_active=true; r.s.traffic_green_active=true; r.tick();
  check(r.out.signal==SignalState::STOPPED_WAIT,"green with red cannot release");
  r.s.traffic_red_active=false; r.s.auto_estop=true; r.tick();
  check(r.out.signal==SignalState::SIGNAL_IDLE && r.out.v_ref==0,"19: green releases only signal, auto estop remains");
  Run both; both.redline(); both.obstacle(); both.s.traffic_stopline_detected=false; both.tick();
  both.s.vehicle_speed=1.f; both.tick(30);
  check(both.out.path_source==MGM_SRC_AVOID && both.out.speed_owner==SpeedOwner::TRAFFIC &&
    both.out.signal==SignalState::APPROACH_STOP_LINE,"20: avoid reference and stop-line speed coexist");
  both.tick(30);
  check(both.out.path_source==MGM_SRC_AVOID && both.out.v_ref==0,"20: avoidance cannot cancel signal stop");
}
void mission()
{
  Run r; r.s.gps_parking_zone=true; r.s.parking_space_found=true; r.tick();
  check(r.out.mission==MissionState::MISSION_IDLE,"legacy parking boolean is not a typed Mission Zone entry");
  r.mission(); check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.mission_start &&
    r.out.path_source==MGM_SRC_PARKING,"21: Mission Zone entry starts typed mission");
  r.s.auto_estop=true; r.obstacle();
  check(r.out.avoid==AvoidState::INACTIVE && r.out.safety!=SafetyState::AUTO_ESTOP,"22: parking masks ordinary avoidance and LiDAR estop");
  r.s.parking_updated=true; r.s.parking_done=false; r.tick();
  check(r.out.v_ref<0 && r.out.speed_owner==SpeedOwner::MISSION,"parking speed owns output after fresh acknowledgement");
  r.s.external_stop=true; r.tick(); check(r.out.v_ref==0,"parking must honor operator/CAN stop");
  r.s.external_stop=false; r.s.auto_estop=false; r.s.avoid_obstacle_detected=false;
  r.s.parking_done=true; r.s.parking_updated=false; r.tick();
  check(r.out.mission==MissionState::MISSION_ACTIVE,"stale done cannot finish mission");
  r.s.parking_updated=true; r.tick();
  check(r.out.mission==MissionState::MISSION_IDLE && r.st.managers.mission_completed[0],"23: fresh done records completion and leaves mission");
  r.mission(); check(r.out.mission==MissionState::MISSION_IDLE,"24: repeated trigger cannot rerun");
  r.mission(20); check(r.out.mission_type==MissionType::PARALLEL_PARKING,"data mapping selects parallel parking");
  r.s.parking_done=true; r.s.parking_updated=true; r.tick();
  check(r.out.mission==MissionState::MISSION_ACTIVE,"previous mission done must not finish new mission");
  r.s.parking_mission_mode=static_cast<uint8_t>(MissionType::PARALLEL_PARKING);
  r.s.parking_done=false; r.tick(); r.s.parking_done=true; r.tick();
  check(r.st.managers.mission_completed[1],"both mission memories retained");
  Run zone; zone.mission(); zone.s.parking_updated=true; zone.tick();
  zone.gps_zone(true); zone.s.parking_done=true; zone.tick();
  check(zone.out.nav==NavState::GPS_ONLY_NAV,"mission exit reevaluates zone");
}
void safety()
{
  Run r; r.tick(); r.s.auto_estop=true; r.tick();
  check(r.out.safety==SafetyState::AUTO_ESTOP && r.out.immediate_stop && r.out.v_ref==0,"25: auto estop brakes");
  r.st.params.escape_after_cycles=1000; // existing documented 10s setting, runtime default stays disabled
  r.tick(998); check(r.out.safety==SafetyState::AUTO_ESTOP,"26: stuck 999 cycles does not reverse");
  r.tick(); check(r.out.safety==SafetyState::REVERSE_RECOVERY && r.out.v_ref<0 &&
    r.out.path_source==MGM_SRC_ESCAPE,"26: existing 1000 cycle condition starts existing recovery ref");
  r.s.auto_estop=false; r.gps_zone(true); r.tick();
  check(r.out.safety==SafetyState::NORMAL && r.out.nav==NavState::GPS_ONLY_NAV &&
    r.out.path_source==MGM_SRC_AVOID && r.out.avoid==AvoidState::AVOID_ACTIVE,"27: recovery end keeps avoidance and observes current zone");
  Run all; all.s.camera_line_valid=all.s.gps_valid=all.s.lidar_valid=false; all.tick();
  check(all.out.safety==SafetyState::SAFE_STOP && all.out.v_ref==0,"28: all sensors unavailable stops");
  all.s.gps_valid=true; all.tick();
  check(all.out.safety==SafetyState::NORMAL && all.out.nav==NavState::GPS_BACKUP,"safe-stop recovery reselects usable GPS");
  Run lidar; lidar.s.lidar_valid=false; lidar.s.estop=true; lidar.tick();
  check(lidar.out.safety==SafetyState::NORMAL && lidar.out.v_ref>0,"LiDAR timeout alone does not stop usable navigation");
  Run finish; finish.tick(); finish.s.gps_at_end=true; finish.tick();
  check(finish.out.top==TopState::FINISH && finish.out.v_ref==0,"29: finish waypoint latches FINISH");
  finish.s.gps_at_end=false; finish.s.estop=finish.s.estop_latch_release=true;
  finish.s.traffic_green_active=true; finish.s.auto_estop=true;
  finish.st.params.escape_after_cycles=1; finish.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0); finish.tick(100);
  check(finish.out.top==TopState::FINISH && finish.out.v_ref==0 && !finish.out.mission_start,"29: all recovery/mission/green/estop events cannot release FINISH");
  finish.s.new_session=true; finish.s.auto_estop=false;  finish.tick();
  check(finish.out.top==TopState::AUTONOMOUS_DRIVE,"explicit new session clears FINISH");
  Run concurrent; concurrent.redline(); concurrent.s.auto_estop=true; concurrent.obstacle();
  check(concurrent.out.path_source==MGM_SRC_AVOID && concurrent.out.n_points==1 &&
    near(concurrent.out.ref_points[0].y,.3f/MGM_NUM_POINTS) && concurrent.out.v_ref==0 &&
    concurrent.out.speed_owner==SpeedOwner::SAFETY,"30: simultaneous requests yield single chosen reference and speed/brake");
  Run enable; enable.s.autonomous_enabled=false; enable.tick();
  check(enable.out.top==TopState::AUTONOMOUS_ENABLE && enable.out.v_ref==0,"enable gate holds output");
  Run low_no_gps; low_no_gps.s.gps_valid=false; low_no_gps.s.lane_confidence=.1f;
  low_no_gps.tick(50);
  check(low_no_gps.out.avoid==AvoidState::AVOID_ACTIVE,"unusable low-confidence LINE + no GPS requests existing LiDAR path");
  low_no_gps.s.lidar_valid=false; low_no_gps.tick();
  check(low_no_gps.out.safety==SafetyState::SAFE_STOP,"low-confidence LINE cannot replace all unavailable sources");
  Run mission_ref; mission_ref.st.params.blend_cycles=10; mission_ref.tick();
  mission_ref.mission();
  check(mission_ref.out.v_ref==0 && near(mission_ref.out.ref_points[0].x,0),
    "mission awaiting ack does not retain navigation geometry");
  mission_ref.s.parking_updated=true; mission_ref.s.parking_mission_active=false;
  mission_ref.tick(); check(mission_ref.out.v_ref==0,"idle parking heartbeat does not acknowledge start");
  mission_ref.s.parking_mission_active=true; mission_ref.tick();
  check(near(mission_ref.out.ref_points[0].x,-1.f),"mission reference is not blended with navigation");
  mission_ref.st.params.a_up=.5f; mission_ref.s.parking_done=true; mission_ref.tick();
  check(mission_ref.out.v_ref==0 && mission_ref.out.immediate_stop,
    "negative mission ramp cannot carry into forward navigation");
  Run recovery_ref; recovery_ref.tick(); recovery_ref.st.params.escape_after_cycles=1;
  recovery_ref.st.params.blend_cycles=10; recovery_ref.s.auto_estop=true; recovery_ref.tick();
  recovery_ref.tick();
  check(recovery_ref.out.v_ref<0 && near(recovery_ref.out.ref_points[0].y,0.f),
    "recovery straight ref must not blend with turning navigation");
  recovery_ref.s.auto_estop=false; recovery_ref.s.camera_line_valid=false;
  recovery_ref.s.gps_valid=false; recovery_ref.s.avoid_path.n=0; recovery_ref.tick();
  check(recovery_ref.out.safety==SafetyState::SAFE_STOP && recovery_ref.out.v_ref==0 &&
    recovery_ref.st.managers.recovery_waiting_reference,
    "recovery exit waits for actual reacquired reference");
  recovery_ref.s.gps_valid=true; recovery_ref.tick();
  check(recovery_ref.out.v_ref==0 && recovery_ref.out.avoid==AvoidState::AVOID_ACTIVE,
    "GPS recovery alone cannot release unfinished obstacle maneuver");
  recovery_ref.s.avoid_maneuver_done=true; recovery_ref.tick();
  check(recovery_ref.out.safety==SafetyState::NORMAL && recovery_ref.out.path_source==MGM_SRC_GPS &&
    recovery_ref.out.avoid==AvoidState::GPS_RETURN,
    "completed maneuver can use recovered GPS within avoidance");
  Run hard_stop; hard_stop.tick(); hard_stop.st.params.escape_after_cycles=1;
  hard_stop.s.auto_estop=true; hard_stop.s.external_stop=true; hard_stop.tick();
  check(hard_stop.out.v_ref==0 && hard_stop.out.safety==SafetyState::SAFE_STOP,
    "external stop prevents recovery entry even during real LiDAR estop");
  Run cancel; cancel.mission(); cancel.s.gps_at_end=true; cancel.tick();
  check(cancel.out.top==TopState::FINISH && cancel.out.mission_cancel && cancel.out.v_ref==0,
    "FINISH cancels active mission without claiming completion");
  check(!cancel.st.managers.mission_completed[0],"cancel is not a mission completion event");
  cancel.tick(); check(!cancel.out.mission_cancel,"mission cancel is a single event");
  Run reset_mission; reset_mission.mission(); reset_mission.s.new_session=true; reset_mission.tick();
  check(reset_mission.out.mission_cancel && reset_mission.out.mission==MissionState::MISSION_IDLE,
    "new session cancels previous module before fresh triggers");
  Run invalidspeed; invalidspeed.redline(); invalidspeed.s.vehicle_speed_valid=false; invalidspeed.tick();
  check(invalidspeed.out.safety==SafetyState::SAFE_STOP && invalidspeed.out.v_ref==0,
    "traffic cannot integrate target speed when vehicle feedback is unavailable");

}
}
int main()
{
  navigation(); avoidance(); signal(); mission(); safety();
  std::printf("manager_state_test: %d checks, %d failures (requirements 1..30)\n",checks,failures);
  return failures ? 1 : 0;
}
