// Third revision: tests 1..25, alongside (not replacing) the original 30 cases.
#include "manager_test_fixture.hpp"
using namespace manager_test;
namespace
{
void navigation_zones()
{
  Run line; line.gps_zone(true); line.tick();
  check(line.out.nav==NavState::GPS_ONLY_NAV,"Z1: LINE -> GPS_ONLY_NAV on spatial entry");
  check(line.out.zones.contexts[1].zone_entered,"Z1: entry edge exists on first membership");
  line.tick(50);
  check(line.out.nav==NavState::GPS_ONLY_NAV && !line.out.zones.contexts[1].zone_entered,
    "Z3: high LINE confidence for 50 cycles cannot leave GPS Zone");
  line.gps_zone(false); line.tick();
  check(line.out.zones.contexts[1].zone_exited,"Z4: valid spatial exit produces exit edge");
  check(line.out.nav==NavState::LINE,"Z5: shared reselect chooses ready LINE");
  line.tick(); check(!line.out.zones.contexts[1].zone_exited,"exit is one cycle, not a level");
  Run backup; backup.s.camera_line_valid=false; backup.tick(); backup.gps_zone(true); backup.tick();
  check(backup.out.nav==NavState::GPS_ONLY_NAV,"Z2: GPS_BACKUP -> GPS_ONLY_NAV on entry");
  backup.gps_zone(false); backup.tick();
  check(backup.out.nav==NavState::GPS_BACKUP,"Z6: exit with LINE unready chooses usable GPS");
  Run lost; lost.gps_zone(true); lost.tick(); lost.s.gps_valid=false;
  lost.s.zones.zone_valid=false; lost.s.zones.count=0; lost.tick(50);
  check(lost.out.safety==SafetyState::SAFE_STOP && lost.out.v_ref==0,"Z7: GPS loss in GPS_ONLY_NAV stops");
  check(lost.out.zones.contexts[1].in_zone && !lost.out.zones.contexts[1].zone_exited &&
    !lost.out.zones.selected.zone_valid && lost.out.nav==NavState::GPS_ONLY_NAV,
    "Z8: unknown zone on GPS loss retains last membership and no exit edge");
  lost.s.gps_valid=true; lost.s.zones.zone_valid=true; lost.gps_zone(true); lost.tick();
  check(!lost.out.zones.contexts[1].zone_entered && lost.out.nav==NavState::GPS_ONLY_NAV,
    "GPS recovery in same Zone does not fabricate another entry");
  lost.s.zones.observations[1]=lost.s.zones.observations[0]; lost.s.zones.count=2; lost.tick();
  check(!lost.out.zones.selected.zone_valid && lost.out.zones.in_gps_only_zone,
    "duplicate IDs invalidate snapshot without erasing last GPS context");
  Run legacy; legacy.s.gps_gps_only_zone=true; legacy.s.gps_parking_zone=true; legacy.s.parking_space_found=true;
  legacy.tick(); check(legacy.out.nav==NavState::LINE && legacy.out.mission==MissionState::MISSION_IDLE,
    "legacy boolean/selected waypoint inputs cannot replace typed Zone membership");
}
void missions()
{
  Run r; r.mission();
  check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.mission_start,"Z9: first Mission Zone preparation ready starts mission");
  int repeats=0; for(int i=0;i<50;++i) {r.tick(); repeats+=r.out.mission_start;}
  check(repeats==0,"Z10: remaining inside cannot repeat start");
  r.s.parking_updated=true; r.tick(); r.s.parking_done=true; r.tick();
  check(r.st.managers.mission_completed[0] && r.out.mission==MissionState::MISSION_IDLE,"Z11: module done records Mission completion");
  r.tick(50); check(!r.out.mission_start && r.out.mission==MissionState::MISSION_IDLE,"Z12: done while still inside does not restart");
  r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false); r.tick(); r.mission();
  check(!r.out.mission_start,"completed mission also cannot restart on reentry");
  r.zone(11,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0); r.tick();
  check(!r.out.mission_start,"different Zone referencing same completed Mission is suppressed");
  r.zone(20,ZoneType::MISSION_ZONE,MissionType::PARALLEL_PARKING,1); r.tick(); r.ready();
  check(r.out.mission_start && r.out.active_mission_id==1,"different Mission of the same session can start");
  Run normal; normal.mission(); normal.s.parking_updated=true; normal.tick(50);
  normal.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false); normal.tick();
  check(normal.out.mission==MissionState::MISSION_ACTIVE,"Z17: Mission Zone exit does not terminate active module");
  normal.s.parking_done=true; normal.tick();
  check(normal.out.mission==MissionState::MISSION_IDLE,"Z18: only existing module done ends mission");
  check(normal.out.nav==NavState::LINE,"Z14: mission end in normal Zone selects ready LINE");
  Run low; low.s.camera_line_valid=false; low.mission(); low.s.parking_updated=true; low.tick();
  low.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false); low.s.parking_done=true; low.tick();
  check(low.out.nav==NavState::GPS_BACKUP,"Z15: mission end with LINE unready selects GPS_BACKUP");
  Run overlap; overlap.tick(50); overlap.obstacle(); overlap.gps_zone(true); overlap.mission();
  check(overlap.out.mission_start && overlap.out.path_source==MGM_SRC_PARKING &&
    overlap.out.avoid==AvoidState::INACTIVE,"Z16: Mission ready preempts running avoidance");
  check(overlap.out.zones.selected.zone_type==ZoneType::MISSION_ZONE && overlap.out.zones.in_gps_only_zone,
    "Mission display priority retains overlapping GPS-only context");
  overlap.s.parking_updated=true; overlap.s.auto_estop=true; overlap.tick();
  check(overlap.out.path_source==MGM_SRC_PARKING && overlap.out.avoid==AvoidState::INACTIVE,
    "Z19: ordinary avoidance never overwrites mission reference");
  check(overlap.out.safety==SafetyState::NORMAL && overlap.out.v_ref<0,
    "Z20: mission masks LiDAR AUTO_ESTOP, module speed owns output");
  overlap.s.gps_valid=false; overlap.s.zones.zone_valid=false; overlap.tick();
  check(overlap.out.path_source==MGM_SRC_PARKING && overlap.out.v_ref<0,
    "active Mission is independent of GPS navigation validity even in overlap");
  overlap.s.external_stop=true; overlap.tick();
  check(overlap.out.immediate_stop && overlap.out.v_ref==0,"Mission still honors external/operator stop");
  overlap.s.external_stop=false; overlap.s.autonomous_enabled=false; overlap.tick();
  check(overlap.out.v_ref==0,"Mission still honors autonomous disable");
  overlap.s.autonomous_enabled=true; overlap.s.parking_done=true; overlap.s.auto_estop=false; overlap.tick();
  check(overlap.out.nav==NavState::GPS_ONLY_NAV && overlap.out.safety==SafetyState::SAFE_STOP,
    "Z13: mission end restores retained GPS Zone; GPS still absent means stop");
  overlap.s.gps_valid=true; overlap.s.zones.zone_valid=true; overlap.s.avoid_obstacle_detected=false; overlap.tick();
  check(overlap.out.nav==NavState::GPS_ONLY_NAV,"Z13: mission end reevaluates current GPS-only context");
  Run invalid; invalid.zone(4,ZoneType::MISSION_ZONE,MissionType::NONE,4); invalid.tick();
  check(invalid.out.mission==MissionState::MISSION_IDLE,"invalid mission_type cannot trigger a mission");
  Run same_type; same_type.mission(); same_type.s.parking_updated=true; same_type.tick();
  same_type.s.parking_done=true; same_type.tick();
  same_type.zone(12,ZoneType::MISSION_ZONE,MissionType::T_PARKING,2); same_type.tick(); same_type.ready();
  check(same_type.out.mission_start && same_type.out.active_mission_id==2,
    "completion memory belongs to Mission ID, not parking type or Zone ID");
  Run session; session.mission(); session.s.parking_updated=true; session.tick();
  session.s.parking_done=true; session.tick(); session.s.new_session=true; session.tick();
  check(!session.st.managers.mission_completed[0],"only explicit new session resets completion memory");
  session.s.new_session=false; session.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false); session.tick();
  session.mission(); check(session.out.mission_start,"Mission can execute on entry in new session");
}
void authority_and_gaps()
{
  Run both; both.gps_zone(true); both.obstacle();
  check(both.out.nav==NavState::GPS_ONLY_NAV && both.out.path_source==MGM_SRC_AVOID,
    "Z21: GPS-only navigation allows avoidance reference override");
  both.redline(); both.s.traffic_stopline_detected=false; both.tick();
  both.s.vehicle_speed=1.f; both.tick(60);
  check(both.out.nav==NavState::GPS_ONLY_NAV && both.out.path_source==MGM_SRC_AVOID &&
    both.out.speed_owner==SpeedOwner::TRAFFIC && both.out.v_ref==0,
    "Z22: GPS-only + avoidance + approach yields one avoid reference and traffic stop");
  Run clear; clear.obstacle();
  clear.s.avoid_obstacle_detected=false; clear.s.avoid_path.n=0;
  clear.s.camera_line_valid=false; clear.s.vehicle_speed=1.f; clear.tick();
  check(clear.out.avoid==AvoidState::INACTIVE && clear.out.reference_available &&
    clear.out.path_source==MGM_SRC_GPS && clear.out.v_ref>0,
    "Z23: empty avoidance with no obstacle and no LINE uses live GPS");
  check(near(clear.out.ref_points[0].y,clear.s.gps_path.pts[0].y),
    "Z23: output uses GPS geometry rather than holding the old avoidance point");
  clear.tick(199);check(clear.out.path_source==MGM_SRC_GPS && clear.out.v_ref>0,
    "Z23: GPS remains available without maneuver completion");
  clear.s.avoid_maneuver_done=true;clear.tick();
  check(clear.out.avoid==AvoidState::INACTIVE && clear.out.path_source==MGM_SRC_GPS,
    "Z23: late producer completion does not restart an old avoidance episode");
  Run empty; empty.s.camera_line_valid=empty.s.gps_valid=false; empty.s.avoid_path.n=0; empty.tick();
  check(empty.out.path_source==MGM_SRC_AVOID && empty.out.n_points==1 &&
    empty.out.ref_points[0].x==0 && empty.out.ref_points[0].y==0 && empty.out.v_ref==0 &&
    !empty.out.reference_available,"Z24 revision 4: initial zero ref is invalid and cannot accompany positive speed");
  Run prior; prior.tick(); const auto nav=prior.out;
  prior.s.camera_line_valid=prior.s.gps_valid=false; prior.s.avoid_path.n=0; prior.tick();
  check(prior.out.path_source==MGM_SRC_AVOID && near(prior.out.ref_points[0].y,nav.ref_points[0].y) &&
    prior.out.n_points==nav.n_points && prior.out.v_ref==0,
    "Z24 revision 4: previous geometry held only with stop; no invented straight ref");
}
void recovery()
{
  Run off; off.tick(); off.s.auto_estop=true; off.tick(1000);
  check(off.out.safety==SafetyState::AUTO_ESTOP && off.st.escape_phase==MGM_ESCAPE_NONE,
    "Z25 A: existing default escape_after_cycles=0 disables recovery");
  off.st.params.escape_after_cycles=1000; off.s.estop_rear_clear=false; off.tick(1000);
  check(off.st.escape_phase==MGM_ESCAPE_NONE,
    "Z25 B: even with configured delay, absent rear_clear prevents entry");
  off.s.estop_rear_clear=true; off.tick();
  check(off.st.escape_phase==MGM_ESCAPE_REVERSING && off.out.path_source==MGM_SRC_ESCAPE,
    "Z25: compiled core has existing recovery ref when all existing inputs are satisfied");
  off.gps_zone(true); off.s.auto_estop=false; off.tick();
  check(off.out.nav==NavState::GPS_ONLY_NAV,"recovery end uses common current-Zone reselect");
  Run takeover; takeover.tick(); takeover.st.params.escape_after_cycles=1;
  takeover.s.auto_estop=true; takeover.tick(2); takeover.mission();
  check(takeover.out.mission_start && takeover.out.safety==SafetyState::SAFE_STOP && takeover.out.v_ref==0 &&
    takeover.out.path_source==MGM_SRC_PARKING,"Mission entry cancels Recovery and stops until valid module acknowledgement");
}
}
int main()
{
  navigation_zones(); missions(); authority_and_gaps(); recovery();
  std::printf("zone_manager_test: %d checks, %d failures (revision 3 tests 1..25)\n",checks,failures);
  return failures ? 1 : 0;
}
