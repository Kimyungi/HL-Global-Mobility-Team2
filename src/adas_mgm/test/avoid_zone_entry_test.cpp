#include "manager_test_fixture.hpp"
using namespace manager_test;

void configure(Run & r)
{
  r.st.params.avoid_zone_only = 1;
  r.st.params.safe_stop_all_sensors_only = 1;
  r.s.sensor_alive_mask = 127;
  r.s.gps_heading_valid = r.s.gps_station_error_valid = true;
}

int main()
{
  Run r; configure(r); r.obstacle();
  check(r.out.avoid == AvoidState::INACTIVE, "obstacle outside zone cannot enter AVOID");
  r.s.camera_line_valid = r.s.gps_valid = false; r.tick();
  check(r.out.avoid == AvoidState::INACTIVE, "nav loss outside zone cannot enter AVOID");
  r.s.camera_line_valid = r.s.gps_valid = true;
  r.s.avoid_obstacle_detected = false; r.s.gps_avoid_zone = true; r.tick();
  check(r.out.avoid == AvoidState::AVOID_ACTIVE && r.out.path_source == MGM_SRC_AVOID,
    "zone entry selects AVOID before obstacle detection");
  r.st.params.avoid_max_cycles = 2; r.s.avoid_maneuver_done = true; r.tick(20);
  check(r.out.avoid == AvoidState::AVOID_ACTIVE, "old done and timeout cannot finish an unstarted maneuver");
  r.s.avoid_path.n = 0; r.tick();
  check(r.out.avoid == AvoidState::AVOID_ACTIVE && r.out.reference_motion_blocked && r.out.v_ref == 0,
    "missing avoidance reference inside zone cannot hand off to healthy GPS or LINE");
  r.s.avoid_path.n = 1; r.redline(); r.s.traffic_red_active = false; r.tick();
  check(r.out.avoid == AvoidState::AVOID_ACTIVE, "signal release preserves zone ownership");
  r.s.gps_valid = false; r.s.gps_avoid_zone = false; r.s.lidar_valid = false; r.tick();
  check(r.st.managers.avoid_zone_inside && r.out.avoid == AvoidState::AVOID_ACTIVE &&
    r.out.reference_motion_blocked, "sensor loss does not certify zone exit");
  r.s.gps_valid = r.s.lidar_valid = true; r.tick();
  check(r.out.avoid == AvoidState::AVOID_ACTIVE, "geographic exit cannot substitute for waypoint return");
  r.s.gps_avoid_zone = true; r.obstacle(); r.s.avoid_obstacle_detected = false; r.tick();
  check(r.out.avoid == AvoidState::GPS_RETURN, "completed maneuver begins GPS alignment even inside marker region");
  r.s.gps_cross_track = .2f; r.tick(20);
  check(r.out.avoid == AvoidState::GPS_RETURN, "zone exit still requires GPS alignment");
  r.s.gps_cross_track = 0; r.tick();
  check(r.out.avoid == AvoidState::INACTIVE, "alignment releases zone episode");
  r.tick(20);
  check(r.out.avoid == AvoidState::INACTIVE, "same consumed marker cannot immediately reactivate avoidance");
  r.obstacle(); check(r.out.avoid == AvoidState::INACTIVE, "completed marker stays consumed with a new detection");

  Run pending; configure(pending); pending.s.gps_avoid_zone = true; pending.obstacle();
  pending.s.gps_avoid_zone = pending.s.avoid_obstacle_detected = false;
  pending.st.params.avoid_max_cycles = 1; pending.tick(20);
  check(pending.out.avoid == AvoidState::AVOID_ACTIVE, "zone exit cannot truncate an unfinished maneuver");
  pending.s.avoid_path.n = 0; pending.tick();
  check(pending.out.reference_motion_blocked && pending.out.avoid == AvoidState::AVOID_ACTIVE,
    "failed return path after exit cannot bypass the pending maneuver");
  pending.s.avoid_path.n = 1; pending.s.avoid_maneuver_done = true; pending.tick();
  check(pending.out.avoid == AvoidState::GPS_RETURN, "completed maneuver outside zone permits GPS return");
  pending.tick(); pending.s.gps_avoid_zone = true; pending.tick();
  check(pending.out.avoid == AvoidState::INACTIVE, "finished episode remains consumed");
  pending.s.gps_avoid_zone = false; pending.tick(); pending.s.gps_avoid_zone = true; pending.tick();
  check(pending.out.avoid == AvoidState::AVOID_ACTIVE, "a separately confirmed zone entry can start the next episode");

  Run filtered; configure(filtered);
  filtered.st.params.zone_enter_confirm_samples = filtered.st.params.zone_exit_confirm_samples = 3;
  filtered.s.gps_avoid_zone = true; filtered.tick();
  for (int i=0; i<20; ++i) {filtered.out = mgm_step(filtered.s, filtered.st);}
  check(filtered.out.avoid == AvoidState::INACTIVE, "republished GPS generation counts once");
  filtered.tick(2);
  check(filtered.out.avoid == AvoidState::AVOID_ACTIVE, "three independent fixes confirm entry");
  filtered.s.gps_avoid_zone = false; filtered.tick(2);
  check(filtered.out.avoid == AvoidState::AVOID_ACTIVE, "exit debounce retains ownership");
  filtered.s.gps_avoid_zone = true; filtered.tick(); filtered.s.gps_avoid_zone = false; filtered.tick(2);
  check(filtered.out.avoid == AvoidState::AVOID_ACTIVE, "boundary chatter restarts exit count");
  filtered.tick(); check(!filtered.st.managers.avoid_zone_inside && filtered.out.avoid == AvoidState::AVOID_ACTIVE,
    "three outside fixes clear membership but not an unfinished episode");
  filtered.s.new_session = true; filtered.tick();
  check(filtered.out.avoid == AvoidState::INACTIVE && !filtered.st.managers.avoid_zone_inside,
    "session reset discards zone memory");

  Run mission; configure(mission); mission.s.gps_avoid_zone = true; mission.tick(); mission.mission();
  check(mission.out.mission == MissionState::MISSION_ACTIVE && mission.out.avoid == AvoidState::INACTIVE,
    "parking retains priority over avoidance zone");
  Run disabled; configure(disabled); disabled.st.params.avoidance_enabled = 0;
  disabled.s.gps_avoid_zone = true; disabled.tick();
  check(disabled.out.avoid == AvoidState::INACTIVE, "explicit disable still overrides zone entry");

  Run reverse; configure(reverse); reverse.st.params.escape_after_cycles = 1;
  reverse.s.vehicle_speed = 1; reverse.tick(2); reverse.st.escape_armed = true;
  reverse.s.auto_estop = true; reverse.tick(4);
  check(reverse.out.safety == SafetyState::REVERSE_RECOVERY && reverse.out.avoid == AvoidState::INACTIVE,
    "independent E-stop recovery outside zone does not enable ordinary AVOID");
  std::printf("avoid_zone_entry_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
