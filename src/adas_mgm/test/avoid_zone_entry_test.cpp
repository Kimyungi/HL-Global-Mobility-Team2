#include "core/legacy_state_ids.hpp"
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
  r.s.gps_cross_track = .31f; r.tick(20);
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
  check(reverse.out.safety == legacy::REVERSE_RECOVERY && reverse.out.avoid == AvoidState::INACTIVE,
    "independent E-stop recovery outside zone does not enable ordinary AVOID");
  Run speeds; configure(speeds);
  speeds.st.params.v_base = 2.f; speeds.st.params.v_avoid = 1.f;
  speeds.s.avoid_v_suggest = 2.f;
  speeds.tick(3);
  check(near(speeds.out.v_ref, 2.f), "ordinary navigation uses 2m/s");
  speeds.s.gps_avoid_zone = true; speeds.tick(3);
  check(speeds.out.avoid == AvoidState::AVOID_ACTIVE && near(speeds.out.v_ref, 1.f),
    "zone entry before detection uses 1m/s");
  speeds.obstacle(); speeds.s.avoid_obstacle_detected = false;
  speeds.s.avoid_maneuver_done = true; speeds.s.gps_cross_track = .5f;
  speeds.tick(3);
  check(speeds.out.avoid == AvoidState::GPS_RETURN && speeds.out.path_source == MGM_SRC_GPS &&
    near(speeds.out.v_ref, 1.f), "GPS return keeps zone speed until alignment");
  speeds.s.gps_cross_track = 0.f; speeds.tick(3);
  check(speeds.out.avoid == AvoidState::INACTIVE && near(speeds.out.v_ref, 2.f),
    "completed zone returns to normal 2m/s");
  // Current v09.17 profile: confidence cannot accumulate during either phase.
  Run lane; configure(lane);
  lane.st.params.revised_v2_enabled = 1; lane.s.revised_v2 = true;
  lane.s.gps_fix_quality = 4;
  lane.tick(lane.st.params.n_cycles);
  check(lane.st.lane_high_cnt == 0, "v2 ignores lane confidence in normal driving");
  lane.s.gps_avoid_zone = true; lane.tick();
  check(lane.out.avoid == AvoidState::AVOID_ACTIVE && lane.st.lane_high_cnt == 0 &&
    lane.st.lane_low_cnt == 0, "avoid entry discards prior confidence immediately");
  const auto navigation = lane.out.nav;
  lane.s.lane_confidence = .1f; lane.tick(100);
  check(lane.st.lane_high_cnt == 0 && lane.st.lane_low_cnt == 0 && lane.out.nav == navigation,
    "low lane confidence cannot change counters or background navigation during avoidance");
  lane.s.lane_confidence = .99f; lane.tick(100);
  check(lane.st.lane_high_cnt == 0 && lane.st.lane_low_cnt == 0,
    "high lane confidence is also ignored throughout avoidance");
  lane.obstacle(); lane.s.avoid_obstacle_detected = false;
  lane.s.avoid_maneuver_done = true; lane.s.gps_cross_track = .31f; lane.tick(100);
  check(lane.out.avoid == AvoidState::AVOID_ACTIVE && lane.st.lane_high_cnt == 0 &&
    lane.st.lane_low_cnt == 0, "completed maneuver inside zone keeps AVOID and suppresses lane confidence");
  lane.s.gps_avoid_zone = false; lane.s.gps_cross_track = 0; lane.tick();
  check(lane.out.avoid == AvoidState::INACTIVE && lane.out.nav == NavState::GPS_BACKUP &&
    lane.st.lane_high_cnt == 0, "alignment resumes GPS without reusing avoidance-time confidence");
  lane.tick(lane.st.params.n_cycles - 1);
  check(lane.out.nav == NavState::GPS_BACKUP, "GPS remains selected after avoidance");
  lane.tick();
  check(lane.out.nav == NavState::GPS_BACKUP && lane.st.lane_high_cnt == 0, "lane never resumes after avoidance");
  Run next; configure(next);
  next.gps_zone(true); next.s.gps_track_index = 336;
  next.s.gps_avoid_zone = true; next.obstacle();
  next.s.avoid_maneuver_done = true;
  next.s.gps_cross_track = 1.f; next.s.gps_station_yaw_error = .8f;
  next.tick(3);
  check(next.out.avoid == AvoidState::AVOID_ACTIVE, "starting zone [1] cannot finish avoidance");
  next.gps_zone(false); next.s.gps_avoid_zone = false;
  next.s.gps_track_index = 400; next.tick();
  next.st.params.zone_enter_confirm_samples = 3;
  next.gps_zone(true); next.s.gps_track_index = 463;
  next.s.avoid_maneuver_done = false; next.tick(3);
  check(next.out.avoid == AvoidState::AVOID_ACTIVE,
    "next zone [1] cannot truncate the active avoidance path");
  next.s.avoid_maneuver_done = true; next.s.gps_valid = false; next.tick();
  check(next.out.avoid == AvoidState::AVOID_ACTIVE, "zone return needs a usable GPS reference");
  next.s.gps_valid = true; next.tick();
  check(next.out.avoid == AvoidState::INACTIVE && next.out.path_source == MGM_SRC_GPS,
    "completed path and next zone [1] release even with large cross track and heading errors");
  next.tick(5);
  check(next.out.avoid == AvoidState::INACTIVE, "producer detection latch cannot reactivate completed zone");

  Run debounced; configure(debounced);
  debounced.s.gps_avoid_zone = true; debounced.obstacle();
  debounced.s.avoid_maneuver_done = true;
  debounced.s.gps_track_index += 10;
  debounced.st.params.zone_enter_confirm_samples = 3;
  debounced.zone(3, ZoneType::GPS_ONLY_ZONE); debounced.tick(3);
  check(debounced.out.avoid == AvoidState::AVOID_ACTIVE, "zone [3] is not an avoidance exit");
  debounced.gps_zone(true); debounced.tick();
  for (int i=0; i<20; ++i) {debounced.out = mgm_step(debounced.s, debounced.st);}
  check(debounced.out.avoid == AvoidState::AVOID_ACTIVE, "duplicate fix cannot confirm next [1]");
  debounced.tick();
  check(debounced.out.avoid == AvoidState::AVOID_ACTIVE, "next [1] obeys entry debounce");
  debounced.tick();
  check(debounced.out.avoid == AvoidState::INACTIVE, "three fresh fixes confirm next [1]");

  std::printf("avoid_zone_entry_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
