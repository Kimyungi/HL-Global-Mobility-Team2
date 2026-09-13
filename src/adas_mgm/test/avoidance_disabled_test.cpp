#include "manager_test_fixture.hpp"
using namespace manager_test;

int main()
{
  Run r; r.st.params.avoidance_enabled = 0;
  r.tick(50);
  r.s.avoid_ttc = .01f; r.s.avoid_path.n = 0;
  r.s.avoid_v_suggest = 0; r.s.gps_avoid_zone = true;
  r.obstacle(); r.tick(400);
  check(r.out.avoid == AvoidState::INACTIVE && r.out.path_source == MGM_SRC_LANE &&
    near(r.out.v_ref, 1.f), "OFF ignores avoidance entry, bad path and avoidance-only TTC stop");
  r.gps_zone(true); r.tick();
  check(r.out.path_source == MGM_SRC_GPS && near(r.out.v_ref, 1.f), "GPS-only still uses GPS");
  r.s.camera_line_valid = r.s.gps_valid = false; r.tick();
  check(r.out.avoid == AvoidState::INACTIVE && r.out.path_source != MGM_SRC_AVOID &&
    r.out.safety == SafetyState::SAFE_STOP && r.out.v_ref == 0,
    "no navigation reference stops instead of using LiDAR fallback");
  r.s.gps_valid = true; r.tick();
  check(r.out.path_source == MGM_SRC_GPS && near(r.out.v_ref, 1.f), "GPS recovery resumes navigation");
  r.s.auto_estop = true; r.tick();
  check(r.out.safety == SafetyState::AUTO_ESTOP && r.out.v_ref == 0,
    "separate LiDAR E-stop remains effective");
  r.s.auto_estop = false; r.s.external_stop = true; r.tick();
  check(r.out.v_ref == 0 && (r.out.safe_stop_reasons & SAFE_STOP_EXTERNAL),
    "external operator/CAN stop remains effective");
  r.s.external_stop = false; r.s.references[MGM_SRC_GPS].age_s = 1; r.tick();
  check(r.out.v_ref == 0 && (r.out.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),
    "reference freshness gate remains effective");
  Run active; active.tick(50); active.obstacle();
  check(active.out.avoid == AvoidState::AVOID_ACTIVE, "ON retains existing avoidance");
  active.st.params.avoidance_enabled = 0; active.st.return_hold_left = 200;
  active.tick();
  check(active.out.avoid == AvoidState::INACTIVE && active.out.path_source == MGM_SRC_LANE &&
    active.st.return_hold_left == 0 && active.st.avoid_ticks == 0 &&
    !active.out.avoid_episode_reference_seen, "OFF clears existing authority and return hold");
  Run clear; clear.tick(50); clear.obstacle(); clear.s.avoid_obstacle_detected = false; clear.tick();
  check(clear.out.avoid == AvoidState::CLEAR_CONFIRM, "fixture enters CLEAR_CONFIRM");
  clear.st.params.avoidance_enabled = 0; clear.tick();
  check(clear.out.avoid == AvoidState::INACTIVE && clear.st.managers.clear_count == 0 &&
    clear.out.path_source == MGM_SRC_LANE, "OFF also clears disappearance confirmation");
  Run signal; signal.st.params.avoidance_enabled = 0; signal.redline();
  signal.s.traffic_stopline_detected = false; signal.tick();
  signal.s.vehicle_speed = 1; signal.tick(60); signal.s.vehicle_speed = 0; signal.tick();
  check(signal.out.signal == SignalState::STOPPED_WAIT && signal.out.v_ref == 0,
    "Signal stopping remains effective");
  Run parking; parking.st.params.avoidance_enabled = 0; parking.mission();
  parking.s.parking_updated = true; parking.tick();
  check(parking.out.mission == MissionState::MISSION_ACTIVE &&
    parking.out.path_source == MGM_SRC_PARKING && parking.out.v_ref < 0,
    "parking retains mission authority");
  std::printf("avoidance_disabled_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
