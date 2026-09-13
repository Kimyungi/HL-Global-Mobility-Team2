#include "manager_test_fixture.hpp"
using namespace manager_test;
int main() {
  Run camera;
  camera.s.start_gate_enabled=true;
  camera.s.camera_available=true;
  camera.s.gps_valid=false; camera.s.gps_path.n=0;
  camera.tick();
  check(camera.out.top==TopState::AUTONOMOUS_DRIVE && camera.out.nav==NavState::LINE && camera.out.v_ref>0,
    "camera-only start does not require GPS or RTK");
  Run gps;
  gps.s.start_gate_enabled=true; gps.s.gps_fixed_ready=true;
  gps.s.camera_line_valid=false; gps.s.lane_path.n=0;
  gps.tick();
  check(gps.out.top==TopState::AUTONOMOUS_DRIVE && gps.out.nav==NavState::GPS_BACKUP && gps.out.v_ref>0,
    "GPS FIXED starts immediately without camera");
  Run missing;
  missing.s.start_gate_enabled=true;
  missing.tick();
  check(missing.out.top==TopState::AUTONOMOUS_ENABLE && missing.out.v_ref==0,
    "no ready sensor cannot accept a bare go request");
  missing.s.camera_available=true; missing.s.camera_line_valid=false; missing.s.lane_path.n=0;
  missing.tick();
  check(missing.out.top==TopState::AUTONOMOUS_DRIVE && missing.out.nav==NavState::GPS_BACKUP,
    "camera frame without detected lane authorizes and selects existing GPS path");
  gps.s.gps_fixed_ready=false; gps.tick();
  check(gps.out.top==TopState::AUTONOMOUS_DRIVE,
    "start requirement is not a continuous FIXED=4 interlock");
  gps.s.autonomous_enabled=false; gps.tick();
  gps.s.autonomous_enabled=true; gps.tick();
  check(gps.out.top==TopState::AUTONOMOUS_ENABLE,
    "stop/restart requires a currently ready sensor again");
  Run fallback;
  fallback.st.managers.nav=NavState::GPS_BACKUP;
  fallback.s.gps_valid=false; fallback.s.lane_confidence=.5f;
  fallback.tick();
  check(fallback.out.nav==NavState::LINE && fallback.out.v_ref>0,
    "GPS loss selects a usable camera immediately without high-confidence dwell");
  Run route;
  route.st.params.route_sequence_enabled=1;
  route.s.route.enabled=true; route.s.route.sequence_id=1; route.s.route.instance_id=2;
  route.s.route.count=2; route.s.route_metadata_fresh=true;
  route.s.gps_valid=false; route.s.gps_path.n=0;
  route.tick();
  check(route.out.route.phase==RoutePhase::RUNNING && route.out.v_ref>0,
    "prepared CSV catalog permits ordinary camera navigation before GPS fix");
  route.s.gps_at_end=true; route.tick(100);
  check(route.out.route.index==0 && !route.out.route.end_reached,
    "camera navigation cannot invent a GPS endpoint or advance a CSV");
  std::printf("startup_navigation_test: %d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
