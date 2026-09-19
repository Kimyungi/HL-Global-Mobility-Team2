#include "manager_test_fixture.hpp"
using namespace manager_test;
struct Halla : Run {
  Halla() {
    auto p = params(); p.revised_v2_enabled = 1; p.halla_stopline_test_enabled = 1;
    p.safe_stop_all_sensors_only = 1;
    mgm_init(st, p); s.gps_fix_quality = 4; s.sensor_alive_mask = 0x77;
    s.start_lidar_ready = true; s.traffic_status_fresh = true;
    zone(3, ZoneType::GPS_ONLY_ZONE); tick();
    s.traffic_status_stamp_ns = s.event_time_ns;
  }
};
int main() {
  Halla r;
  r.s.traffic_stopline_detected = true; r.s.traffic_status_fresh = false; r.tick();
  check(r.out.signal == SignalState::SIGNAL_IDLE, "stale detection cannot enter");
  r.s.traffic_status_fresh = true; r.tick();
  check(r.out.signal == SignalState::APPROACH_STOP_LINE, "stopline enters without red");
  r.tick(299);
  check(r.out.signal == SignalState::APPROACH_STOP_LINE, "2.99 seconds cannot release");
  r.s.traffic_red_active = true; r.tick();
  check(r.out.signal == SignalState::SIGNAL_IDLE, "3 seconds releases even with red");
  check(!r.st.traffic_distance_latched, "release clears distance");
  r.tick(10); check(r.out.signal == SignalState::SIGNAL_IDLE, "same visible line cannot reenter");
  r.s.traffic_stopline_detected = false; r.tick();
  r.s.traffic_stopline_detected = true; r.tick();
  check(r.out.signal == SignalState::APPROACH_STOP_LINE, "new line rearms");
  r.s.traffic_stopline_detected = false; r.tick();
  check(near(r.out.traffic_remaining_m, 1.5f), "loss retains distance seed");
  r.st.traffic_stopline_distance = .9f; r.tick();
  check(r.out.signal == SignalState::STOPPED_WAIT && r.out.v_ref == 0, "existing stop retained");
  r.s.traffic_status_fresh = false; r.tick(298);
  check(r.out.signal == SignalState::SIGNAL_IDLE, "timer starts at detection, independent of red freshness");
  r.zone(3, ZoneType::GPS_ONLY_ZONE, MissionType::NONE, 0, false); r.tick();
  check(!r.st.managers.halla_stopline_wait_clear && r.st.managers.halla_stopline_start_ns == 0,
    "zone exit resets test episode");
  r.s.traffic_status_fresh = true; r.s.traffic_stopline_detected = true; r.tick();
  check(r.out.signal == SignalState::SIGNAL_IDLE, "outside traffic zone ignores line");
  Halla normal; normal.st.params.halla_stopline_test_enabled = 0;
  normal.s.traffic_stopline_detected = true; normal.tick();
  check(normal.out.signal == SignalState::SIGNAL_IDLE, "default retains red condition");
  normal.s.traffic_red_active = true; normal.tick(301);
  check(normal.out.signal == SignalState::APPROACH_STOP_LINE, "default has no timed release");
  normal.s.traffic_red_active = false; normal.tick();
  check(normal.out.signal == SignalState::SIGNAL_IDLE, "default releases on no red");
  std::printf("halla_stopline_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
