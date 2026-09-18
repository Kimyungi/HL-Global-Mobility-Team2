#include "manager_test_fixture.hpp"
using namespace manager_test;
int main()
{
  Run r;
  r.st.params.revised_v2_enabled = 1;
  r.st.params.v_base = 2.f;
  r.s.sensor_alive_mask = 0x77;
  r.s.gps_fix_quality = 4;
  r.tick();
  check(near(r.out.v_ref, 2.f), "outside zone uses session speed");
  r.zone(3, ZoneType::GPS_ONLY_ZONE); r.tick();
  check(near(r.out.v_ref, 1.f) && r.out.path_source == MGM_SRC_GPS,
    "zone 3 caps navigation before red or stopline detection");
  r.tick(60);
  check(near(r.out.v_ref, 1.f), "cap persists throughout zone");
  r.st.params.v_base = .6f; r.tick();
  check(near(r.out.v_ref, .6f), "cap never raises lower speed");
  r.st.params.v_base = 2.f;
  r.s.external_stop = true; r.tick();
  check(r.out.v_ref == 0, "operator stop remains zero");
  r.s.external_stop = false; r.s.traffic_status_fresh = true;
  r.s.traffic_status_stamp_ns = r.s.event_time_ns + 10'000'000;
  r.redline();
  check(near(r.out.v_ref, 1.f), "unseeded stopline approach retains zone cap");
  r.s.traffic_stopline_detected = false; r.tick();
  r.st.traffic_stopline_distance = .9f; r.tick();
  check(r.out.v_ref == 0, "traffic stop remains zero");
  r.s.traffic_red_active = false; r.tick();
  check(near(r.out.v_ref, 1.f), "traffic release inside zone remains capped");
  r.zone(3, ZoneType::GPS_ONLY_ZONE, MissionType::NONE, 0, false); r.tick();
  check(near(r.out.v_ref, 2.f), "zone exit restores session speed");
  r.zone(2, ZoneType::GPS_ONLY_ZONE); r.tick();
  check(near(r.out.v_ref, 2.f), "other GPS-only zone is not capped");
  std::printf("traffic_zone_speed_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
