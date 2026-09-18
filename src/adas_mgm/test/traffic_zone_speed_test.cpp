#include "manager_test_fixture.hpp"
#include "core/zone_step.hpp"
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
  check(!r.st.managers.traffic_zone_active && !in_traffic_zone(r.out.zones),
    "zone 2 cannot enable the zone 3 signal detector");
  r.s.traffic_status_stamp_ns = r.s.event_time_ns + 10'000'000;
  r.redline();
  check(r.out.signal == SignalState::SIGNAL_IDLE && near(r.out.v_ref, 2.f),
    "zone 2 ignores zone 3 red/stopline inputs");
  r.zone(2, ZoneType::GPS_ONLY_ZONE, MissionType::NONE, 0, false);
  r.zone(3, ZoneType::GPS_ONLY_ZONE); r.tick();
  check(r.st.managers.traffic_zone_active && in_traffic_zone(r.out.zones) && near(r.out.v_ref, 1.f),
    "zone 3 signal gate and cap use the same configured identity");
  r.zone(3, ZoneType::GPS_ONLY_ZONE, MissionType::NONE, 0, false);
  r.s.gps_gps_only_zone = true; r.s.gps_accel_zone = true;
  r.st.params.v_accel_zone = .5f; r.tick();
  check(r.out.nav == NavState::GPS_ONLY_NAV && near(r.out.v_ref, .5f),
    "CSV zone 4 selects GPS and its configured approach speed");
  check(!in_traffic_zone(r.out.zones) && !r.st.managers.traffic_zone_active,
    "physical CSV waypoint zone cannot enable the traffic detector");
  std::printf("traffic_zone_speed_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
