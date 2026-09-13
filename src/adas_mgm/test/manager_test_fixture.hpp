#ifndef ADAS_MGM__TEST__MANAGER_TEST_FIXTURE_HPP_
#define ADAS_MGM__TEST__MANAGER_TEST_FIXTURE_HPP_
#include "core/mgm_step.hpp"
#include <cmath>
#include <cstdio>
using namespace adas_mgm;
namespace manager_test
{
int failures = 0;
int checks = 0;
void check(bool ok, const char * text)
{
  ++checks;
  if (!ok) {++failures; std::fprintf(stderr, "FAIL: %s\n", text);}
}
bool near(float a, float b) {return std::fabs(a-b) < 1e-5f;}
CoreParams params()
{
  CoreParams p{};
  p.base_state_machine_enabled = 1;
  p.avoidance_enabled = 1;  // existing avoidance scenarios explicitly enable the manager
  p.zone_enter_confirm_samples = p.zone_exit_confirm_samples = 1;  // test-only edge fixtures
  // Synthetic test-only limits; deliberately absent from operating defaults.
  p.parking_search_timeout = 30.; p.max_parking_search_distance = 30.;
  p.lane_conf_exit = .35f; p.lane_conf_return = .7f; p.n_cycles = 50;
  p.avoid_return_hold_cycles = 300;
  p.v_base = 1.f; p.v_accel_zone = 1.f; p.v_avoid = .6f; p.v_narrow = .2f;
  p.ttc_stop = 1.3f; p.a_up = 100.f; p.a_down = 100.f;
  p.wrongway_yaw = 2.1f; p.wrongway_cycles = 50;
  p.traffic_state_enabled = 1; p.traffic_ramp_distance_m = 1.5f; p.traffic_stop_offset = 1.f;
  p.escape_after_cycles = 0; p.escape_max_cycles = 200;
  p.escape_require_rear_clear = 1; p.v_escape = -.3f;
  return p;
}
struct Run
{
  CoreState st{};
  CoreSnapshot s{};
  CoreOutput out{};
  Run()
  {
    mgm_init(st, params());
    s.autonomous_enabled = true;
    s.monotonic_ns = s.event_time_ns = 1'000'000'000;
    s.gps_position_valid = true; s.gps_track_index = 30;
    for (auto & ref : s.references) {ref = ReferenceSample{1, 0.0f, 0.5f};}
    s.zones.zone_valid = true;
    s.camera_line_valid = s.gps_valid = s.lidar_valid = s.parking_valid = true;
    s.lane_confidence = .9f;
    s.lane_path.n = s.gps_path.n = s.avoid_path.n = s.parking_path.n = 1;
    for (int i = 0; i < 1; ++i) {
      s.lane_path.pts[i] = CorePoint{float(i+1), .1f, 0, 0};
      s.gps_path.pts[i] = CorePoint{float(i+1), .2f, 0, 0};
      s.avoid_path.pts[i] = CorePoint{float(i+1), .3f, 0, 0};
      s.parking_path.pts[i] = CorePoint{-float(i+1), .4f, 0, 0};
    }
    s.avoid_ttc = 100.f; s.avoid_v_suggest = .6f; s.parking_v_suggest = -.3f;
    s.vehicle_speed_valid = true;
    s.estop_rear_clear = true;
    s.rear_sensor_valid = true; s.rear_corridor_state = RearCorridorState::CLEAR;
    s.parking_mission_active = true;
    s.parking_mission_mode = static_cast<uint8_t>(MissionType::T_PARKING);
    s.lane_updated = s.gps_updated = s.avoid_updated = true;
  }
  CoreOutput tick(int n = 1) {for (int i=0;i<n;++i) {s.monotonic_ns+=10'000'000; s.event_time_ns+=10'000'000; ++s.zones.generation; out=mgm_step(s,st);} return out;}
  void obstacle() {s.avoid_obstacle_detected=true; s.avoid_avoidable=true; tick();}
  void redline() {s.traffic_red_active=true; s.traffic_stopline_detected=true; tick();}
  void zone(uint8_t id, ZoneType type, MissionType kind=MissionType::NONE,
    uint8_t mission_id=0, bool inside=true)
  {
    for (int i=0; i<s.zones.count; ++i) {
      if (s.zones.observations[i].zone_id==id) {
        s.zones.observations[i] = ZoneObservation{id,type,kind,mission_id,inside,-1.0f}; return;
      }
    }
    s.zones.observations[s.zones.count++] = ZoneObservation{id,type,kind,mission_id,inside,-1.0f};
  }
  void gps_zone(bool inside)
  {
    s.gps_gps_only_zone=inside;  // retained legacy flag is not the new trigger
    zone(1,ZoneType::GPS_ONLY_ZONE,MissionType::NONE,0,inside);
  }
  void ready()
  {
    if (st.managers.mission != MissionState::MISSION_PREPARE) {return;}
    const bool updated=s.parking_updated;
    const auto mode=s.parking_mission_mode;
    s.parking_updated=true;
    s.parking_request_id=st.managers.request.request_id;
    s.parking_mission_mode=static_cast<uint8_t>(st.managers.request.mission_type);
    s.parking_search_active=s.parking_search_space_found=s.parking_preparation_ready=true;
    s.parking_preparation_reference=ReferenceSample{static_cast<uint64_t>(s.event_time_ns),0,.5f};
    s.references[MGM_SRC_PARKING]=s.parking_preparation_reference;
    tick();
    s.parking_updated=updated;
    s.parking_mission_mode=mode;
  }
  void prepare(int id=10)
  {
    zone(static_cast<uint8_t>(id),ZoneType::MISSION_ZONE,
      id==10 ? MissionType::T_PARKING : MissionType::PARALLEL_PARKING,
      id==10 ? 0 : 1);
    tick();
  }
  void mission(int id=10) {prepare(id); ready();}
};
}
#endif
