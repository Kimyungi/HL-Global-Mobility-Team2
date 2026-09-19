#include "manager_test_fixture.hpp"
#include "src/estop_scan.hpp"
#include <limits>
using namespace manager_test;
struct V2 : Run {
  V2() {
    auto p = params(); p.revised_v2_enabled = 1; p.safe_stop_all_sensors_only = 1;
    p.avoid_zone_only = 1; p.traffic_stop_offset = 1.1f; p.v_base = 2;
    mgm_init(st, p); s.gps_fix_quality = 4; s.sensor_alive_mask = 0x77;
    s.start_lidar_ready = true; s.traffic_status_fresh = true;
  }
  void arm_estop() {
    st.params.estop_station_zone_id=9;
    zone(9, ZoneType::NORMAL_ZONE); tick(5);
  }
  void traffic(bool red, bool line) {
    s.traffic_red_active = red; s.traffic_stopline_detected = line;
    s.traffic_status_stamp_ns = s.event_time_ns + 10'000'000;
    tick();
  }
  void scan(int sensor, float clearance, uint64_t generation) {
    s.estop_clearance_m[sensor] = clearance;
    if (sensor == 0) {s.estop_front_obstacle_width_m=clearance; s.estop_front_clear=clearance==0;}
    s.estop_scans[sensor] = ReferenceSample{generation,0,.35f}; tick();
  }
  void recovered(bool done=false) {
    s.recovery_request_id = st.managers.estop_request_id;
    s.recovery_reference = ReferenceSample{static_cast<uint64_t>(s.event_time_ns+1),0,.35f};
    s.recovery_path.n = 1; s.recovery_path.pts[0] = CorePoint{-1,0,0,0};
    s.recovery_speed = -.2f; s.recovery_done = done; tick();
  }
};
int main() {
  { V2 r;
    r.scan(0,.3f,1);r.scan(0,.3f,2);r.scan(0,.3f,3);
    check(!r.st.managers.estop_active,"unconfigured production station disables detection");
    r.st.params.estop_station_zone_id=9;
    r.scan(0,.3f,4);r.scan(0,.3f,5);r.scan(0,.3f,6);
    check(!r.st.managers.estop_active,"outside configured station cannot enter");
    r.arm_estop();r.s.autonomous_enabled=false;
    r.scan(0,.3f,7);r.scan(0,.3f,8);r.scan(0,.3f,9);
    check(!r.st.managers.estop_active,"authorization required");
  }

  { V2 r;
    check(r.st.params.parking_zone_entry_active == 1,"v09.16 fixes active-entry policy even with old false parameter");
    r.tick(); r.prepare();
    check(r.out.mission == MissionState::MISSION_ACTIVE && r.out.mission_prepare,
      "zone entry starts preparation inside ACTIVE, without a PREPARE state");
    r.tick(5);
    check(r.out.mission == MissionState::MISSION_ACTIVE,
      "missing readiness cannot fall back to removed PREPARE state"); }

  { V2 r;
    r.tick(100);
    check(r.out.nav==NavState::GPS_BACKUP && r.out.path_source==MGM_SRC_GPS && r.out.v_ref>0,
      "normal navigation always selects GPS even with high confidence lane");
    check(!r.out.references[MGM_SRC_LANE].valid && r.st.lane_high_cnt==0 && r.st.lane_low_cnt==0,
      "lane provider and confidence counters disabled");
    for (float confidence : {.1f, .5f, .9f, std::numeric_limits<float>::quiet_NaN()}) {
      r.s.gps_fix_quality=5; r.s.lane_confidence=confidence; r.tick(100);
      check(r.out.nav==NavState::GPS_BACKUP && r.out.v_ref==0,
        "FLOAT never falls back to any camera confidence");
      r.s.gps_fix_quality=4; r.tick();
      check(r.out.path_source==MGM_SRC_GPS && r.out.v_ref>0,
        "FIXED resumes GPS independently of lane confidence");
    }
    r.s.gps_valid=false; r.tick(100);
    check(r.out.v_ref==0 && r.out.path_source!=MGM_SRC_LANE,"GPS outage never selects lane");
  }
  { V2 r; r.s.start_gate_enabled=true; r.s.camera_available=true;
    r.s.gps_fixed_ready=false; r.s.gps_fix_quality=5; r.tick();
    check(r.out.top==TopState::AUTONOMOUS_ENABLE && r.out.v_ref==0,"camera alone cannot authorize v2 start");
    r.s.gps_fixed_ready=true; r.s.gps_fix_quality=4; r.tick();
    check(r.out.top==TopState::AUTONOMOUS_DRIVE && r.out.path_source==MGM_SRC_GPS,
      "GPS FIXED authorizes v2 start");
  }
  { V2 r; r.zone(3, ZoneType::GPS_ONLY_ZONE); r.s.gps_fix_quality=5; r.tick(60);
    check(r.out.zones.in_gps_only_zone && r.out.v_ref==0,"FLOAT classifies zone but cannot drive even with good lane");
    r.s.gps_fix_quality=4; r.tick(); check(r.out.v_ref>0 && r.out.path_source==MGM_SRC_GPS,"turn zone resumes FIXED"); }
  { V2 r; r.s.auto_estop=true; r.tick(1200); check(r.out.v_ref>0,"legacy independent estop excluded");
    r.s.sensor_alive_mask=8; r.tick(); check(r.out.safety==SafetyState::SAFE_STOP,"rear alone does not satisfy runtime health");
    r.s.sensor_alive_mask=64; r.tick(); check(r.out.v_ref>0,"one monitored sensor recovers"); }
  { V2 r; r.traffic(true,true); check(r.out.signal==SignalState::SIGNAL_IDLE,"traffic outside zone ignored");
    r.zone(3, ZoneType::GPS_ONLY_ZONE); r.tick(); r.traffic(true,true); r.traffic(true,false);
    check(near(r.out.traffic_remaining_m,1.5f),"stopline loss seeds 1.5");
    r.s.vehicle_speed=1; r.tick(10); check(r.out.traffic_remaining_m<1.5f,"actual speed integrates");
    r.traffic(true,true); r.traffic(true,false); check(near(r.out.traffic_remaining_m,1.5f),"every loss reseeds");
    r.s.vehicle_speed_valid=false; r.tick(10); check(r.out.v_ref>0 && r.out.traffic_remaining_m<1.5f,"last actual speed used without speed-loss stop");
    r.tick(40); check(r.out.v_ref==0 && r.out.signal==SignalState::APPROACH_STOP_LINE,"distance stops without fabricating stopped proof");
    r.s.vehicle_speed_valid=true; r.s.vehicle_speed=0; r.tick(); check(r.out.signal==SignalState::STOPPED_WAIT,"actual zero confirms stopped");
    r.s.traffic_status_fresh=false; r.traffic(false,false); check(r.out.v_ref==0,"stale no-red cannot release established stop");
    r.s.traffic_status_fresh=true; r.traffic(false,false); check(r.out.v_ref>0,"fresh red absence resumes same GPS");
    r.zone(3, ZoneType::GPS_ONLY_ZONE, MissionType::NONE, 0, false); r.tick(); check(!r.st.managers.traffic_zone_active && !r.st.traffic_distance_latched,"zone exit resets signal distance"); }
  { V2 r; r.s.vehicle_speed_valid=false; r.zone(3, ZoneType::GPS_ONLY_ZONE); r.tick(); r.traffic(true,true); r.traffic(true,false); r.tick();
    check(r.out.traffic_remaining_m<1.5f && r.out.v_ref>0,"no actual history integrates previous output command"); }
  { V2 r; r.arm_estop(); r.scan(0,.18f,1); r.tick(20);
    check(!r.st.managers.estop_active,"held scan never counts twice");
    r.scan(1,.3f,1);r.scan(2,.3f,1);
    check(!r.st.managers.estop_active,"side scans never trigger");
    r.scan(0,.18f,2);r.s.estop_scans[0].age_s=.36f;r.tick();r.scan(0,.18f,3);
    check(r.out.state==MGM_STATE_ESTOP && r.out.v_ref==0,"three front width hits enter stop-only ESTOP");
    r.recovered();check(r.out.v_ref==0,"old recovery reverse ignored");
    r.recovered(true);check(r.st.managers.estop_active,"old recovery done ignored");
    r.s.sensor_alive_mask=0;r.s.estop_scans[0].age_s=.36f;r.tick(100);
    check(r.st.managers.estop_active && r.out.v_ref==0,"stale/all-lost sensors keep stopped");
    r.s.sensor_alive_mask=0x77;r.scan(0,.05f,4);r.scan(0,.05f,5);r.scan(0,.05f,6);
    check(r.st.managers.estop_active,"remaining narrow obstacle prevents clear");
    r.scan(0,0,7);r.tick(20);check(r.st.managers.estop_active,"held clear scan cannot release");
    r.scan(0,0,8);r.s.external_stop=true;r.scan(0,0,9);
    check(r.st.managers.estop_active && r.out.v_ref==0,"operator stop prevents release");
    r.s.external_stop=false;r.scan(0,0,10);r.scan(0,0,11);r.scan(0,0,12);
    check(!r.st.managers.estop_active && r.st.managers.estop_station_completed,
      "three fresh empty scans restore previous navigation");
    r.scan(0,.3f,13);r.scan(0,.3f,14);r.scan(0,.3f,15);
    check(!r.st.managers.estop_active,"station completion prevents reentry");
    r.s.zones.zone_valid=false;r.tick();r.s.external_stop=true;r.tick();
    r.s.external_stop=false;r.s.zones.zone_valid=true;r.tick();
    check(r.st.managers.estop_station_completed,"GPS loss or go cycling cannot rearm");
    r.zone(9,ZoneType::NORMAL_ZONE,MissionType::NONE,0,false);r.tick(5);
    check(!r.st.managers.estop_station_completed,"confirmed station exit rearms");
    r.arm_estop();r.scan(0,.3f,16);r.scan(0,.3f,17);r.scan(0,.3f,18);
    check(r.st.managers.estop_active,"new station visit can trigger again"); }
  { V2 r; r.arm_estop(); r.st.managers.mission=MissionState::MISSION_ACTIVE;
    // Preserve an active parking episode while ESTOP owns the output.
    r.st.managers.request.active=true; r.st.managers.request.request_id=10;
    r.st.managers.mission_type=MissionType::T_PARKING;
    r.s.parking_request_id=10;
    r.scan(0,.2f,1); r.scan(0,.2f,2); r.scan(0,.2f,3);
    check(r.st.managers.estop_active && r.st.managers.estop_return_mission==MissionState::MISSION_ACTIVE,"parking permits ESTOP"); }
  { V2 r; r.st.managers.route.phase=RoutePhase::FAULT; r.tick();
    check(r.out.v_ref==0 && (r.out.safe_stop_reasons & SAFE_STOP_ROUTE_SEQUENCE),"route fault stops even with healthy camera"); }
  { const std::vector<double> mount{.76,0,0,-180,180,0,0,12};
    check(near(body_clearance({.25f},0,1,0,12,mount,.76,.09,.31),.25f),"front distance from bumper");
    const std::vector<double> side{0,.2,90,-180,180,0,0,12};
    check(near(body_clearance({.26f},0,1,0,12,side,.76,.09,.31),.15f),"side distance from body exterior");
    check(std::isnan(body_clearance({NAN},0,1,0,12,side,.76,.09,.31)),"invalid scan never clears counter"); }
  { V2 r;
    r.st.params.route_sequence_enabled=1;
    r.s.route.enabled=true; r.s.route.sequence_id=11; r.s.route.instance_id=22; r.s.route.count=2;
    auto fix = [&](bool end) {++r.s.references[MGM_SRC_GPS].generation; r.s.gps_at_end=end; r.tick();};
    fix(false); r.s.vehicle_speed_valid=false; fix(true);
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.v_ref>0,"middle CSV needs no stationary feedback; GPS continues");
    const auto id=r.out.route.request_id; r.tick(500);
    check(r.out.route.request_id==id && r.out.v_ref>0,"no ACK timeout and no GPS stop");
    r.s.route.index=1; r.s.route.acknowledged_request=id; fix(false);
    check(r.out.route.changed && r.out.nav==NavState::GPS_BACKUP && r.out.v_ref>0,"ACK preserves GPS ownership");
    fix(false); fix(true);
    check(r.out.route.phase==RoutePhase::WAIT_STOP && r.out.v_ref==0,"last CSV commands zero but waits for actual speed");
    r.s.vehicle_speed_valid=true; r.s.vehicle_speed=.5f; fix(true);
    check(r.out.top!=TopState::FINISH,"moving final CSV cannot finish");
    r.s.vehicle_speed=0; fix(true); check(r.out.top==TopState::FINISH,"actual stationary final CSV finishes");
    r.s.autonomous_enabled=false; r.tick(); r.s.autonomous_enabled=true; r.tick();
    check(r.out.top==TopState::FINISH,"go alone cannot reset FINISH");
  }
  { V2 r; r.s.camera_line_valid=false; r.s.lane_path.n=0;
    r.st.params.route_sequence_enabled=1;
    r.s.route.enabled=true; r.s.route.sequence_id=11; r.s.route.instance_id=22; r.s.route.count=2;
    r.tick(); ++r.s.references[MGM_SRC_GPS].generation; r.s.gps_at_end=true; r.tick();
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.v_ref>0,"GPS handoff continues valid old reference");
    r.s.route.index=1; r.s.route.acknowledged_request=r.out.route.request_id;
    r.s.gps_path.n=0; ++r.s.references[MGM_SRC_GPS].generation; r.tick();
    check(r.out.v_ref>0 && r.out.route.phase==RoutePhase::WAIT_ACK,"old fresh cached reference is not a new-route ACK");
    r.tick(51); check(r.out.v_ref==0 && r.out.route.phase==RoutePhase::WAIT_ACK,"expired cache cannot drive or acknowledge");
    r.s.gps_path.n=1; ++r.s.references[MGM_SRC_GPS].generation; r.s.gps_at_end=false; r.tick();
    check(r.out.route.changed && r.out.v_ref>0,"new valid route automatically resumes GPS");
  }
  { V2 r; r.st.params.route_sequence_enabled=1;
    r.s.route.enabled=true; r.s.route.sequence_id=11; r.s.route.instance_id=22; r.s.route.count=2;
    r.s.route.next_connecting=true; r.tick();
    ++r.s.references[MGM_SRC_GPS].generation; r.s.gps_at_end=true; r.tick();
    r.s.route.index=1; r.s.route.connecting=true; r.s.route.next_connecting=false;
    r.s.route.acknowledged_request=r.out.route.request_id; ++r.s.references[MGM_SRC_GPS].generation;
    r.s.gps_at_end=false; r.tick(60);
    check(r.out.route.connecting && r.out.nav==NavState::GPS_BACKUP,"CSV connector uses GPS outside turn zone");
  }
  std::printf("revised v2: %d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
