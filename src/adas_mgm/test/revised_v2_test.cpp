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
    s.vehicle_speed=.03f; tick(); tick(200); s.vehicle_speed=0;
  }
  void traffic(bool red, bool line) {
    s.traffic_red_active = red; s.traffic_stopline_detected = line;
    s.traffic_status_stamp_ns = s.event_time_ns + 10'000'000;
    tick();
  }
  void scan(int sensor, float clearance, uint64_t generation) {
    s.estop_clearance_m[sensor] = clearance;
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
    r.s.autonomous_enabled=false; r.s.vehicle_speed=.03f;
    r.scan(0,.1f,1); r.scan(0,.1f,2); r.scan(0,.1f,3); r.tick(300);
    check(!r.st.managers.estop_motion_seen && !r.st.managers.estop_active,"no authorization cannot start ESTOP timer");
    r.s.autonomous_enabled=true;
    for (float speed : {0.f,.02f,-.03f,std::numeric_limits<float>::quiet_NaN()}) {
      r.s.vehicle_speed=speed; r.tick(250);
      check(!r.st.managers.estop_motion_seen,"zero, threshold, reverse and NaN do not arm");
    }
    r.s.vehicle_speed=.03f; r.s.vehicle_speed_valid=false; r.tick(250);
    check(!r.st.managers.estop_motion_seen,"invalid actual speed does not arm");
    r.s.vehicle_speed_valid=true; r.tick();
    r.s.vehicle_speed=0; r.tick(198); r.scan(0,.1f,4);
    check(!r.st.managers.estop_detection_enabled && !r.st.managers.estop_active,"1.99 seconds still ignores hazards");
    r.scan(0,.1f,5);
    check(r.st.managers.estop_detection_enabled && !r.st.managers.estop_active,"two seconds arms and discards boundary scan");
    r.tick(10); r.scan(0,.1f,6); r.scan(0,.1f,7);
    check(!r.st.managers.estop_active,"preactivation hits cannot contribute");
    r.scan(0,.1f,8); check(r.st.managers.estop_active,"three new hits trigger even when stopped after activation");
  }
  { V2 r; r.s.vehicle_speed=.03f; r.tick(); r.tick(100);
    r.s.autonomous_enabled=false; r.tick(); r.s.autonomous_enabled=true;
    r.s.vehicle_speed=0; r.tick(300);
    check(!r.st.managers.estop_detection_enabled && !r.st.managers.estop_motion_seen,"revoked go resets pending timer");
    r.arm_estop(); r.s.external_stop=true; r.tick(); r.s.external_stop=false; r.tick(300);
    check(!r.st.managers.estop_detection_enabled,"operator stop resets an enabled gate");
    r.arm_estop(); r.s.new_session=true; r.tick(); r.s.new_session=false;
    check(!r.st.managers.estop_detection_enabled,"new session requires fresh forward motion");
  }

  { V2 r;
    check(r.st.params.parking_zone_entry_active == 1,"v09.16 fixes active-entry policy even with old false parameter");
    r.tick(); r.prepare();
    check(r.out.mission == MissionState::MISSION_ACTIVE && r.out.mission_prepare,
      "zone entry starts preparation inside ACTIVE, without a PREPARE state");
    r.tick(5);
    check(r.out.mission == MissionState::MISSION_ACTIVE,
      "missing readiness cannot fall back to removed PREPARE state"); }

  { V2 r; r.s.gps_fix_quality=5; r.s.lane_confidence=.1f; r.tick(50);
    check(r.out.v_ref==0 && r.out.nav==NavState::GPS_BACKUP,"low lane + FLOAT stops");
    r.s.lane_confidence=.5f; r.tick(60); check(r.out.v_ref==0,"mid-confidence cannot bypass recovery hysteresis");
    r.s.lane_confidence=.8f; r.tick(49); check(r.out.v_ref==0,"49 high ticks still waiting");
    r.tick(); check(r.out.nav==NavState::LINE && r.out.v_ref>0,"50 high ticks resume lane without FIXED/go"); }
  { V2 r; r.s.lane_confidence=.1f; r.s.gps_fix_quality=5; r.tick(50);
    r.s.gps_fix_quality=4; r.tick(); check(r.out.v_ref>0 && r.out.path_source==MGM_SRC_GPS,"one FIXED resumes GPS");
    r.s.gps_fix_quality=5; r.s.lane_confidence=.8f; r.tick();
    check(r.out.v_ref>0 && r.out.nav==NavState::LINE,"after GPS recovery, a new GPS loss uses ordinary immediate valid-lane fallback"); }
  { V2 r; r.s.lane_confidence=.1f; r.tick(50);
    r.s.gps_fix_quality=5; r.tick(); r.s.lane_confidence=.5f; r.tick(60);
    check(r.out.v_ref==0,"low lane at GPS quality loss also requires full lane recovery");
  }
  { V2 r; r.st.managers.nav=NavState::GPS_BACKUP; r.s.gps_fix_quality=5; r.tick();
    check(r.out.nav==NavState::LINE && r.out.v_ref>0,"ordinary GPS loss permits immediate valid lane"); }
  { V2 r; r.gps_zone(true); r.s.gps_fix_quality=5; r.tick(60);
    check(r.out.zones.in_gps_only_zone && r.out.v_ref==0,"FLOAT classifies zone but cannot drive even with good lane");
    r.s.gps_fix_quality=4; r.tick(); check(r.out.v_ref>0 && r.out.path_source==MGM_SRC_GPS,"turn zone resumes FIXED"); }
  { V2 r; r.s.auto_estop=true; r.tick(1200); check(r.out.v_ref>0,"legacy independent estop excluded");
    r.s.sensor_alive_mask=8; r.tick(); check(r.out.safety==SafetyState::SAFE_STOP,"rear alone does not satisfy runtime health");
    r.s.sensor_alive_mask=64; r.tick(); check(r.out.v_ref>0,"one monitored sensor recovers"); }
  { V2 r; r.traffic(true,true); check(r.out.signal==SignalState::SIGNAL_IDLE,"traffic outside zone ignored");
    r.gps_zone(true); r.tick(); r.traffic(true,true); r.traffic(true,false);
    check(near(r.out.traffic_remaining_m,1.5f),"stopline loss seeds 1.5");
    r.s.vehicle_speed=1; r.tick(10); check(r.out.traffic_remaining_m<1.5f,"actual speed integrates");
    r.traffic(true,true); r.traffic(true,false); check(near(r.out.traffic_remaining_m,1.5f),"every loss reseeds");
    r.s.vehicle_speed_valid=false; r.tick(10); check(r.out.v_ref>0 && r.out.traffic_remaining_m<1.5f,"last actual speed used without speed-loss stop");
    r.tick(40); check(r.out.v_ref==0 && r.out.signal==SignalState::APPROACH_STOP_LINE,"distance stops without fabricating stopped proof");
    r.s.vehicle_speed_valid=true; r.s.vehicle_speed=0; r.tick(); check(r.out.signal==SignalState::STOPPED_WAIT,"actual zero confirms stopped");
    r.s.traffic_status_fresh=false; r.traffic(false,false); check(r.out.v_ref==0,"stale no-red cannot release established stop");
    r.s.traffic_status_fresh=true; r.traffic(false,false); check(r.out.v_ref>0,"fresh red absence resumes same GPS");
    r.gps_zone(false); r.tick(); check(!r.st.managers.traffic_zone_active && !r.st.traffic_distance_latched,"zone exit resets signal distance"); }
  { V2 r; r.s.vehicle_speed_valid=false; r.gps_zone(true); r.tick(); r.traffic(true,true); r.traffic(true,false); r.tick();
    check(r.out.traffic_remaining_m<1.5f && r.out.v_ref>0,"no actual history integrates previous output command"); }
  { V2 r; r.arm_estop(); r.scan(0,.25f,1); r.tick(20); check(!r.st.managers.estop_active,"held scan never counts as new detection");
    r.scan(1,.15f,1); r.scan(2,.15f,1); check(!r.st.managers.estop_active,"different sensors do not sum");
    r.scan(0,.25f,2); r.s.estop_scans[0].age_s=.36f; r.tick(); r.scan(0,.25f,3);
    check(r.out.safety==SafetyState::ESTOP && r.out.v_ref==0,"third same-sensor hit enters ESTOP; missing executor holds zero");
    r.s.recovery_request_id=1; r.s.recovery_done=true; r.tick(); check(r.st.managers.estop_active,"wrong episode done ignored");
    r.recovered(); check(near(r.out.v_ref,-.2f) && r.out.ref_points[0].x==-1,"executor geometry/speed used without legacy reverse generator");
    r.s.sensor_alive_mask=0; r.tick(); check(r.out.state==MGM_STATE_ESTOP && r.out.v_ref<0 && r.out.safety==SafetyState::ESTOP,"all sensor loss does not interrupt recovery");
    r.s.external_stop=true; r.tick(); check(r.out.v_ref==0,"operator/CAN stop overrides recovery");
    r.s.external_stop=false; r.s.sensor_alive_mask=1; r.recovered(true);
    check(!r.st.managers.estop_active && r.out.nav==NavState::LINE,"completion returns previous state");
    r.scan(0,.2f,4); r.scan(0,.2f,5); r.scan(0,.2f,6); check(!r.st.managers.estop_active,"continuing front condition cannot retrigger");
    r.scan(1,.5f,2); r.scan(1,.1f,3); r.scan(1,.1f,4); r.scan(1,.1f,5);
    check(r.st.managers.estop_active,"another rearmed sensor can trigger"); }
  { V2 r; r.arm_estop(); r.st.managers.mission=MissionState::MISSION_ACTIVE;
    // Preserve an active parking episode while ESTOP owns the output.
    r.st.managers.request.active=true; r.st.managers.request.request_id=10;
    r.st.managers.mission_type=MissionType::T_PARKING;
    r.s.parking_request_id=10;
    r.scan(0,.1f,1); r.scan(0,.1f,2); r.scan(0,.1f,3);
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
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.v_ref>0,"middle CSV needs no stationary feedback; camera continues");
    const auto id=r.out.route.request_id; r.tick(500);
    check(r.out.route.request_id==id && r.out.v_ref>0,"no ACK timeout and no camera stop");
    r.s.route.index=1; r.s.route.acknowledged_request=id; fix(false);
    check(r.out.route.changed && r.out.nav==NavState::LINE && r.out.v_ref>0,"ACK preserves camera ownership");
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
    check(r.out.route.connecting && r.out.nav==NavState::LINE,"CSV connector outside turn zone allows camera");
  }
  std::printf("revised v2: %d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
