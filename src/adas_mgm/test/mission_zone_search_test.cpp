#include "manager_test_fixture.hpp"
#include <initializer_list>
using namespace manager_test;
namespace {
void configure(Run & r) {
  r.st.params.parking_search_zone_only=1;
  r.st.params.parking_search_timeout=r.st.params.max_parking_search_distance=-1;
  r.st.params.zone_enter_confirm_samples=r.st.params.zone_exit_confirm_samples=5;
}
void start(Run & r, MissionType type=MissionType::T_PARKING) {
  r.zone(10,ZoneType::MISSION_ZONE,type,0,true); r.tick(5);
  check(r.out.mission==MissionState::MISSION_PREPARE && r.out.mission_prepare,
    "five independent fixes start PREPARE without numeric limits");
}
void ready_input(Run & r) {
  r.s.parking_updated=true; r.s.parking_request_id=r.out.mission_request.request_id;
  r.s.parking_mission_mode=static_cast<uint8_t>(r.out.mission_request.mission_type);
  r.s.parking_search_active=r.s.parking_preparation_ready=true;
  r.s.parking_preparation_reference=ReferenceSample{static_cast<uint64_t>(r.s.event_time_ns),0,.5f};
  r.s.references[MGM_SRC_PARKING]=r.s.parking_preparation_reference;
}
void fix(Run & r, bool endpoint=false) {
  r.s.gps_at_end=endpoint; ++r.s.references[MGM_SRC_GPS].generation; r.tick();
}
}
int main() {
  for (auto type : {MissionType::T_PARKING, MissionType::PARALLEL_PARKING}) {
    Run r; configure(r); start(r,type);
    check(r.out.parking_calibration==CalibrationState::NOT_REQUIRED,"unused numeric limits reported NOT_REQUIRED");
    r.st.params.parking_search_timeout=.01; r.st.params.max_parking_search_distance=.001;
    r.s.vehicle_speed=.5f; r.s.monotonic_ns+=3'600'000'000'000; r.tick();
    check(r.out.mission==MissionState::MISSION_PREPARE && r.out.mission_request.elapsed_s>3600 &&
      r.out.mission_request.travel_distance>100,"time/distance telemetry never bounds source-Zone search");
    r.zone(10,ZoneType::MISSION_ZONE,type,0,false); r.tick(4);
    check(r.out.mission==MissionState::MISSION_PREPARE,"four outside fixes do not confirm exit");
    ready_input(r); r.tick();
    check(r.out.mission==MissionState::MISSION_IDLE && r.out.mission_cancel &&
      r.out.mission_request.cancel_reason==MissionCancelReason::ZONE_EXIT && r.out.active_mission_failed &&
      !r.out.active_mission_completed,"fifth exit beats simultaneous ready; failure is not success");
    check(r.out.v_ref>0 && r.out.top==TopState::AUTONOMOUS_DRIVE,"failed search resumes normal driving");
    r.zone(10,ZoneType::MISSION_ZONE,type,0,true); r.tick(5);
    check(r.out.mission==MissionState::MISSION_IDLE && r.st.managers.mission_failed[0],"failed ID does not retry on reentry");
    r.s.new_session=true; r.tick(); r.s.new_session=false; r.tick(5);
    check(!r.st.managers.mission_failed[0] && r.out.mission==MissionState::MISSION_IDLE,"session clears failure without starting inside Zone");
    r.zone(10,ZoneType::MISSION_ZONE,type,0,false);r.tick(5);start(r,type);
  }
  {
    Run r;configure(r);start(r); const auto request=r.out.mission_request.request_id;
    r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false);r.tick(3);
    r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,true);r.tick();
    r.zone(20,ZoneType::MISSION_ZONE,MissionType::PARALLEL_PARKING,1,true);r.tick(5);
    r.zone(20,ZoneType::MISSION_ZONE,MissionType::PARALLEL_PARKING,1,false);r.tick(5);
    check(r.out.mission_request.request_id==request && r.out.mission_request.active,"chatter/other Zone exit do not end source request");
    r.s.gps_valid=false;ready_input(r);r.tick(20);
    check(r.out.mission==MissionState::MISSION_PREPARE && !r.out.mission_start && r.out.v_ref==0 &&
      (r.out.safe_stop_reasons&SAFE_STOP_MISSION_ZONE_UNKNOWN),"unknown GPS/Zone holds request and stops, never a false exit or handoff");
    r.s.gps_valid=true;r.s.parking_preparation_ready=false;r.tick();
    check(r.out.mission==MissionState::MISSION_PREPARE && r.out.v_ref>0 &&
      !(r.out.safe_stop_reasons&SAFE_STOP_MISSION_ZONE_UNKNOWN),"inside recovery resumes same search");
    ready_input(r);r.tick();
    check(r.out.mission==MissionState::MISSION_ACTIVE,"fresh readiness inside source Zone takes authority");
    r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false);r.tick(5);
    check(r.out.mission==MissionState::MISSION_ACTIVE && !r.out.active_mission_failed,"ACTIVE maneuver survives Zone exit");
    r.s.parking_done=true;r.tick();
    check(r.out.active_mission_completed && !r.out.active_mission_failed,"normal done stays success");
  }
  {
    Run r;configure(r);r.st.params.route_sequence_enabled=1;
    r.s.route.enabled=true;r.s.route.sequence_id=1;r.s.route.instance_id=2;r.s.route.count=2;
    r.s.route.required_count=1;r.s.route.required_missions[0]=0;
    fix(r);start(r);r.s.external_stop=true;
    r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false);r.tick(5);
    check(r.out.active_mission_failed && r.out.v_ref==0,"exit failure preserves external stop");
    r.s.external_stop=false;fix(r);
    check(r.out.route.index==0 && r.out.route.request_id==0 && r.out.route.phase==RoutePhase::RUNNING && r.out.v_ref>0,
      "failure continues current CSV; no immediate next-CSV command");
    r.s.vehicle_speed=.4f;fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_STOP && r.out.v_ref==0,"CSV endpoint still requires actual stop");
    r.s.vehicle_speed=0;fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.route.requested_index==1,"failure satisfies terminal Mission requirement at endpoint");
    r.s.route.index=1;r.s.route.acknowledged_request=r.out.route.request_id;r.s.route.required_count=0;
    fix(r);fix(r);
    check(r.out.route.index==1 && r.st.managers.mission_failed[0] && !r.st.managers.mission_completed[0],"failure memory survives CSV change");
  }
  std::printf("mission_zone_search_test: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
