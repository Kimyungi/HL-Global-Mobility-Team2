#include "manager_test_fixture.hpp"
#include <initializer_list>
#include <limits>
using namespace manager_test;
namespace {
void configure(Run & r, bool sequence=true) {
  r.st.params.parking_zone_entry_active=1;
  r.st.params.parking_search_zone_only=1;  // new policy must override an old launch argument
  r.st.params.parking_search_timeout=r.st.params.max_parking_search_distance=-1;
  r.st.params.zone_enter_confirm_samples=r.st.params.zone_exit_confirm_samples=5;
  r.s.lane_path.n=r.s.gps_path.n=r.s.avoid_path.n=r.s.parking_path.n=1;
  r.s.parking_mission_active=false;
  if (sequence) {
    r.st.params.route_sequence_enabled=1;
    r.s.route.enabled=true; r.s.route.sequence_id=11; r.s.route.instance_id=22;
    r.s.route.count=2; r.s.route.required_count=1; r.s.route.required_missions[0]=0;
  }
  r.tick();
}
void start(Run & r, MissionType type=MissionType::T_PARKING) {
  r.zone(10,ZoneType::MISSION_ZONE,type,0); r.tick(4);
  check(r.out.mission==MissionState::MISSION_IDLE,"entry still requires five independent fixes");
  r.tick();
  check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.state==MGM_STATE_PARKING &&
    r.out.mission_start && r.out.mission_prepare,"fifth fix immediately takes Parking authority and requests preparation");
  check(r.out.path_source==MGM_SRC_PARKING && r.out.v_ref==0 && r.out.immediate_stop,
    "entry holds zero with Parking reference ownership, never Nav continuation");
  check(r.out.mission_request.handoff.recorded && !r.out.mission_request.ready.recorded &&
    r.out.parking_calibration==CalibrationState::NOT_REQUIRED,"authority handoff precedes readiness; limits not required");
}
void status(Run & r) {
  r.s.parking_valid=r.s.parking_updated=r.s.parking_search_active=true;
  r.s.parking_request_id=r.out.mission_request.request_id;
  r.s.parking_mission_mode=static_cast<uint8_t>(r.out.mission_request.mission_type);
}
void ready(Run & r) {
  status(r);
  r.s.parking_search_space_found=r.s.parking_preparation_ready=true;
  r.s.parking_preparation_reference=ReferenceSample{static_cast<uint64_t>(r.s.event_time_ns),0,.5f};
  r.s.references[MGM_SRC_PARKING]=r.s.parking_preparation_reference;
  r.tick();
}
void execute(Run & r) {
  ready(r);
  check(r.out.mission_request.preparation_ready && r.out.v_ref==0 &&
    !r.st.managers.mission_feedback_seen,"ready does not substitute for execution acknowledgement");
  r.s.parking_mission_active=true; r.tick();
  check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.v_ref<0 && r.out.n_points==1,
    "matching execution ack and one-point parking path allow maneuver");
}
void endpoint(Run & r) {
  r.s.gps_at_end=true; ++r.s.references[MGM_SRC_GPS].generation; r.tick();
}
}
int main() {
  for (auto type : {MissionType::T_PARKING,MissionType::PARALLEL_PARKING}) {
    Run r;configure(r);start(r,type);const auto id=r.out.mission_request.request_id;
    status(r);r.tick();
    r.zone(10,ZoneType::MISSION_ZONE,type,0,false);r.tick(5);
    r.s.monotonic_ns+=3'600'000'000'000; r.s.vehicle_speed=.4f;r.tick();
    check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.mission_request.request_id==id &&
      !r.out.mission_cancel && !r.out.active_mission_failed && r.out.v_ref==0,
      "confirmed Zone exit, elapsed time and travel never release Parking");
    ready(r);r.s.parking_done=true;r.tick();
    check(!r.out.active_mission_completed && r.out.mission_request.active,"done before execution ack is rejected");
    r.s.parking_done=false;execute(r);
    r.s.parking_done=true;r.s.parking_mission_active=false;r.tick();
    check(r.out.mission==MissionState::MISSION_IDLE && r.out.active_mission_completed &&
      !r.out.active_mission_failed && !r.out.mission_cancel,"current acknowledged done returns navigation with success");
    check(r.out.v_ref==0,"negative parking command is cleared on navigation handoff");
    r.tick();check(r.out.v_ref>0 && r.out.path_source!=MGM_SRC_PARKING,"navigation resumes after completed maneuver");
    r.zone(10,ZoneType::MISSION_ZONE,type,0,true);r.tick(5);
    check(r.out.mission==MissionState::MISSION_IDLE,"completed mission cannot restart on reentry");
  }
  for (bool executing : {false,true}) {
    Run r;configure(r);start(r);if(executing)execute(r);
    r.s.gps_at_end=true;r.tick();
    check(r.out.mission==MissionState::MISSION_ACTIVE,"repeated GPS generation cannot create endpoint");
    r.s.references[MGM_SRC_GPS].age_s=1;endpoint(r);
    check(r.out.mission==MissionState::MISSION_ACTIVE,"stale GPS endpoint cannot cancel Parking");
    r.s.references[MGM_SRC_GPS].age_s=0;r.s.vehicle_speed=.4f;endpoint(r);
    check(r.out.mission==MissionState::MISSION_IDLE && r.out.mission_cancel &&
      r.out.mission_request.cancel_reason==MissionCancelReason::ROUTE_END && r.out.active_mission_failed &&
      !r.out.active_mission_completed,"fresh current endpoint ends preparation or execution as a failed attempt");
    check(r.out.route.phase==RoutePhase::WAIT_STOP && r.out.v_ref==0,"endpoint returns through actual stop gate");
    r.s.vehicle_speed=0;r.tick();
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.route.requested_index==1,"ended mission permits next CSV request");
    status(r);r.s.parking_mission_active=r.s.parking_done=true;r.tick();
    check(!r.out.active_mission_completed && r.out.mission==MissionState::MISSION_IDLE,"late done cannot convert endpoint failure to success");
  }
  {
    Run r;configure(r);start(r);execute(r);r.s.parking_done=true;r.s.vehicle_speed=0;endpoint(r);
    check(r.out.active_mission_completed && !r.out.active_mission_failed && !r.out.mission_cancel,
      "acknowledged done beats simultaneous endpoint");
  }
  {
    Run r;configure(r);start(r);status(r);r.s.parking_preparation_ready=true;
    r.s.parking_preparation_reference=ReferenceSample{1,0,.5f};r.tick();
    check(!r.out.mission_request.preparation_ready,"pre-request preparation generation rejected");
    ready(r);execute(r);r.s.parking_request_id--;r.tick();
    check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.v_ref==0,"wrong request cannot drive or release Parking");
    status(r);r.s.parking_search_active=r.s.parking_mission_active=false;r.tick();
    check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.v_ref==0 && !r.out.mission_cancel,
      "module abort holds Parking instead of returning to navigation");
    r.s.gps_valid=r.s.vehicle_speed_valid=false;r.s.vehicle_speed=std::numeric_limits<float>::quiet_NaN();r.tick();
    check(r.out.mission_request.active && r.out.v_ref==0,"lost GPS/motion do not end the request");
    r.s.external_stop=true;r.tick();
    check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.v_ref==0 &&
      (r.out.safe_stop_reasons&SAFE_STOP_EXTERNAL),"external stop preserves Parking request");
    r.s.mission_cancel_requested=true;r.tick();
    check(r.out.mission_cancel && r.out.mission_request.cancel_reason==MissionCancelReason::EXPLICIT,
      "explicit operator cancellation remains available");
  }
  {
    Run r;configure(r,false);start(r);endpoint(r);
    check(r.out.top==TopState::FINISH && r.out.active_mission_failed && r.out.mission_cancel &&
      r.out.mission_request.cancel_reason==MissionCancelReason::ROUTE_END,"single CSV endpoint ends Parking then finishes");
  }
  {
    Run r;configure(r);start(r);const auto id=r.out.mission_request.request_id;
    r.s.new_session=true;r.tick();
    check(r.out.mission_cancel && r.out.mission_request.cancel_reason==MissionCancelReason::SESSION_RESET &&
      r.out.mission_request.request_id==id,"session reset cancels old module request");
  }
  std::printf("parking_entry_test: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
