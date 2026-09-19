#include "core/legacy_state_ids.hpp"
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
    !r.out.mission_start && r.out.mission_prepare,"fifth fix enters Parking search and requests preparation");
  check(r.out.path_source==MGM_SRC_GPS && r.out.v_ref==0 &&
    r.out.n_points==1 && near(r.out.ref_points[0].y,r.s.gps_path.pts[0].y),
    "Parking entry stops immediately while retaining the GPS point");
  check(!r.out.mission_request.handoff.recorded && !r.out.mission_request.ready.recorded &&
    r.out.parking_calibration==CalibrationState::NOT_REQUIRED,"handoff waits for readiness; limits not required");
  r.s.parking_valid=r.s.parking_updated=r.s.parking_search_active=true;
  r.s.parking_request_id=r.out.mission_request.request_id;
  r.s.parking_mission_mode=static_cast<uint8_t>(type);
  r.s.parking_preparation_ready=true;
  r.s.parking_preparation_reference=ReferenceSample{static_cast<uint64_t>(r.s.event_time_ns),0,.5f};
  r.s.parking_wall_acquisition_complete=false; r.tick(4);
  check(r.out.v_ref==0 && !r.out.mission_request.preparation_ready && !r.out.mission_start,
    "early planner readiness cannot skip collection or release the entry stop");
  r.s.parking_preparation_ready=false;
  r.s.parking_wall_acquisition_complete=true; ++r.s.parking_request_id; r.tick();
  check(r.out.v_ref==0,"another request's completion cannot release stop");
  --r.s.parking_request_id; r.tick();
  check(r.out.v_ref>0 && r.out.path_source==MGM_SRC_GPS && !r.out.mission_request.preparation_ready,
    "five collected frames resume GPS before a parking plan is ready");
  r.s.parking_updated=false; r.tick();
  check(r.out.v_ref>0,"fresh completion remains usable between status publications");
  r.s.parking_valid=false; r.tick();
  check(r.out.v_ref==0,"lost status stops GPS search");
  r.s.parking_valid=r.s.parking_updated=true; r.tick();
}
void status(Run & r) {
  r.s.parking_valid=r.s.parking_updated=r.s.parking_search_active=true;
  r.s.parking_request_id=r.out.mission_request.request_id;
  r.s.parking_mission_mode=static_cast<uint8_t>(r.out.mission_request.mission_type);
}
void ready(Run & r) {
  const bool first=!r.out.mission_request.preparation_ready;
  status(r);
  r.s.parking_search_space_found=r.s.parking_preparation_ready=true;
  r.s.parking_preparation_reference=ReferenceSample{static_cast<uint64_t>(r.s.event_time_ns),0,.5f};
  r.s.references[MGM_SRC_PARKING]=r.s.parking_preparation_reference;
  r.tick();
  if (first) {
    check(r.out.mission_start && r.out.mission_request.handoff.recorded &&
      r.out.mission_request.handoff.time_ns==r.out.mission_request.ready.time_ns &&
      r.out.path_source==MGM_SRC_PARKING && r.out.v_ref==0,
      "fresh ready hands off once and stops until execution acknowledgement");
  }
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
      !r.out.mission_cancel && !r.out.active_mission_failed && r.out.v_ref>0 && r.out.path_source==MGM_SRC_GPS,
      "confirmed Zone exit, elapsed time and travel keep GPS search until this CSV endpoint");
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
    // Parallel parking retains the historical endpoint-cancel policy.
    Run r;configure(r);start(r,MissionType::PARALLEL_PARKING);if(executing)execute(r);
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
    r.s.route.index=r.out.route.requested_index;
    r.s.route.acknowledged_request=r.out.route.request_id;
    r.s.route.required_count=0;r.s.gps_at_end=false;
    r.s.gps_path.pts[0]=CorePoint{2.5f,-.7f,.1f,.05f};
    r.tick();
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.v_ref==0,
      "next CSV metadata alone cannot reuse the old reference generation");
    ++r.s.references[MGM_SRC_GPS].generation;r.tick();
    check(r.out.route.index==1 && r.out.route.changed && r.out.v_ref==0,
      "fresh next CSV ack changes route automatically with a stopped handoff tick");
    r.tick();
    check(r.out.route.phase==RoutePhase::RUNNING && r.out.mission==MissionState::MISSION_IDLE &&
      r.out.path_source==MGM_SRC_GPS && r.out.v_ref>0 && near(r.out.ref_points[0].y,-.7f),
      "navigation automatically resumes the next CSV point after unsuccessful Parking");
  }
  {
    Run r;configure(r);start(r);
    r.s.parking_valid=false;r.s.parking_path.n=0;r.tick(60);
    check(r.out.mission_request.active && r.out.path_source==MGM_SRC_GPS && r.out.v_ref==0 &&
      r.out.speed_owner==SpeedOwner::MISSION && !r.out.safe_stop_reasons,
      "absent Parking status stops search without returning to a high-confidence LINE");
    r.s.gps_valid=false;r.tick();
    check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.path_source==MGM_SRC_GPS &&
      r.out.v_ref==0 && (r.out.safe_stop_reasons&SAFE_STOP_REFERENCE_INVALID),
      "GPS loss during search stops without LINE or LiDAR fallback");
    status(r);
    r.s.gps_valid=true;r.s.avoid_obstacle_detected=r.s.avoid_avoidable=true;r.tick();
    check(r.out.v_ref>0 && r.out.path_source==MGM_SRC_GPS && r.out.avoid==AvoidState::INACTIVE,
      "GPS recovery resumes search; ordinary Avoidance cannot take over the search route");
    r.s.auto_estop=true;r.tick();
    check(r.out.v_ref>0 && r.out.safety!=legacy::AUTO_ESTOP && r.out.avoid==AvoidState::INACTIVE,
      "Parking search masks LiDAR E-stop and ordinary avoidance before readiness");
    r.s.auto_estop=false;r.s.external_stop=true;r.tick();
    check(r.out.v_ref==0 && (r.out.safe_stop_reasons&SAFE_STOP_EXTERNAL),"external stop applies during GPS search");
    r.s.external_stop=false;r.s.traffic_fail_safe_stop=true;r.tick();
    check(r.out.v_ref==0 && (r.out.safe_stop_reasons&SAFE_STOP_TRAFFIC_INPUT),"traffic input fault applies during GPS search");
    r.s.traffic_fail_safe_stop=false;r.redline();
    r.s.traffic_stopline_detected=false;r.s.vehicle_speed=0;r.tick();
    r.s.vehicle_speed=1;r.s.monotonic_ns+=600'000'000;r.tick();
    check(r.out.path_source==MGM_SRC_GPS && r.out.v_ref==0 && r.out.speed_owner==SpeedOwner::TRAFFIC,
      "Signal stop profile constrains GPS search while keeping Parking state");
    r.s.vehicle_speed_valid=false;r.tick();
    check(r.out.safe_stop_reasons&SAFE_STOP_VEHICLE_SPEED,"Signal speed freshness guard applies during search");
    r.s.vehicle_speed_valid=true;r.s.traffic_green_active=true;r.s.traffic_red_active=false;r.tick();
    check(r.out.v_ref>0 && r.out.mission_request.active,"green automatically resumes the same GPS search request");
  }
  {
    Run r;configure(r);start(r,MissionType::PARALLEL_PARKING);status(r);
    r.s.parking_preparation_ready=true;
    r.s.parking_preparation_reference=ReferenceSample{static_cast<uint64_t>(r.s.event_time_ns),0,.5f};
    r.s.vehicle_speed=0;endpoint(r);
    check(r.out.mission_cancel && r.out.mission_request.cancel_reason==MissionCancelReason::ROUTE_END &&
      !r.out.mission_start && !r.out.mission_request.handoff.recorded && r.out.route.phase==RoutePhase::WAIT_ACK,
      "CSV endpoint beats simultaneous ready and automatically requests the next route");
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
    Run r;configure(r,false);start(r,MissionType::PARALLEL_PARKING);endpoint(r);
    check(r.out.top==TopState::FINISH && r.out.active_mission_failed && r.out.mission_cancel &&
      r.out.mission_request.cancel_reason==MissionCancelReason::ROUTE_END,"single CSV endpoint ends Parking then finishes");
  }
  {
    Run r;configure(r);start(r);const auto id=r.out.mission_request.request_id;
    r.s.new_session=true;r.tick();
    check(r.out.mission_cancel && r.out.mission_request.cancel_reason==MissionCancelReason::SESSION_RESET &&
      r.out.mission_request.request_id==id,"session reset cancels old module request");
  }
  for (auto type : {MissionType::T_PARKING,MissionType::PARALLEL_PARKING}) {
    Run r;configure(r);r.gps_zone(true);r.tick(5);
    r.s.avoid_obstacle_detected=r.s.avoid_avoidable=true;r.tick();
    check(r.out.avoid==AvoidState::AVOID_ACTIVE && r.out.path_source==MGM_SRC_AVOID && r.out.v_ref>0,
      "enabled avoidance takes control during ordinary GPS-only navigation");
    r.s.auto_estop=true;r.tick();
    check(r.out.v_ref==0 && r.out.safety==legacy::AUTO_ESTOP,
      "ordinary driving honors a fresh LiDAR E-stop");
    start(r,type);
    check(r.out.avoid==AvoidState::INACTIVE && r.out.safety!=legacy::AUTO_ESTOP && r.out.v_ref>0,
      "Zone entry clears existing avoidance and E-stop in the same control tick");
    execute(r);
    check(r.out.avoid==AvoidState::INACTIVE && r.out.safety!=legacy::AUTO_ESTOP && r.out.v_ref<0,
      "Parking execution also masks continuously asserted ordinary danger");
    r.s.external_stop=true;r.tick();
    check(r.out.v_ref==0 && (r.out.safe_stop_reasons&SAFE_STOP_EXTERNAL),
      "Parking masks neither operator nor CAN external stops");
    r.s.external_stop=false;r.s.parking_path.n=0;r.tick();
    check(r.out.v_ref==0 && (r.out.safe_stop_reasons&SAFE_STOP_REFERENCE_INVALID),
      "Parking still stops for an unavailable maneuver reference");
    r.s.parking_path.n=1;r.s.parking_done=true;r.s.parking_mission_active=false;r.tick();
    check(r.out.mission==MissionState::MISSION_IDLE && r.out.v_ref==0 &&
      r.out.safety==legacy::AUTO_ESTOP,
      "leaving Parking immediately restores LiDAR E-stop");
    r.s.auto_estop=false;r.tick(2);
    check(r.out.avoid==AvoidState::AVOID_ACTIVE && r.out.path_source==MGM_SRC_AVOID && r.out.v_ref>0,
      "leaving Parking restores ordinary avoidance without another enable command");
  }
  {
    Run r;configure(r);start(r);r.s.parking_wall_acquisition_complete=false;
    endpoint(r);
    check(r.out.mission_request.active && !r.out.mission_cancel && r.out.v_ref==0,
      "T endpoint while unresolved holds the request; cannot skip parking to route 04");
    r.s.parking_wall_acquisition_complete=true;r.tick();
    check(r.out.v_ref==0 && r.out.mission_request.active,
      "T no-space search also stops at endpoint even if old wall collection was complete");
    execute(r);
    r.st.params.v_base=2.f;r.st.params.v_accel_zone=.5f;
    r.st.params.revised_v2_enabled=1;r.s.sensor_alive_mask=0x77;r.s.gps_fix_quality=4;r.s.gps_accel_zone=true;
    r.s.parking_v_suggest=.55f;r.tick();
    check(near(r.out.v_ref,.55f) && r.out.path_source==MGM_SRC_PARKING,
      "T adapter owns forward route-3 remainder with bounded provider speed");
    r.s.parking_v_suggest=-1.f;r.tick();
    check(near(r.out.v_ref,-1.f) && r.out.mission_request.active &&
      r.out.route.phase!=RoutePhase::WAIT_ACK,"reverse remains 1m/s; route stays 03");
    r.s.parking_v_suggest=0;r.s.vehicle_speed=0;r.tick(1100);
    check(r.out.mission_request.active && !r.out.active_mission_completed &&
      r.out.route.index==0 && r.out.route.phase!=RoutePhase::WAIT_ACK,
      "ten seconds stationary at wall is NOT done and must not select route 04");
    r.s.parking_v_suggest=.55f;r.tick();
    check(r.out.v_ref>0 && r.out.mission_request.active,"forward exit keeps Parking authority");
    r.s.parking_valid=false;r.tick();
    check(r.out.v_ref==0 && !r.out.mission_cancel && r.out.mission_request.active,
      "status loss at endpoint stops without releasing the request");
    status(r);r.s.parking_v_suggest=0;r.s.parking_done=true;r.tick();
    check(r.out.active_mission_completed && r.out.route.phase==RoutePhase::WAIT_ACK &&
      r.out.route.requested_index==1,"only acknowledged exit done and measured stop request route 04");
    r.s.gps_accel_zone=false;
    r.s.route.index=1;r.s.route.acknowledged_request=r.out.route.request_id;
    r.s.route.required_count=0;r.s.gps_at_end=false;
    ++r.s.references[MGM_SRC_GPS].generation;r.tick();r.tick();
    check(r.out.route.index==1 && (r.out.path_source==MGM_SRC_GPS ||
      r.out.path_source==MGM_SRC_LANE) && near(r.out.v_ref,2.f),
      "fresh route-04 acknowledgement resumes 2m/s normal navigation");
  }
  {
    Run r; configure(r); start(r,MissionType::PARALLEL_PARKING); execute(r);
    r.st.params.revised_v2_enabled=1; r.s.revised_v2=true;
    r.s.sensor_alive_mask=0x7f; r.s.gps_fix_quality=4;
    r.st.params.v_base=2.f; r.s.parking_v_suggest=-1.f;
    endpoint(r);
    check(r.out.mission_request.active && !r.out.mission_cancel && near(r.out.v_ref,-1.f),
      "Yongin parallel entry retains CSV mission at route endpoint and module speed");
    r.s.parking_v_suggest=1.f; r.tick();
    check(r.out.mission_request.active && near(r.out.v_ref,1.f),
      "Yongin parallel retrace keeps forward module speed");
    r.s.parking_v_suggest=0; r.s.vehicle_speed=0; r.s.parking_done=true; r.tick();
    check(r.out.active_mission_completed && r.out.route.phase==RoutePhase::WAIT_ACK,
      "Yongin parallel releases next route only after acknowledged exit completion");
  }
  {
    Run r; configure(r); start(r,MissionType::PARALLEL_PARKING);
    r.st.params.revised_v2_enabled=1; r.s.revised_v2=true;
    r.s.sensor_alive_mask=0x7f; r.s.gps_fix_quality=4;
    endpoint(r);
    check(r.out.mission_request.active && !r.out.mission_cancel && r.out.v_ref==0,
      "Yongin parallel without ready stays stopped at route endpoint");
  }
  for (auto type : {MissionType::T_PARKING,MissionType::PARALLEL_PARKING}) {
    Run r; configure(r); start(r,type); execute(r);
    r.st.params.revised_v2_enabled=1; r.s.revised_v2=true;
    r.s.sensor_alive_mask=0x7f; r.s.gps_fix_quality=4;
    r.st.params.v_base=2.f; r.s.parking_v_suggest=1.f;
    r.s.vehicle_speed=.7f; endpoint(r);
    r.s.parking_done=true; r.tick();
    check(r.out.active_mission_completed && r.out.route.phase==RoutePhase::WAIT_ACK &&
      r.out.v_ref>0 && !r.out.immediate_stop,
      "moving CSV parking exit requests next route without an intermediate stop");
    r.tick();
    check(r.out.v_ref>0 && !r.out.immediate_stop,
      "valid reference carries forward motion while next-route acknowledgement arrives");
    r.s.route.index=1; r.s.route.acknowledged_request=r.out.route.request_id;
    r.s.route.required_count=0; r.s.gps_at_end=false;
    ++r.s.references[MGM_SRC_GPS].generation; r.tick();
    check(r.out.route.index==1 && r.out.v_ref>0 && !r.out.immediate_stop,
      "fresh next-route acknowledgement continues forward without stopping");
    r.s.external_stop=true; r.tick();
    check(r.out.v_ref==0,"operator stop still overrides moving parking handoff");
  }
  std::printf("parking_entry_test: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
