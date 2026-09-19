#include "core/legacy_state_ids.hpp"
#include "manager_test_fixture.hpp"
#include <limits>
#include <initializer_list>
using namespace manager_test;
namespace
{
void acknowledge_search(Run & r, bool ready=false)
{
  r.s.parking_updated=true;
  r.s.parking_request_id=r.st.managers.request.request_id;
  r.s.parking_mission_mode=static_cast<uint8_t>(r.st.managers.request.mission_type);
  r.s.parking_search_active=true;
  r.s.parking_search_space_found=ready;
  r.s.parking_preparation_ready=ready;
  r.s.parking_preparation_reference=ReferenceSample{
    static_cast<uint64_t>(r.s.event_time_ns),0,.5f};
  r.s.references[MGM_SRC_PARKING]=r.s.parking_preparation_reference;
}
void latch_and_handoff()
{
  Run r; r.tick(50); r.s.parking_valid=false; r.s.parking_path.n=0;
  r.s.gps_x=5.; r.s.gps_y=8.; r.s.gps_track_index=30; r.prepare();
  const auto request=r.out.mission_request;
  check(r.out.mission==legacy::MISSION_PREPARE && r.out.mission_prepare &&
    r.out.path_source==MGM_SRC_LANE && r.out.v_ref>0,"P1/P20: entry prepares while Navigation owns reference/speed");
  check(request.active && request.mission_id==0 && request.source_zone_id==10 &&
    request.mission_type==MissionType::T_PARKING && request.zone_entry.x==5. &&
    request.zone_entry.y==8. && request.zone_entry.track_index==30,"request latches IDs, time and measured start position");
  check(!(r.out.safe_stop_reasons & (SAFE_STOP_REFERENCE_INVALID|SAFE_STOP_MISSION_FEEDBACK)),
    "P6: missing Mission feedback/reference is normal in PREPARE");
  r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false); r.tick();
  check(r.out.mission_request.active && r.out.mission_request.request_id==request.request_id &&
    r.out.mission==legacy::MISSION_PREPARE,"P2: zone exit preserves request");
  r.obstacle();
  check(r.out.mission==legacy::MISSION_PREPARE && r.out.path_source==MGM_SRC_AVOID &&
    r.out.speed_owner==SpeedOwner::AVOIDANCE,"P4: PREPARE allows ordinary avoidance");
  r.s.auto_estop=true; r.tick();
  check(r.out.safety==legacy::AUTO_ESTOP && r.out.v_ref==0 && r.out.mission_request.active,
    "P5: PREPARE keeps LiDAR brake");
  r.s.auto_estop=false; r.redline(); r.s.traffic_stopline_detected=false; r.tick();
  r.s.vehicle_speed=1.f; r.tick(60);
  check(r.out.mission==legacy::MISSION_PREPARE && r.out.path_source==MGM_SRC_AVOID &&
    r.out.speed_owner==SpeedOwner::TRAFFIC && r.out.v_ref==0,
    "P19: PREPARE + Avoidance + Traffic keeps lateral and speed arbitration");
  r.s.parking_valid=true; acknowledge_search(r); r.tick();
  check(r.out.mission_request.search_start.recorded && r.out.mission_request.search_acknowledged,
    "search start acknowledgement has separate calibration observation");
  r.s.parking_search_space_found=true; r.tick();
  check(r.out.mission==legacy::MISSION_PREPARE && r.out.mission_request.space.recorded,
    "space/plan alone waits for existing localization readiness");
  acknowledge_search(r,true); r.tick();
  check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.path_source==MGM_SRC_PARKING &&
    r.out.mission_start && !r.out.zones.contexts[10].in_zone,
    "P3/P7: late current-request ready outside Zone hands control to Mission");
  check(r.out.v_ref==0 && !r.out.selected_reference.valid && r.out.ref_points[0].x==0,
    "handoff tick discards Navigation geometry and waits for execution acknowledgement");
  check(r.out.mission_request.ready.recorded && r.out.mission_request.handoff.recorded &&
    r.out.mission_request.ready.travel_distance>=r.out.mission_request.space.travel_distance,
    "calibration retains separate space/ready/authority milestones and real travel");
  r.s.parking_mission_active=true; r.s.parking_path.n=1; r.tick();
  check(r.out.selected_reference.valid && r.out.v_ref<0 && r.out.speed_owner==SpeedOwner::MISSION &&
    r.out.avoid==AvoidState::INACTIVE,"P20: acknowledged ACTIVE owns signed speed and reference");
  r.s.parking_path.n=0; r.tick();
  check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.path_source==MGM_SRC_PARKING &&
    r.out.v_ref==0 && (r.out.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),
    "P8: missing ACTIVE Mission reference stops without Navigation replacement");
}
void lifetime()
{
  Run timeout; timeout.st.params.parking_search_timeout=.1; timeout.prepare();
  const auto first=timeout.out.mission_request.request_id;
  timeout.tick(9); check(timeout.out.mission_request.active,"timeout waits until configured boundary");
  acknowledge_search(timeout,true); timeout.tick();
  check(!timeout.out.mission_request.active && timeout.out.mission_cancel &&
    timeout.out.mission_request.cancel_reason==MissionCancelReason::SEARCH_TIMEOUT &&
    !timeout.st.managers.mission_completed[0] && timeout.out.mission==MissionState::MISSION_IDLE,
    "P9: timeout wins over same-tick ready; cancellation is not completion");
  timeout.zone(20,ZoneType::MISSION_ZONE,MissionType::PARALLEL_PARKING,1); timeout.tick();
  check(timeout.out.mission==legacy::MISSION_PREPARE && timeout.out.active_mission_id==1 &&
    timeout.out.mission_request.request_id>first,"P11/P17: later Mission Zone starts new request after timeout");
  Run distance; distance.st.params.max_parking_search_distance=.05;
  distance.s.vehicle_speed=-.5f; distance.prepare(); distance.tick(9);
  check(distance.out.mission_request.active,"distance waits before configured limit");
  distance.tick(2);  // definitely exceeds limit despite binary floating-point rounding
  check(distance.out.mission_request.cancel_reason==MissionCancelReason::TRAVEL_DISTANCE &&
    !distance.st.managers.mission_completed[0],"P10: actual reverse travel counts toward distance limit");
  Run actual; actual.s.vehicle_speed=0; actual.prepare(); actual.tick(100);
  check(actual.out.v_ref>0 && actual.out.mission_request.travel_distance==0,
    "positive command cannot fabricate measured travel while vehicle is stationary");
  actual.s.vehicle_speed_valid=false; actual.tick();
  check(actual.out.mission_request.cancel_reason==MissionCancelReason::MOTION_UNAVAILABLE &&
    actual.out.path_source==MGM_SRC_LANE,"unobservable travel cancels search, normal Navigation continues");
  for (double value : {-1.,0.,std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()}) {
    Run unset; unset.st.params.parking_search_timeout=value; unset.prepare();
    check(unset.out.mission_request.cancel_reason==MissionCancelReason::CALIBRATION_REQUIRED &&
      !unset.out.mission_prepare && unset.out.v_ref>0,
      "uncalibrated/nonfinite search limit never starts unlimited search");
  }
  Run unset_distance; unset_distance.st.params.max_parking_search_distance=-1; unset_distance.prepare();
  check(unset_distance.out.mission_request.cancel_reason==MissionCancelReason::CALIBRATION_REQUIRED,
    "both independent calibration limits must be configured");
  Run cancel; cancel.prepare(); cancel.s.mission_cancel_requested=true; cancel.tick();
  check(cancel.out.mission_request.cancel_reason==MissionCancelReason::EXPLICIT &&
    cancel.out.mission_cancel && !cancel.st.managers.mission_completed[0],"explicit operator Mission cancel clears request");
  Run finish; finish.prepare(); finish.s.gps_at_end=true; finish.tick();
  check(finish.out.top==TopState::FINISH && !finish.out.mission_request.active &&
    finish.out.mission_request.cancel_reason==MissionCancelReason::FINISH && finish.out.mission_cancel,
    "P18: FINISH cancels PREPARE");
  Run session; session.prepare(); const auto before=session.out.mission_request.request_id;
  session.s.new_session=true; session.tick();
  check(session.out.mission_request.cancel_reason==MissionCancelReason::SESSION_RESET &&
    session.out.mission_cancel && session.out.mission_request.request_id==before,
    "session reset sends cancel for old session before a fresh request");
  session.s.new_session=false; session.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false);
  session.tick(); session.s.event_time_ns=1; session.prepare();
  check(session.out.mission_request.request_id>before,"session reset/ROS clock rollback cannot reuse request ID");
  Run stop; stop.prepare(); stop.s.external_stop=true; stop.tick(30);
  check(stop.out.mission_request.active && stop.out.mission_request.elapsed_s>=.3 &&
    stop.out.v_ref==0 && (stop.out.safe_stop_reasons & SAFE_STOP_EXTERNAL),
    "temporary external stop retains request and real elapsed timeout");
}
void generations_and_completion()
{
  Run r; r.prepare(); const auto id=r.out.mission_request.request_id;
  acknowledge_search(r,true); r.s.parking_request_id=id-1; r.tick();
  check(r.out.mission==legacy::MISSION_PREPARE && !r.out.mission_request.search_acknowledged,
    "P12: old session true space/ready cannot start new Mission");
  acknowledge_search(r,true); r.s.parking_preparation_reference.generation=1; r.tick();
  check(r.out.mission==legacy::MISSION_PREPARE,"pre-request readiness timestamp rejected even with current ID");
  acknowledge_search(r,true); r.s.parking_preparation_reference.age_s=.6f; r.tick();
  check(r.out.mission==legacy::MISSION_PREPARE,"delayed/frozen readiness generation rejected despite live heartbeat");
  acknowledge_search(r,true); r.s.parking_updated=false; r.tick();
  check(r.out.mission==legacy::MISSION_PREPARE,"no new status cannot acknowledge readiness");
  acknowledge_search(r,true); r.s.parking_mission_mode=2; r.tick();
  check(r.out.mission==legacy::MISSION_PREPARE,"wrong Mission type cannot acknowledge readiness");
  acknowledge_search(r); r.tick();
  const auto start=r.out.mission_request.start_time_ns;
  for(int i=0;i<20;++i) {
    r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,i%2==0); r.tick();
    check(r.out.mission_request.request_id==id && r.out.mission_request.start_time_ns==start &&
      !r.out.mission_prepare,"P14: Zone chatter never resets request or search clock");
  }
  r.zone(20,ZoneType::MISSION_ZONE,MissionType::PARALLEL_PARKING,1); r.tick();
  check(r.out.mission_request.request_id==id && r.out.mission_request.mission_id==0,
    "P17: another Mission entry cannot overwrite pending request");
  acknowledge_search(r,true); r.tick();
  check(r.out.mission==MissionState::MISSION_ACTIVE,"P13: fresh current session readiness is accepted");
  r.s.parking_mission_active=true; r.s.parking_request_id=id-1; r.tick();
  check(!r.out.selected_reference.valid && r.out.v_ref==0,"old request execution/ref cannot authorize ACTIVE speed");
  r.s.parking_request_id=id; r.s.references[MGM_SRC_PARKING].generation=1; r.tick();
  check(!r.out.selected_reference.valid && r.out.v_ref==0,"pre-request reference generation rejected in ACTIVE");
  acknowledge_search(r,true); r.tick(); r.gps_zone(true); r.s.parking_done=true;
  r.s.parking_request_id=id-1; r.tick();
  check(r.out.mission==MissionState::MISSION_ACTIVE && !r.st.managers.mission_completed[0],
    "stale other-session done cannot mark current Mission complete");
  r.s.parking_request_id=id; r.tick();
  check(!r.out.mission_request.active && r.st.managers.mission_completed[0] &&
    r.out.mission==MissionState::MISSION_IDLE && r.out.nav==NavState::GPS_ONLY_NAV,
    "P15: done clears request, remembers Mission ID and reevaluates current GPS Zone");
  r.tick(); check(r.out.mission==MissionState::MISSION_IDLE,"ignored second Zone is not implicitly queued");
  r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false); r.tick(); r.prepare();
  check(r.out.mission==MissionState::MISSION_IDLE,"P16: completed Mission cannot repeat on reentry");
  Run abort; abort.prepare(); acknowledge_search(abort); abort.tick();
  abort.s.parking_search_active=false; abort.tick();
  check(abort.out.mission_request.cancel_reason==MissionCancelReason::MODULE_ABORT,
    "existing module cancel after search acknowledgement aborts request");
}
}
int main()
{
  latch_and_handoff(); lifetime(); generations_and_completion();
  std::printf("mission_preparation_test: %d checks, %d failures (revision 5 tests 1..20)\n",checks,failures);
  return failures ? 1 : 0;
}
