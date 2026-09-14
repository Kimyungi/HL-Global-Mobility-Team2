#include "manager_test_fixture.hpp"
#include <type_traits>
using namespace manager_test;
static_assert(std::is_standard_layout<RouteControl>::value && std::is_trivially_copyable<RouteControl>::value);
static_assert(std::is_standard_layout<RouteFeedback>::value && std::is_trivially_copyable<RouteFeedback>::value);
void enable(Run & r, int count=2) {
  r.st.params.route_sequence_enabled = 1;
  r.s.route.enabled = true; r.s.route.sequence_id = 11; r.s.route.instance_id = 22;
  r.s.route.count = count;
}
void fix(Run & r, bool end=false) {
  ++r.s.references[MGM_SRC_GPS].generation; r.s.gps_at_end = end; r.tick();
}
void ack(Run & r) {
  r.s.route.index = r.st.managers.route.requested_index;
  r.s.route.acknowledged_request = r.st.managers.route.request_id;
  r.s.route.required_count = 0;
  r.s.route.connecting = r.st.managers.route.requested_connecting;
  r.s.route.next_connecting = false;
}
int main() {
  {
    Run r; enable(r); fix(r);
    check(r.out.route.phase == RoutePhase::RUNNING && r.out.v_ref > 0, "initial nonterminal drive");
    r.s.vehicle_speed=.4f; fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_STOP && r.out.v_ref==0 && r.out.top!=TopState::FINISH, "middle endpoint stops immediately without FINISH");
    r.s.vehicle_speed_valid=false; fix(r,true);
    check(r.out.route.request_id==0, "unknown actual speed never requests next");
    r.s.vehicle_speed_valid=true; r.s.vehicle_speed=0; r.s.external_stop=true; fix(r,true);
    check(r.out.route.request_id==0, "external stop retains route");
    r.s.external_stop=false; fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.route.requested_index==1 && r.out.v_ref==0, "stopped requests next route");
    const auto request=r.out.route.request_id; r.tick(20);
    check(r.out.route.request_id==request && r.out.v_ref==0, "retransmit uses one request");
    ack(r); r.s.gps_at_end=false; r.tick();
    check(r.out.route.phase==RoutePhase::WAIT_ACK, "same generation cannot acknowledge new geometry");
    r.st.params.avoid_zone_only = 1;
    r.st.managers.avoid = AvoidState::AVOID_ACTIVE;
    r.st.managers.avoid_zone_inside = r.st.managers.avoid_zone_maneuver_seen = true;
    fix(r);
    check(r.out.route.index==1 && r.out.route.changed && r.out.v_ref==0, "fresh acknowledgement handoff remains stopped for this tick");
    check(r.out.avoid==AvoidState::INACTIVE && !r.st.managers.avoid_zone_inside &&
      !r.st.managers.avoid_zone_maneuver_seen, "CSV handoff discards previous avoidance marker and episode");
    fix(r);
    check(r.out.v_ref>0 && r.out.route.seen_nonterminal, "same go automatically resumes next route");
    fix(r,true);
    check(r.out.top==TopState::FINISH && r.out.route.phase==RoutePhase::FINISHED, "only final endpoint finishes");
    r.s.new_session=true; fix(r);
    check(r.out.top!=TopState::FINISH && r.out.route.phase==RoutePhase::WAIT_ACK && r.out.route.requested_index==0 && r.out.route.request_id>request, "new session requests route zero with a new ID");
    r.s.new_session=false; ack(r); fix(r); fix(r);
    check(r.out.route.index==0 && r.out.v_ref>0, "reset handoff resumes first route");
  }
  {
    Run r; enable(r); r.s.route.required_count=1; r.s.route.required_missions[0]=7; fix(r); fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_MISSION && r.out.v_ref==0, "uncompleted Mission blocks route");
    r.st.managers.request.cancel_reason=MissionCancelReason::EXPLICIT; fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_MISSION, "cancel is not completion");
    r.st.managers.mission_completed[7]=true; fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_ACK, "completion unlocks stopped handoff");
    ack(r); fix(r);
    check(r.st.managers.mission_completed[7], "handoff preserves completed Mission memory");
  }
  {
    Run r; enable(r); fix(r,true); r.tick(100);
    check(!r.out.route.seen_nonterminal && r.out.route.request_id==0 && r.out.v_ref==0, "startup at endpoint cannot skip route");
    r.s.gps_valid=false; r.s.gps_path.n=0; fix(r);
    check(r.out.v_ref>0 && !(r.out.safe_stop_reasons & SAFE_STOP_ROUTE_SEQUENCE) && r.out.route.index==0, "GPS outage allows camera on ordinary CSV without advancing the route");
  }
  {
    Run r; enable(r); fix(r); r.s.route.instance_id++; fix(r);
    check(r.out.route.phase==RoutePhase::FAULT && r.out.v_ref==0, "GPS producer restart faults instead of rewinding");
    r.s.new_session=true; fix(r); r.s.new_session=false; ack(r); fix(r); fix(r);
    check(r.out.route.phase==RoutePhase::RUNNING, "explicit session can accept restarted producer");
  }
  {
    Run r; enable(r); fix(r); r.s.route.index=1; fix(r);
    check(r.out.route.phase==RoutePhase::FAULT, "unsolicited route change rejected");
  }
  {
    Run r; enable(r); fix(r); r.mission(); fix(r,true);
    check(r.out.mission==MissionState::MISSION_ACTIVE && r.out.top!=TopState::FINISH && r.out.route.end_reached, "Parking retains authority while remembering passed endpoint");
    r.s.gps_valid=false; fix(r);
    check(!(r.out.safe_stop_reasons & SAFE_STOP_ROUTE_SEQUENCE), "GPS loss does not override ACTIVE Parking");
  }
  {
    Run r; enable(r); fix(r); r.redline(); fix(r,true);
    check(r.out.route.request_id==0, "red stop blocks route request");
    r.s.traffic_green_active=true; r.s.traffic_red_active=false; fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_ACK, "green allows route request if other conditions clear");
  }
  {
    Run r; fix(r,true);
    check(r.out.top==TopState::FINISH, "single CSV keeps existing final stop");
    Run unexpected; unexpected.s.route.enabled=true; fix(unexpected);
    check(unexpected.out.v_ref==0, "sequence GPS cannot silently run with MGM sequence disabled");
  }
  {
    Run r; enable(r); r.s.route.required_count=1; r.s.route.required_missions[0]=0;
    fix(r); r.mission(); fix(r,true);
    r.s.parking_updated=true; fix(r);  // execution acknowledgement after ACTIVE
    r.s.parking_done=true; fix(r);     // Mission finishes beyond the old endpoint
    check(r.out.route.phase==RoutePhase::WAIT_ACK && !r.s.gps_at_end,
      "passed endpoint survives Mission; no return to old CSV endpoint");
  }
  {
    Run r; enable(r); r.s.route.completion=RouteCompletion::MISSIONS_COMPLETE;
    r.s.route.required_count=1; r.s.route.required_missions[0]=0;
    fix(r); r.mission(); fix(r); r.s.parking_updated=true; fix(r);
    check(r.out.route.request_id==0, "mission-only boundary waits for completion");
    r.s.parking_done=true; fix(r);
    check(r.out.route.phase==RoutePhase::WAIT_ACK && !r.out.route.end_reached,
      "explicit Mission completion can replace endpoint condition");
  }
  {
    Run r; enable(r); r.s.route.completion=RouteCompletion::MISSIONS_COMPLETE; fix(r);
    check(r.out.route.phase==RoutePhase::FAULT, "empty mission-only condition is invalid");
    Run unknown; enable(unknown); unknown.s.route.completion=static_cast<RouteCompletion>(9); fix(unknown);
    check(unknown.out.route.phase==RoutePhase::FAULT, "unknown transition condition is invalid");
  }
  {
    Run r; enable(r); fix(r); fix(r,true); ack(r); fix(r,true); fix(r,true);
    check(r.out.route.index==1 && r.out.route.requested_index==1 && r.out.v_ref==0 &&
      !r.out.route.seen_nonterminal, "shared endpoint cannot skip the next numbered route");
  }
  {
    Run r; enable(r); fix(r); r.s.new_session=true; fix(r); r.s.new_session=false;
    ack(r); r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,true);
    fix(r); fix(r);
    check(r.out.mission==MissionState::MISSION_IDLE && r.st.managers.zones.contexts[10].mission_entry_suppressed,
      "route reset acknowledgement preserves inside-Zone Mission suppression");
    r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false); fix(r);
    r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,true); fix(r);
    check(r.out.mission==MissionState::MISSION_PREPARE, "confirmed exit/reentry permits Mission after route reset");
  }
  {
    Run r; enable(r); fix(r); r.s.monotonic_ns-=1'000'000'000; fix(r,true);
    check(r.out.route.phase==RoutePhase::FAULT && r.out.v_ref==0, "clock regression stops sequence");
  }
  {
    Run r; enable(r); r.s.route.next_connecting=true; fix(r); fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.route.requested_connecting,
      "no intervening Mission requests the next entry connection");
    ack(r); fix(r); fix(r); r.tick(60);
    check(r.out.route.connecting && r.out.nav==NavState::GPS_BACKUP && r.out.path_source==MGM_SRC_GPS && r.out.v_ref>0,
      "valid connection drives GPS despite high LINE confidence");
    r.s.route.required_count=0;
    r.s.vehicle_speed=.3f; fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_STOP && r.out.v_ref==0 && r.out.top!=TopState::FINISH,
      "last route connection endpoint stops without final FINISH");
    r.s.vehicle_speed=0; r.s.auto_estop=true; fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_STOP, "auto estop still blocks connection handoff");
    r.s.auto_estop=false; fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_ACK && r.out.route.requested_index==1 && !r.out.route.requested_connecting,
      "connection end requests same-index CSV with a new stage");
    const auto id=r.out.route.request_id;
    r.s.route.acknowledged_request=id; // old connection stage may not acknowledge the CSV
    fix(r);
    check(r.out.route.phase==RoutePhase::FAULT && r.out.v_ref==0,
      "wrong-stage acknowledgement is rejected even with the right index and ID");
  }
  {
    Run r; enable(r); r.s.route.next_connecting=true; fix(r); fix(r,true); ack(r); fix(r); fix(r);
    r.s.gps_valid=false; fix(r);
    check(r.out.v_ref==0, "GPS loss stops connection while camera remains valid");
    r.s.gps_valid=true; fix(r,true); ack(r); fix(r); fix(r);
    check(!r.out.route.connecting && r.out.route.index==1 && r.out.v_ref>0,
      "connection completion resumes the same numbered CSV from its start");
    fix(r,true);
    check(r.out.top==TopState::FINISH, "final FINISH waits for the final CSV after its connection");
  }
  {
    Run r; enable(r); r.s.route.next_connecting=true; r.s.route.required_count=1;
    r.s.route.required_missions[0]=7; fix(r); fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_MISSION && !r.out.route.requested_connecting,
      "entry connection cannot bypass the previous route Mission");
    r.st.managers.mission_completed[7]=true; fix(r,true); ack(r);
    r.s.route.required_count=1; r.s.route.required_missions[0]=8; fix(r); fix(r);
    r.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,8); fix(r);
    check(r.out.mission==MissionState::MISSION_IDLE, "destination Mission cannot start during connection");
    fix(r,true);
    check(r.out.route.phase==RoutePhase::WAIT_ACK && !r.out.route.requested_connecting,
      "connection completion does not wait for destination Mission");
  }
  std::printf("Route sequence: %d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
