#include "manager_test_fixture.hpp"
#include "core/last_mission_step.hpp"
using namespace manager_test;

void setup(Run & r, bool zone=true) {
  r.st.params.revised_v2_enabled = 1;
  r.st.params.route_sequence_enabled = 1;
  r.st.params.zone_enter_confirm_samples = 2;
  r.s.gps_fix_quality = 4; r.s.revised_v2 = true; r.s.sensor_alive_mask = 0x41;
  r.s.route.enabled = true; r.s.route.sequence_id = 11; r.s.route.instance_id = 22;
  r.s.route.count = 3; r.s.route.last_mission_enabled = true;
  r.s.route.left_index = 1; r.s.route.right_index = 2;
  if (zone) {r.zone(20, ZoneType::LAST_MISSION_ZONE);}
}
void observe(Run & r, int cls, uint64_t request=0) {
  r.s.exit_request_id = request ? request : r.st.managers.last_mission.request_id;
  r.s.exit_class_id = cls; r.s.exit_confidence = .9f;
  r.s.exit_reference = ReferenceSample{static_cast<uint64_t>(r.s.event_time_ns),0,.5f};
  r.tick();
}
void finish_window(Run & r) {
  r.s.monotonic_ns = r.st.managers.last_mission.started_ns + 10'000'000'000LL;
  r.tick();
}
int main() {
  for (int cls : {0, 1, -1}) {
    Run r; setup(r); r.s.vehicle_speed = .5f; r.tick(2);
    check(r.out.last_mission.phase == LastMissionPhase::STOPPING && r.out.v_ref == 0,
      "confirmed zone enters braking with immediate output stop");
    r.tick(1100);
    check(r.out.last_mission.phase == LastMissionPhase::STOPPING, "moving is not ten stationary seconds");
    r.s.vehicle_speed = 0; r.tick();
    check(r.out.last_mission.phase == LastMissionPhase::JUDGING, "actual stop begins observation");
    observe(r, cls); r.tick(990);
    check(r.out.last_mission.phase == LastMissionPhase::JUDGING && r.out.v_ref == 0,
      "a detection never permits early departure");
    finish_window(r);
    const int selected = cls == 0 ? 2 : 1;
    check(r.out.route.phase == RoutePhase::WAIT_ACK && r.out.route.requested_index == selected &&
      r.out.v_ref == 0 && r.out.last_mission.phase == LastMissionPhase::WAIT_ROUTE,
      "Left/no detection requests 06, Right requests 07 and remains stopped");
    check(r.out.last_mission.fallback == (cls == -1), "fallback telemetry distinguishes no decision");
    const auto request = r.out.route.request_id;
    r.tick(100); check(r.out.v_ref == 0 && r.out.route.request_id == request, "missing ACK stays stopped");
    r.s.route.index = selected; r.s.route.acknowledged_request = request;
    r.s.route.last_mission_enabled = false; r.s.route.terminal = true;
    r.s.route.left_index = r.s.route.right_index = 0;
    r.s.zones.count = 0;
    r.tick(); check(r.out.v_ref == 0 && r.out.route.phase == RoutePhase::WAIT_ACK, "same GPS generation cannot release");
    r.s.gps_valid = false; ++r.s.references[MGM_SRC_GPS].generation; r.tick();
    check(r.out.route.phase == RoutePhase::WAIT_ACK, "unlocalized GPS is not new route evidence");
    r.s.gps_valid = true; ++r.s.references[MGM_SRC_GPS].generation; r.tick();
    check(r.out.last_mission.phase == LastMissionPhase::DONE && r.out.v_ref == 0,
      "matching new GPS ACK completes mission with one stationary handoff tick");
    r.tick(); check(r.out.v_ref > 0 && r.out.nav == NavState::GPS_BACKUP, "GPS navigation resumes after acknowledged branch");
    r.s.gps_at_end = true; ++r.s.references[MGM_SRC_GPS].generation; r.tick();
    check(r.out.top == TopState::FINISH, "either terminal branch finishes instead of driving 06 then 07");
    r.s.new_session = true; r.tick();
    check(r.out.last_mission.phase == LastMissionPhase::IDLE, "new session resets completion memory");
  }
  {
    Run r; setup(r, false); r.tick(1200);
    check(r.out.last_mission.phase == LastMissionPhase::IDLE && r.out.v_ref > 0,
      "no configured zone entry means no last mission activation");
  }
  {
    Run r; setup(r); r.tick(2); observe(r,0);
    r.tick(50); check(r.out.last_mission.right_votes == 1, "one camera generation gets one vote");
    r.s.exit_request_id++; r.s.exit_reference.generation++; r.tick();
    check(r.out.last_mission.right_votes == 1, "wrong request is rejected");
    r.s.vehicle_speed_valid=false; r.tick();
    check(r.out.last_mission.phase==LastMissionPhase::STOPPING, "speed loss resets window");
    r.s.vehicle_speed_valid=true; r.tick(); observe(r,1); observe(r,0);
    finish_window(r); check(r.out.last_mission.fallback && r.out.last_mission.route_id==6, "tie falls back to 06");
  }
  {
    Run r; setup(r); r.tick(2); observe(r,0);
    r.s.external_stop=true; r.tick(1100);
    check(r.out.v_ref==0 && r.out.last_mission.phase==LastMissionPhase::STOPPING, "operator stop suspends judgment");
    r.s.external_stop=false; r.tick();
    check(r.out.last_mission.right_votes==0, "resume starts a new observation window");
    for (int i=0;i<3;++i) {
      r.s.estop_scans[0]=ReferenceSample{static_cast<uint64_t>(100+i),0,.35f};
      r.s.estop_clearance_m[0]=.1f; r.tick();
    }
    check(r.out.estop_active && r.out.last_mission.phase==LastMissionPhase::STOPPING, "upper ESTOP preempts last mission");
    r.s.recovery_request_id=r.out.estop_request_id;
    r.s.recovery_reference=ReferenceSample{r.out.estop_request_id,0,.35f}; r.s.recovery_done=true; r.tick();
    r.tick(); check(!r.out.estop_active && r.out.last_mission.phase==LastMissionPhase::JUDGING, "recovery returns to stationary judgment");
  }
  {
    Run r; setup(r); r.tick(2); r.s.route.instance_id++; r.tick();
    check(r.out.route.phase==RoutePhase::FAULT && r.out.v_ref==0, "GPS restart faults during mission");
    Run moving; setup(moving); moving.tick(2); observe(moving,0); moving.s.vehicle_speed=.1f;
    finish_window(moving); check(moving.out.last_mission.phase==LastMissionPhase::STOPPING, "motion at deadline prevents completion");
  }
  {
    Run r; setup(r); r.s.route.required_count=1; r.s.route.required_missions[0]=4; r.tick(3);
    check(r.out.last_mission.phase==LastMissionPhase::IDLE, "prior required missions cannot be bypassed");
    r.st.managers.mission_completed[4]=true; r.tick();
    check(r.out.last_mission.phase==LastMissionPhase::JUDGING, "completed prior mission allows entry");
    r.s.exit_request_id=r.out.last_mission.request_id;
    r.s.exit_reference=ReferenceSample{static_cast<uint64_t>(r.s.event_time_ns),1.,.5f};
    r.s.exit_class_id=0; r.s.exit_confidence=.9f; r.tick();
    check(r.out.last_mission.right_votes==0, "stale camera result is not a vote");
    finish_window(r);
    r.s.route.index=r.out.route.requested_index; r.s.route.acknowledged_request=r.out.route.request_id;
    r.s.route.last_mission_enabled=false; r.s.route.left_index=r.s.route.right_index=0;
    r.s.route.required_count=0; ++r.s.references[MGM_SRC_GPS].generation; r.tick();
    check(r.out.route.phase==RoutePhase::FAULT && r.out.v_ref==0, "exit ACK must certify a terminal route");
  }
  std::printf("Last mission: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
