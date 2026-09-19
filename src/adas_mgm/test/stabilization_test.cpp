#include "core/legacy_state_ids.hpp"
#include "manager_test_fixture.hpp"
#include "core/mission_step.hpp"
#include "core/manager_step.hpp"
#include "core/reference_safety.hpp"
#include <limits>
#include <type_traits>
using namespace manager_test;
#define BUS(T) static_assert(std::is_standard_layout<T>::value && std::is_trivially_copyable<T>::value, #T " must map to fixed MBD bus")
BUS(CoreSnapshot); BUS(CoreState); BUS(CoreParams); BUS(CoreOutput); BUS(ZoneState);
BUS(MissionRequest); BUS(ReferenceStatus); BUS(MissionObservation); BUS(RecoveryDiagnostics);
namespace {
CoreOutput fix(Run & r, bool inside, uint64_t generation, bool valid=true, int copies=10)
{
  r.gps_zone(inside); r.s.zones.generation=generation; r.s.gps_valid=valid;
  for (int i=0;i<copies;++i) {
    r.s.monotonic_ns+=10'000'000; r.s.event_time_ns+=10'000'000;
    r.out=mgm_step(r.s,r.st);
  }
  return r.out;
}
void zones()
{
  Run r; r.st.params.zone_enter_confirm_samples=3; r.st.params.zone_exit_confirm_samples=3;
  fix(r,true,1);
  check(!r.out.zones.in_gps_only_zone && r.out.zones.contexts[1].enter_count==1,
    "Z1: ten control copies count as one independent GNSS fix");
  fix(r,false,2); check(!r.out.zones.in_gps_only_zone,"Z1: isolated IN does not enter");
  fix(r,true,3); fix(r,true,4); fix(r,true,5,true,1);
  check(r.out.zones.contexts[1].zone_entered && r.out.zones.in_gps_only_zone,"Z2: third independent IN enters once");
  fix(r,true,5); check(!r.out.zones.contexts[1].zone_entered,"Z2: repeated generation emits no second edge");
  fix(r,false,6); fix(r,true,7); fix(r,false,8); fix(r,false,9);
  check(r.out.zones.contexts[1].stable_in_zone && !r.out.zones.contexts[1].raw_in_zone,
    "Z3/Z4: interior and exit-boundary jitter preserve stable membership");
  fix(r,false,10,false);
  check(r.out.zones.in_gps_only_zone && !r.out.zones.selected.zone_valid &&
    !r.out.zones.selected.zone_exited && r.out.v_ref==0,"Z5: invalid GPS preserves stable GPS-only and stops");
  fix(r,false,11); check(r.out.zones.in_gps_only_zone,"Z6: first contrary recovered fix cannot exit");
  fix(r,true,12); check(!r.out.zones.selected.zone_entered,"Z6: same-zone recovery emits no synthetic entry");
  fix(r,false,13); fix(r,false,14); fix(r,false,15,true,1);
  check(!r.out.zones.in_gps_only_zone && r.out.zones.contexts[1].zone_exited,"confirmed exit emits one edge");
  fix(r,true,14); check(!r.out.zones.selected.zone_valid,"regressing fix generation cannot be counted");
  fix(r,true,16); fix(r,true,17);
  check(!r.out.zones.in_gps_only_zone,"invalid generation resets pending confirmation");

  Run baseline, stable;
  stable.st.params.zone_enter_confirm_samples=stable.st.params.zone_exit_confirm_samples=3;
  int before=0, after=0; auto prev_before=baseline.out.nav, prev_after=stable.out.nav;
  for (int i=0;i<100;++i) {
    fix(baseline,i%2==0,i+1); fix(stable,i%2==0,i+1);
    before+=baseline.out.nav!=prev_before; after+=stable.out.nav!=prev_after;
    prev_before=baseline.out.nav; prev_after=stable.out.nav;
  }
  std::printf("Z9 synthetic 10s boundary: raw-policy navigation=%d, confirmed=%d\n",before,after);
  check(before==100 && after==0,"Z7/Z8/Z9: alternating membership from parallel/crossing indices suppressed");
  fix(stable,true,101);fix(stable,true,102);fix(stable,true,103);
  check(stable.out.zones.in_gps_only_zone,"persistent wrong segment still confirms: debounce is not localization");

  Run mission; mission.st.params.zone_enter_confirm_samples=mission.st.params.zone_exit_confirm_samples=3;
  mission.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0);
  mission.tick(3); const auto id=mission.out.mission_request.request_id;
  for (int i=0;i<100;++i) {
    mission.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,i%2==0); mission.tick();
  }
  check(mission.out.mission==legacy::MISSION_PREPARE && mission.out.mission_request.request_id==id,
    "Z10: PREPARE latch survives raw/stable source-zone changes");
  Run reset;reset.st.params.zone_enter_confirm_samples=reset.st.params.zone_exit_confirm_samples=3;
  reset.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0);
  reset.s.new_session=true;reset.tick();reset.s.new_session=false;reset.tick(10);
  check(reset.out.mission==MissionState::MISSION_IDLE && reset.out.zones.contexts[10].mission_entry_suppressed,
    "session reset inside zone suppresses delayed confirmation entry too");
  reset.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,false);reset.tick(3);
  reset.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,true);reset.tick(3);
  check(reset.out.mission==legacy::MISSION_PREPARE,"confirmed exit/reentry after reset permits new request");
  Run unset; unset.st.params.zone_enter_confirm_samples=unset.st.params.zone_exit_confirm_samples=0;
  unset.gps_zone(true); unset.tick();
  check(unset.out.zones.calibration==CalibrationState::UNCALIBRATED && unset.out.v_ref==0 &&
    (unset.out.safe_stop_reasons & SAFE_STOP_ZONE_CONTEXT_UNAVAILABLE),"unset confirmation with defined zones is explicit stop");
  check(unset.out.zones.contexts[1].raw_in_zone,"uncalibrated raw zone still observable");
  unset.st.params.zone_enter_confirm_samples=-1;unset.tick();
  check(unset.out.zones.calibration==CalibrationState::INVALID_CONFIG,"negative confirmation is invalid configuration");
}
void parking()
{
  Run r; r.st.params.parking_search_timeout=-1; r.prepare();
  check(r.out.parking_calibration==CalibrationState::UNCALIBRATED &&
    r.out.mission_request.cancel_reason==MissionCancelReason::CALIBRATION_REQUIRED &&
    r.out.path_source!=MGM_SRC_PARKING,"P1: unset limit never takes parking authority");
  auto p=params(); check(parking_calibration(p)==CalibrationState::CALIBRATED,"P2: explicit test limits calibrated");
  p.parking_search_timeout=0; check(parking_calibration(p)==CalibrationState::INVALID_CONFIG,"zero limit invalid");
  p.parking_search_timeout=-1;p.max_parking_search_distance=std::numeric_limits<double>::infinity();
  check(parking_calibration(p)==CalibrationState::INVALID_CONFIG,"invalid value takes priority over unset sentinel");
  Run moving;moving.prepare();moving.st.params.parking_search_timeout=-1;moving.tick();
  check(!moving.out.mission_request.active && !moving.st.managers.mission_completed[0],
    "removing calibration during PREPARE cancels without completion");
}
void traffic()
{
  Run r; r.tick(50); r.redline(); r.s.vehicle_speed=-2; r.s.traffic_stopline_detected=false; r.tick();
  check(near(r.out.traffic_remaining_m,1.5f),"T1/A: loss seeds exact 1.5m once, no same-tick integration");
  r.st.v=0; r.s.monotonic_ns+=90'000'000; r.tick();
  check(near(r.out.traffic_remaining_m,1.3f),"T2/B/D: actual absolute speed times real 0.1s, not command speed");
  r.s.traffic_stopline_detected=true; r.s.vehicle_speed=1;r.tick();
  r.s.traffic_stopline_detected=false;r.tick();
  check(near(r.out.traffic_remaining_m,1.28f),"T5/E: flicker never reseeds");
  r.tick(29); check(near(r.out.traffic_remaining_m,.99f) && r.out.v_ref==0,
    "T3/C: zero-speed target at remaining 1.0m within one integration step of 0.5m actual travel");
  r.st.traffic_stopline_distance=1.f;r.s.vehicle_speed=0;r.tick();
  check(r.out.signal==SignalState::STOPPED_WAIT && r.out.traffic_stop_in_success_region,
    "T4: measured stopped at 1m belongs to success region");
  r.s.traffic_red_active=false;r.tick();check(r.out.signal==SignalState::SIGNAL_IDLE && r.out.path_source==MGM_SRC_GPS && r.out.v_ref>0,"F: loss of red alone releases signal to GPS");
  r.s.external_stop=true;r.s.lane_path.n=0;r.s.gps_path.n=0;r.s.avoid_path.n=0;
  r.s.traffic_green_active=true;r.tick();
  check(r.out.signal==SignalState::SIGNAL_IDLE && r.out.v_ref==0 &&
    (r.out.safe_stop_reasons & SAFE_STOP_EXTERNAL) && (r.out.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),
    "T6/G/H: green releases Signal only; independent stops stay");
  r.s.external_stop=false;r.tick();check(r.out.v_ref==0,"clearing one reason does not clear invalid reference");
  Run overshoot;overshoot.redline();overshoot.s.traffic_stopline_detected=false;overshoot.tick();
  overshoot.s.vehicle_speed=10;overshoot.tick(20);overshoot.s.vehicle_speed=0;overshoot.tick();
  check(overshoot.out.traffic_remaining_m<0 && !overshoot.out.traffic_stop_in_success_region,
    "T4: signed overshoot never counted as successful stop");
  overshoot.st.traffic_stopline_distance=0;overshoot.tick();
  check(!overshoot.out.traffic_stop_in_success_region,"T4: exactly zero is outside open lower boundary");
  overshoot.st.traffic_stopline_distance=.5;overshoot.tick();
  check(overshoot.out.traffic_stop_in_success_region,"T4: 0.5m stationary observation is within region");
  Run gps;gps.gps_zone(true);gps.tick();gps.s.external_stop=true;gps.tick();gps.s.external_stop=false;gps.tick();
  check(gps.out.nav==NavState::GPS_ONLY_NAV && gps.out.v_ref>0,"SAFE_STOP release reselects current GPS-only, not LINE");
}
void recovery()
{
  Run r;r.tick(2);r.s.auto_estop=true;r.tick(50);
  check(!r.out.recovery.configured && !r.out.recovery.eligible && r.st.escape_phase==MGM_ESCAPE_NONE,
    "R1: operating zero delay stays disabled");
  r.st.params.escape_after_cycles=2;r.st.params.escape_max_cycles=6;
  r.s.rear_corridor_state=RearCorridorState::UNKNOWN;r.tick(5);
  check(r.out.recovery.block_reason==RecoveryBlockReason::REAR_UNKNOWN && r.out.v_ref==0,"R2: UNKNOWN vetoes legacy rear_clear true");
  r.s.rear_corridor_state=RearCorridorState::BLOCKED;r.tick(5);
  check(r.out.recovery.block_reason==RecoveryBlockReason::REAR_BLOCKED && r.out.v_ref==0,"R3: BLOCKED prohibits recovery");
  r.s.rear_corridor_state=RearCorridorState::CLEAR;r.s.vehicle_speed=-.2;r.tick();
  check(r.st.escape_phase==MGM_ESCAPE_REVERSING && r.out.v_ref<0 && r.out.recovery.eligible,
    "R4: explicit test config and certified CLEAR permit existing recovery");
  r.tick(2);
  check(r.out.recovery.attempt_count==1 && r.out.recovery.command_time_s>0 &&
    r.out.recovery.measured_distance_m>0 && r.out.recovery.measured_distance_complete,
    "R7: attempt, actual command interval and measured reverse distance accumulate");
  auto bad=r.out;bad.ref_points[0].x=std::numeric_limits<float>::quiet_NaN();final_reference_gate(bad,r.st);
  check(bad.v_ref==0 && (bad.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),"R5: final gate blocks malformed reverse reference");
  r.s.rear_sensor_valid=false;r.tick();
  check(r.st.escape_phase==MGM_ESCAPE_NONE && r.out.v_ref==0 &&
    r.out.recovery.last_reason==RecoveryReason::REAR_LOST,"R6: rear freshness loss ends reverse and stops");
  r.s.rear_sensor_valid=true;r.tick(3);
  check(r.out.recovery.attempt_count==2,"R7: later attempts counted without invented total cap");
  r.s.vehicle_speed_valid=false;r.tick();
  check(!r.out.recovery.measured_distance_complete,"missing speed makes cumulative distance explicitly incomplete");
  r.s.auto_estop=false;r.s.rear_corridor_state=RearCorridorState::UNKNOWN;r.tick(2);
  check(r.out.v_ref>0,"rear UNKNOWN does not prevent ordinary forward navigation");
  Run projection;projection.tick();projection.s.traffic_red_active=true;projection.s.traffic_stopline_detected=true;projection.tick();
  check(projection.out.state==MGM_STATE_TRAFFIC && projection.out.path_source==MGM_SRC_LANE,"legacy traffic byte cannot encode lateral owner");
  projection.obstacle();check(projection.out.state==MGM_STATE_AVOID && projection.out.speed_owner==SpeedOwner::TRAFFIC,
    "legacy AVOID byte cannot encode simultaneous Traffic speed owner");
}
}
int main(){zones();parking();traffic();recovery();std::printf("stabilization_test: %d checks, %d failures\n",checks,failures);return failures?1:0;}
