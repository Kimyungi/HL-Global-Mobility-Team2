#include "manager_test_fixture.hpp"
#include "core/reference_safety.hpp"
#include "src/reference_clock.hpp"
#include <limits>
using namespace manager_test;
namespace
{
bool reason(const Run & r, uint32_t mask) {return (r.out.safe_stop_reasons & mask) == mask;}
void initial_and_geometry()
{
  Run boot; boot.s.camera_line_valid = boot.s.gps_valid = false;
  boot.s.avoid_path.n = 0; boot.tick();
  check(boot.out.v_ref == 0 && !boot.out.selected_reference.valid,
    "R1: initial zero ref + positive speed prohibited");
  check(reason(boot, SAFE_STOP_REFERENCE_INVALID) && !reason(boot, SAFE_STOP_ALL_SENSORS_LOST),
    "R13: live LiDAR with no drivable path is reference failure, not all sensors lost");
  boot.s.avoid_path.n = 1; boot.tick();
  check(boot.out.selected_reference.valid && boot.out.v_ref > 0 && boot.out.avoid_episode_reference_seen,
    "R2: existing valid LiDAR path permits low-speed fallback");
  boot.s.lidar_valid = false; boot.tick();
  check(reason(boot, SAFE_STOP_ALL_SENSORS_LOST | SAFE_STOP_REFERENCE_INVALID),
    "sensor lost and invalid ref can coexist");
  for (uint8_t source = 0; source < MGM_SRC_ESCAPE; ++source) {
    Run r; CorePath * paths[] = {&r.s.lane_path,&r.s.gps_path,&r.s.avoid_path,&r.s.parking_path};
    auto & path = *paths[source];
    path = CorePath{}; path.n = 1;
    check(!provider_reference(r.s, source).valid, "all providers reject zero-filled geometry");
    path.pts[0].x = 1.0f;
    check(provider_reference(r.s, source).valid, "one nonzero control point is valid");
    path.n = 2;
    check(!provider_reference(r.s, source).valid, "multiple provider points violate v2 control contract");
    path.n = 1;
    path.pts[0].yaw = std::numeric_limits<float>::quiet_NaN();
    check(!provider_reference(r.s, source).valid, "all providers reject NaN");
    path.pts[0].yaw = 0; path.pts[0].curvature = std::numeric_limits<float>::infinity();
    check(!provider_reference(r.s, source).valid, "all providers reject Inf");
    path.pts[0].curvature = 0; path.n = MGM_NUM_POINTS + 1;
    check(!provider_reference(r.s, source).valid, "point count outside fixed bus rejected");
    path.n = 1; r.s.references[source].generation = 0;
    check(!provider_reference(r.s, source).valid, "old/default buffer without actual generation rejected");
  }
  Run confidence; confidence.s.lane_confidence = 1.1f; confidence.tick();
  check(confidence.out.nav == NavState::GPS_BACKUP, "out-of-domain LINE confidence is invalid");
}
void generation_and_clear()
{
  ReferenceClock clock;
  constexpr int64_t sec = 1'000'000'000;
  auto first = clock.observe(10*sec, 10*sec, sec, sec/2);
  check(first.generation == 10*sec && near(first.age_s,0), "actual generation establishes age");
  Run r; r.obstacle(); r.s.avoid_obstacle_detected = false;
  r.s.references[MGM_SRC_AVOID] = clock.observe(10*sec, 10*sec+200'000'000, sec+200'000'000, sec/2);
  r.tick();
  check(near(r.out.selected_reference.age_s,.2f) && r.out.selected_reference.generation == first.generation,
    "R3/R6: republish increments generation age, not generation ID");
  check(r.out.avoid == AvoidState::AVOID_ACTIVE && r.out.selected_reference.valid && r.out.v_ref > 0,
    "R4: valid existing path within existing timeout remains usable");
  r.redline(); r.s.traffic_stopline_detected = false; r.tick();
  check(r.out.path_source == MGM_SRC_AVOID && r.out.speed_owner == SpeedOwner::TRAFFIC,
    "R21: avoidance ref and stop-line profile remain separate");
  r.s.avoid_obstacle_detected = true;
  r.s.references[MGM_SRC_AVOID] = clock.observe(10*sec, 10*sec+501'000'000, sec+501'000'000, sec/2);
  r.tick();
  check(!r.out.selected_reference.fresh && r.out.v_ref == 0 && reason(r, SAFE_STOP_REFERENCE_INVALID),
    "R5: unchanged generation exceeds existing 0.5s timeout despite live publications");
  r.s.avoid_obstacle_detected = false;
  r.s.traffic_red_active = false; r.s.traffic_green_active = true; r.tick();
  check(r.out.signal == SignalState::SIGNAL_IDLE && r.out.path_source == MGM_SRC_GPS &&
    r.out.avoid == AvoidState::INACTIVE && r.out.v_ref > 0,
    "R22: signal exit with obstacle gone releases stale avoid reference to valid GPS");
  r.s.avoid_obstacle_detected = true;
  r.tick();
  check(r.out.v_ref == 0 && reason(r,SAFE_STOP_REFERENCE_INVALID),
    "a new obstacle still rejects the stale avoid generation");
  r.s.references[MGM_SRC_AVOID] = clock.observe(11*sec, 11*sec, 2*sec, sec/2); r.tick();
  check(r.out.selected_reference.valid && r.out.v_ref > 0, "new sensor-backed generation recovers same owner");
  r.s.avoid_obstacle_detected = false;
  r.s.avoid_path.n = 0; r.tick();
  check(r.out.reference_available && r.out.v_ref > 0 && r.out.path_source == MGM_SRC_GPS,
    "empty avoidance after obstacle clears uses current GPS without stale path grace");
  auto delayed = clock.observe(12*sec, 13*sec, 3*sec, sec/2);
  check(near(delayed.age_s,1.f), "transport delay counts against generation freshness");
  auto backward = clock.observe(11*sec, 13*sec, 3*sec, sec/2);
  check(backward.generation == 0, "out-of-order old generation cannot renew lease");
  auto future = clock.observe(15*sec, 13*sec, 3*sec, sec/2);
  check(future.generation == 0, "future timestamp cannot authorize a reference");
  auto missing = clock.observe(0, 13*sec, 3*sec, sec/2);
  check(missing.generation == 0 && missing.age_s >= 1, "missing metadata is invalid without forgetting last age");
  auto paused_ros = clock.observe(12*sec,13*sec,4*sec,sec/2);
  check(near(paused_ros.age_s,2), "monotonic age keeps increasing if ROS time is paused");
}
void ownership_and_reasons()
{
  Run lane; lane.s.lane_path.n = 0; lane.tick();
  check(lane.out.nav == NavState::GPS_BACKUP && lane.out.selected_reference.valid,
    "R7: invalid LINE uses existing GPS fallback");
  Run mission; mission.mission(); mission.s.parking_updated = true; mission.tick();
  mission.s.parking_path.pts[0].x = std::numeric_limits<float>::quiet_NaN(); mission.tick();
  check(mission.out.mission == MissionState::MISSION_ACTIVE && mission.out.path_source == MGM_SRC_PARKING &&
    mission.out.v_ref == 0 && reason(mission,SAFE_STOP_REFERENCE_INVALID),
    "R8: invalid active Mission keeps authority and stops both speed signs");
  check(reference_geometry_valid(mission.out.ref_points,mission.out.n_points),
    "malformed mission geometry never enters assembled stop buffer");
  Run avoid; avoid.obstacle(); avoid.s.avoid_path.n = 0; avoid.tick();
  check(avoid.out.avoid == AvoidState::AVOID_ACTIVE && avoid.out.path_source == MGM_SRC_AVOID && avoid.out.v_ref == 0,
    "R9: invalid active avoidance does not silently choose Navigation");
  check(reason(avoid,SAFE_STOP_REFERENCE_INVALID) && !reason(avoid,SAFE_STOP_ALL_SENSORS_LOST),
    "R13: all sensors alive, selected reference invalid uses distinct reason");
  avoid.s.external_stop = true; avoid.tick();
  check(reason(avoid,SAFE_STOP_EXTERNAL | SAFE_STOP_REFERENCE_INVALID), "R14: independent simultaneous reasons");
  avoid.s.avoid_path.n = 1; avoid.tick();
  check(reason(avoid,SAFE_STOP_EXTERNAL) && !reason(avoid,SAFE_STOP_REFERENCE_INVALID) && avoid.out.v_ref == 0,
    "R14: recovered reference does not clear external stop");
  avoid.s.external_stop = false; avoid.tick();
  check(avoid.out.safe_stop_reasons == 0 && avoid.out.v_ref > 0, "all reasons must clear to resume");
  Run gps; gps.gps_zone(true); gps.tick(); gps.s.gps_path.pts[0].x = std::numeric_limits<float>::infinity(); gps.tick();
  check(gps.out.nav == NavState::GPS_ONLY_NAV && gps.out.v_ref == 0 &&
    reason(gps,SAFE_STOP_GPS_ONLY_GPS_LOSS | SAFE_STOP_REFERENCE_INVALID),
    "R12: GPS-only invalid GPS ref preserves Zone and sets GPS reason");
  Run gate; gate.tick(); gate.out.selected_reference.valid = false; gate.out.v_ref = 1.0f;
  final_reference_gate(gate.out,gate.st);
  check(gate.out.v_ref == 0 && gate.st.v == 0, "R10: final gate blocks positive speed independent of arbitration");
  gate.out.v_ref = 0; final_reference_gate(gate.out,gate.st);
  check(gate.out.immediate_stop && gate.out.v_ref == 0, "R11: invalid reference may carry a safe zero-speed command");
}
void recovery_and_jitter()
{
  Run off; off.tick(); off.s.auto_estop = true; off.tick(1001);
  check(off.st.params.escape_after_cycles == 0 && off.st.escape_phase == MGM_ESCAPE_NONE,
    "R15: zero means disabled, never immediate recovery");
  off.st.params.escape_after_cycles = 1; off.s.estop_rear_clear = false;
  off.st.params.escape_require_rear_clear = 1; off.tick(1001);
  check(off.st.escape_phase == MGM_ESCAPE_NONE && off.out.v_ref == 0,
    "R16: rear certification enabled prevents auto-reverse without rear_clear");
  off.s.estop_rear_clear = true; off.tick();
  check(off.out.path_source == MGM_SRC_ESCAPE && off.out.selected_reference.valid && off.out.v_ref < 0,
    "existing Escape reference remains valid with all existing inputs and rear confirmation");
  off.out.ref_points[0].yaw = std::numeric_limits<float>::quiet_NaN();
  final_reference_gate(off.out,off.st);
  check(off.out.v_ref == 0 && !off.out.selected_reference.valid, "R18: invalid assembled Escape blocks reverse");
  off.s.estop_rear_clear = false; off.tick();
  check(off.st.escape_phase == MGM_ESCAPE_NONE && off.out.v_ref == 0, "rear loss terminates active reverse");
  Run jitter; jitter.tick(50); int starts=0, entries=0, exits=0, nav_switches=0;
  NavState previous=jitter.out.nav;
  for (int i=0;i<100;++i) {
    const bool inside=i%2==0;
    jitter.gps_zone(inside);
    jitter.zone(10,ZoneType::MISSION_ZONE,MissionType::T_PARKING,0,inside);
    jitter.s.parking_updated=true; jitter.s.parking_done=i>=2;
    jitter.tick();
    entries+=jitter.out.zones.contexts[1].zone_entered;
    exits+=jitter.out.zones.contexts[1].zone_exited;
    jitter.ready(); starts+=jitter.out.mission_start;
    nav_switches+=jitter.out.nav!=previous; previous=jitter.out.nav;
    jitter.tick(9);  // hold each GPS sample across ten MGM 100Hz cycles
  }
  check(entries==50 && exits==50 && nav_switches==100,
    "R19: alternating boundary membership exposes chatter, no invented debounce");
  check(starts==1 && jitter.st.managers.mission_completed[0],
    "R20: boundary chatter cannot restart active/completed Mission");
  std::printf("Zone core synthetic 10Hz/10s: entries=%d exits=%d nav_switches=%d mission_starts=%d\n",
    entries,exits,nav_switches,starts);
}
}
int main()
{
  initial_and_geometry(); generation_and_clear(); ownership_and_reasons(); recovery_and_jitter();
  std::printf("reference_safety_test: %d checks, %d failures (revision 4)\n",checks,failures);
  return failures ? 1 : 0;
}
