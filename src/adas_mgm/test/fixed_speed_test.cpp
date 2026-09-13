#include "manager_test_fixture.hpp"
#include <initializer_list>
#include <limits>

using namespace manager_test;
namespace
{
void motion()
{
  Run r;
  r.st.params.v_base = .8f;  // test value, independent of operating YAML
  r.st.params.a_up = .5f; r.st.params.a_down = 1.5f;
  r.tick();
  check(near(r.out.v_ref, .8f) && near(r.st.v, .8f), "LINE starts at fixed target without ramp");
  r.s.gps_accel_zone = true; r.st.params.v_accel_zone = 2.f; r.tick();
  check(near(r.out.v_ref, .8f), "acceleration zone cannot change fixed target");
  r.gps_zone(true); r.tick();
  check(r.out.path_source == MGM_SRC_GPS && near(r.out.v_ref, .8f), "GPS uses same target");
  r.obstacle();
  r.s.avoid_v_suggest = 2.f;
  r.st.params.v_avoid = .6f; r.st.params.v_narrow = .2f;
  r.tick(40);
  check(r.out.path_source == MGM_SRC_AVOID && near(r.out.v_ref,.6f),
    "AVOID is the main-compatible exception to fixed v2 navigation speed");
  r.s.avoid_narrow_gap = true; r.tick(40);
  check(near(r.out.v_ref,.2f), "AVOID retains main narrow-gap cap");
  r.s.avoid_v_suggest = 0.f; r.tick();
  check(near(r.out.v_ref, .185f), "provider zero request retains normal stop deceleration");
  r.tick(60); check(r.out.v_ref == 0.f, "provider zero reaches stop");
  r.s.avoid_v_suggest = .01f; r.tick();
  check(near(r.out.v_ref, .005f), "avoidance resumes through main acceleration ramp");
}

void stops()
{
  Run r; r.st.params.a_up = .5f; r.st.params.a_down = 1.5f;
  r.obstacle(); r.s.avoid_ttc = .5f; r.tick();
  check(r.out.safety == SafetyState::AUTO_ESTOP && r.out.v_ref == 0 && r.st.v == 0,
    "TTC still immediately stops fixed-speed motion");
  r.s.avoid_ttc = 100.f; r.tick();
  check(near(r.out.v_ref, .005f), "TTC release uses avoidance acceleration ramp");
  r.s.external_stop = true; r.tick();
  check(r.out.v_ref == 0 && r.out.immediate_stop, "operator/CAN external stop preserved");
  r.s.external_stop = false; r.s.auto_estop = true; r.tick();
  check(r.out.v_ref == 0, "LiDAR stop preserved");
  r.s.auto_estop = false; r.s.references[MGM_SRC_AVOID].age_s = 1.f; r.tick();
  check(r.out.v_ref == 0 && (r.out.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),
    "stale reference vetoes fixed speed");
  Run disabled; disabled.s.autonomous_enabled = false; disabled.tick();
  check(disabled.out.v_ref == 0, "go wait preserved");
  Run finish; finish.tick(); finish.s.gps_at_end = true; finish.tick();
  check(finish.out.top == TopState::FINISH && finish.out.v_ref == 0, "FINISH stop preserved");
}

void parking_and_recovery()
{
  Run r; r.st.params.a_up = .5f; r.st.params.a_down = 1.5f;
  r.tick(); r.mission();
  check(r.out.v_ref == 0, "mission handoff still waits for execution ack");
  r.s.parking_updated = true; r.tick();
  check(near(r.out.v_ref, -1.f), "parking reverse uses negative fixed target");
  r.s.parking_v_suggest = 0.f; r.tick(210);  // existing a_up=.5 takes 2s from -1 to 0
  check(r.out.v_ref == 0, "parking phase stop preserved");
  r.s.parking_v_suggest = .05f; r.tick();
  check(near(r.out.v_ref, 1.f), "parking forward uses positive fixed target");
  r.s.parking_path_blocked = true; r.tick(100);
  check(r.out.v_ref == 0, "parking blocked stop preserved");
  r.s.parking_path_blocked = false; r.s.parking_v_suggest = -.1f; r.tick();
  r.s.parking_done = true; r.tick();
  check(r.out.v_ref == 0 && r.out.immediate_stop, "reverse-to-navigation handoff stays zero");
  r.tick(); check(near(r.out.v_ref, 1.f), "navigation resumes at fixed speed after handoff");
  Run escape; escape.st.params.escape_after_cycles = 1;
  escape.st.params.v_escape = -.8f; escape.tick();
  escape.s.auto_estop = true; escape.tick(2);
  check(escape.out.path_source == MGM_SRC_ESCAPE && near(escape.out.v_ref, -.8f),
    "recovery speed is independent of navigation magnitude");
  escape.s.rear_sensor_valid = false; escape.tick();
  check(escape.out.v_ref == 0, "rear loss still terminates reverse motion");
}

void rc_recovery()
{
  Run r;
  r.st.params.escape_after_cycles = 1000;
  r.st.params.escape_max_cycles = 162;
  r.st.params.v_escape = -.8f;
  r.st.params.escape_require_rear_clear = 0;
  r.s.estop_rear_clear = r.s.rear_sensor_valid = false;
  r.s.rear_corridor_state = RearCorridorState::UNKNOWN;
  r.tick();  // Forward output arms the existing recovery algorithm.
  r.s.auto_estop = true;
  r.tick(999);
  check(r.out.v_ref == 0 && r.st.escape_phase == MGM_ESCAPE_NONE,
    "RC recovery waits for the complete E-stop delay");
  r.tick();
  check(r.out.path_source == MGM_SRC_ESCAPE && near(r.out.v_ref, -.8f),
    "explicit rear opt-out permits configured recovery with UNKNOWN input");
  check(!(r.out.safe_stop_reasons & SAFE_STOP_REAR_UNAVAILABLE) &&
    !r.st.managers.recovery.rear_sensor_valid &&
    r.st.managers.recovery.rear_corridor_state == RearCorridorState::UNKNOWN,
    "rear opt-out does not fabricate CLEAR diagnostics");
  int reverse_commands = 0;
  for (int i = 0; i < 200 && r.out.v_ref < 0; ++i) {
    ++reverse_commands;
    check(near(r.out.v_ref, -.8f), "every recovery command uses configured speed");
    r.tick();
  }
  check(reverse_commands == 162 && r.out.v_ref == 0 &&
    near(reverse_commands * .01f * .8f, 1.296f),
    "RC recovery ends after 162 commands, nominally 1.296 metres");

  Run stop;
  stop.st.params.escape_after_cycles = 1;
  stop.st.params.escape_require_rear_clear = 0;
  stop.s.estop_rear_clear = stop.s.rear_sensor_valid = false;
  stop.s.rear_corridor_state = RearCorridorState::BLOCKED;
  stop.tick(); stop.s.auto_estop = true; stop.tick();
  check(stop.out.v_ref < 0, "rear opt-out also ignores BLOCKED input");
  stop.s.external_stop = true; stop.tick();
  check(stop.out.v_ref == 0 && stop.out.immediate_stop,
    "operator and CAN external stop still ends opted-out recovery immediately");
  stop.s.external_stop = false; stop.tick(3);
  check(stop.out.v_ref < 0, "recovery can rearm after the existing delay");
  stop.st.params.escape_require_rear_clear = 1; stop.tick();
  check(stop.out.v_ref == 0 && stop.st.escape_phase == MGM_ESCAPE_NONE,
    "reenabling rear certification restores its veto during reverse");

  Run mission;
  mission.st.params.escape_after_cycles = 1;
  mission.st.params.escape_require_rear_clear = 0;
  mission.tick(); mission.mission();
  mission.s.auto_estop = true; mission.tick(3);
  check(mission.st.escape_phase == MGM_ESCAPE_NONE &&
    mission.out.path_source != MGM_SRC_ESCAPE,
    "Mission ownership excludes recovery even with rear opt-out");
}

void signal_and_invalid()
{
  Run r; r.st.params.a_up = .5f; r.st.params.a_down = 1.5f;
  r.redline();
  check(near(r.out.v_ref, 1.f), "unseeded Signal approach remains fixed speed");
  r.s.traffic_stopline_detected = false; r.tick();  // seed 1.5m
  r.s.vehicle_speed = .5f; r.tick(20);
  check(r.out.speed_owner == SpeedOwner::TRAFFIC && near(r.out.v_ref, 1.4f/1.5f),
    "seeded Signal stop profile remains distance-dependent");
  r.tick(220); r.s.vehicle_speed = 0; r.tick();
  check(r.out.v_ref == 0 && r.out.signal == SignalState::STOPPED_WAIT, "Signal reaches stop");
  r.s.traffic_red_active = false; r.s.traffic_green_active = true; r.tick();
  check(near(r.out.v_ref, 1.f), "green release restores fixed speed");
  Run bad; bad.obstacle(); bad.s.avoid_v_suggest = std::numeric_limits<float>::quiet_NaN(); bad.tick();
  check(bad.out.v_ref == 0 && (bad.out.safe_stop_reasons & SAFE_STOP_REFERENCE_INVALID),
    "normalization cannot hide nonfinite provider speed");
  Run config; config.st.params.v_base = -1.f; config.tick();
  check(config.out.v_ref == 0, "invalid common magnitude cannot command reverse navigation");
  Run legacy; legacy.st.params.base_state_machine_enabled = 0;
  legacy.st.params.a_up = .5f; legacy.tick();
  check(near(legacy.out.v_ref, .005f), "legacy still uses original motion ramp");
}
}
int main()
{
  motion(); stops(); parking_and_recovery(); rc_recovery(); signal_and_invalid();
  std::printf("fixed_speed_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
