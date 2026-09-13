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
  for (float suggestion : {.02f, .6f, 2.f}) {
    r.s.avoid_v_suggest = suggestion;
    r.s.avoid_narrow_gap = true;
    r.st.params.v_narrow = 0.f; r.st.params.v_avoid = .1f;
    r.tick();
    check(r.out.path_source == MGM_SRC_AVOID && near(r.out.v_ref, .8f),
      "AVOID ignores non-stop suggestion magnitude and narrow/avoid caps");
  }
  r.s.avoid_v_suggest = 0.f; r.tick();
  check(near(r.out.v_ref, .785f), "provider zero request retains normal stop deceleration");
  r.tick(60); check(r.out.v_ref == 0.f, "provider zero reaches stop");
  r.s.avoid_v_suggest = .01f; r.tick();
  check(near(r.out.v_ref, .8f), "motion resumes at fixed target");
}

void stops()
{
  Run r; r.st.params.a_up = .5f; r.st.params.a_down = 1.5f;
  r.obstacle(); r.s.avoid_ttc = .5f; r.tick();
  check(r.out.safety == SafetyState::AUTO_ESTOP && r.out.v_ref == 0 && r.st.v == 0,
    "TTC still immediately stops fixed-speed motion");
  r.s.avoid_ttc = 100.f; r.tick();
  check(near(r.out.v_ref, 1.f), "TTC release has no acceleration ramp");
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
  Run escape; escape.st.params.escape_after_cycles = 1; escape.tick();
  escape.s.auto_estop = true; escape.tick(2);
  check(escape.out.path_source == MGM_SRC_ESCAPE && near(escape.out.v_ref, -1.f),
    "certified recovery uses same fixed magnitude and reverse sign");
  escape.s.rear_sensor_valid = false; escape.tick();
  check(escape.out.v_ref == 0, "rear loss still terminates reverse motion");
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
  motion(); stops(); parking_and_recovery(); signal_and_invalid();
  std::printf("fixed_speed_test: %d checks, %d failures\n", checks, failures);
  return failures ? 1 : 0;
}
