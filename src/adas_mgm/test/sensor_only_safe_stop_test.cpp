#include "core/legacy_state_ids.hpp"
#include "manager_test_fixture.hpp"
#include "core/reference_safety.hpp"
#include <limits>
using namespace manager_test;
int main()
{
  // Exhaustive seven-sensor truth table; physical health is independent of paths.
  for (int mask=0; mask<128; ++mask) {
    Run r; r.st.params.safe_stop_all_sensors_only=1;
    r.s.sensor_alive_mask=static_cast<uint8_t>(mask); r.tick();
    check((r.out.safety==SafetyState::SAFE_STOP)==(mask==0), "all seven loss only");
    check(r.out.safe_stop_reasons==(mask==0 ? SAFE_STOP_ALL_SENSORS_LOST : 0u), "only bit 2");
    if (mask==0) {
      check(r.out.v_ref==0, "total loss stops");
      r.s.sensor_alive_mask=2; r.tick();
      check(r.out.safety!=SafetyState::SAFE_STOP && r.out.v_ref>0, "one sensor recovery releases SAFE_STOP");
    }
  }
  Run faults; faults.st.params.safe_stop_all_sensors_only=1;
  faults.s.sensor_alive_mask=2; // only traffic camera alive
  faults.s.camera_line_valid=faults.s.gps_valid=faults.s.lidar_valid=false;
  faults.s.traffic_fail_safe_stop=true; faults.s.parking_valid=false;
  faults.s.start_gate_enabled=true; // already driving before loss
  faults.st.managers.top=TopState::AUTONOMOUS_DRIVE;
  faults.st.params.route_sequence_enabled=1; faults.tick();
  check(faults.out.safety!=SafetyState::SAFE_STOP && faults.out.safe_stop_reasons==0,
    "route, traffic, lidar, reference and nav-loss do not create SAFE_STOP");
  check(faults.out.v_ref==0 && faults.out.reference_motion_blocked,
    "absent control reference remains an explicit motion hold");

  Run operator_stop; operator_stop.st.params.safe_stop_all_sensors_only=1;
  operator_stop.s.sensor_alive_mask=127; operator_stop.s.external_stop=true; operator_stop.tick();
  check(operator_stop.out.safety!=SafetyState::SAFE_STOP && operator_stop.out.safe_stop_reasons==0,
    "operator stop is independent of SAFE_STOP");
  check(operator_stop.out.v_ref==0 && operator_stop.out.immediate_stop, "operator stop still works");

  Run obstacle; obstacle.st.params.safe_stop_all_sensors_only=1;
  obstacle.s.sensor_alive_mask=127; obstacle.obstacle();
  obstacle.s.avoid_path.n=0; obstacle.tick();
  check(obstacle.out.avoid==AvoidState::AVOID_ACTIVE && obstacle.out.safety!=SafetyState::SAFE_STOP,
    "empty avoidance target keeps ownership without SAFE_STOP");
  check(obstacle.out.reference_motion_blocked && obstacle.out.v_ref==0, "no fabricated steering target");
  obstacle.s.avoid_path.n=1; obstacle.tick();
  check(!obstacle.out.reference_motion_blocked && obstacle.out.v_ref>0, "fresh target resumes motion");
  obstacle.s.auto_estop=true; obstacle.tick();
  check(obstacle.out.safety==legacy::AUTO_ESTOP && obstacle.out.v_ref==0, "AUTO_ESTOP independent");

  Run nan; nan.st.params.safe_stop_all_sensors_only=1; nan.s.sensor_alive_mask=127; nan.tick();
  nan.out.v_ref=std::numeric_limits<float>::quiet_NaN(); final_reference_gate(nan.out,nan.st);
  check(nan.out.v_ref==0 && nan.out.reference_motion_blocked && nan.out.safe_stop_reasons==0,
    "nonfinite output rejected without SAFE_STOP");

  Run old; old.s.traffic_fail_safe_stop=true; old.tick();
  check(old.out.safety==SafetyState::SAFE_STOP && (old.out.safe_stop_reasons & SAFE_STOP_TRAFFIC_INPUT),
    "legacy policy unchanged when opt-out is disabled");
  std::printf("%d checks, %d failures\n",checks,failures);
  return failures ? 1 : 0;
}
