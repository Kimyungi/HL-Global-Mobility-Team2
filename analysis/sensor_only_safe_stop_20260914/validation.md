# Sensor-only SAFE_STOP validation — 2026-09-14

v2 config: safe_stop_all_sensors_only=true.
SAFE_STOP iff all seven physical sensor availability bits are zero, independent of mission state.
Bits: lane camera, traffic camera, raw a1/a2/b1/b2, GPS.
Timeouts: lane camera 1.0s; traffic camera/GPS 0.5s; each LiDAR 0.35s.
GPS requires fresh valid position/fix, not specifically RTK FIXED or valid heading.

Operator/CAN stop, AUTO_ESTOP, traffic/mission speed arbitration and original go admission remain independent.
No usable steering reference/nonfinite speed still causes an explicit output hold (reference_motion_blocked),
without creating SAFE_STOP_REFERENCE_INVALID in this policy. This does not fix cubic path generation failures.

Validation:
- Build: 13 packages, success, no compiler warnings in final build.
- Core CTest: 23/23 passed; sensor_only_safe_stop_test 268 checks includes all 128 availability combinations.
- Launch/traffic initialization tests: 28 passed, including successful-frame heartbeat and no heartbeat on failed camera read.
- Mock sensor ROS integration: 18 PASS conditions, real MGM on localhost ROS_DOMAIN_ID=188, no CAN driver.
  Every individual sensor prevents SAFE_STOP; repeated old timestamps expire; FLOAT is accepted; NO FIX loses GPS bit.
- v29 synthetic dump replay: 1,100 rows with sensor_alive_mask/reference_motion_blocked columns populated.
- scripts/v2 check: passed. git diff --check: passed.

No integrated vehicle launch/go performed. v28 replay binary retained at
analysis/run_20260914_063951_diagnosis/core_replay_v28 for historical logs.
