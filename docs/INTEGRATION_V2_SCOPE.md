# Integration v2 반영 목록

기반: origin/main `c76f287` (PR #84 포함).
입력: feat/state-machine `19464ae` 위 현재 미커밋 MGM 6차 작업.
브랜치: `integration/v2_main`. main에 직접 병합하거나 push하지 않았다.

## 필수로 함께 반영한 69개 파일

아래 파일은 하나의 메시지/생산자/소비자 계약으로 반영했다. GPS node와 통합 런처,
CLAUDE는 main의 PR #84 변경과 3-way 병합했다. 다른 코어/provider 소스는 현재
6차 작업 사본을 유지한다. 런처의 로그/자산 경로 인자는 v2 분리 과정에서 추가했다.

- `CLAUDE.md`
- `docs/MBD_KIT.md`
- `docs/MGM_BASE_STATE_MACHINE.md`
- `docs/MGM_MBD_STATE_MACHINE_SPEC.md`
- `docs/MGM_MISSION_PREPARATION.md`
- `docs/MGM_REFERENCE_SAFETY.md`
- `docs/MGM_STABILIZATION_REPORT.md`
- `docs/history/MGM_BASE_STATE_MACHINE_REV3.md`
- `docs/history/MGM_MISSION_PREPARATION_REV5.md`
- `docs/history/MGM_REFERENCE_SAFETY_REV4.md`
- `src/adas_mgm/CMakeLists.txt`
- `src/adas_mgm/README.md`
- `src/adas_mgm/config/params.yaml`
- `src/adas_mgm/core/manager_step.cpp`
- `src/adas_mgm/core/manager_step.hpp`
- `src/adas_mgm/core/manager_types.hpp`
- `src/adas_mgm/core/mgm_step.cpp`
- `src/adas_mgm/core/mgm_types.hpp`
- `src/adas_mgm/core/mission_step.cpp`
- `src/adas_mgm/core/mission_step.hpp`
- `src/adas_mgm/core/reference_safety.cpp`
- `src/adas_mgm/core/reference_safety.hpp`
- `src/adas_mgm/core/zone_step.cpp`
- `src/adas_mgm/core/zone_step.hpp`
- `src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py`
- `src/adas_mgm/src/decision_backend.cpp`
- `src/adas_mgm/src/decision_backend.hpp`
- `src/adas_mgm/src/generated_adapter.cpp`
- `src/adas_mgm/src/mgm_node.cpp`
- `src/adas_mgm/src/reference_clock.hpp`
- `src/adas_mgm/test/manager_ros_smoke.py`
- `src/adas_mgm/test/manager_state_test.cpp`
- `src/adas_mgm/test/manager_test_fixture.hpp`
- `src/adas_mgm/test/mission_preparation_test.cpp`
- `src/adas_mgm/test/reference_hold_test.cpp`
- `src/adas_mgm/test/reference_safety_test.cpp`
- `src/adas_mgm/test/stabilization_test.cpp`
- `src/adas_mgm/test/test_mission_calibration.py`
- `src/adas_mgm/test/test_reference_metadata.py`
- `src/adas_mgm/test/zone_manager_test.cpp`
- `src/adas_mgm/tools/analyze_mission_calibration.py`
- `src/adas_mgm/tools/core_replay.cpp`
- `src/adas_mgm/tools/dump_format.hpp`
- `src/fma_interfaces/CMakeLists.txt`
- `src/fma_interfaces/msg/AvoidStatus.msg`
- `src/fma_interfaces/msg/EstopRequest.msg`
- `src/fma_interfaces/msg/GpsPath.msg`
- `src/fma_interfaces/msg/LanePath.msg`
- `src/fma_interfaces/msg/MgmState.msg`
- `src/fma_interfaces/msg/MissionObservation.msg`
- `src/fma_interfaces/msg/ParkingCommand.msg`
- `src/fma_interfaces/msg/ParkingStatus.msg`
- `src/fma_interfaces/msg/ZoneContext.msg`
- `src/fma_interfaces/package.xml`
- `src/lidar_fusion_v2/lidar_fusion_v2/fusion_node.py`
- `src/stack_avoid/stack_avoid/node.py`
- `src/stack_estop/stack_estop/node.py`
- `src/stack_gps/config/mission_zones.example.yaml`
- `src/stack_gps/setup.py`
- `src/stack_gps/stack_gps/node.py`
- `src/stack_gps/stack_gps/zones.py`
- `src/stack_gps/test/conftest.py`
- `src/stack_gps/test/test_zone_chatter.py`
- `src/stack_gps/test/test_zone_stability_integration.py`
- `src/stack_gps/test/test_zones.py`
- `src/stack_lane/stack_lane/node.py`
- `src/stack_parking/launch/parking.launch.py`
- `src/stack_parking/stack_parking/node.py`
- `src/stack_parking/test/test_mission_preparation.py`

## 이번 반영에서 제외한 기존 작업

아래는 원래 작업 폴더에 그대로 남아 있다. 학습/라벨링과 GPU·고속 주행·단독 시험은
별도 변경 단위로 다룬다. 런북의 기존 편집/이름 변경도 가져오지 않았으며 v2 사용법은
INTEGRATION_V2.md가 정본이다. Traffic은 main 버전 그대로 빌드·시험한다.
`.gitignore`는 기존 수정 대신 v2 build/install/log 제외 항목만 별도로 추가했다.

- `.gitignore`
- `README.md`
- `docs/lane_gps_2mps_test.md`
- `docs/lunar_lake_gpu_setup.md`
- `docs/traffic_incremental_80.md`
- `docs/traffic_labeling_center.md`
- `docs/traffic_labeling_yongin.md`
- `docs/traffic_training_4000.md`
- `scripts/gpu/add_intel_gpu_runtime_repo.sh`
- `scripts/gpu/check_xpu.py`
- `scripts/gpu/install_supported_xe_kernel.sh`
- `scripts/gpu/remove_test_kernel.sh`
- `scripts/gpu/rollback_mesa_xorg.sh`
- `scripts/gpu/setup_lunar_lake.sh`
- `scripts/gpu/use_xorg_for_xe.sh`
- `scripts/run_lane_gps_2mps.sh`
- `scripts/training/add_coco_traffic.py`
- `scripts/training/build_mixed_traffic.py`
- `scripts/training/prepare_incremental_80.py`
- `scripts/training/prepare_reviewed_traffic.py`
- `scripts/training/preserve_traffic_head.py`
- `scripts/training/test_preserve_traffic_head.py`
- `scripts/training/train_incremental_80.py`
- `scripts/training/train_traffic_4000.py`
- `src/adas_mgm/RUNBOOK_full_measurement_20260830.md`
- `src/adas_mgm/RUNBOOK_full_measurement_20260904.md`
- `src/adas_mgm/RUNBOOK_full_operation_20260904.md`
- `src/adas_mgm/RUNBOOK_lane_gps.md`
- `src/adas_mgm/RUNBOOK_mbd_lane_gps.md`
- `src/stack_parking/launch/parallel_parking_test.launch.py`
- `src/stack_traffic/launch/central_traffic_test.launch.py`
- `src/stack_traffic/launch/stopline_distance_test.launch.py`
- `src/stack_traffic/package.xml`
- `src/stack_traffic/setup.py`
- `src/stack_traffic/stack_traffic/labeling.py`
- `src/stack_traffic/stack_traffic/logic.py`
- `src/stack_traffic/stack_traffic/node.py`
- `src/stack_traffic/test/test_labeling.py`
- `src/stack_traffic/test/test_node_initialization.py`
- `src/stack_traffic/test/test_target_selection.py`

## v2 분리를 위해 추가한 변경

- `docs/INTEGRATION_V2.md`, 이 목록: 브랜치/실행/반영 범위와 시험 결과.
- `scripts/v2`, `scripts/v2_check.py`: clean ROS 환경, 독립 build/install, prefix/메시지 검사.
- `scripts/test_v2_launch.py`: 벤치 토픽 격리와 실차 런처 거부/경로 검증.
- `src/adas_mgm/launch/integration_v2_bench.launch.py`: MGM만 실행하는 별도 벤치.
- `src/adas_mgm/launch/REAL_VEHICLE_integration_v2.launch.py`: 기존 전체 통합 구성의 v2 진입점.
- `src/adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py`: 전체 구성 재사용 함수 및 경로 인자.
- `.gitignore`, `README.md`, `CLAUDE.md`: v2 산출물 제외와 전용 사용법 연결.

main의 `bridge_dspace`, `ydlidar_ros2_driver`, `ydlidar_sdk`, `multi_lidar_fusion`은
알고리즘 변경 없이 v2 설치에 다시 빌드한다. 모델 가중치/실측 센서 자료는 새로 추가하지 않는다.
