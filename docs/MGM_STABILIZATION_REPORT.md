# MGM 6차 통합 안정화 보고서

2026-09-11, `feat/state-machine`. 현재 작업 파일 기준. 실제 차량 구동 없이 구현·모의 검증했다.
정본: [MGM_MBD_STATE_MACHINE_SPEC.md](MGM_MBD_STATE_MACHINE_SPEC.md).

## 1. 발견한 실제 위험

- raw Zone membership의 즉시 전이가 10초 합성 jitter에서 Navigation을 100회 전환했다.
  전역 nearest index에는 이전 segment/검색 window를 유지하는 연속성 제약이 없었다.
- Parking 제한 -1은 의도된 미설정 gate였지만 설정 상태 구분과 이벤트 통계 도구가 없었다.
- **현재 노드와 운용 YAML의 Traffic offset은 이미 1.0m**였다. 일부 core/MBD 주석이 0.5m로
  낡아 있었고, 병행 core의 signed speed×고정 10ms 적분이 abs(actual speed)×실제 dt 요구와 달랐다.
- Recovery enum은 있었지만 운영 delay=0, 후방 corridor 인증 생산자 없음이 진단에서 분명하지 않았다.
- MBD_KIT의 flat 5-state chart/초록이면 LINE/실제 estop으로 FINISH 해제/double 금지 지시는
  현재 병행 core와 맞지 않았다.
- confirmation 도입 후 reset 내부 Mission의 지연 entry가 새 요청을 만들 수 있는 경계를 발견했다.
  확정된 exit/reentry 전까지 해당 Mission entry를 억제하도록 보완했다.

## 2. 구현 / Calibration / 실차 검증 구분

| 분류 | 항목 |
|---|---|
| IMPLEMENTED | 독립 GNSS Zone 확인, raw/stable 진단, calibration enum/CSV 분석, Traffic abs(v)·dt 적분, typed rear gate/진단, 반복 기록, 단일 MBD 명세 |
| CALIBRATION_REQUIRED | Zone 확인 수/공간 hysteresis, Parking 제한/trigger 위치, rear corridor 여유·coverage, Recovery delay/총 반복 상한 |
| FIELD_VALIDATION_REQUIRED | 실제 GPS segment 선택/진입 지연, 범퍼 1.5m seed와 최종 정차 위치, 센서 지연/참조점 추종, 후방 corridor 인증 producer, 실제 CAN/제동 동작 |

코드 구현만으로 실차 운용 준비가 끝났다고 판단하지 않는다.

## 3. Zone Stability 구현 구조

기존 GPS ZoneMap raw membership → `ZoneSnapshot.generation` → `core/zone_step.cpp` 확인 →
stable ZoneContext → Navigation/Mission. 새로운 주행 State는 없다. 50/200/300 MGM timer와 분리했다.

## 4. Raw vs Stable Zone Context

각 Zone ID에 raw_in_zone, stable_in_zone, enter_count, exit_count, stable entered/exited를 노출한다.
MGM 출력 in_zone은 stable의 호환 별칭이다. GPS 메시지의 in_zone은 raw다.
전체 configured Zone을 MgmState와 선택적 zone_observations.csv로 관측할 수 있다.
GPS invalid/역행은 stable membership과 metadata를 보존하고 가짜 edge를 만들지 않는다.

## 5. Entry/Exit confirmation

서로 다른 유효 GNSS reference_stamp만 계수한다. 같은 fix를 100Hz로 반복해도 count는 늘지 않는다.
반대 membership/invalid는 후보 count를 초기화하며 복구 첫 반대 fix로 즉시 전환하지 않는다.
**운영 기본은 enter=0/exit=0, 미설정**이다. 정의된 Zone이 있는데 기준이 없으면 일반 주행을
ZONE_CONTEXT_UNAVAILABLE(256)로 정지한다. 사용자가 측정한 양수 정수를 설정하고 재시작해야 한다.
확인 수 1/3은 테스트에만 넣었다. reset 내부 Mission은 확정 exit/reentry까지 억제한다.

## 6. Track continuity 처리

기존 `_nearest_idx`는 전역 최근접점, `_target_idx`는 전방 reference 목표 검색이다.
후자는 현재 위치의 segment continuity가 아니므로 전용하지 않았다.
기존 heading/pose/fix 정보로 현재·이전 idx/ENU/표본 이동량/heading validity를 기록한다.
Zone 경계 관측값은 start/end 끝점까지 최소 유클리드 거리이며 실제 signed membership 경계 거리가 아니다.
새 global localization/검색 window/공간 폭은 추가하지 않았다. 지속적인 잘못된 segment 선택은 미해결이다.

## 7. Chatter Before / After

| 합성 10초 / 독립 GNSS 100표본 | Raw edge | Stable edge (시험용 N=3) | 최근접 idx 변경 |
|---|---:|---:|---:|
| 정지 경계 ±1cm | 100 | 0 | 99 |
| 평행 track 20cm 간격 중간 ±1cm | 100 | 0 | 99 |
| 교차 track 두 축 1.1cm 교대 | 100 | 0 | 99 |

실제 PathEngine 출력을 생산 C++ zone_step에 공급한 결과다. MGM Nav 전환도 기존 확인 1표본
시험에서는 100회, 3표본 확인에서는 0회였다. 저속 합성 0.1m/s 진입은 raw entry보다 2개 새 fix 뒤
정확히 한 번 확정됐다. 잘못된 membership이 3표본 지속되면 확정되는 한계도 시험했다.
수치와 위치는 전부 test-only이며 현장 정책으로 채택하지 않았다.

## 8. Parking calibration 현재 상태

운영 timeout=-1.0s, max distance=-1.0m로 **UNCALIBRATED**다.
두 유한 양수만 CALIBRATED, -1 표식은 UNCALIBRATED, 0/기타 음수/비유한 값은 INVALID_CONFIG다.
두 미허가 상태 모두 CALIBRATION_REQUIRED 취소하며 Parking 제어권을 얻지 않는다.
ROS 파라미터 변경을 control snapshot 경계에 적용한다. raw dump 기록 중에는 header 고정을 위해
런타임 변경을 거부하고 YAML 변경/재시작을 요구한다. CALIBRATED는 설정 상태이며 현장 인증이 아니다.

## 9. Calibration CSV 도구

[`analyze_mission_calibration.py`](../src/adas_mgm/tools/analyze_mission_calibration.py)는
Mission 타입별 entry→search/space/ready/handoff 시간, ready/handoff 실제 누적 거리,
성공/timeout/distance cancel/기타 cancel/incomplete와 mean/max/p50/p90/p95/p99를 계산한다.
JSON에 event의 ENU/idx/실제 속도/Zone/Mission ID를 보존한다. 누락 이벤트는 통계에서 제외한다.
8개 단위시험과 실제 ROS mock이 쓴 CSV를 CLI로 분석하는 연결 시험을 통과했다.

```bash
python3 src/adas_mgm/tools/analyze_mission_calibration.py /path/to/mission_events.csv \
  --json /tmp/calibration.json --markdown /tmp/calibration.md
```

## 10. 기존 데이터 자동 설정 여부

운영값/YAML을 자동 생성하지 않았다. 도구 출력도 CALIBRATION_REQUIRED로 유지한다.
기존 parallel 단일 37.92s/2.188m 기록은 현재 lifecycle 제한값으로 채택하지 않았다.
Search trigger는 측정한 기존 index_range/start/end로 별도 지정할 수 있으며 임의 확대하지 않았다.

## 11. Traffic 1.0m 구현 위치

`mgm_node.cpp`의 traffic_stop_offset_m 기본과 config/params.yaml은 원래 1.0m였고 그대로 유지했다.
`manager_step.cpp` Signal 거리 추적 및 기존 `mgm_step.cpp::prioritize()`의 d<=offset 정지 요구가 이를 사용한다.
새 수치 보상이 아니라 **앞범퍼 잔여거리 목표**로 의미를 정리하고 낡은 주석/MBD 지시를 수정했다.
노드는 finite seed > target > 0 설정을 검사한다.

## 12. Traffic 수식 / 검증

최초 loss 틱은 정확히 1.5m. 이후 `d -= abs(actual_v)*monotonic_dt`다.
소실 뒤 약 0.5m 실제 이동하면 목표 잔여 1.0m에 도달한다. command speed를 적분하지 않는다.
기존 감속 profile/merge는 변경하지 않았다. 실제 최종 정차는 차량 동역학과 제동 지연의 영향을 받는다.

T1~T6/A~H: 최초 seed, 실제 절댓값 속도와 가변 dt, 목표 경계, `0<d<=1` 정지 진단,
0/음수 overshoot 실패, flicker 재시드 금지, red 소실 유지, green&&!red 해제 및 다른 reason 유지 통과.
부동소수 적분의 경계 오차는 기존 single 정밀도 범위이며 exact 1.0m 조건과 한 적분 step 이내 통과를 각각 검사했다.
정지 후 밀림도 remaining에 반영한다. 범퍼 기준 seed 일치와 최종 실차 위치는 미검증이다.

## 13. Rear Clear evaluator 설계

[단일 명세 §9](MGM_MBD_STATE_MACHINE_SPEC.md#9-rear-escape-corridor-evaluator--설계--비활성)에
실제 a2 장착/FOV/차체 footprint, 예정 후진 corridor, 측면·정지 여유, coverage/최소점,
missing/dynamic obstacle 처리 계약을 정리했다.
기존 Parking rear sector와 0.20m 도착 조건은 corridor CLEAR 인증으로 사용하지 않았다.
UNKNOWN/BLOCKED/CLEAR, 실제 rear_reference_stamp/valid 소비 gate를 구현했으며 인증 생산자는 아직 없다.

## 14. Recovery 활성화 가능 여부

**운용 불가**다. escape_after_cycles=0과 rear corridor UNKNOWN을 유지했다.
recovery_configured/rear_sensor_valid/rear_corridor_state/eligible/block_reason을 ROS와 replay에 노출한다.
rear_clear bool 단독 또는 UNKNOWN/BLOCKED로 자동 후진할 수 없다. 일반 전진은 rear UNKNOWN만으로 막지 않는다.
시험용 enable+CLEAR에서 기존 Recovery 동작, 후방 상실 종료, invalid reverse reference 정지를 검증했다.
횟수·명령 후진 실제 경과 시간·실측 후진 거리·거리 누락 여부·마지막 이유를 기록하며 새 총 반복 상한은 없다.

## 15. SAFE_STOP SET/CLEAR 표

[단일 명세 §8](MGM_MBD_STATE_MACHINE_SPEC.md#8-safe_stop-set--clear--매-틱-독립-계산)의 표가 정본이다.
GPS_ONLY_GPS_LOSS / ALL_SENSORS_LOST / REFERENCE_INVALID / EXTERNAL / MISSION_FEEDBACK /
TRAFFIC_INPUT / VEHICLE_SPEED / REAR_UNAVAILABLE / ZONE_CONTEXT_UNAVAILABLE 각각의 SET/CLEAR,
발생 영역, latch, 자동 복구, PREPARE/ACTIVE 적용 범위를 명시했다.
TRAFFIC_INPUT은 producer의 camera_fault/startup_hold/오류 출력 또는 MGM 수신 이력 뒤 timeout이다.
red 자체와 구분한다. 한 reason만 해제한 경우와 GPS-only에서 전체 해제한 경우를 시험했다.

## 16. Legacy byte mapping

단일 함수 `legacy_state_projection()`:
ACTIVE Mission→PARKING, 실제 reverse 또는 Avoid ACTIVE/CLEAR→AVOID,
Signal APPROACH/STOPPED 표시→TRAFFIC, Nav LINE→LANE, 그 외→WAYPOINT.
Top/Safety는 별도다. AVOID byte에서 Traffic 속도 제약이 동시에 존재할 수 있다.
byte 하나가 reference/speed/Safety 전체를 표현하지 못한다.

## 17. MBD authoritative 상태

TopState, NavState, AvoidState, SignalState, SafetyState, MissionState가 정본이다.
NAV_RESELECT는 공통 함수/Junction이다. Stateflow에서 flat 5-state로 축약하지 않는다.
주요 bus의 standard_layout/trivially_copyable compile assertion을 추가했다.
ROS/STL container/heap을 core 상태에 추가하지 않았으며 double/int64/uint64도 실제 헤더대로 매핑한다.

## 18. 단일 명세 위치

**[docs/MGM_MBD_STATE_MACHINE_SPEC.md](MGM_MBD_STATE_MACHINE_SPEC.md)**.
기존 3/4/5차 문서는 현재 안내로 정리하고 당시 원문/검증 기록은 docs/history로 옮겼다.
CLAUDE.md/MBD_KIT/패키지 README도 이 명세를 우선하도록 연결했다.

## 19. 이번 변경 파일

5차 및 기존 사용자 미커밋 변경과 구분한 6차 범위다.

| 영역 | 경로 |
|---|---|
| 명세/보고 | CLAUDE.md, docs/MBD_KIT.md, docs/MGM_MBD_STATE_MACHINE_SPEC.md, docs/MGM_STABILIZATION_REPORT.md |
| 기존 문서 정리 | docs/MGM_BASE_STATE_MACHINE.md, docs/MGM_REFERENCE_SAFETY.md, docs/MGM_MISSION_PREPARATION.md, docs/history/MGM_BASE_STATE_MACHINE_REV3.md, docs/history/MGM_REFERENCE_SAFETY_REV4.md, docs/history/MGM_MISSION_PREPARATION_REV5.md |
| Core | src/adas_mgm/core/manager_types.hpp, mgm_types.hpp, manager_step.hpp/.cpp, zone_step.hpp/.cpp, mission_step.hpp/.cpp, mgm_step.cpp |
| MGM 연결/설정 | src/adas_mgm/src/mgm_node.cpp, decision_backend.hpp/.cpp; src/adas_mgm/config/params.yaml, launch/REAL_VEHICLE_lane_gps_can.launch.py, CMakeLists.txt, README.md |
| 메시지 | src/fma_interfaces/msg/ZoneContext.msg, GpsPath.msg, EstopRequest.msg, MgmState.msg |
| GPS 관측 | src/stack_gps/stack_gps/node.py |
| 도구 | src/adas_mgm/tools/analyze_mission_calibration.py, core_replay.cpp, dump_format.hpp |
| 시험 | src/adas_mgm/test/stabilization_test.cpp, test_mission_calibration.py, manager_test_fixture.hpp, manager_ros_smoke.py; src/stack_gps/test/test_zone_stability_integration.py |

축약 파일은 같은 셀의 직전 디렉터리 기준이다. 수정 전 사본/patch는 `/tmp/mgm-before-stabilization-revision`에 있다.
기존 Parking detector/planner/mission/ICP/localization, GPS path_engine, Estop node는 이 사본과 동일하다.
기존 update_escape/build_escape_ref/assemble/merge/prioritize 본문도 동일함을 비교했다.

## 20. 전체 시험 결과

| 검증 | 결과 |
|---|---|
| 격리 build | 9 packages 성공: interfaces, MGM, GPS, Lane, Avoid, Estop, Parking, fusion, Traffic |
| CTest | **15/15 통과** |
| 기존 core 검사 | Manager 78, Zone 49, Reference 60, Preparation 65 유지 |
| 새 6차 core | **48 checks**, 실패 0; Z/P/T/R·reset·bus·projection 포함 |
| Python 전체 관련 패키지 | **338 passed, 3 skipped** |
| 새 Python | CSV 8개, 실제 PathEngine→C++ Zone/관측 5개 포함 |
| ROS mock | **32 PASS + 추가 assertion**, MGM 하나/합성 입력/단일 TargetRef 송신자 |
| 보호 알고리즘/형식 | 위 본문 동일 비교, Python 문법, git diff --check 통과 |

Python skip 3개는 DepthAI 2.x 전용 serialization 환경 조건이다. 기존 SciPy가 NumPy 1.26.4에 대해
지원 버전 경고 1건을 냈다. 시험 실패로 세지 않았으며 이 의존성 조합을 변경하지 않았다.

빌드 `/tmp/mgm-stabilization-build`, 설치 `/tmp/mgm-stabilization-install`.
시험 로그는 `/tmp/mgm-stabilization-{build-final,ctest,python-final,ros-final}.log`다.
기존 운용 install을 덮어쓰지 않았다. driver/bridge underlay 경로가 빌드 검색에 존재할 수 있지만 실행하지 않았다.

```bash
source /opt/ros/humble/setup.bash
source /tmp/mgm-stabilization-install/setup.bash
ctest --test-dir /tmp/mgm-stabilization-build/adas_mgm --output-on-failure
PYTHONPATH="$PWD/src/stack_parking:$PWD/src/stack_gps:$PWD/src/stack_estop:$PWD/src/stack_avoid:$PWD/src/lidar_fusion_v2:$PWD/src/stack_traffic:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q src/stack_gps/test src/stack_estop/test src/stack_parking/test \
  src/stack_avoid/test src/lidar_fusion_v2/test src/stack_traffic/test \
  src/adas_mgm/test/test_reference_metadata.py src/adas_mgm/test/test_mission_calibration.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=173 /usr/bin/python3 src/adas_mgm/test/manager_ros_smoke.py \
  /tmp/mgm-stabilization-install/adas_mgm/lib/adas_mgm/mgm_node
```

## 21. 남은 실차 Calibration

Zone 확인 수/공간 hysteresis 및 평행·교차 segment 오선택, Parking timeout/최대 실제 탐색 거리/
trigger 위치, 범퍼 기준 소실 seed와 최종 정차 위치, provider 지연/Reference 추종,
rear corridor margin/coverage/dynamic 처리, Recovery activation delay/총 반복 제한을 측정해야 한다.
현재 운영 숫자를 임의 확정하지 않았다. 새 인터페이스는 provider와 MGM을 같은 overlay에서 함께 빌드해야 한다.
raw dump는 v12이고 구 버전은 해당 버전 빌드가 필요하다.

## 22. 실제 차량 구동 여부

**차량·센서 드라이버·CAN bridge를 실행하지 않았다.** localhost ROS domain 173에서 MGM과 합성 입력만 사용했다.
커밋·push·merge는 하지 않았으며 기존 사용자 변경을 보존했다.
