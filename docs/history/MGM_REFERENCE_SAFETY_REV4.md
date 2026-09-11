> **역사적 4차 기록 — 현재 운용/MBD 명세 아님.** 현행 기준은 [6차 단일 명세](../MGM_MBD_STATE_MACHINE_SPEC.md)다. 아래 구현/시험 결과/미해결 항목은 당시 시점의 기록이다.

# MGM 4차 Reference 안전 통합

> **5차 적용:** [Mission Preparation 보고서](../MGM_MISSION_PREPARATION.md)를 함께 적용한다.
> PREPARE에서는 Mission reference 공백이 정상이며 기존 Navigation/Avoidance reference를 검사한다.
> ACTIVE는 기존 Mission reference gate에 요청 ID·요청 이후 생성 조건을 추가한다. Dump는 v11이다.

2026-09-11, `feat/state-machine`. 3차 병행 Manager/Zone 구조를 유지하고 Reference 유효성,
생성 시각, 최종 속도 gate, SAFE_STOP reason을 추가했다. 3차 문서의 빈 Reference에 대한
양의 속도 허용은 이번 정지 정책으로 대체한다. 실차 구동은 수행하지 않았다.

## 1. Reference Provider의 실제 생성 위치

| Provider | 파일 / 함수 | 실제 생성과 재발행 구분 |
|---|---|---|
| LINE | `src/stack_lane/stack_lane/node.py::StackLaneNode.tick`, `lane_path.py::estimate_lane_path` | 새 camera frame에서 정상 estimate 생성. HELD/SEARCH는 `_held_estimate` 재사용 |
| GPS | `src/stack_gps/stack_gps/node.py::StackGpsNode.tick`, `path_engine.py::PathEngine.snapshot` | 기존 GNSS `fix_t`로 새 위치 표본 식별. 같은 fix의 timer 출력은 같은 generation |
| Avoidance | `src/stack_avoid/stack_avoid/node.py::StackAvoidNode.on_scan`, `_gap_target` | 입력 scan으로 gap 경로 또는 기존 clearance 유지점 생성. `scan.header.stamp` 재사용 |
| Mission | `src/stack_parking/stack_parking/node.py::_process_slam`, `_tick`; `mission.py::ParkingMission.tick` | 기존 Mission 경로의 로컬 preview. 실제 accepted localization scan 시각이 유효성 근거 |
| Recovery | `src/adas_mgm/core/mgm_step.cpp::build_escape_ref`, `assemble` | 기존 Escape 기하를 활성 후진 cycle마다 조립. 외부 heartbeat에 의존하지 않음 |

경로 생성 식/점/속도 profile은 바꾸지 않았다. Producer에는 `reference_stamp` 메타데이터만
추가했다. LINE의 HELD/SEARCH와 Parking의 rejected/미갱신 localization도 실제 생성으로
오인하지 않도록 각각 마지막 정상 estimate/accepted localization 시각을 유지한다.

## 2. available / valid / fresh 정의

`ReferenceSample`과 `ReferenceStatus`는 `core/manager_types.hpp`의 고정 크기 구조다.
`CoreSnapshot.references[5]`가 generation/age/기존 timeout을 공급하고,
`CoreOutput.references[5]`와 `selected_reference`가 검사 결과를 노출한다.

- **available**: 현재 provider가 하나 이상의 점을 제공한다. 경로 사용 가능 판정과 별개다.
- **fresh**: 실제 generation이 알려져 있고 그 나이가 기존 provider timeout 이내다.
  같은 generation은 곧바로 invalid가 되지 않고 기존 유효기간 동안 사용할 수 있다.
- **valid**: available + fresh + 센서/모듈 유효성 + 유효 점 개수 + 모든 x/y/yaw/curvature가
  finite + 전체 경로가 원점으로만 채워져 있지 않음 + 해당 provider 사용 조건.
- LINE은 confidence가 finite, 0..1이어야 하며 기존 low 50-cycle 조건을 유지한다.
  GPS는 fix/메시지 유효성이 필요하다. Mission은 active와 matching fresh module ack가
  필요하다. Escape는 실제 후진 phase, 기존 설정, rear_clear 및 조립 기하를 확인한다.

원점이 경로 첫 점이라는 이유만으로 정상 다점 경로를 거부하지 않는다. 모든 점의 x/y가
정확히 0인 기본 버퍼를 거부하며 새 거리 문턱은 만들지 않았다. 정상 Point 수는 1..20이다.

| 기존 설정 | 노드 기본값 | 현재 운용 YAML |
|---|---:|---:|
| `lane_stale_timeout_sec` | 0.5s | **1.0s** |
| `gps_stale_timeout_sec` | 0.5s | 0.5s |
| `avoid_stale_timeout_sec` | 0.5s | 0.5s |
| `parking_stale_timeout_sec` | 0.5s | 별도 지정 없음 → 기본값 |

`src/adas_mgm/config/params.yaml`과 `MgmNode`의 기존 값을 그대로 전달한다. 별도 Reference
유효기간이나 임의 추천값은 적용하지 않았다. 기존 GPS 자체 fix timeout 1.5s, Parking cloud
0.35s / SLAM 0.6s 등 내부 센서 검사는 그대로이며 MGM의 경로 유효성 검사는 별도로 적용한다.

## 3. generation과 publish 구분

기존 네 path 메시지에 `builtin_interfaces/Time reference_stamp`를 추가했다.
`header.stamp`의 발행/관측 시각과 분리하며 zero는 생성 근거 없음이다.

- LINE `_set_reference_stamp`: 정상 estimate 때 실제 capture 시각을 기록한다.
  HELD/SEARCH는 이전 시각을 유지한다. 기존 header가 오래된 capture에 대해 발행 시각으로
  fallback하더라도 reference_stamp는 원래 나이를 유지한다. capture 미상은 valid로 만들지 않는다.
- GPS `_set_reference_stamp`: 기존 `fix_t`가 바뀔 때만 GNSS 표본 시각을 갱신한다.
  같은 fix로 재계산/재발행하거나 동일 좌표를 유지하는 경우에도 새 fix인지 구분한다.
- Avoidance: 실제 입력 scan 시각을 사용한다. 새로운 scan에서 기존 `(1.5,0)` clearance
  유지점을 계산한 것은 정상 생성이다. 동일 scan을 다시 받는 것은 나이를 초기화하지 않는다.
- Parking: accepted SLAM 입력의 `stamp_s`를 저장한다. timer의 preview 재발행,
  pose 미갱신, rejected ICP로 나이를 초기화하지 않는다.

추적 중 발견한 상류 시각 세탁도 수정했다. `lidar_fusion_v2::FusionNode._publish`는 이제
현재 기여한 센서 중 **가장 최근 실제 입력 stamp**를 unified scan/cloud에 사용한다.
새 센서 입력이 없으면 timer가 돌아도 stamp가 유지된다. 각 raw cloud는 해당 센서 stamp를
쓴다. 최신 입력을 쓰므로 새 전방 scan이 들어왔을 때 `stack_estop`의 stamp 중복 제거가
그 입력을 놓치지 않는다. 기존 max_age 0.30s, 기여 센서 선택, 점 융합/FOV 알고리즘은 유지한다.
이 metadata는 모든 센서의 시야가 동시에 정상이라는 보증은 아니다.

`src/adas_mgm/src/reference_clock.hpp::ReferenceClock`은 실제 stamp마다 수신 지연을 포함한
나이를 설정하고 이후 monotonic 경과 시간으로 증가시킨다. 같은 stamp 재발행으로 초기화하지
않으며 0/미래/역행 stamp를 새 generation으로 수용하지 않는다. ROS 시간이 멈춰도 나이는
증가한다. 시계 역행 후에는 정상 시간 관계가 회복될 때까지 경로를 허가하지 않는다.
좌표 전체를 매 10ms 비교하는 새로운 generation 검출은 없다.

## 4. CLEAR_CONFIRM 처리

현재 주어진 기존 Avoidance path가 valid하고 timeout 이내면 계속 사용한다. generation이
동일하면 age는 증가하며, 단순 재발행은 timeout을 연장하지 않는다. 신호 감속/정지 override는
이 동안 그대로 적용된다.

경로가 비거나 NaN/Inf/기본값이거나 timeout을 넘으면 SAFE_STOP_REFERENCE_INVALID.
CLEAR_CONFIRM과 200-cycle 확인, 300-cycle LINE 보류는 유지한다. 임의 hold 유예나 다른
Reference를 생성하지 않는다. `avoid_episode_reference_seen`은 해당 회피 episode에서 유효한
경로를 소비한 이력이 있는지 보여주며 episode가 끝나면 초기화한다.

## 5. LiDAR fallback의 빈 path

LiDAR scan이 살아 있어도 주행 가능한 path가 없으면 정지한다. 회피 Manager/source 문맥은
유지하며 LINE/GPS로 몰래 교체하지 않는다. `lidar_valid=true`와 `avoid_ref_valid=false`를
동시에 관측할 수 있다. 센서 가용성과 Reference 가용성을 별도로 표현한다.

## 6. 초기 (0,0) 위험 제거

기존 `mgm_init`의 zero buffer와 점 1개는 정지 출력용 초기값으로 남는다.
실제 generation 이전, empty vector, zero-filled path는 valid=false다.
이 버퍼에 양의 속도를 붙이는 경우를 재현한 뒤 차단했다. 새 직진점/이전점 복제/임의 waypoint는
추가하지 않았다. malformed path는 assembler에 넣지 않으므로 기존 정상 버퍼에 NaN을 섞지 않는다.

## 7. 최종 speed gate

`manager_decision()`에서 선택 provider의 status를 확인하여 invalid면 즉시 정지한다.
`mgm_step()`은 valid provider만 기존 assembler에 전달하고, 기존 speed merge 이후
`reference_safety.cpp::final_reference_gate()`에서 다시 선택 validity/freshness/source,
최종 출력 기하와 속도의 finite 여부를 검사한다. 무효이면 속도와 내부 ramp 상태를 0으로 한다.

`MgmNode::tick`도 `TargetRef` 발행 직전에 선택 validity/기하/속도를 확인한다.
양의 속도뿐 아니라 Mission·Recovery의 음수 속도도 invalid Reference와 함께 허용하지 않는다.
유효하지 않은 기존 buffer와 **0 속도**의 정지 명령은 허용한다. CAN 송신 counter를 멈추는
방식에 의존하지 않고 새 정지 속도 명령을 계속 보낸다.

## 8. SAFE_STOP reason mask와 진단

새 최상위 State를 만들지 않고 기존 SAFE_STOP에 독립 uint32 mask를 연결했다.

| Bit | Reason |
|---:|---|
| 1 | GPS_ONLY_GPS_LOSS — fix뿐 아니라 사용할 GPS Reference가 없는 경우 포함 |
| 2 | ALL_SENSORS_LOST — 일반 주행 센서 입력이 모두 없는 경우 |
| 4 | REFERENCE_INVALID — 선택 경로 없음/무효/stale 또는 최종 명령 gate 실패 |
| 8 | EXTERNAL — 운전자/CAN 등 기존 외부 정지 |
| 16 | MISSION_FEEDBACK — Mission 모듈 입력 미수신/timeout |
| 32 | TRAFFIC_INPUT — 기존 traffic fail-safe 입력 |
| 64 | VEHICLE_SPEED — 신호 정지 중 실제 속도 입력 무효 |
| 128 | REAR_UNAVAILABLE — Escape Reference 승인 단계에서 후방 확인 없음 |

매 cycle 현재 조건으로 각 reason을 독립 계산한다. 하나가 해제되어도 다른 reason이 있으면
정지한다. FINISH, disable, AUTO_ESTOP의 기존 상위 정지 규칙도 유지한다. 후방 미확인은
평상시 전진을 막는 전역 정지 사유로 사용하지 않는다. 정상 전이에서는 후진 종료/진입 차단으로
먼저 처리하고 Escape 승인 gate에도 확인을 둔다.

`/adas/mgm_state`에 selected source, valid/fresh/age/generation, provider별 valid,
avoid episode 이력, active_safe_stop_reasons를 추가했다. 기존 source/available도 유지한다.
전이 로그는 throttle하고 CSV와 core replay에 유효성·나이·reason을 기록한다.
코어 검사는 고정 5 provider와 최대 20점으로 한정된다. 새 동적 할당이나 무제한 console 출력은 없다.

## 9. 허용/금지 fallback

일반 구간의 LINE Reference 무효 → usable GPS의 GPS_BACKUP은 기존 정책으로 허용한다.
GPS Reference 무효 → 기존 LINE_RETURN_READY를 만족하는 LINE 복귀도 유지한다.
LINE high 50-cycle 조건과 GPS 불가 시 남은 LINE hold만 생략하는 정책은 그대로다.
GPS-only Zone에서는 GPS Reference가 없다고 LINE/LiDAR로 대신하지 않는다.

Mission 또는 Avoidance가 활성이고 그 Reference가 무효이면 다른 provider로 조용히
fallback하지 않는다. Mission done, 회피 200-cycle 완료 등 **기존 종료 전이** 뒤의
Navigation 선택은 계속 허용된다.

## 10. Mission Reference invalid

Mission state/type/ID/완료 기억을 유지하면서 SAFE_STOP_REFERENCE_INVALID로 정지한다.
matching active ack 이전에도 정지하며, 경로가 회복되면 같은 Mission이 제어권을 유지한다.
invalid 때문에 새로운 Mission 완료/실패 판정을 만들지 않았다. fresh 기존 done이 종료를 결정한다.

## 11. Avoidance Reference invalid

AVOID_ACTIVE/CLEAR_CONFIRM의 소유권을 유지하고 속도는 0이다.
새 유효 generation이 도착하면 같은 회피 경로를 사용하고, 다른 정지 reason이 없을 때만 움직인다.
실제 장애물 소실과 기존 timer에 따른 정상 종료는 별도로 진행한다. green이 들어와도
Reference invalid reason은 해제되지 않는다.

## 12. Recovery 실제 설정과 반복 제한

- 위치: `src/adas_mgm/config/params.yaml`, `src/mgm_node.cpp`의 파라미터 선언.
  실차 launch는 YAML의 `escape_after_cycles`를 읽어 전달한다.
- 현재 `escape_after_cycles=0`: **disable**이다. `update_escape`는 반드시 `>0`을 요구한다.
  즉시 시작의 의미가 아니다. 최초 도입 commit `4e76172`와 후속 `a612af3` 설정도 0이었다.
  주석의 1000 cycle=10s는 예시다. 기존 단위시험의 20-cycle 값은 시험용이다.
- `v_escape=-0.3m/s`, `escape_max_cycles=200`, `escape_require_rear_clear=true` 유지.
- 기존 진입: 이전 전진 명령으로 armed, 실제 auto-estop 연속 유지, 양수 delay/max,
  음수 속도, 후방 여유, DRIVE 및 다른 강제 정지/Mission/신호 등과 충돌하지 않음.
- 종료: 시간 상한, 후방 막힘/미확인, 실제 estop 해제, 설정 또는 실행 자격 해제.
  종료 후 NAV_RESELECT와 기존 Zone/Avoidance/Signal/Sensor 조건을 다시 평가한다.
- 반복: 종료 시 지속 counter를 지우므로 설정 delay를 다시 채워야 한다.
  **총 반복 횟수나 전체 누적 후진 거리의 별도 상한은 기존 코드에 없다.**
  -0.3m/s × 2s = 0.6m는 한 phase의 명령상 계산이며 실측 이동거리 보증이 아니다.

이번 base Manager에서는 legacy `escape_require_rear_clear=false`를 주더라도 후방 확인 없이
자동 후진을 시작하지 않는다. 기존 Recovery 알고리즘/기하를 바꾸지 않고 제어권 승인과
최종 Reference gate에서 막는다. generated v1.88은 별도 legacy 경로이며 기존 지원 제한을 유지한다.

## 13. rear_clear 생산 가능성

코드/설정 기준으로 **A: 후방 데이터 경로가 존재하지만 해당 판단이 연결되지 않은 상태**다.
실제 이 PC에 연결된 센서의 전원/장착/유효 시야는 이번 무구동 시험으로 확인하지 않았다(E).
센서 자체가 없다고 단정할 근거는 없다.

| 확인 항목 | 근거 및 재사용 가능 범위 |
|---|---|
| 후방 센서/토픽 | `lidar_fusion_v2/config/fixed_geometry.yaml`: a2, `/lidar/a2/scan`; 기존 `/lidar/a2/cloud`와 unified scan 경로도 존재 |
| 후방 거리 생산부 | `stack_parking/node.py::_on_rear_scan`, `_rear_clearance` → `latest_rear_clearance_m` |
| 기존 주차 판정값 | sector 중심 -90°, 반폭 12°, offset 0.069m, 최소 5점, cluster 0.04m, scan timeout 0.35s; 주차 완료 거리 0.20m |
| 현재 의미 | 주차 후진 중 뒤쪽 벽에 도착했는지 판정하는 재료. 장애물 없는 후진 통로 전체의 안전 승인과 다름 |
| 기존 충돌 판정 | Parking의 static map / dynamic path-blocked는 Mission 경로를 대상으로 함. 일반 Escape rear_clear와 동등한 입력이 아님 |
| MGM 소비 | `EstopRequest.rear_clear` → wrapper → `estop_rear_clear` → `update_escape` 및 Reference 승인 |
| 생산 누락 | `stack_estop/node.py::publish_current_level`은 estop/scan_valid만 설정. rear_clear는 기본 false |

기존 거리 함수와 센서 입출력 코드는 향후 재사용 후보지만, 위 0.20m 등을 일반 후진 허용
threshold로 치환하지 않았다. 장착 좌표/FOV, 차량 폭을 포함한 후진 통로, 정지 여유, missing scan의
false 처리까지 정의·검증되어야 한다. 현재 주차용 좁은 sector 판정만으로 자동 rear_clear=true를
발행할 수 있다고 결론내리지 않는다.

## 14. 현재 Recovery 활성화 가능 여부

현재는 **운용 활성화 조건이 충족되지 않았다**. config OFF와 rear_clear 생산 누락 모두 남아 있다.
rear_clear를 임의 true로 넣거나 delay를 바꾸지 않았다. 실제 Escape 경로 생성과 state 연결은
모의 입력으로 검증했으며, valid Escape 기하/후방 확인 없이는 후진 속도를 출력하지 않는다.

## 15. Zone boundary chatter 결과

기존 `_nearest_idx`는 전체 track의 최근접 점을 찾고 `_in_ranges`는 포함 여부를 바로 판단한다.
Zone hysteresis/debounce는 없다. GPS 기본 10Hz 입력, MGM 100Hz를 기준으로 모의시험했다.

| 합성 입력, 10초 / 100 GPS 표본 | Entry | Exit |
|---|---:|---:|
| 정지 상태에서 경계 양쪽으로 ±1cm 교대 | 50 | 50 |
| 20cm 간격 평행 track의 중간에서 ±1cm 교대 | 50 | 50 |
| 교차 track에서 두 축 방향 1.1cm 교대 | 50 | 50 |
| 경계에서 seed 고정 Gaussian 위치 잡음 σ=1.5cm | 23 | 23 |
| 같은 잡음이 경계에서 떨어진 내부에 존재 | 1 | 0 |

교대 membership을 core에 10Hz로 공급한 시험에서는 Navigation 전환 100회/10초,
Mission 시작 **1회**였다. 활성 Mission/완료 기억은 반복 실행을 막지만 GPS-only Navigation
떨림을 제거하지는 않는다. 위 좌표/폭은 시험 입력이며 실차 Zone이나 설정에 추가하지 않았다.

후보는 경계 진입/이탈 폭 분리, 연속 유효 GNSS 표본으로 edge 확정이다. 평행/교차 track에는
이전 segment·진행 방향 문맥을 사용하는 판정도 검토 대상이다. 실제 위치오차/track 간격과
허용 진입 지연을 측정한 뒤 폭/표본 수를 결정해야 하며 이번에는 임의 수치를 적용하지 않았다.

## 16. 추가/변경 테스트

- `reference_safety_test.cpp`: 요구 1–16, 18–22와 geometry/시계 경계 조건 **60 checks**.
  요구 17 후방 입력 재사용 가능성은 §13의 실제 코드/설정 추적으로 검증했다.
- 기존 `manager_state_test.cpp` **78 checks**, `zone_manager_test.cpp` **49 checks** 유지.
  3차에서 위험 동작을 기록하던 empty-path 양의 속도 기대값을 이번 정지 정책으로 바꾸고,
  Recovery 경로 대기는 SAFE_STOP + 기존 대기 문맥으로 검증한다.
- `test_reference_metadata.py` **6 tests**: 실제 producer wrapper 메서드를 센서/GPU 생성 없이
  실행한다. LINE HELD/SEARCH, 오래된/미상 capture, GPS 동일 fix, Parking pose 재사용,
  Avoidance 새/반복 scan, fusion timer 재발행과 실제 새 센서 입력을 검증한다.
- `stack_gps/test/test_zone_chatter.py` **5 tests**: 위 공간 경계/평행/교차/잡음 재현.
- `manager_ros_smoke.py`: 기존 연결과 함께 generation 고정 heartbeat의 stale 정지,
  Mission 후진 경로의 stale 정지와 같은 Mission의 복구를 추가했다.

수정 전에 초기 zero ref/빈 CLEAR_CONFIRM/NaN Mission의 3개 위험을 재현했고 모두 실패했다.
동일 재현은 안전 계층 적용 뒤 통과했다.

## 17. 전체 시험 결과

- 영향 패키지 **8개 빌드 성공**: fma_interfaces, adas_mgm, stack_lane, stack_gps,
  stack_avoid, stack_estop, stack_parking, lidar_fusion_v2.
- **CTest 13/13**, Manager 78 + Zone 49 + 새 Reference 60 checks 통과.
  기존 generated parity, Recovery, reference hold, traffic, parking 회귀 포함.
- **Python 175/175 통과**: GPS, E-stop, Parking, Avoidance, fusion, metadata 시험.
  설치된 SciPy/NumPy 버전 조합 경고 1건이 있었으나 시험 실패는 없었다.
- **ROS 모의 연결 23단계와 단일 송신자/중복 Mission 방지 검사 통과**.
  DDS domain 172, localhost 한정, MGM과 합성 입력만 실행했다.
- fusion의 새 전방 scan 처리까지 확인한 마지막 metadata 보완 후 관련 Python **19/19 재통과**.
- `git diff --check`, Python 문법 검사 통과. 실제 CAN bridge/센서/차량은 실행하지 않았다.

시험 결과는 `/tmp/mgm-reference-build`, `/tmp/mgm-reference-install`에 분리했다.
운용 install을 덮어쓰지 않았다. 저장소 루트에서 재현한다.

```bash
source /opt/ros/humble/setup.bash
colcon --log-base /tmp/mgm-reference-build-log build \
  --base-paths src/fma_interfaces src/adas_mgm src/stack_gps src/stack_lane src/stack_avoid src/stack_estop src/stack_parking src/lidar_fusion_v2 \
  --build-base /tmp/mgm-reference-build --install-base /tmp/mgm-reference-install \
  --packages-select fma_interfaces adas_mgm stack_gps stack_lane stack_avoid stack_estop stack_parking lidar_fusion_v2 \
  --allow-overriding fma_interfaces \
  --cmake-args -DBUILD_TESTING=ON -DADAS_MGM_ENABLE_GENERATED_BACKEND=ON
ctest --test-dir /tmp/mgm-reference-build/adas_mgm --output-on-failure
source /tmp/mgm-reference-install/setup.bash
python3 -m pytest src/stack_gps/test src/stack_estop/test src/stack_parking/test src/stack_avoid/test src/lidar_fusion_v2/test src/adas_mgm/test/test_reference_metadata.py -q
ROS_DOMAIN_ID=172 ROS_LOCALHOST_ONLY=1 python3 src/adas_mgm/test/manager_ros_smoke.py /tmp/mgm-reference-install/adas_mgm/lib/adas_mgm/mgm_node
```

## 18. 이번 4차 변경 파일

| 영역 | 파일 |
|---|---|
| Core | `src/adas_mgm/core/manager_types.hpp`, `mgm_types.hpp`, `manager_step.cpp`, `mgm_step.cpp`, 새 `reference_safety.hpp/.cpp` |
| Wrapper | `src/adas_mgm/src/mgm_node.cpp`, 새 `reference_clock.hpp` |
| 빌드/설정/재생 | `src/adas_mgm/CMakeLists.txt`, `config/params.yaml`의 설명, `tools/dump_format.hpp`, `tools/core_replay.cpp` |
| 메시지 | `src/fma_interfaces/msg/LanePath.msg`, `GpsPath.msg`, `AvoidStatus.msg`, `ParkingStatus.msg`, `MgmState.msg`, `CMakeLists.txt`, `package.xml` |
| Producer metadata | `src/stack_lane/stack_lane/node.py`, `src/stack_gps/stack_gps/node.py`, `src/stack_avoid/stack_avoid/node.py`, `src/stack_parking/stack_parking/node.py`, `src/lidar_fusion_v2/lidar_fusion_v2/fusion_node.py` |
| Tests | `src/adas_mgm/test/manager_test_fixture.hpp`, `manager_state_test.cpp`, `zone_manager_test.cpp`, `manager_ros_smoke.py`, 새 `reference_safety_test.cpp`, `test_reference_metadata.py`; 새 `src/stack_gps/test/test_zone_chatter.py` |
| 문서 | `CLAUDE.md`, `docs/MGM_REFERENCE_SAFETY.md`, `docs/MGM_BASE_STATE_MACHINE.md`, `docs/MBD_KIT.md`, `src/adas_mgm/README.md` |

raw snapshot dump는 **v10**이다. 이전 dump는 해당 버전 빌드로 재생한다.
메시지 변경을 반영하려면 provider와 MGM을 함께 빌드/source해야 한다. reference_stamp를
제공하지 않는 옛 producer의 경로는 새 base core에서 invalid다. 생성 v1.88은 legacy 비교용이며
이번 Reference gate를 구현한 Stateflow 모델로 취급하지 않는다.

기존 사용자 변경은 보존했다. 이번 수정 전 백업은 `/tmp/mgm-before-reference-revision/`이다.
commit/push는 수행하지 않았다.

## 19. 기존 알고리즘 내부 변경 여부

LINE/GPS/Avoidance/Parking의 경로 생성, GPS Zone 공간 판정, Signal perception,
E-stop 계산, 조향/속도 제어, Escape path 생성은 수정하지 않았다.
기존 `build_escape_ref`, `assemble`, `merge` 함수 본문은 3차 백업과 동일하다.
새 코드는 그 앞뒤의 validity/generation/stop gate 및 metadata 계층에 있다.
LINE/GPS/주차의 실제 생성 근거와 fusion의 실제 입력 timestamp를 노출하는 producer wrapper
변경은 포함한다. 생성 ADAS_MGR2 C 코드, 임계값, 실차 좌표는 변경하지 않았다.

## 20. 남은 실차 확인

- 새 메시지와 producer metadata가 실제 capture/GNSS/scan/localization 주기에 맞게 전달되는지,
  기존 timeout에서 지연 여유가 충분한지, 실제 100Hz 부하에서 확인.
- valid로 표기된 기하의 현장 추종 정확도, 무효 전환 시 실제 제동 거리/조향 유지,
  Mission 정지·복구 및 CAN 수신/하드웨어 E-stop. 이번 검증은 명령 선택·송신값 검증이다.
- 측정한 Zone 경계, 평행/교차 구간의 실제 chatter와 허용 지연을 바탕으로 안정화 정책 결정.
- rear 센서 장착/FOV/신선도/후진 통로와 정지 여유를 검증하고 rear_clear 생산 정책 결정.
  현재는 자동 Recovery OFF 유지. 총 반복 제한을 추가할지는 기존 동작과 별도로 결정할 사항이다.
