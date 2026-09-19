> **역사적 5차 기록 — 현재 운용/MBD 명세 아님.** 현행 기준은 [6차 단일 명세](../MGM_MBD_STATE_MACHINE_SPEC.md)다. 아래 구현/시험 결과/미해결 항목은 당시 시점의 기록이다.

# MGM Mission Preparation — 5차 설계

2026-09-11, `feat/state-machine`. 코드 수정 전 작성한 설계 기준.

MISSION_ZONE entry는 Mission ID/type/source Zone ID를 갖는 요청을 latch한다.
`MISSION_PREPARE=2`를 추가한다(IDLE=0, ACTIVE=1 wire 값 유지). PREPARE 동안
Navigation/Avoidance/Signal/Safety를 그대로 실행하며 Parking reference 부재는 정상이다.
현재 요청의 준비 완료를 받은 틱에 ACTIVE로 전환하고 그때부터 Parking만 제어권을 갖는다.
4차 Reference validity/final gate, 외부·운전자·CAN 정지, 50/200/300 cycle은 유지한다.

## 요청과 모듈 연결

MissionRequest에는 active, request_id, Mission ID/type, source Zone ID, 시작 monotonic 시각,
시작 위치/track index, 경과 시간, 실제 이동거리, 준비 응답, 취소 사유와 측정 milestone을 저장한다.
요청 ID는 ROS 시각과 보존된 high-water counter로 생성하며 session reset에서도 재사용하지 않는다.
ParkingCommand(PREPARE/ACTIVATE/CANCEL)는 request_id를 전달한다. 모듈은 ID를 응답하고
명령 재전송을 멱등 처리한다. 늦게 온 이전 ID의 명령/space_found/ready/done/ref는 거부한다.

PREPARE 명령은 기존 mission/detector/plan reset, SLAM/prior/map/pipeline reset을 사용한다.
보관된 LiDAR/ICP/reference 입력도 비워 이전 결과가 새 session에 섞이지 않게 한다.
기존 PipelineController의 SLAM→MAPPING→LOCALIZATION→PARKING 및 생성된 plan,
기존 localization freshness가 준비 완료의 근거다. 공간 검출과 준비 완료를 따로 노출한다.
준비 동안 경로 실행 tick은 보류하여 ACTIVE 명령 전에 approach/reverse 단계가 진행되지 않는다.
검출·ICP·플래너·주차 진행 알고리즘 자체는 변경하지 않는다.

## 수명과 제한

Zone exit/chatter 및 다른 Zone entry는 latch된 요청을 변경하지 않는다. queue는 없다.
done은 현재 요청/모드의 실행 응답 이후만 인정하고 Mission ID별 완료 기억과 nav_reselect를 유지한다.
취소는 timeout/distance/explicit/FINISH/session reset/module cancel로 구분하며 완료로 기록하지 않는다.
외부 stop은 기존 임시 정지 level이므로 요청을 유지하되 timeout은 실제 경과 시간대로 진행한다.

`parking_search_timeout`(s), `max_parking_search_distance`(m)는 기존값/측정 데이터가 없다.
기본 `-1.0`은 미설정 표식이며 운영 제한값이 아니다. 둘 다 유한 양수여야 탐색을 시작한다.
미설정이면 CALIBRATION_REQUIRED로 요청을 취소하고 일반 주행을 유지한다.
실제 속도 VehicleVector.v의 절댓값과 monotonic 경과 시간을 적분한다(명령 속도 사용 금지).
속도 freshness를 잃거나 시간이 역행하면 제한을 검증할 수 없으므로 MOTION_UNAVAILABLE로 취소한다.
테스트에만 시간/거리 숫자를 넣는다. 기존 cloud/slam timeout은 탐색 수명 설정으로 전용하지 않는다.

## Calibration

ZoneDefinition의 독립 index/range로 탐색 trigger 위치를 표현할 수 있으므로 구간을 임의로
넓히거나 선행 거리를 만들어 넣지 않는다. maneuver 위치는 기존 detector/planner가 결정한다.
Zone entry, Search 시작 응답, space 발견, ready, ACTIVE 각각의 시각·GPS ENU 위치·track index·
실제 속도·현재 Zone ID와 누적 실제 이동거리를 MgmState와 선택적 CSV로 기록한다.
위치는 기존 GPS ENU/idx를 메시지로 노출하며 경로 기하는 변경하지 않는다.
Search 시작/검출/ready 시점은 MGM의 현재 세션 응답 수신 관측 시각이다(통신 지연 포함).
정확한 센서 입력 시각은 기존 reference_stamp로 별도 추적한다.

구현과 아래 모의 검증을 완료했다. 실차·센서 드라이버·CAN bridge는 구동하지 않았다.

## 구현 결과 — 요청한 23항목

| 번호 | 보고 항목 | 결과 / 구현 위치 |
|---|---|---|
| 1 | 기존 `parking_zone && space_found` 위치 | `src/adas_mgm/core/mgm_step.cpp::transition()`의 LANE/WAYPOINT→PARKING 두 곳, `src/adas_mgm/src/transition_log.cpp`의 legacy 전이 검증. Generated v1.88도 기존 계약이다. **수정 전 4차 병행 core에는 이 AND 조건이 없었으며, Zone entry 즉시 ACTIVE로 진입하고 reference를 기다리는 구조였다.** |
| 2 | 제거/변경 여부 | 현행 `base_state_machine_enabled=true`는 `mission_step()`의 latch된 request + 현재 session readiness로 전환하며 Zone occupancy를 요구하지 않는다. 과거 dump/generated parity용 `false` 경로의 두 조건과 로그는 유지했다. 이를 현행 Mission 경로로 사용하지 않는다. |
| 3 | PREPARE 구현 | `core/manager_types.hpp::MissionState`, `core/mission_step.cpp::mission_step()`. IDLE=0 / ACTIVE=1 값 호환을 유지하고 PREPARE=2 추가. |
| 4 | Request 데이터 구조 | `MissionRequest`: active, request_id, mission_id/type, source_zone_id, start_time_ns, last_update_ns, elapsed_s, travel_distance, search_acknowledged, space_found, preparation_ready, cancel_reason, 다섯 MissionObservation. 시작 위치/index는 zone_entry에 저장. 별도 RequestState 머신 없음. |
| 5 | Zone entry 생성 흐름 | `zone_step()`→유효 MISSION_ZONE entry/type/미완료 ID/다른 요청 없음 확인→request latch→PREPARE→`ParkingCommand.PREPARE`. 기존 autonomous enable/session/FINISH gate 유지. |
| 6 | Zone exit 이후 유지 | active request 분기는 Zone 포함 여부를 검사하지 않는다. Zone edge는 계속 관측하고 source_zone_id/시작 시각은 변경하지 않는다. |
| 7 | Parking Search 초기화 | `stack_parking/node.py::_on_mission_command()`→`_cancel_search()`→`_start_mission(...force_reset=True)`→기존 mission.trigger/reset, SLAM/prior/pipeline reset. legacy map-reset 옵션이 false여도 새 MGM request는 초기화한다. |
| 8 | space/ready 생성 위치 | 기존 `ParkingMission.observe_map()`에서 detector.update와 planner.plan 성공 시 space/plan 저장. `_process_slam()`이 PipelineController.plan_ready()를 호출하여 LOCALIZATION 진입. 기존 accepted scan 확인 후 PARKING. `_preparation_ready()`가 이 단계와 실제 plan/path, 기존 localization validity/시각을 노출한다. |
| 9 | 이전 결과 reset | Mission reset의 space/plan/path/progress/detector 초기화를 재사용한다. SLAM map/prior, cloud pair queue, pending merged cloud, accepted ICP, reference generation, 후방 거리, debug cache도 비운다. |
| 10 | stale 결과 방지 | uint64 request ID의 명령/응답 일치 + fresh 수신 + mode 일치. 준비 완료는 요청 이후 `preparation_stamp` 생성과 기존 parking freshness timeout까지 검사한다. 이전 ID 명령은 거부, 같은 PREPARE는 멱등, 먼저 받은 CANCEL은 같은 ID의 늦은 PREPARE를 막는다. 새 Search 전 timestamp의 queued scan도 거부한다. ACTIVE reference도 요청 ID 및 요청 이후 생성 시각을 검사한다. |
| 11 | PREPARE→ACTIVE 조건 | active request, 아직 timeout/distance 미초과, 관측 가능한 실제 속도, 현재 request/mode의 새 status, search_active, 기존 pipeline readiness, 유효한 요청 이후 preparation generation. **Zone 내부 조건 없음.** |
| 12 | 정확한 제어권 인계 | 위 조건이 성립하는 `mission_step()` 틱에서 ACTIVE 및 mission_start=true. 같은 틱 manager_decision은 Parking reference/속도 소유자를 선택하고 일반 Avoidance/AUTO_ESTOP을 마스킹한다. 기존 handoff 처리가 Nav geometry를 지우고, ACTIVATE 실행 응답과 유효 Mission reference가 올 때까지 0 속도 정지한다. 모듈은 ACTIVATE 전에 mission.tick의 실행 진행을 보류한다. |
| 13 | Search Timeout | 시작 monotonic 시각으로부터 실제 경과 시간과 `parking_search_timeout`(s) 비교. 준비 중 ready보다 timeout 취소가 우선한다. 외부 stop 중에도 실제 경과 시간이 흐른다. ACTIVE 주차 진행 시간 제한으로 전용하지 않는다. |
| 14 | Search Distance Limit | `abs(VehicleVector.v) × monotonic_dt`를 PREPARE 동안 적분하여 `max_parking_search_distance`(m)와 비교. 실제 속도 기반 누적 추정이며 명령 v_ref/목표속도를 적분하지 않는다. 후진도 거리로 누적. 기존 0.2s vehicle freshness를 잃으면 MOTION_UNAVAILABLE 취소. |
| 15 | 설정 상태 | 두 제한값은 ROS/운용 YAML/launch 모두 **-1.0=미설정**. 임의 운영 숫자 없음. 하나라도 미설정/0/비유한 값이면 CALIBRATION_REQUIRED 취소, Parking 탐색 명령 없이 일반 주행 유지. 테스트의 30s/30m 및 짧은 경계값은 test-only. 아래 기존 자료 조사 참고. |
| 16 | Cancel Reason | NONE=0, SEARCH_TIMEOUT=1, TRAVEL_DISTANCE=2, EXPLICIT=3, FINISH=4, SESSION_RESET=5, CALIBRATION_REQUIRED=6, MOTION_UNAVAILABLE=7, MODULE_ABORT=8. `/operator/cancel_mission` Bool true가 명시 취소 입력. 모듈의 기존 cancel/reset/stop 응답도 현재 Search ack 이후 MODULE_ABORT로 인식. 취소는 완료 기억을 쓰지 않는다. |
| 17 | 다음 Mission Zone | 요청이 있는 동안 다른 entry는 덮어쓰거나 queue하지 않는다. 취소/완료 이후 **새 entry**가 있으면 미완료 Mission ID에 새 request를 만든다. 이미 지나간/계속 머무는 다른 Zone을 암묵적으로 재생하지 않는다. |
| 18 | 완료 후 Zone 재평가 | 현재 session 실행 ack 이후 기존 ParkingMission.done만 완료 인정. request.active 해제, `mission_completed[mission_id]` 기록 후 공통 `nav_reselect()`. 현재 GPS_ONLY_ZONE 우선, 나머지는 기존 LINE high 50/hold 300 조건과 GPS_BACKUP 사용. |
| 19 | Reference Safety 관계 | PREPARE는 일반 Nav/Avoidance 선택 경로에 기존 available/fresh/valid와 final gate를 그대로 적용한다. Mission ref 부재로 Mission stop reason을 만들지 않는다. ACTIVE의 invalid/stale Mission ref는 제어권 유지 + 0 속도 정지. 외부/operator/CAN 정지와 독립 reason mask 유지. |
| 20 | 테스트 | CTest **14/14**, 기존 Manager **78**, Zone **49**, Reference **60**, 새 Preparation **65** checks 통과. 관련 Python **183 passed**(새 wrapper 8 포함). ROS 모의 연결 **28 PASS** 및 추가 assertion 통과. 아래 재현 방법/한계 참고. |
| 21 | 변경 파일 | 아래 파일 목록 참조. 4차 이전의 기존 미커밋 변경은 보존했다. |
| 22 | Parking 알고리즘 내부 변경 | `mission.py`, `path_planner.py`, `space_detector.py`, `icp_slam.py`, `localization.py` 및 GPS `path_engine.py` **변경 없음**. Wrapper에 요청 초기화/실행 허용/기존 readiness 노출만 추가했다. |
| 23 | 실차 calibration | Search timeout, 최대 실제 탐색 거리, Search trigger 시작/끝 위치와 선행 거리, Zone 크기, GPS boundary debounce/hysteresis를 측정해야 한다. debounce/hysteresis에 새 임의 숫자는 넣지 않았다. 아래 기록 필드로 향후 결정한다. |

## 기존 값·측정 자료 조사

기존 PipelineController는 SLAM accepted **10 scan**, localization accepted **3 scan**,
최소 map **80 point**를 사용한다. cloud stale **0.35s**, SLAM stale **0.6s**,
MGM Parking reference/status stale **0.5s**는 입력 품질 제한이다. 탐색 전체 수명이나
허용 이동거리로 사용할 근거가 없어 그대로 두었다.

기존 [parallel parking tick CSV](analysis/run_0904_005737_parallel_parking_ticks.csv)에는
`parallel_searching_for_gap` 구간 3,792 sample, 다음 `parallel_waiting_to_pass_valid_point`까지
**37.920s**가 기록되어 있다. `abs(act_v) × sample_dt` 적분은 **2.188m**다.
이는 이전 parallel/wall-gap 시험의 단일 구간 관측값이다. 현재 SLAM→MAPPING→LOCALIZATION
파이프라인의 Zone entry/초기화/ready 시각, 여러 속도/실패 조건의 최대 탐색값이 없어
새 구조의 timeout/거리 제한으로 채택하지 않았다. 실제 속도 입력 freshness까지 소급
증명하는 수치는 아니므로 정밀 측정 거리로 간주하지 않는다.

PC의 `/home/sangmin/FMA_ws/drive_logs/run_0910_*` 네 run의 transitions.csv에는
LANE→WAYPOINT 각 한 건, parking_zone/found/done=0만 있었다. 새 Search lifetime을
보정할 이벤트 시계열이 아니다. `stack_parking/simulation.py`는 synthetic map을
직접 observe_map에 공급한 뒤 주차 maneuver ticks를 계산하므로 실제 탐색 시간 분포가 아니다.
이 자료들과 코드에서 검증된 현재 Search 제한값은 찾지 못했다.

Search trigger는 기존 ZoneDefinition의 `index_range` 또는 측정한 start/end 좌표로 표현한다.
Maneuver 위치는 detector/planner가 찾는다. 기존 parking range를 앞선 explicit trigger로
대체할 때는 기존 range 설정도 제거/대체해야 한다. 서로 다른 range를 둘 다 남기면
자동 생성된 별도 Mission ID가 생긴다. 이번에는 좌표·폭·선행 거리를 임의 변경하지 않았다.

## 진단과 보정 기록

`MgmState`는 Mission state, request active/ID/type/source Zone, monotonic start time,
경과 시간, 실제 속도 기반 누적 거리, search ack/space/ready, cancel reason, 완료 기억과
현재 Zone ID/type을 함께 노출한다. 다섯 Observation은 종료 뒤에도 다음 request까지 보존한다.

| 이벤트 | 시각 및 측정 데이터 |
|---|---|
| zone_entry | request latch 시각, GPS ENU position/validity, current track index, 실제 속도/validity, 현재 Zone ID |
| search_start | 현재 request의 첫 Search ack 관측 시각과 같은 측정 필드 |
| space_found | 기존 space+plan 생성 응답의 첫 관측 시각과 해당 누적 거리 |
| ready | 기존 localization readiness를 유효하게 받은 시각과 해당 누적 거리 |
| handoff | core가 ACTIVE로 바꾼 시각과 같은 측정 필드 |
| cancel/done | 마지막 request metadata, 종료 시각/위치/속도, cancel reason |

Search/space/ready의 관측은 MGM 입력 수신 기준이므로 통신/발행 지연을 포함한다.
실제 준비 pose 생성 시각은 `ParkingStatus.preparation_stamp`, 실행 ref는 `reference_stamp`다.
GPS ENU/index는 PathEngine의 기존 GNSS ENU/idx를 그대로 노출한다. freshness가 없으면
position_valid=false/index=-1로 기록한다. 목표점 index나 명령 속도를 측정값으로 사용하지 않는다.

`mission_events_csv_path`가 비어 있지 않으면 위 이벤트를 CSV로 기록한다. 실차 통합 launch는
run 디렉터리의 `mission_events.csv`를 설정하며 `/adas/mgm_state`, `/parking/mission_command`,
`/operator/cancel_mission`도 rosbag 기록 목록에 포함한다. 이번 작업에서는 실차 launch를 실행하지 않았다.

## 검증과 재현

격리 build/install은 `/tmp/mgm-preparation-build`, `/tmp/mgm-preparation-install`이다.
공통 interfaces, MGM, GPS, Parking 및 나머지 인지 provider를 포함한 8개 패키지가 빌드됐다.
이전에 빌드된 로컬 underlay 일부 driver/bridge 의존성은 빌드 검색 경로에 존재했지만,
검증 중 실행한 제어 노드는 mock 입력을 받는 MGM 하나다. 센서/CAN bridge를 띄우지 않았다.

```bash
source /opt/ros/humble/setup.bash
source /tmp/mgm-preparation-install/setup.bash
ctest --test-dir /tmp/mgm-preparation-build/adas_mgm --output-on-failure
PYTHONPATH="$PWD/src/stack_parking:$PWD/src/stack_gps:$PWD/src/stack_estop:$PWD/src/stack_avoid:$PWD/src/lidar_fusion_v2:$PYTHONPATH" \
  /usr/bin/python3 -m pytest -q src/stack_gps/test src/stack_estop/test src/stack_parking/test \
  src/stack_avoid/test src/lidar_fusion_v2/test src/adas_mgm/test/test_reference_metadata.py
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=172 /usr/bin/python3 src/adas_mgm/test/manager_ros_smoke.py \
  /tmp/mgm-preparation-install/adas_mgm/lib/adas_mgm/mgm_node
```

새 core test는 요청한 20개 상황에 더해 stale ready timestamp, 잘못된 ID/mode/done/ref,
기한과 ready 동시 도착, session reset 후 ID 재사용 방지, 미설정 제한, 실제 속도 상실을 검사한다.
Wrapper test 8개는 기존 detector/planner를 실제 사용하여 두 주차 타입의 plan→localization→activate,
새 요청 reset, 중복/늦은 명령, scan stale, 모듈 cancel, 수동 명령 overwrite 차단을 검증한다.
ROS test는 실제 GPS Zone 직렬화→MGM, typed command/status, Zone 밖 handoff, CAN fault 입력,
reference stale, 명시 취소, CSV 필드·시각·track index를 검증한다.

Python 전체 시험에서 기존 시스템 SciPy가 설치 NumPy 1.26.4에 대해 지원 버전 경고를 1건 출력했다.
시험 실패는 없다. 실제 차량 동역학, 공간별 탐색 성공률/지연, CAN 하드웨어는 이 검증 범위에 없다.
메시지 계약이 확장되어 운용 시 관련 패키지를 같은 overlay에서 함께 다시 빌드해야 한다.
raw dump는 **v11**이며 v10 이전 기록은 해당 버전 빌드로 재생한다.

## 이번 5차 변경 파일

| 영역 | 파일 |
|---|---|
| 설계/운용 문서 | `CLAUDE.md`, `docs/MGM_MISSION_PREPARATION.md`, `docs/MGM_BASE_STATE_MACHINE.md`, `docs/MGM_REFERENCE_SAFETY.md`, `docs/MBD_KIT.md`, `src/adas_mgm/README.md` |
| 순수 core | `src/adas_mgm/core/manager_types.hpp`, `core/mgm_types.hpp`, `core/manager_step.cpp`, 신규 `core/mission_step.hpp`, 신규 `core/mission_step.cpp` |
| MGM 연결/설정 | `src/adas_mgm/src/mgm_node.cpp`, `config/params.yaml`, `launch/REAL_VEHICLE_lane_gps_can.launch.py`, `CMakeLists.txt`, `tools/dump_format.hpp` |
| 메시지 | `src/fma_interfaces/CMakeLists.txt`, `msg/GpsPath.msg`, `msg/ParkingStatus.msg`, `msg/MgmState.msg`, 신규 `msg/ParkingCommand.msg`, 신규 `msg/MissionObservation.msg` |
| GPS 생산부 | `src/stack_gps/stack_gps/node.py`, `src/stack_gps/config/mission_zones.example.yaml` |
| Parking 연결부 | `src/stack_parking/stack_parking/node.py`, 신규 `src/stack_parking/test/test_mission_preparation.py` |
| core/ROS 시험 | 신규 `src/adas_mgm/test/mission_preparation_test.cpp`, `test/manager_test_fixture.hpp`, `test/zone_manager_test.cpp`, `test/reference_safety_test.cpp`, `test/manager_ros_smoke.py` |

같은 셀 안의 축약 경로는 첫 파일의 패키지를 기준으로 한다.
수정 전 tracked diff와 core/test/msg/생산부 사본은 `/tmp/mgm-before-preparation-revision`에 보존했다.
커밋·push·merge는 수행하지 않았다.
