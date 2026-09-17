# MBD 착수 킷 — 김재민 (Simulink/Stateflow → 생성 C 코드 트랙)

> **2026-09-13 PR 검토 상태:** [현재 범위·검증 결과](INTEGRATION_V2_PR_STATUS_20260913.md). 13개 패키지 빌드 완료, Python 471 통과/3 skip.
> 전체 CTest는 11/20 통과이며 회귀 정리가 남아 있다. 아래 과거 미빌드/전체 통과 표기보다 이 결과를 우선한다.

> **구현 상태 정정:** 아래 전체 provider 1점 계약은 통합 미완료 설계다. MGM 변경 소스는 미빌드이며
> GPS/LINE은 station preview 1점 반환을 오프라인 검증했다. 나머지 생산부와 MGM 설치본 통합은 미완료다.

> **2026-09-12:** v2 제어 경로는 입력/출력 모두 유효 목표점 1개다(`MGM_CONTROL_POINTS=1`).
> CorePath/Output 배열의 20 용량은 legacy 호환 저장 공간이며 현재 n은 1 또는 무효 입력 0이다.
> n>1도 무효다. MBD에서 1→20 보간을 구현하지 않는다. xy/yaw/curvature와 실제 generation을 보존한다.
> 현재 dump는 v18이며 과거 기록은 해당 역사적 빌드로 재생한다.

> **6차 단일 기준:** [MGM_MBD_STATE_MACHINE_SPEC.md](MGM_MBD_STATE_MACHINE_SPEC.md). 현재 MBD 정본은 병행 Top/Nav/Avoid/Signal/Safety/Mission이며 legacy 5-state byte는 호환 projection이다.
> Zone 확인은 독립 GNSS sample이며 0=미설정. Parking 제한 -1, Recovery OFF 유지. 새 bus/dump는 v12이며 이전 버전 설명/시험 절차는 역사적 비교 범위다.
> 실제 운용 전 Zone/Parking/후방 corridor calibration과 현장 검증이 필요하다.

> **2026-09-11 공통 베이스 변경:** 새 C++ core의 정본은
> [MGM_BASE_STATE_MACHINE.md](MGM_BASE_STATE_MACHINE.md)와 `core/manager_step.cpp`다.
> 4차 Reference 안전 계층은 [MGM_REFERENCE_SAFETY.md](MGM_REFERENCE_SAFETY.md)와
> `core/reference_safety.cpp`를 함께 적용한다.
> 5차 준비/수명은 [MGM_MISSION_PREPARATION.md](MGM_MISSION_PREPARATION.md)와
> `core/mission_step.cpp`를 대응시킨다.
> 아래 5상태 차트/생성 v1.88 설명은 legacy 비교용이며 새 병행 Manager와 동치가 아니다.

### 새 버스와 Stateflow 대응

- `manager_types.hpp`: `TopState`, `NavState`, `AvoidState`, `SignalState`,
  `SafetyState`, `MissionState`, `MissionType`, `SpeedOwner`, `ManagerState`와
  `ZoneType`, `ZoneObservation`, `ZoneSnapshot`, `ZoneContext`, `ZoneState`,
  `ReferenceSample`, `ReferenceStatus`, `SafeStopReason`, `CalibrationState`,
  `RearCorridorState`, `RecoveryDiagnostics`.
- Top과 Navigation/Avoidance/Signal/Safety/Mission을 병행으로 보존한다. DRIVE는 시작/주행 승인 guard다.
  ENABLE에서도 Zone 관측과 active 요청의 lifetime 갱신이 멈추지 않도록 모델링한다.
  `nav_reselect()`는 Junction/선택 함수이며 저장 State를 만들지 않는다.
- `CoreSnapshot` 추가 입력: `autonomous_enabled`, `new_session`, `external_stop`,
  `camera_line_valid`, `gps_valid`, `lidar_valid`, `auto_estop`, `parking_valid`,
  `parking_updated`, `parking_mission_active`, `parking_mission_mode`, `zones`, `references[5]`.
  waypoint reached/pass/target 변경은 Mission 입력이 아니다.
- `ReferenceSample`: uint64 generation, float age_s/timeout_s. Wrapper가 실제 sensor-input
  `reference_stamp`를 monotonic age로 변환한다. timeout은 기존 provider별 설정을 재사용한다
  (노드 기본 각 0.5s, 운용 YAML의 LINE은 1.0s). 주기마다 publish됐다는 이유로 나이를 0으로
  만들면 안 된다. LINE HELD/SEARCH, 동일 GPS fix, 미갱신 주차 pose도 새 생성이 아니다.
- `CoreParams`의 새 구조 선택은 `base_state_machine_enabled`다.
  Mission Trigger 설정 배열은 제거했다. Zone 경계/타입/ID 설정은 GPS의
  `ZoneDefinition`에서 처리하고, core에는 전체 포함 여부 snapshot만 전달한다.
- `CoreState.managers`는 고정 크기 상태/완료 기억이다. Zone ID 1..255별 edge를
  `zone_step()`에서 raw/stable과 독립 GNSS generation 확인을 거쳐 계산하며 GPS invalid에는 stable을 보존한다.
  zone_enter_confirm_samples/zone_exit_confirm_samples는 0=미설정이다. reset 내부 Mission은 확정 exit/reentry 전 억제한다.
  Zone 0은 암시적 Normal이다. `mission_completed[256]`는 별도 Mission ID 0..255로
  색인하며 명시적 새 session에서 초기화한다. 256은 uint8 표현 용량이지 주행 임계가 아니다.
  기존 lane 카운터, return_hold, escape, traffic distance 필드를 재사용한다.
- `CoreOutput`은 manager 상태, `speed_owner`, `mission_start`, `mission_cancel`,
  `reference_available`, `zones`, `active_mission_id`, `references[5]`, `selected_reference`,
  `safe_stop_reasons`, `avoid_episode_reference_seen`을 추가한다.
  `ReferenceStatus`는 source/available/valid/fresh/age_s/generation이다.
  SAFE_STOP reason은 uint32 독립 mask이며 하나의 해제로 다른 reason을 지우지 않는다.
  Reference 무효는 센서 상실과 구별하고 Mission/Avoidance 제어권을 유지한 채 정지한다.
  기존 assemble/merge 뒤의 `final_reference_gate`도 모델에 대응시켜야 한다.
  CAN의 `state` 바이트는 기존 경로 표시용 투영이다.
  병행 상태 관측은 `/adas/mgm_state` (`MgmState`), 실제 제어 출력은
  `/adas/target_ref` 한 곳이다.
- `GpsPath.zones`는 전체 Zone membership 입력이다. `MgmState.zone`은 표시 우선순위
  Mission > GPS-only > Normal의 대표값이며, `MgmState.zones`는 겹친 구간과 exit edge도
  보존한다. 대표값만 사용하여 GPS-only 문맥을 지우면 안 된다.
- MissionState wire 값은 IDLE=0 / ACTIVE=1 / PREPARE=2다. PREPARE는 일반 병행 주행을 유지한다.
  `MissionRequest`(고정 크기 POD)에 요청 ID/type/source Zone, monotonic 시작/경과 시간,
  실제 속도 적분 거리, milestone과 cancel reason을 보존한다. Zone exit로 clear하지 않는다.
  별도 RequestState chart를 중복 생성하지 않는다. `mission_prepare`는 탐색 명령,
  `mission_start`는 ready 이후 제어권 인계 pulse다. Session reset에서 `last_request_id`를 보존한다.
- 새 입력은 monotonic/event time, explicit cancel, parking_request_id/search_active/search_space_found/
  preparation_ready/parking_preparation_reference, GPS ENU/track index다.
  새 파라미터 `parking_search_timeout` / `max_parking_search_distance`는 실측 전 미설정이다.
  모델에 임의 제한값을 넣지 않는다. 두 값이 유효하지 않으면 CALIBRATION_REQUIRED 취소다.
  실제 VehicleVector.v의 절댓값×monotonic dt로 누적하며 stale 속도는 MOTION_UNAVAILABLE 취소다.
- 6차 입력/출력: zones.generation, raw/stable/count/mission_entry_suppressed, Parking/Zone calibration,
  typed rear valid/corridor, Recovery 누적/차단 진단, Traffic 실제 잔여거리/정지 성공 구간 진단.
  상세 bus는 core 헤더, SET/CLEAR 및 실행 순서는 단일 명세를 사용한다.
- raw dump 버전은 **12**. 이전 덤프는 해당 버전의 빌드로 재생한다.
  생성 v1.88 선택 시 새 manager flag는 false여야 하며, true 조합은 시작 시 거부한다.
- 검증: `test/manager_state_test.cpp`의 기존 1~30 상황(78 checks),
  `test/zone_manager_test.cpp`의 3차 1~25 상황(49 checks),
  `test/reference_safety_test.cpp`의 4차 안전 검사(60 checks),
  `test/mission_preparation_test.cpp`의 5차 1~20 상황(65 checks),
  `test/manager_ros_smoke.py`의 GPS Zone 생산부→MGM 모의 ROS 연결을 사용한다.

> 스펙의 단일 소스는 `docs/MGM_MBD_STATE_MACHINE_SPEC.md`. 이 문서는 착수에 필요한 파일·설정·합격 기준만 정리한다.
> 레퍼런스 구현: `src/adas_mgm/core/` (김윤기). 모델이 이것과 **같은 입력에 같은 출력**을 내면 합격.

## 1. 입·출력 버스 정의 = `src/adas_mgm/core/mgm_types.hpp`

이 헤더가 곧 Simulink 버스 정의다. 필드 그대로 버스를 만들 것 (형·순서·이름 일치 권장).

| 구조체 | Simulink 대응 | 비고 |
|---|---|---|
| `CoreSnapshot` | 입력 버스 | 인지 입력 + traffic 적/녹/정지선 거리 + dSPACE 실차속도. bool→boolean, float→single, int32_t→int32 |
| `CoreOutput` | 출력 버스 | state, path_source, immediate_stop, v_ref, **n_points**, ref_points[20] (유효분은 n_points개, legacy state는 projection) |
| `CoreState` | 모델 내부 상태 | Stateflow 차트 상태 + Data Store (params 포함) |
| `CoreParams` | tunable parameter | params.yaml과 1:1 |
| `CorePoint` / `CorePath` | 서브 버스 | CorePoint = RefPointWire와 동일 레이아웃 (float×4) |
| 상수 | `MGM_NUM_POINTS=20`, `MGM_PERIOD_S=0.01` | 고정 배열 크기·주기 |

## 2. Stateflow 차트 스펙 = MGM_MBD_STATE_MACHINE_SPEC.md

- Top/Nav/Avoid/Signal/Safety/Mission 병행 상태를 구성한다. flat 5-state chart로 만들지 않는다.
- Mission은 IDLE→PREPARE→ACTIVE, 요청 ID/완료 기억/준비와 실행 ack를 분리한다.
- Zone stability는 별도 context layer이며 GNSS generation으로 확인한다. MGM cycle timer와 다르다.
- NAV_RESELECT는 현재 GPS-only/LINE/GPS를 재평가하는 공통 함수다.
- Traffic은 최초 정지선 소실 edge에서 1.5m seed, 실제 abs(v)와 monotonic dt 적분, 잔여 1.0m 정지 목표다.
  green && !red만 Signal 요구를 해제하며 다른 정지 reason은 유지한다.
- FINISH 해제는 explicit new_session만 허용한다. 기존 legacy estop_latch_release 해제로 대체하지 않는다.
- reference와 speed 선택을 분리하고 기존 assemble/merge 뒤 final gate를 둔다.
- `docs/state_machine_detail.drawio`의 과거 flat chart는 현행 모델 정본이 아니다.

## 3. 코드 생성 설정 (Embedded Coder)

- step 함수 이름 **`mgm_step`** (레퍼런스와 동일 — wrapper 교체 탑재의 전제).
- **정적 메모리만** — 동적 할당(malloc) 금지 옵션.
- 자료형은 core 헤더 그대로 single/double/boolean/int32/uint8/uint32/int64/uint64를 매핑한다.
  시각/요청 ID/ENU/누적량을 임의 single로 줄이지 않는다. 고정 배열과 정적 POD만 사용한다.
- 솔버: discrete, 고정 스텝 0.01s.

## 4. 역사적 legacy 예제의 back-to-back (하네스: `src/adas_mgm/tools/`)

입력 덤프와 정답 CSV는 결정론 도구로 재생성한다 (바이너리 커밋 안 함):

```bash
colcon build --packages-select adas_mgm && source install/setup.bash
ros2 run adas_mgm make_sample_dump sample.bin      # 합성 시나리오 1100틱(11s) — 전 스테이트·전이 통과
ros2 run adas_mgm core_replay sample.bin golden.csv  # 레퍼런스 정답 출력
```

생성 코드를 같은 방식으로 재생(core_replay.cpp에서 `mgm_step` 호출부만 생성 코드로 링크)하여
`diff golden.csv gen.csv`가 0이면 합격. 실차 rosbag 덤프(`mgm_node`의 `snapshot_dump_path` 파라미터로 기록)도 같은 절차.

현재 체크인된 `ADAS_MGR2 v1.88` 생성 C는 기존 flat 4상태 비교 전용이다.
production core는 6차 병행 Manager를 실행한다. generated backend는 base manager 및
traffic_state_enabled=true를 거부하며 위 legacy 합성 예제로 새 병행 모델의 동등성을 입증할 수 없다.
6차 이식 검증은 stabilization/manager/zone/reference/preparation 시험의 입력과 실제 v12 snapshot을 사용한다.

### 정답 CSV 스테이트 전이 체크포인트 (빠른 육안 확인용)

| tick | state | 의미 |
|---|---|---|
| 319 | 0→1 | 차선 신뢰도 저하 20주기 → waypoint |
| 450 | 1→2 | 장애물+회피 가능 → avoid |
| 549 | 2→1 | 기동 완료 → 진입했던 waypoint로 복귀 |
| 579 | 1→0 | 신뢰도 복귀 20주기 → lane |
| 660 | 0→2 | avoid 진입 + TTC<0.8 → immediate_stop=1, v_ref=0 |
| 679 | 2→0 | 복귀 |
| 700 | 0→3 | 주차구간+공간 인식 → parking |
| 899 | 3→0 | 주차 완료 |
| 950 | — | estop → immediate_stop=1, v_ref 즉시 0 (스테이트는 lane 유지 — "정지는 스테이트가 아니다") |

## 5. 주의

- 스펙 변경은 CLAUDE.md §4 갱신이 선행 — 모델·레퍼런스 어느 쪽도 임의 변경 금지.
- 덤프 바이너리는 같은 머신·같은 ABI에서만 호환 (`tools/dump_format.hpp` 참조).
- float 연산 순서 차이로 마지막 자리 수 diff가 나면 허용 오차 비교(예: 1e-5)로 완화하되, 스테이트·immediate_stop·path_source는 **완전 일치**여야 한다.

## 2026-09-16 revised v2 / dump v32

`CoreParams.revised_v2_enabled`를 추가했다. 운영 prepare/drive는 true, 역사적 테스트/생성 백엔드의 기본은 false다.
`CoreSnapshot` 끝에 FIXED 품질, 출발 raw LiDAR 준비, 신호 상태 freshness, 센서별 ESTOP clearance/reference,
회복 episode ID·경로·속도·완료·reference를 추가했다. `CoreOutput`에는 ESTOP 활성과 episode ID를 추가했다.
`SafetyState::ESTOP=4`이며 기존 enum 숫자 및 CAN 5-state projection은 유지한다.
새 입력은 dump v32로만 재생한다. 구버전 dump는 당시 코드로 재생하며 필드를 추정 보충하지 않는다.
생성 백엔드로 새 정책을 요청하면 시작 시 거부한다. 현재 요구사항과 제외 내역은
[V2_STATE_MACHINE_CHANGE_REPORT.md](V2_STATE_MACHINE_CHANGE_REPORT.md)를 따른다.


## 2026-09-16 통합 로그 v33 / PR #106

현재 기록기 `mgm_node`, 샘플 생성기 `make_sample_dump`, `core_replay`,
`parity_replay`는 공통 `dump_format.hpp`의 v33을 사용한다.
v33은 인터뷰 기반 v2와 PR #106 T 주차 전체 순서·종점 유지·저속 전달을 통합한다.
두 재생기는 동일한 `dump_reader.hpp`로 버전, snapshot/params 크기와 파일 완전성을 확인한다.
누락된 파라미터를 0으로 보충하거나 잘린 레코드를 정상 종료로 처리하지 않는다.
`core_replay` CSV에는 dump_version, revised_v2, estop_active, estop_request_id도 기록한다.

이 PC에서 새 로그를 재생하는 명령(실차/CAN 기동 없음):

```bash
cd /home/sangmin/Desktop/HL-Global-Mobility-Team2-v2_main
install_v2/adas_mgm/lib/adas_mgm/core_replay drive_logs/실제_세션/mgm_snapshots.bin /tmp/replay_v33.csv
```

v32는 인터뷰 개발본과 PR #106 독립 개발본이 같은 번호를 서로 다르게 사용했다.
헤더 번호만 33으로 바꾸거나 임의 변환하지 않는다. 이 PC의 기존 인터뷰 v32 재생
실행 파일은 `build_v2/replay_archive/v32_local/`에 복사해 보존했다(해시는 README.txt).
그 파일들은 **이 PC의 변경 전 v32 로그**에만 사용하며, PR #106 독립 v32 또는 다른 ABI의
로그는 해당 기록 당시 빌드가 필요하다. 기존 로그를 새 정책으로 재해석하지 않는다.
`build_v2`를 삭제하기 전에는 이 보존 디렉터리도 별도로 백업해야 한다.

`parity_replay`는 이전 Simulink 생성 모델과의 비교 도구이며, 모델이 지원하지 않는
새 v2 동작의 동일성을 보장하지 않는다. 현재 실차 정책의 재생은 `core_replay`를 사용한다.

## 2026-09-16 ESTOP v34

CAN 상태 5와 회복 실행기 계약으로 raw dump v34를 사용한다. 현재 core_replay와
parity_replay는 v33을 명확히 거부한다. 이전 로그는 해당 버전 실행기로 재생한다.
이 PC의 v33 실행기는 `build_v2/replay_archive/v33_local`에 보관했다.
[적용 계약](V2_PR108_INTEGRATION.md) 참고.

## 스테이트 v09.16 상태 제외 — v35

현재 상태 선언에서 CLEAR_CONFIRM, AUTO_ESTOP, REVERSE_RECOVERY, MISSION_PREPARE를 제외했다.
과거 상태 번호는 재사용하지 않는다. 현재 재생 도구는 v35를 사용하며, v34는
`build_v2/replay_archive/v34_local` 또는 `f21ef6d` 빌드의 도구로 재생한다.

## 스테이트 v09.17 마지막 미션 통합 — v36

현재 정본은 [STATE_V09_17.md](STATE_V09_17.md)다. `CoreSnapshot`에
exit_request_id/exit_reference/exit_class_id/exit_confidence가 추가됐고,
RouteFeedback/RouteControl에는 terminal/last_mission_enabled/left_index/right_index가
추가됐다. ManagerState와 CoreOutput의 LastMissionControl 버스는 phase, request_id,
last_frame, started_ns, started_event_ns, left_votes, right_votes, selected_index,
route_id, source_zone_id, fallback을 가진다. 형식은 core/manager_types.hpp를 따른다.

새 ZoneType::LAST_MISSION_ZONE=3과 LastMissionPhase 0~5는 ROS 메시지 상수와 일치한다.
CAN 상태 번호는 유지한다. 생성 v1.88 백엔드에는 이 상태를 연결하지 않는다.
raw dump는 v36이며, v35 로그는 당시 도구로 재생해야 한다. 이 PC의 도구는
build_v2/replay_archive/v35_local에 보관했다.

## 스테이트 v09.17 회피 중 차선 판정 중단 — v37

현재 raw dump는 v37이다. 버스 크기는 같지만 AVOID_ACTIVE/GPS_RETURN에서
차선 신뢰도 카운터를 초기화하고 판정을 중단하도록 전이 의미가 바뀌었다.
v36 덤프는 `build_v2/replay_archive/v36_local`의 당시 도구로 재생한다.
현재 도구는 v36 덤프를 거부해 다른 전이 의미로 재생되는 것을 방지한다.
