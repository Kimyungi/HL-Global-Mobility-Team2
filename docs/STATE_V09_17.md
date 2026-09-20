> 2026-09-20 최종 ESTOP: CSV zone [6] (경로 04 idx 622~828)만 감지 활성화. state=6 범위는 폐기. 구역 이탈만으로 정지 해제하지 않음.

> 2026-09-20 회피: 현재 CSV zone [5]에서만 AVOID_ACTIVE, 유효 위치가 구역 밖이면 즉시 종료. state=4와 경로 정렬 완료 조건은 v2에서 사용하지 않음.

> 2026-09-20: CSV state=6부터 같은 path_id 끝까지 ESTOP 감지 구간으로 연결. 마커가 없는 경로는 비활성화. [현재 계약](ESTOP_STATION_20260919.md) 참조.

> 2026-09-19 ESTOP 변경: [스테이션 계약](ESTOP_STATION_20260919.md)이 아래 과거 후진 회복 설명보다 우선합니다. 현재 런처는 스테이션 지정 전까지 ESTOP 감지 비활성화이며, 새 내부 동작은 장애물 소실까지 정지 유지입니다.

# 스테이트 v09.17

2026-09-17 사용자 지시에 따라 마지막 미션을 전체 상태 머신에 통합하고
현재 명칭을 **스테이트 v09.17**로 변경했다. 기존 v09.16에 MGM 마지막 미션 전이,
YOLO 관측 노드, GPS 분기 인계와 종료 처리를 추가한 버전이다.
대상 실행 경로는 RUN_BOOK_HALLA_FINAL.md의 `scripts/v2 prepare/drive`, `revised_v2_enabled=true`다.
소스에 남은 과거 프로파일과 실험 모드는 이 명칭의 실행 범위에 포함하지 않는다.
이 이름은 상태 머신 명세 버전이다. raw dump 형식 버전 v38과는 별개다.

## 빈 상태 점검 결과

정적 코드 점검에서 **현재 운용하는 주요 상태에 실행기 없이 자리만 있는 상태는 발견하지 않았다.**
후속 요청으로 미사용 선언 1개, 구형 안전 상태 2개 및 별도 준비 상태를 현재 상태 정의에서 제외했다.
아래의 “구현됨”은 실차 동작 검증 완료를 뜻하지 않는다. 정상 주행 정책은 유지하며, 별도 미션 준비 상태로 되돌리는 설정은 허용하지 않는다.

| 계층 | 상태 | 확인 결과 |
|---|---|---|
| CAN 출력 | 0 LANE / 1 WAYPOINT / 2 AVOID / 3 PARKING / 4 TRAFFIC / 5 ESTOP | 0~5 연속 정의. 각 출력·참조 경로 선택 구현됨 |
| 최상위 | AUTONOMOUS_ENABLE / AUTONOMOUS_DRIVE / FINISH | 준비·인가, 주행, 완료 정지 처리 있음 |
| 주행 | GPS_BACKUP / GPS_ONLY_NAV | FIXED GPS 사용. LINE은 v2에서 사용하지 않음. 유효 GPS 참조가 없으면 일반 주행 정지 |
| 회피 | INACTIVE / AVOID_ACTIVE | 현재 zone [5] 진입·이탈로 전이. 구역 안에서 기동이 끝나도 유지 |
| 회피 | GPS_RETURN | 과거 프로파일용 선언 보존. 현재 v2 zone [5] 모드에서는 진입하지 않음 |
| 회피 | CLEAR_CONFIRM | **현재 enum에서 삭제. 진입·처리 없음** |
| 신호 | SIGNAL_IDLE / RED_DETECTED / APPROACH_STOP_LINE / STOPPED_WAIT | zone [3] 모듈 활성화, 적색·정지선·거리 감속·정차·적색 해제 전이 있음 |
| 미션 | MISSION_IDLE / MISSION_ACTIVE | 요청 ID, 준비 응답, 실행 인계, 완료/취소 처리 있음 |
| 미션 | MISSION_PREPARE | **현재 enum·ROS 상수에서 제외. ACTIVE로 직접 진입**. v09.17 코어는 활성 진입을 고정하고 ROS 노드는 false 설정을 거부. 준비 절차는 ACTIVE 내부에서 수행 |
| 안전 | NORMAL / SAFE_STOP / ESTOP | 정상, 정지 게이트, 상위 ESTOP 회복 계약 구현됨 |
| 안전 | AUTO_ESTOP / REVERSE_RECOVERY | **현재 enum에서 제외**. 과거 재생용 식별자는 legacy_state_ids.hpp에 격리. v09.17에서는 진입하지 않음 |
| 경로 | DISABLED / RUNNING / WAIT_MISSION / WAIT_STOP / WAIT_ACK / FINISHED / FAULT | 초기화, 미션/종점/실제 정차/새 경로 응답, 완료·고장 처리 있음 |
| T 주차 실행기 | STOP_SELECT / ADVANCE_3 / STOP_REVERSE / REVERSE / WAIT_3 / EXIT / DONE / FAULT | 선택→경로 종점→후진→주차 완료 정차→3초→전진 탈출→다음 경로로 주행. |
| ESTOP 실행기 | IDLE / HOLD / REVERSE / SETTLE / DONE / FAULT | 대기·6초 정차·1m 실측 후진·정차 확인·완료·고장 처리 있음 |

용인 T·평행 주차는 같은 참조 경로 실행기를 사용한다. state=1은 주차 경로 01·02와 별도 탈출 경로, state=2는 03·04를 사용하고 진입 경로로 되돌아 나온다.

## 라인 주행 전면 중지 (2026-09-19)

사용자 지시로 revised_v2 전체 구간에서 라인 추론·경로 발행·신뢰도 판정과
LINE 전이 및 GPS 소실 시 카메라 대체 주행을 중지했다. 일반 주행과 CSV 연결은
GPS를 사용하며, 유효 GPS 참조가 없으면 라인으로 대체하지 않고 정지한다.
출발 인가는 기존 필수 LiDAR 수신과 GPS FIXED 준비를 요구한다.
회피·주차·신호·상위 ESTOP 및 마지막 미션의 기존 실행 권한은 유지한다.

차선 카메라 노드는 `camera_only=true`로 실행한다. 차선 모델·호모그래피를
로드하지 않으며 일반 구간에서도 라인 추론을 재개하지 않는다.
`/perception/lane_image_raw`와 `/perception/lane_camera`는 계속 제공하므로
마지막 미션 출구 검출기의 원본 영상 입력은 유지된다.

## Last_mission_state 통합 (2026-09-17)

이 상태는 MGM 10ms 코어의 병렬 상태 `ManagerState.last_mission`으로 실행된다.
GPS zone [2] 입력 → YOLO 검출·주행 유지 → CSV state=3 정지 명령 → 실제 정차 → 3초 판별 → GPS 경로 요청·응답 →
주행 복귀까지 런처와 ROS 메시지에 연결됐다. CAN 상태 바이트 0~5는 기존 횡방향
소스/ESTOP 투영을 유지하며, 새 상태 단계는 `MgmState.last_mission_phase`로 공개한다.

| 단계 | 전이·동작 |
|---|---|
| IDLE | 사전 로드한 분기 원본 경로의 확정 LAST_MISSION_ZONE 도달. 인가·유효 위치, 주차/회피/신호 비활성, 선행 필수 미션 종료 시 APPROACH |
| APPROACH | 출구 YOLO 검출·일반 주행 유지. 현재 station이 CSV state=3 도달 시 STOPPING |
| STOPPING | MGM 즉시 정지 출력. 유효 실제 속도 절댓값 ≤0.001m/s 확인 후 JUDGING |
| JUDGING | 단조 시계로 정차 3초를 모두 관측. 신뢰도 0.5 이상, 새 프레임별 1표 다수결. 중복·낡은·다른 요청 관측 제외 |
| SELECTED | Left→06, Right→07, 무검출·동률→06. 인가·실제 정차·유효 GPS·기존 정지 조건을 확인한 후 선택 경로 요청 |
| WAIT_ROUTE | 기존 sequence/instance/request/index와 새 유효 GPS 관측으로 ACK 확인할 때까지 정지 유지 |
| DONE | ACK 틱은 정지. 다음 틱부터 정상 GPS 주행·기존 내비게이션 전이 재개. 선택된 06 또는 07 종점에서 FINISH |

운전자 정지·속도 소실·이동·상위 ESTOP은 판별 창을 초기화한다. ESTOP은 기존
회복 계약대로 우선 실행되며 복귀 후 실제 정차부터 다시 3초를 센다. 이미 선택한
경로는 인계 대기 중 유지하고, 명시적인 새 세션만 완료·판별 기억을 초기화한다.
카메라/모델 실패 또는 무검출도 MGM 타이머를 막지 않으며 3초 후 06을 선택한다.

현재 용인 CSV의 경로 05, zone [2](idx 84~103, state=3은 idx 92)를
LAST_MISSION_ZONE으로 연결한다. 경로 05를 포함한 실행은 06·07을 함께 사전 로드하며
판별 결과로 하나의 출구에 인계한다. 06 다음에 07을 연속 주행하지 않는다.
06 또는 07에서 시작하는 경우에는 선택한 단일 출구만 실행한다.

[전이·zone 설정·검증 계약](LAST_MISSION_STATE.md),
[MGM 상태 구현](../src/adas_mgm/core/last_mission_step.hpp),
[YOLO 관측 노드](../src/stack_exit_decision/stack_exit_decision/node.py).

## 빈 구현으로 오해하면 안 되는 항목

- ESTOP FAULT: 자동 재시도 없이 정지하는 명시적 동작이다. 정상 회복 완료를 보내지 않는다.
- ESTOP HOLD: 실행기의 정상 진입 관측, 실제 정차, 인가 및 후방 관측이 충족되지 않으면 계속 대기한다.
- WAIT_ACK: 새 경로의 일치하는 요청 응답과 유효 GPS 관측을 기다린다. 대기시간 만료만으로 통과시키는 로직은 없다.
- WAIT_MISSION: 필요한 미션의 완료/실패 기록을 기다린다. 완료 이벤트가 없으면 계속 대기할 수 있다.
- FINISH 및 FAULT: 정상 주행으로 자동 전이하지 않는 종료/고장 상태도 실행 동작이 있는 상태다.
- MgmState의 recovery_* 진단 필드는 구형 실행기용이다. v09.17 ESTOP 단계·실측 거리는
  `/planning/estop_recovery_status`를 사용한다. 이 진단 연결의 차이는 빈 회복 실행기를 의미하지 않는다.
- 지도 CSV의 state=5 지정 정지점과 CAN TargetRef의 state=5 ESTOP은 서로 다른 번호 체계다.

## 과거 문서와 구분

초기 상태 머신 변경 보고서의 “ESTOP 실행기 보류”, “turn_zones 비어 있음”,
“회피·주차 내부 교체 보류”는 초기 작업 당시 기록이다.
현재는 PR #103 회피, PR #105 한라대 경로/zone [3], PR #106 T 주차,
PR #108 기능을 조정 통합한 PR #109 ESTOP 실행기를 기준으로 한다.

## 코드 근거

- [상태 선언](../src/adas_mgm/core/manager_types.hpp), [CAN 상태 번호](../src/adas_mgm/core/mgm_types.hpp)
- [주행·회피·신호·안전 전이/출력](../src/adas_mgm/core/manager_step.cpp)
- [미션 전이](../src/adas_mgm/core/mission_step.cpp), [경로 전이](../src/adas_mgm/core/route_step.cpp)
- [상위 ESTOP](../src/adas_mgm/core/estop_state.hpp), [회복 실행기](../src/adas_mgm/tools/estop_recovery_core.py)
- [T 주차 순서](../src/stack_parking/stack_parking/t_parking_sequence.py), [주차 연결](../src/stack_parking/stack_parking/node.py)
- [회피 실행 연결](../src/stack_avoid/stack_avoid/waypoint_node.py)
- [현재 런처](../src/adas_mgm/launch/REAL_VEHICLE_integration_v2_drive.launch.py)
- [ESTOP 적용·검증 범위](V2_PR108_INTEGRATION.md)

## 기존 상태 제외와 검증 이력

현재 상태 번호는 재사용·재번호 부여하지 않는다. 회피 2, 안전 1·2, 미션 2는
v09.17의 유효 상태가 아니다. 과거 로그·과거 프로파일의 재현에만 legacy 식별자를 사용한다.
`mission_prepare` 출력 및 ParkingCommand.PREPARE는 주차 모듈 초기화 명령이므로 유지한다.
이는 제거한 MISSION_PREPARE **상태**와 다르며, 삭제하면 ACTIVE 내부 준비 연결이 끊긴다.

현재 로그는 v38이다. 아래는 v09.16의 기존 검증 이력이며, 새 통합 검증은 LAST_MISSION_STATE.md를 따른다.
v35 도구는 `build_v2/replay_archive/v35_local`에 보관했다. 당시 로그는 v35로 구분했다. 이 PC의 이전 v34 실행기는
`build_v2/replay_archive/v34_local`에 보관하며 v34는 해당 도구로 재생한다.

검증: 전체 14개 패키지 빌드, C++ 29개(직접 ACTIVE 진입 회귀시험 포함),
런처 31개, 회복 단위시험 11개 통과. 격리 ROS 전이 시험 및 v35 반복 재생,
v34 보관 도구 재생/새 도구의 구버전 거부를 확인했다. 실차 주행 시험은 수행하지 않았다.

2026-09-17 후속 지시: 상위 ESTOP 감지는 출발 인가 후 유효한 실제 전진 속도가
0.02m/s를 초과한 첫 시점부터 2초 뒤 활성화한다. 그 이전 및 활성화 경계의 관측은
3회 연속 감지에 포함하지 않는다. 활성화 후 정차해도 감지는 유지한다.
인가 해제·운전자 정지·새 세션은 초기화하며, 이미 진행 중인 ESTOP 회복 계약은 유지한다.

### 한라대 정지선 인식 시험 임시 조건 (2026-09-18)

한라대 정지선 단독 시험용 `halla_stopline_test_enabled` 옵션을 보관한다. 현재 용인 `scripts/v2 drive`와 기본 런처에서는 `false`이며 적색+정지선 조건을 사용한다. 아래 전이는 시험용 옵션을 명시적으로 켠 경우에만 적용된다.

- zone [3]의 최신 정지선 인식만으로 `SIGNAL_IDLE → APPROACH_STOP_LINE`에 진입한다. 적색 조건은 임시 생략하며, 검출기도 적색 확정 전부터 정지선을 처리한다.
- 최초 정지선 인식 시점부터 3초 후 `APPROACH_STOP_LINE` 또는 `STOPPED_WAIT → SIGNAL_IDLE`로 탈출한다. 정차 완료 후 3초 대기가 아니며 신호 색상과 무관하다.
- 정지선 소실 후 거리 계산·감속·정차 동작은 유지한다. 탈출 후 같은 정지선의 연속 검출은 재진입하지 않으며, 최신 미검출 후 다음 검출부터 재진입한다. zone 이탈 시 시험 타이머도 초기화한다.
- 임시 조건 복구: 한라대 launch의 `halla_stopline_test_enabled` 전달값을 `False`로 바꾸면 기존 적색+정지선 진입 / 적색 해제 탈출 및 적색 후 검출로 돌아간다.
