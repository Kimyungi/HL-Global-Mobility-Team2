# 스테이트 v09.17

2026-09-17 사용자 지시에 따라 마지막 미션을 전체 상태 머신에 통합하고
현재 명칭을 **스테이트 v09.17**로 변경했다. 기존 v09.16에 MGM 마지막 미션 전이,
YOLO 관측 노드, GPS 분기 인계와 종료 처리를 추가한 버전이다.
대상 실행 경로는 RUN_BOOK_FINAL.md의 `scripts/v2 prepare/drive`, `revised_v2_enabled=true`다.
소스에 남은 과거 프로파일과 실험 모드는 이 명칭의 실행 범위에 포함하지 않는다.
이 이름은 상태 머신 명세 버전이다. raw dump 형식 버전 v36와는 별개다.

## 빈 상태 점검 결과

정적 코드 점검에서 **현재 운용하는 주요 상태에 실행기 없이 자리만 있는 상태는 발견하지 않았다.**
후속 요청으로 미사용 선언 1개, 구형 안전 상태 2개 및 별도 준비 상태를 현재 상태 정의에서 제외했다.
아래의 “구현됨”은 실차 동작 검증 완료를 뜻하지 않는다. 정상 주행 정책은 유지하며, 별도 미션 준비 상태로 되돌리는 설정은 허용하지 않는다.

| 계층 | 상태 | 확인 결과 |
|---|---|---|
| CAN 출력 | 0 LANE / 1 WAYPOINT / 2 AVOID / 3 PARKING / 4 TRAFFIC / 5 ESTOP | 0~5 연속 정의. 각 출력·참조 경로 선택 구현됨 |
| 최상위 | AUTONOMOUS_ENABLE / AUTONOMOUS_DRIVE / FINISH | 준비·인가, 주행, 완료 정지 처리 있음 |
| 주행 | LINE / GPS_BACKUP / GPS_ONLY_NAV | 카메라·FIXED GPS 선택 및 복귀, 불충족 정지 처리 있음 |
| 회피 | INACTIVE / AVOID_ACTIVE / GPS_RETURN | waypoint_avoid_node 연결, 회피 완료 후 GPS 정렬 복귀 조건 있음 |
| 회피 | CLEAR_CONFIRM | **현재 enum에서 삭제. 진입·처리 없음** |
| 신호 | SIGNAL_IDLE / RED_DETECTED / APPROACH_STOP_LINE / STOPPED_WAIT | zone [3] 모듈 활성화, 적색·정지선·거리 감속·정차·적색 해제 전이 있음 |
| 미션 | MISSION_IDLE / MISSION_ACTIVE | 요청 ID, 준비 응답, 실행 인계, 완료/취소 처리 있음 |
| 미션 | MISSION_PREPARE | **현재 enum·ROS 상수에서 제외. ACTIVE로 직접 진입**. v09.17 코어는 활성 진입을 고정하고 ROS 노드는 false 설정을 거부. 준비 절차는 ACTIVE 내부에서 수행 |
| 안전 | NORMAL / SAFE_STOP / ESTOP | 정상, 정지 게이트, 상위 ESTOP 회복 계약 구현됨 |
| 안전 | AUTO_ESTOP / REVERSE_RECOVERY | **현재 enum에서 제외**. 과거 재생용 식별자는 legacy_state_ids.hpp에 격리. v09.17에서는 진입하지 않음 |
| 경로 | DISABLED / RUNNING / WAIT_MISSION / WAIT_STOP / WAIT_ACK / FINISHED / FAULT | 초기화, 미션/종점/실제 정차/새 경로 응답, 완료·고장 처리 있음 |
| T 주차 실행기 | STOP_SELECT / ADVANCE_3 / STOP_REVERSE / REVERSE / WAIT_10 / EXIT / EXIT_STOP / DONE / FAULT | 선택→03 종점→후진→후방 벽 정차→10초→전진 복귀→정차 완료. 모든 단계 처리 있음 |
| ESTOP 실행기 | IDLE / HOLD / REVERSE / SETTLE / DONE / FAULT | 대기·10초 정차·1m 실측 후진·정차 확인·완료·고장 처리 있음 |

T 주차는 PR #106 기반 어댑터, 평행 주차는 기존 ParkingMission 연결을 사용한다.
평행 주차를 새 T 주차 알고리즘으로 교체한 것은 아니다.

## Last_mission_state 통합 (2026-09-17)

이 상태는 MGM 10ms 코어의 병렬 상태 `ManagerState.last_mission`으로 실행된다.
GPS zone 입력 → 정지 명령 → 실제 정차 → YOLO 10초 판별 → GPS 경로 요청·응답 →
주행 복귀까지 런처와 ROS 메시지에 연결됐다. CAN 상태 바이트 0~5는 기존 횡방향
소스/ESTOP 투영을 유지하며, 새 상태 단계는 `MgmState.last_mission_phase`로 공개한다.

| 단계 | 전이·동작 |
|---|---|
| IDLE | 사전 로드한 분기 원본 경로의 확정 LAST_MISSION_ZONE 도달. 인가·유효 위치, 주차/회피/신호 비활성, 선행 필수 미션 종료 시 STOPPING |
| STOPPING | MGM 즉시 정지 출력. 유효 실제 속도 절댓값 ≤0.001m/s 확인 후 JUDGING |
| JUDGING | 단조 시계로 정차 10초를 모두 관측. 신뢰도 0.5 이상, 새 프레임별 1표 다수결. 중복·낡은·다른 요청 관측 제외 |
| SELECTED | Left→06, Right→07, 무검출·동률→06. 인가·실제 정차·유효 GPS·기존 정지 조건을 확인한 후 선택 경로 요청 |
| WAIT_ROUTE | 기존 sequence/instance/request/index와 새 유효 GPS 관측으로 ACK 확인할 때까지 정지 유지 |
| DONE | ACK 틱은 정지. 다음 틱부터 정상 GPS 주행·기존 내비게이션 전이 재개. 선택된 06 또는 07 종점에서 FINISH |

운전자 정지·속도 소실·이동·상위 ESTOP은 판별 창을 초기화한다. ESTOP은 기존
회복 계약대로 우선 실행되며 복귀 후 실제 정차부터 다시 10초를 센다. 이미 선택한
경로는 인계 대기 중 유지하고, 명시적인 새 세션만 완료·판별 기억을 초기화한다.
카메라/모델 실패 또는 무검출도 MGM 타이머를 막지 않으며 10초 후 06을 선택한다.

**현재 한라대에는 LAST_MISSION_ZONE을 생성하지 않았다.** 따라서 현재 지도에서는
이 상태가 진입하지 않고 검출기도 실행되지 않는다. 기존 `end_waypoint` 선택
(기본 07)이 적용된다. zone을 지정한 코스에서는 06·07을 함께 사전 로드하고
판별 결과로 즉시 인계한다. 06 다음에 07을 연속 주행하지 않는다.

[전이·zone 설정·검증 계약](LAST_MISSION_STATE.md),
[MGM 상태 구현](../src/adas_mgm/core/last_mission_step.hpp),
[YOLO 관측 노드](../src/stack_exit_decision/stack_exit_decision/node.py).

## 빈 구현으로 오해하면 안 되는 항목

- ESTOP FAULT: 자동 재시도 없이 정지하는 명시적 동작이다. 정상 회복 완료를 보내지 않는다.
- ESTOP HOLD: 전진 관측에 의한 활성화, 실제 정차, 인가 및 후방 관측이 충족되지 않으면 계속 대기한다.
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

현재 로그는 v36이다. 아래는 v09.16의 기존 검증 이력이며, 새 통합 검증은 LAST_MISSION_STATE.md를 따른다.
v35 도구는 `build_v2/replay_archive/v35_local`에 보관했다. 당시 로그는 v35로 구분했다. 이 PC의 이전 v34 실행기는
`build_v2/replay_archive/v34_local`에 보관하며 v34는 해당 도구로 재생한다.

검증: 전체 14개 패키지 빌드, C++ 29개(직접 ACTIVE 진입 회귀시험 포함),
런처 31개, 회복 단위시험 11개 통과. 격리 ROS 전이 시험 및 v35 반복 재생,
v34 보관 도구 재생/새 도구의 구버전 거부를 확인했다. 실차 주행 시험은 수행하지 않았다.
