# Integration v2 — 주차 상태에서 GPS 탐색 / 종점 자동 전환

2026-09-13 사용자 변경. 이 문서는 과거 Zone 진입 즉시 정지 및 PREPARE/Zone 이탈 실패 설명보다 우선한다.

- `parking_zone_entry_active=true`: 기존 stable Mission Zone entry(5개 독립 fix)에서
  `MISSION_IDLE → MISSION_ACTIVE`, legacy byte `PARKING=3`으로 전환한다.
- 준비 절차는 주차 상태 안에서 수행한다. 기존 PREPARE 명령과 SLAM→MAPPING→LOCALIZATION을
  사용한다. **탐색 중에는 현재 CSV의 GPS 목표점 1개를 따라 v_base로 주행**한다.
  LINE 신뢰도가 높아도 GPS를 선택하며 일반 회피/Recovery로 제어권을 넘기지 않는다.
  주차 status/경로 부재만으로 탐색 주행을 정지하지 않는다. GPS reference 유효성,
  신호 정지·외부/CAN 정지는 탐색 중에도 적용한다. 일반 LiDAR E-stop과 회피는
  Mission ACTIVE 진입부터 준비·실행 전체에서 마스킹하고 종료 틱부터 다시 적용한다.
  현재 요청/타입의 fresh preparation generation을 확인한 틱에 Parking 제어권으로 인계하고
  ACTIVATE를 전송한다. 이때부터 실행 ack와 유효 Parking reference까지 0 속도로 대기한다.
  `mission_start`와 handoff observation은 Zone entry가 아니라 이 ready 틱에 기록한다.
- 완료: 현재 ID/타입의 실행 ack 이후 done만 성공으로 기억하고 공통 nav_reselect로 복귀한다.
- 현재 CSV 종점: 유효 GPS와 현재 route의 새 generation/종점 확인을 사용한다. 미완료 주차는
  `ROUTE_END=10`으로 취소하고 실패 기억을 남긴다. 같은 틱의 유효 done은 성공이 우선한다.
  다음 CSV는 기존 실제 정지·필수 Mission 종료·새 경로 응답을 확인한 뒤 인계한다.
  **03번에서 경로를 못 찾으면 03 종점 → ROUTE_END 실패/CANCEL → WAIT_STOP →
  WAIT_ACK(04 요청) → 04의 새 GPS 응답 → RUNNING**을 상태 머신이 자동 수행한다.
  수동 CSV 선택이나 추가 go 명령은 필요 없다. Zone 이탈 자체로 04로 건너뛰지 않는다.
- Zone 이탈, GPS 공백, 숫자 탐색 시간/거리 제한은 이 모드의 주차 종료 조건이 아니다.
  ready 이전 모듈 중단/잘못된 응답은 GPS 탐색을 계속하고 종점에서 실패 처리한다.
  ready 이후 모듈 중단/잘못된 실행 응답/Parking reference 상실은 Parking 상태를 유지하고 정지한다.
  이미 시작한 주차 기동에서 GPS 탐색으로 되돌아가지 않는다.
  명시 취소, 새 session, 최종 FINISH 및 외부/CAN 정지는 기존 상위 중단으로 유지한다.
- 단일 CSV의 종점은 주차 요청을 종료한 뒤 Top FINISH다. 종료되지 않은 다른 필수 Mission을
  임의 성공 처리하지 않는다. 성공·종점 실패한 Mission ID는 같은 session에서 재실행하지 않는다.

이 모드는 기존 `parking_search_zone_only`보다 우선하며 기본 YAML은 즉시 진입 true,
과거 Zone 탐색 false다. 두 과거 준비 모드는 즉시 진입 false에서만 사용한다.
MgmState에 현재 정책과 CANCEL_ROUTE_END를 기록하고 raw dump는 v20이다.
9월 12일 v18, 9월 13일 변경 전 v19 기록은 원본을 보존하며 해당 버전 빌드로 재생한다.
TargetRef의 state byte는 탐색 중에도 PARKING이다. 실제 reference_source가 GPS이면 GNSS
dx/dy/dyaw/update를, Parking이면 SLAM 값을 전달한다. state byte만으로 위치 정보 소스를 선택하지 않는다.

GPS 종점은 기존 station/at_end 판정을 사용한다. 별도 9월 13일 GPS 변경으로 탐색 반경은
`abs(v_ref) * sample_time * 1.5 + 0.5m`이며 목표속도 0에서도 반경 0.5m를 사용한다.
CSV 종점 판정은 기존 `idx >= 점 개수 - 2` 기준을 유지한다. 공간 검출/주차 경로 생성 알고리즘은 변경하지 않았다.

신호 정지 seed는 1.5m 그대로다. 차량/CAN 실행 없이 core 및 ROS mock으로 검증한다.

## 이번 수정 검증

2026-09-13 추가 요청: 일반 주행의 `lidar_estop_enabled=true`, `avoidance_enabled=true`.
운영은 `scripts/v2 vehicle`을 사용한다. `vehicle-no-estop`은 이름 그대로 E-stop 제외 시험용이다.
주차 중 GPS 탐색/종점 조건은 유지한다. 물리 비상정지·운전자/CAN 정지·주차 경로 유효성 정지는 유지한다.
추가 검증: `parking_entry_test` 134 checks 및 격리 ROS `parking_entry_ros_smoke.py` 통과.
일반 GPS-only 회피/E-stop, 두 주차 타입의 진입·준비·실행 마스킹, 종료 후 재적용을 검사했다.

GPS 탐색 구현 시점의 검증 이력(위 134 checks로 확장):

- `parking_entry_test`: 104 checks 통과. 두 주차 타입의 GPS 탐색, 높은 LINE 신뢰도,
  Parking status 부재, GPS 상실, 일반 정지, ready 인계, 종점 실패/새 CSV ack/자동 재출발을 검사한다.
- 관련 CTest 3/3 통과(주차 진입·주차 복귀·신호 상태).
- `parking_entry_ros_smoke.py`: 설치된 MGM에서 GPS/SLAM pose delta 선택과 1점 출력,
  준비/실행 ack, 완료·종점 실패·다음 CSV 응답 후 자동 주행을 검사한다.
- `parking_search_route_ros_smoke.py`: 업로드된 한라대 01/03/04 CSV/Zone과 실제 GPS wrapper,
  MGM을 연결한다. 센서만 mock이며, 전진 명령이 있어야 모의 위치가 다음 표본으로 이동한다.
  03 주차 탐색에 ready를 주지 않고 종점 실패 → GPS wrapper의 04 적용 → 재출발을 검사한다.

센서·차량·CAN bridge는 실행하지 않는다. 아래는 변경 전 v19 검증 이력이다.

## 변경 전 v19 구현·검증 이력

- `mission_step.cpp`: 즉시 제어권, 주차 상태 내부 준비/실행 ack, done/CSV 종점 종료.
- `mgm_node.cpp`: ACTIVE이더라도 준비 전에는 PREPARE 명령, ready 후 ACTIVATE 명령을 전송한다.
- `params.yaml`과 일반/no-estop v2 런처에 기본 정책을 연결했다. 과거 준비 모드는 명시적으로 끌 때만 사용한다.
- `./scripts/v2 build`: 13개 패키지 빌드/설치 완료. `./scripts/v2 check`에서 새 정책/메시지 확인.
- 코어 `parking_entry_test` 75 checks 통과. 신호/과거 주차 복귀 시험 포함 선택 CTest 3/3 통과.
- 주차 wrapper·런처·CSV 분석 Python 시험 38개 통과. CSV 종점 실패를 `route_end_failure`로 집계한다.
- `parking_entry_ros_smoke.py`: 격리 localhost domain 178에서 설치된 MGM의 즉시 진입,
  Zone 밖 준비 유지, PREPARE→ACTIVATE, 실행 ack 이후 1점 후진 출력, done 복귀,
  CSV 종점 CANCEL/다음 CSV 요청까지 12개 모의 연결 확인 통과.

실제 센서/차량/CAN은 실행하지 않았다. 전역 테스트 전체를 재실행한 결과는 아니며,
준비 중 공간 검출 성공률과 실제 종점 도달은 현장 확인 대상이다.
