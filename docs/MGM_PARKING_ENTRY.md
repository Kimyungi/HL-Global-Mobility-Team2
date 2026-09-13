# Integration v2 — Zone 진입 즉시 주차 제어권

2026-09-13 사용자 변경. 이 문서는 과거 PREPARE 주행/Zone 이탈 실패 설명보다 우선한다.

- `parking_zone_entry_active=true`: 기존 stable Mission Zone entry(5개 독립 fix)에서
  `MISSION_IDLE → MISSION_ACTIVE`, legacy byte `PARKING=3`, reference 소유자 Parking.
- 준비 절차는 주차 상태 안에서 수행한다. 기존 PREPARE 명령과 SLAM→MAPPING→LOCALIZATION을
  사용하고, 준비 중에는 속도 0이다. 현재 요청/타입의 fresh preparation generation을 확인한 뒤
  ACTIVATE를 전송한다. 실행 ack와 유효 reference를 받기 전에는 이동하지 않는다.
- 완료: 현재 ID/타입의 실행 ack 이후 done만 성공으로 기억하고 공통 nav_reselect로 복귀한다.
- 현재 CSV 종점: 유효 GPS와 현재 route의 새 generation/종점 확인을 사용한다. 미완료 주차는
  `ROUTE_END=10`으로 취소하고 실패 기억을 남긴다. 같은 틱의 유효 done은 성공이 우선한다.
  다음 CSV는 기존 실제 정지·필수 Mission 종료·새 경로 응답을 확인한 뒤 인계한다.
- Zone 이탈, GPS 공백, 숫자 탐색 시간/거리 제한은 이 모드의 주차 종료 조건이 아니다.
  모듈 중단/잘못된 응답/유효 reference 부재는 Parking 상태를 유지하고 정지한다.
  명시 취소, 새 session, 최종 FINISH 및 외부/CAN 정지는 기존 상위 중단으로 유지한다.
- 단일 CSV의 종점은 주차 요청을 종료한 뒤 Top FINISH다. 종료되지 않은 다른 필수 Mission을
  임의 성공 처리하지 않는다. 성공·종점 실패한 Mission ID는 같은 session에서 재실행하지 않는다.

이 모드는 기존 `parking_search_zone_only`보다 우선하며 기본 YAML은 즉시 진입 true,
과거 Zone 탐색 false다. 두 과거 준비 모드는 즉시 진입 false에서만 사용한다.
MgmState에 현재 정책과 CANCEL_ROUTE_END를 기록하고 raw dump는 v19다.
9월 12일 기록은 원본 그대로 보존하며 v18 빌드로 재생한다.

GPS 종점은 기존 station/at_end 판정을 사용한다. 정지 중 목표속도 0으로 station 탐색 범위가
0이 되는 현재 GPS 정책은 이번 변경에 포함하지 않는다. 주차 모듈이 경로를 만들지 못하면
주차 상태에서 정지 대기하며, 상태 변경만으로 탐색 성공을 보장하지 않는다.

신호 정지 seed는 1.5m 그대로다. 차량/CAN 실행 없이 core 및 ROS mock으로 검증한다.

## 구현·검증 결과

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
