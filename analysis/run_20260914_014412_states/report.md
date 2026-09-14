# 마지막 주행 전체 스테이트 보고

대상: `v2_20260914_014412_894120` · 코드 `886a061` · 2026-09-14 KST.
분석 구간: **01:44:13.086–01:48:13.546**, 240.460초, **24,047틱**.
동일 빌드의 core_replay로 원본 스냅샷을 재생했다. ABI/version을 검증했으며 원본 transitions.csv 38개 레코드의 주요 상태·경로 소스·속도·정지 사유가 모두 일치한다.
통합 노드 종료 및 CAN zero 30회 송신을 확인했다. 아래 마지막 값은 zero 송신 전 마지막 기록값이며, 물리 정지를 입증하는 후속 속도 기록은 없다.

## 결과

- 경로는 **01→03→04**까지 진행했다. 05·07에는 진입하지 않았으며 FINISH도 발생하지 않았다.
- T자 주차: 01:45:48.086 진입, GPS 탐색 후 01:45:58.085 경로 종점에서 취소. `CANCEL_ROUTE_END=10`, 이동거리 7.628m, 경과 약 10초. space_found/ready/handoff 이벤트 없음. 평행 주차는 진입하지 않았다.
- 01:46:23.235 AVOID_ACTIVE 진입. 01:46:36.185 새 회피 입력에서 경로 점이 1개→0개, generation=0, done=false가 됐다. 이때부터 REFERENCE_INVALID로 v_ref=0. GPS_RETURN/회피 완료 전환은 없었다.
- 위 첫 보호정지 시 ref_age는 0.306초였다. 단순히 0.5초 timeout을 넘어서 생긴 정지로 해석하면 안 된다. 빈 회피 경로가 직접 관측되며, 발행 측이 빈 경로를 만든 근본 원인은 이 보고서에서 확정하지 않는다.
- 01:47:40.646 외부 정지 입력이 추가돼 stop_reasons가 4→12로 변했다. 01:48:05.575 적색 감지 상태로 들어갔지만, 이미 안전 상태가 속도 결정권을 갖고 있었다.
- 마지막 목표속도는 **0m/s**지만 마지막 CAN 수신 속도는 **1.3924m/s**였다. 명령과 수신 피드백이 불일치하므로 물리적으로 멈췄다고 단정할 수 없다.

## 마지막 기록 상태

| 항목 | 값 |
|---|---|
| 최상위 | AUTONOMOUS_DRIVE (1) |
| 내비게이션 | LINE (0) |
| 회피 | AVOID_ACTIVE (1) |
| 신호 | RED_DETECTED (1) |
| 안전 | SAFE_STOP (3) |
| 미션 | MISSION_IDLE (0) |
| 미션 종류 | NONE (0) |
| 경로 소스 | AVOID (2) |
| 속도 결정권 | SAFETY (4) |
| legacy/MGM 통합 state | AVOID (2) |
| 경로 | 04, route_index=2 / 5, RUNNING, end_reached=0 |
| 미션 결과 | failed=1, cancel_reason=10(CANCEL_ROUTE_END); 현재는 MISSION_IDLE |
| Reference | available=0, valid=0, fresh=0, generation=0, age=97.667초 |
| 정지 사유 | 12 = REFERENCE_INVALID(4) + EXTERNAL(8) |
| 목표속도 / CAN 수신 속도 | 0 / 1.3924 m/s |
| 후진 복구 | configured=1, eligible=0, attempt_count=0, block_reason=FORCED_STOP(7) |
| 후방 복구 센서 | rear_sensor_valid=0, corridor=UNKNOWN |
| 보정 상태 | parking=NOT_REQUIRED(3), zone=CALIBRATED(1) |
| 마지막 GPS 로그 | RTK FIXED, 융합 헤딩은 종료 직전 ROS 수신에서 확인 |

## 전체 전환 38건

시간은 스냅샷의 실제 event_time_ns를 사용한다. 변경 열에 없는 상태는 직전 값을 유지한다. 동일 상태가 기록된 행도 생략하지 않았다.

| 번호 | KST | tick | 변경 상태 | v_ref | 정지 사유 |
|---:|---|---:|---|---:|---|
| 1 | 01:44:13.086 | 0 | 최상위=AUTONOMOUS_ENABLE; 내비게이션=LINE; 회피=INACTIVE; 신호=SIGNAL_IDLE; 안전=SAFE_STOP; 미션=MISSION_IDLE; 미션 종류=NONE; 경로 소스=LANE; 속도 결정권=SAFETY | 0 | 526: ALL_SENSORS_LOST + REFERENCE_INVALID + EXTERNAL + ROUTE_SEQUENCE |
| 2 | 01:44:13.985 | 90 | 내비게이션=GPS_BACKUP; 경로 소스=GPS; 정지 사유 변경 | 0 | 8: EXTERNAL |
| 3 | 01:44:27.285 | 1420 | 최상위=AUTONOMOUS_DRIVE; 안전=NORMAL; 속도 결정권=NAVIGATION; 정지 사유 변경 | 1 | 0: 없음 |
| 4 | 01:44:27.435 | 1435 | 내비게이션=LINE; 경로 소스=LANE | 1 | 0: 없음 |
| 5 | 01:45:08.685 | 5560 | 안전=SAFE_STOP; 속도 결정권=SAFETY; 정지 사유 변경 | 0 | 512: ROUTE_SEQUENCE |
| 6 | 01:45:10.286 | 5720 | 내비게이션=GPS_BACKUP; 경로 소스=GPS | 0 | 512: ROUTE_SEQUENCE |
| 7 | 01:45:10.296 | 5721 | 안전=NORMAL; 속도 결정권=NAVIGATION; 정지 사유 변경 | 1 | 0: 없음 |
| 8 | 01:45:12.025 | 5894 | 내비게이션=LINE; 경로 소스=LANE | 1 | 0: 없음 |
| 9 | 01:45:15.625 | 6254 | 내비게이션=GPS_BACKUP; 경로 소스=GPS | 1 | 0: 없음 |
| 10 | 01:45:16.085 | 6300 | 내비게이션=GPS_ONLY_NAV | 1 | 0: 없음 |
| 11 | 01:45:40.885 | 8780 | 내비게이션=GPS_BACKUP | 1 | 0: 없음 |
| 12 | 01:45:41.805 | 8872 | 내비게이션=LINE; 경로 소스=LANE | 1 | 0: 없음 |
| 13 | 01:45:42.385 | 8930 | 내비게이션=GPS_ONLY_NAV; 경로 소스=GPS | 1 | 0: 없음 |
| 14 | 01:45:48.086 | 9500 | 미션=MISSION_ACTIVE; 미션 종류=T_PARKING; 속도 결정권=MISSION | 0 | 0: 없음 |
| 15 | 01:45:48.185 | 9510 | 내비게이션=GPS_BACKUP | 0 | 0: 없음 |
| 16 | 01:45:49.285 | 9620 | 주요 상태 동일, 정지/벽 수집 구간 유지 | 0 | 0: 없음 |
| 17 | 01:45:49.975 | 9689 | 속도 결정권=NAVIGATION | 1 | 0: 없음 |
| 18 | 01:45:58.085 | 10500 | 안전=SAFE_STOP; 미션=MISSION_IDLE; 미션 종류=NONE; 속도 결정권=SAFETY; 정지 사유 변경 | 0 | 512: ROUTE_SEQUENCE |
| 19 | 01:46:05.796 | 11271 | 안전=NORMAL; 속도 결정권=NAVIGATION; 정지 사유 변경 | 1 | 0: 없음 |
| 20 | 01:46:08.785 | 11570 | 내비게이션=GPS_ONLY_NAV | 1 | 0: 없음 |
| 21 | 01:46:19.085 | 12600 | 내비게이션=GPS_BACKUP | 1 | 0: 없음 |
| 22 | 01:46:20.605 | 12752 | 내비게이션=LINE; 경로 소스=LANE | 1 | 0: 없음 |
| 23 | 01:46:23.235 | 13015 | 회피=AVOID_ACTIVE; 경로 소스=AVOID; 속도 결정권=AVOIDANCE | 0.985 | 0: 없음 |
| 24 | 01:46:26.905 | 13382 | 내비게이션=GPS_BACKUP | 0.6 | 0: 없음 |
| 25 | 01:46:36.185 | 14310 | 안전=SAFE_STOP; 속도 결정권=SAFETY; 정지 사유 변경 | 0 | 4: REFERENCE_INVALID |
| 26 | 01:46:36.295 | 14321 | 내비게이션=LINE | 0 | 4: REFERENCE_INVALID |
| 27 | 01:46:36.415 | 14333 | 내비게이션=GPS_BACKUP | 0 | 4: REFERENCE_INVALID |
| 28 | 01:47:40.646 | 20756 | 정지 사유 변경 | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 29 | 01:47:41.685 | 20860 | 내비게이션=LINE | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 30 | 01:47:42.985 | 20990 | 내비게이션=GPS_BACKUP | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 31 | 01:47:45.996 | 21291 | 내비게이션=LINE | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 32 | 01:47:47.475 | 21439 | 내비게이션=GPS_BACKUP | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 33 | 01:47:49.286 | 21620 | 내비게이션=GPS_ONLY_NAV | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 34 | 01:47:56.385 | 22330 | 내비게이션=GPS_BACKUP | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 35 | 01:47:58.075 | 22499 | 내비게이션=LINE | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 36 | 01:48:05.575 | 23249 | 신호=RED_DETECTED | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 37 | 01:48:07.655 | 23457 | 내비게이션=GPS_BACKUP | 0 | 12: REFERENCE_INVALID + EXTERNAL |
| 38 | 01:48:11.257 | 23817 | 내비게이션=LINE | 0 | 12: REFERENCE_INVALID + EXTERNAL |

## 경로 상태 전환 전체

| KST | 경로 | Route phase |
|---|---|---|
| 01:44:13.086 | 01 | DISABLED |
| 01:44:13.985 | 01 | RUNNING |
| 01:45:08.685 | 01 | WAIT_STOP |
| 01:45:10.186 | 01 | WAIT_ACK |
| 01:45:10.286 | 03 | RUNNING |
| 01:45:58.085 | 03 | WAIT_STOP |
| 01:46:05.745 | 03 | WAIT_ACK |
| 01:46:05.786 | 04 | RUNNING |

## 발생하지 않은 상태

- FINISH, AVOID GPS_RETURN/CLEAR_CONFIRM, AUTO_ESTOP, REVERSE_RECOVERY.
- 신호 APPROACH_STOP_LINE/STOPPED_WAIT, 평행 주차, 경로 05/07.
- MISSION_PREPARE enum은 관측되지 않았다. 즉시 MISSION_ACTIVE에 진입하는 설정이며, search_start 이벤트는 존재한다.

## 전체 타임라인

![state timeline](state_timeline.png)

## 원본 및 분석 파일

- [전환 38건: 이름·시각 포함 CSV](state_transitions_decoded.csv)
- [전체 24,047틱 상태 재생 CSV](replay_all_ticks.csv)
- [틱별 입력·시각·CAN 속도 CSV](input_telemetry.csv)
- 원본: `../../drive_logs/v2_20260914_014412_894120/`.
- 원본 로그와 제어 코드는 수정하지 않았다.
