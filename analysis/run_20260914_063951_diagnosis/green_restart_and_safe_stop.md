# 06:39:51 주행: 초록불 재출발 및 SAFE_STOP 분석

대상: drive_logs/v2_20260914_063951_118844. KST 기준.
mgm_snapshots.bin 18,310틱을 기록 당시 파라미터로 재생했고 transitions.csv의 공통 필드가 전부 일치했다.
원본 로그는 변경하지 않았다. 제어 코드 변경 없음.

## 초록불 이후 정지

| 시각 | 관측 |
|---|---|
| 06:40:28.656 | 신호 STOPPED_WAIT, v_ref=0 |
| 06:40:31.186 | 장애물 감지로 AVOID_ACTIVE. 회피 점 0개, SAFE_STOP_REFERENCE_INVALID=4 |
| 06:41:41.836 | red=0, green=1. SIGNAL_IDLE로 해제되지만 회피 점 0개 및 SAFE_STOP=4 유지 |
| 06:41:56.536 | 회피 점 1개 회복. GPS_RETURN, safety=NORMAL, v_ref=0.005부터 재개 |

초록 해제 이후 14.700초 동안 명령 정지가 지속됐다. 회피 점은 총 85.350초(8,535틱) 연속 비었다.
이 구간은 lidar_valid=1, gps_valid=1. 초록 해제 시 external_stop=0, traffic_fail_safe=0, auto_estop=0이다.
장애물 플래그는 간헐적으로만 참이었다. 초록 해제 순간에는 obstacle=0이었다.

stack_avoid 로그 1789335631.182563997: no collision-free cubic path with waypoint return at +2m.
GPS station=19.53639845747219, anchor=20.53639845747219, target=False, side=None, return=None.
1789335716.532907025: obstacle cleared; cubic GPS return, target=True, updates=1002.
따라서 SAFE_STOP이 플래너 실행을 막은 것이 아니라, 재계산에도 경로가 비어서 제어 명령이 막혔다.
단, 마지막 메시지는 후보 없음/범위/곡률/충돌/발행 실패를 구분하지 않는 공통 사유다.
원시 scan/후보별 거절 사유가 이 세션에 없어 정확한 기하 실패 원인은 확정하지 않는다.
코드는 현재 관측뿐 아니라 최초/최근 장애물 표면을 보존해 복귀 곡선도 검사한다. 지속 실패의 검토 대상이다.
06:41:56.526에 실제 속도 약 1.054m/s인데 MGM v_ref=0이었다. 따라서 56.536의 회복을 차량이 자동 재출발한 증거로 해석하지 않는다.

## SAFE_STOP 비트 전체

여기서 주차 경로 권한은 주차 미션 활성 중 GPS 탐색 단계를 제외한 상태다.

| 값 | 조건 |
|---:|---|
| 1 | 주차 경로 권한이 없고 GPS 전용 구역에서 유효 GPS 레퍼런스 상실 |
| 2 | 주차 경로 권한이 없고 camera_line_valid/gps_valid/lidar_valid 모두 false |
| 4 | 선택 경로의 점 수가 1개가 아님, 좌표/각도/곡률 비유한, 원점뿐인 점, 오래되거나 생성 시각 없는 참조, 소스 무효/불일치, 최종 속도 비유한 |
| 8 | external_stop: operator stop, go 대기/해제, 또는 CAN 상태 수신 이후 통신 고장/래치 |
| 16 | 주차 경로 권한을 가진 상태에서 주차 피드백 무효/지연 |
| 32 | 주차 경로 권한이 없고 traffic_fail_safe_stop. 카메라 실패/인지 예외 래치, 초기 준비 대기, traffic 메시지 수신 이후 0.5초 초과 단절 |
| 64 | 신호 접근/대기 중 주차 경로 권한이 없고 실속도 무효(수신 지연 0.2초 포함), 비유한 실속도 또는 단조 시계 역행 |
| 128 | 후진 복구 출력에서 후방 허용 조건 실패. 현재 escape_require_rear_clear=false이므로 후방 조건으로는 발생하지 않음 |
| 256 | 주차 경로 권한이 없고 Zone 정의는 수신했지만 좌표 보정 상태가 CALIBRATED가 아님 |
| 512 | 경로 순서 FAULT, 변경/전환 중, RUNNING 미도달, 유효 GPS 없이 차선 대체도 불가능, 또는 정상 중간 지점 관측 전 CSV 끝 감지. 종료 완료는 제외, 활성 주차는 FAULT 이후 검사에서 제외 |
| 1024 | parking_search_zone_only 활성 + MISSION_PREPARE + 탐색 Zone 불명 |
| 2048 | start_gate_enabled 활성 + 라이다 유효성 실패. 4개 raw scan 각각 0.35초 초과/미수신/미래 시각, avoid 지연/scan_valid=false, 활성 estop 지연/scan_valid=false |

추가 sensor_stop 분기도 SAFE_STOP을 만든다(전용 비트 없음):
- 주차 경로 권한이 없고 GPS 전용 구역 또는 GPS 주차 탐색 중 유효 GPS 참조가 없음.
- 주차 경로 권한이 없고 현재 Navigation과 GPS가 모두 불가하며, 라이다까지 무효이거나 회피 기능이 꺼짐.

이유는 OR 비트마스크이며 매 틱 다시 계산한다. 초록불은 신호 제약만 해제하며 다른 이유를 지우지 않는다.
AUTO_ESTOP은 별도 Safety 상태: 주차 미션이 아닐 때 라이다 auto_estop, 또는 회피 중 유효 라이다에서 TTC<ttc_stop/장애물 존재+avoidable=false.
적색 STOPPED_WAIT, 출발 인가 전, FINISH, 지정 정지 및 미션 대기 역시 v_ref=0을 만들 수 있지만 그 자체로 모두 SAFE_STOP인 것은 아니다.

근거: src/adas_mgm/core/{manager_step.cpp,manager_types.hpp,reference_safety.cpp,route_step.cpp},
src/adas_mgm/src/mgm_node.cpp, src/stack_traffic/stack_traffic/node.py,
src/stack_avoid/stack_avoid/gps_cubic_path.py, log_v2/ros/python3_40301_1789335591769.log.
