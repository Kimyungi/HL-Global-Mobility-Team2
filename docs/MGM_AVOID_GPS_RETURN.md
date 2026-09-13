# 회피 상태 내부의 GPS 복귀 — 2026-09-13

일반 회피가 완료되면 회피 상태를 유지하며 GPS waypoint reference를 추종한다.
현재 station 기준 횡오차 절댓값 **0.1m 이하**와 방향오차 절댓값 **20° 이하**를
동시에 만족해야 정상 회피 종료와 공통 Navigation 재선택을 허용한다.

실행 순서:

1. 장애물 회피: `AvoidState::AVOID_ACTIVE=1`, 기존 회피 reference/속도 사용.
2. 기존 `maneuver_done` 또는 `avoid_max_cycles` 도달: `GPS_RETURN=3` 진입.
   legacy state는 계속 `AVOID=2`, reference는 GPS, 속도는 기존 GPS 목표와 복귀 ramp다.
3. 두 오차 조건과 fresh GPS/유효 실제 heading을 만족하면 `INACTIVE=0`으로 종료한다.
   기존 GPS-only/LINE confidence 조건으로 Navigation을 재선택한다.
   종전 회피 종료 후 300틱 GPS hold는 이 오차 조건으로 대체하며 추가로 기다리지 않는다.

GPS_RETURN에서는 시간 경과나 LINE 신뢰도만으로 회피를 끝내지 않는다.
GPS가 invalid/stale이면 GPS reference 소유권과 회피 상태를 유지하고 최종 gate에서 정지한다.
heading이 미상/접선 fallback이거나 오차가 비유한 값이면 복귀 완료로 인정하지 않는다.
새 장애물이 `obstacle_detected && avoidable && !maneuver_done`과 기존 구간 허용 조건을
충족하면 다시 AVOID_ACTIVE로 들어가 회피 reference를 사용한다.
외부/CAN/신호/TTC/E-stop 정지와 Mission 우선권, FINISH, 명시적 disable/reset은 유지한다.
장애물 없이 Nav 입력만 잃었던 LiDAR fallback의 복구는 기존 Nav 재선택을 유지한다.

E-stop 자동후진을 시작하는 **같은 tick**에 실제 AvoidState를 AVOID_ACTIVE로 설정한다
(일반 회피가 ON인 구성). 후진 중에는 Safety=REVERSE_RECOVERY가 Escape reference와
음수 속도를 선택한다. 후진이 끝나면 회피 상태에서 회피 reference를 사용하며,
회피 완료 뒤 위 GPS_RETURN을 거쳐 종료한다. 후진 시간은 후속 회피 maneuver 시간 상한에
포함하지 않는다. 회피 reference가 없으면 그 상태에서 정지하고 입력을 기다린다.

GPS `StationPath.heading()`은 현재 station 양쪽의 기존 경로 tangent를 각도 wrap하여 보간한다.
`GpsPath.station_yaw_error_rad = wrap(station heading - vehicle heading)`이며
`station_error_valid`는 실제 heading 관측이 있을 때만 true다.
기존 `cross_track_m`은 차량과 bounded station 투영 위치 사이의 거리다. station 추적이
지연되면 진행방향 차이도 포함할 수 있어 이 경우에도 0.1m 조건을 만족해야 한다.
현재 station 탐색 범위, 2.5m preview, 90% snap/보간, CAN 한 점 출력은 바꾸지 않는다.
2.5m 앞 preview의 yaw와 현재 station의 yaw error를 서로 대신 사용하지 않는다.

`MgmState.avoidance=3`이 GPS_RETURN을 나타낸다. 같은 메시지의
`avoid_return_cross_track_m`, `avoid_return_yaw_error_rad`, `avoid_return_error_valid`로
판정 입력을 확인한다. GPS 메시지와 raw dump에도 오차/validity를 기록한다.
메시지 변경에 따라 interfaces와 의존 패키지를 함께 빌드해야 한다. raw dump는 **v23**이며
기존 주행의 v22 기록은 해당 버전 빌드로 재생한다.

검증: 관련 13개 패키지 빌드 성공, CTest 21/21, GPS·메시지 Python 177개 통과.
`test/avoid_gps_return_ros_smoke.py`로 localhost의 별도 ROS_DOMAIN_ID=188에서 MGM만
실행하여 7개 연결 검사를 통과했다. GPS_RETURN의 실제 ROS state/reference/오차 필드,
GPS timeout 정지, 두 임계값 충족 시 종료, 후진 시작과 종료의 AVOID 유지를 검사했다.
차량·센서·CAN bridge를 구동하는 주행 시험은 실행하지 않았다.
