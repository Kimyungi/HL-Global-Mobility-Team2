# MGM Mission Preparation — 현재 기준 안내

현재 계약의 정본은 **[6차 단일 명세](MGM_MBD_STATE_MACHINE_SPEC.md) §6**이다.
5차 구현의 23항목 보고·당시 시험 결과·이전 자료 조사는 [역사적 5차 기록](history/MGM_MISSION_PREPARATION_REV5.md)에 보존했다.
현재 시험 결과는 [6차 보고서](MGM_STABILIZATION_REPORT.md)를 사용한다.

## 요청 lifecycle

stable MISSION_ZONE entry → IDLE→PREPARE 요청 latch → 현재 request의 준비 완료 → ACTIVE → DONE/CANCEL.
Zone exit/다른 Zone/chatter는 active request를 덮어쓰거나 취소하지 않는다. queue는 없다.
PREPARE는 기존 일반 주행/회피/신호/안전을 실행하고, ACTIVE에서만 Parking에 제어권을 준다.
ParkingCommand PREPARE/ACTIVATE/CANCEL 및 ParkingStatus의 request_id/mode/실제 생성 시각으로
늦은 이전 명령·space·ready·done·ref를 거부한다. 초기화는 기존 pipeline/SLAM/mission reset을 재사용한다.
ACTIVATE 전 maneuver tick을 진행하지 않는다. 실행 ack 이후 기존 done만 완료 기억에 기록한다.

## 제한과 상태 진단

parking_search_timeout / max_parking_search_distance는 운영 -1.0 / -1.0 미설정이다.
CALIBRATED=두 유한 양수, UNCALIBRATED=-1 포함, INVALID_CONFIG=0/그 밖의 음수/비유한 값이다.
유효하지 않은 설정은 CALIBRATION_REQUIRED 취소이며 주차 제어권을 얻지 않는다.
PREPARE 동안 실제 monotonic 경과 시간과 integral(abs(actual VehicleVector.v)*dt)를 사용한다.
외부 stop 동안에도 timeout은 진행하고, 속도 freshness 상실/시계 역행은 MOTION_UNAVAILABLE로 취소한다.
시간/거리 제한은 ACTIVE 실행 시간 상한으로 사용하지 않는다. 취소는 완료로 기록하지 않는다.

## Calibration workflow

mission_events_csv_path에 zone_entry/search_start/space_found/ready/handoff/cancel/done을 기록한다.
각 이벤트는 request/Mission/type/source Zone, ROS 시각, monotonic elapsed, ENU/idx/실측 속도/누적 거리를 갖는다.
Search/space/ready는 MGM이 현재 요청 응답을 받은 관측 시각이므로 통신 지연을 포함한다.
센서 생성 시각은 preparation_stamp/reference_stamp와 별도로 추적한다.

```bash
python3 src/adas_mgm/tools/analyze_mission_calibration.py /path/to/mission_events.csv \
  --json /tmp/mission_calibration.json --markdown /tmp/mission_calibration.md
```

타입별 시간/거리 평균·최대·백분위수·표본 수, 성공/timeout/cancel을 집계하며 누락값은 제외한다.
운영값/YAML은 생성하지 않는다. 기존 parallel 37.92s/2.188m 단일 로그도 운영 상한에 채택하지 않았다.
Search trigger는 기존 ZoneDefinition index_range/start/end로 maneuver 위치보다 앞에 둘 수 있다.
실측 entry→ready 거리를 바탕으로 기존 range를 대체하며 폭/선행 거리를 임의 지정하지 않는다.

6차 Zone 확인 수 역시 0=미설정이다. 정의된 Zone이 있으나 확인 기준이 없으면
ZONE_CONTEXT_UNAVAILABLE로 일반 주행을 정지한다. Parking 제한 미설정 진단과 별개다.
현재 bus/dump는 v12다. 이전 v11 모의 검증 숫자를 현 빌드 검증 결과로 혼동하지 않는다.
