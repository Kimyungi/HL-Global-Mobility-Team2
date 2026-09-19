# MGM Reference Safety — 현재 기준 안내

현재 규칙과 SAFE_STOP SET/CLEAR 표는 **[6차 단일 명세](MGM_MBD_STATE_MACHINE_SPEC.md)**가 정본이다.
4차 위험 재현·생산자 조사·당시 시험 결과는 [역사적 4차 기록](history/MGM_REFERENCE_SAFETY_REV4.md)에 보존했다.
6차 구현/검증 결과는 [6차 보고서](MGM_STABILIZATION_REPORT.md)에 있다.

## 유지한 Reference 계약

available은 점 존재, fresh는 실제 reference_stamp에 근거한 생성 나이가 기존 timeout 이내,
valid는 위 조건과 센서/모듈 사용 조건, 정상 점 개수, 모든 기하 finite, zero-filled 기본 버퍼 아님을 뜻한다.
LINE HELD/SEARCH, GPS 동일 fix, Avoid 동일 scan, Parking 미갱신 localization, fusion timer 재발행은
실제 생성 나이를 초기화하지 않는다. ROS 시간이 멈춰도 monotonic age는 증가한다.

현재 timeout은 운용 YAML LINE 1.0s, GPS/Avoid/Parking 기존 0.5s다.
유효하지 않은 provider는 assembler에 넣지 않는다. 기존 assemble/merge 뒤 final_reference_gate와
ROS 발행 직전 gate가 선택 소스/기하/속도를 재검사하여 양·음 속도를 모두 0으로 차단한다.
빈 경로에서 새 직진점이나 가짜 유효 경로를 생성하지 않는다.

## Mission / Avoidance

PREPARE는 일반 Nav/Avoidance reference를 검사한다. Mission ref가 없어도 정상이다.
ACTIVE는 현재 request ID/mode/실행 ack/요청 이후 실제 generation을 추가 검사한다.
Mission 또는 Avoidance ref가 invalid이면 제어권을 유지하고 정지하며, 정상 종료 전이 전에는
다른 provider로 자동 교체하지 않는다. LINE/GPS의 기존 허용 fallback은 유지한다.

## 독립 SAFE_STOP mask

기존 GPS_ONLY_GPS_LOSS, ALL_SENSORS_LOST, REFERENCE_INVALID, EXTERNAL,
MISSION_FEEDBACK, TRAFFIC_INPUT, VEHICLE_SPEED, REAR_UNAVAILABLE를 유지한다.
6차는 미설정 Zone 확인 기준을 ZONE_CONTEXT_UNAVAILABLE(256)로 노출한다.
각 reason은 독립적으로 계산하며 green 또는 한 입력 복구가 다른 정지 이유를 지우지 않는다.
정확한 SET/CLEAR, latch/자동 복구, PREPARE/ACTIVE 적용 범위는 단일 명세 §8 표를 사용한다.

## 6차에서 확정한 관측/한계

Zone은 raw/stable을 구분하고 독립 GNSS 확인을 거친다. 표본 수/공간 hysteresis 값은 실측 전 미설정이다.
기존 전역 nearest index의 지속적인 잘못된 segment 선택은 debounce로 해결되지 않는다.
Recovery는 UNKNOWN/BLOCKED/CLEAR와 rear actual generation freshness를 요구하며 운영 OFF를 유지한다.
후방 전체 corridor의 인증 알고리즘/여유/coverage는 아직 확정하지 않았다.
실차 제동 거리/조향 유지/실제 센서 타이밍/CAN watchdog 동작을 이번 모의 검증으로 인증하지 않는다.
메시지 변경 후 관련 provider와 MGM은 동일 overlay로 재빌드해야 한다. 현재 dump는 v12다.
