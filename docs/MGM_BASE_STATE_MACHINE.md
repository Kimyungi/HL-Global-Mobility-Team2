# MGM 공통 베이스 상태 머신 — 현재 기준 안내

현재 C++ 병행 Manager와 향후 Stateflow의 **단일 기준은 [MGM_MBD_STATE_MACHINE_SPEC.md](MGM_MBD_STATE_MACHINE_SPEC.md)**다.
3차 구현 당시 보고서와 빈 Reference 위험 재현은 [역사적 3차 기록](history/MGM_BASE_STATE_MACHINE_REV3.md)에 보존했다.
그 문서의 시험 결과/미해결 사항은 당시 기록이며 현재 운용 명세로 사용하지 않는다.
6차 구현·검증·Calibration 상태는 [6차 보고서](MGM_STABILIZATION_REPORT.md)에 있다.

## 현재 구조

TopState 아래 Navigation/Avoidance/Signal/Safety/Mission을 병행 실행한다.
NAV_RESELECT는 저장 상태가 아닌 공통 선택 함수다. 50/200/300 MGM cycle 조건은 유지한다.
legacy LANE/WAYPOINT/AVOID/PARKING/TRAFFIC byte는 CAN/debug 호환 표시이며 전체 FSM이 아니다.

## Zone entry

기존 GPS ranges/ZoneDefinition은 raw membership을 제공한다. 새로운 유효 GNSS generation마다
Zone별 entry/exit 확인 수를 쌓아 stable edge를 만든다. 동일 fix의 100Hz 반복은 새 표본이 아니다.
GPS invalid는 stable 문맥을 보존하고 fake edge를 만들지 않는다. GPS-only GPS 상실은 정지한다.
zone_enter_confirm_samples/zone_exit_confirm_samples는 0=미설정이다. 정의된 Zone이 있으나
기준이 없으면 일반 주행에 ZONE_CONTEXT_UNAVAILABLE 정지를 적용한다. 실제 표본 수는 측정 후 설정한다.

## Mission entry와 제어권

**MISSION_IDLE → MISSION_PREPARE → MISSION_ACTIVE**다.
stable MISSION_ZONE entry는 request ID/Mission ID/type/source Zone을 latch하며 PREPARE 동안
Navigation/Avoidance/Signal/Safety는 그대로 실행한다. Zone exit나 chatter는 요청을 취소하지 않는다.
현재 요청의 준비 완료와 시간/실제 이동거리 제한을 확인한 틱에 ACTIVE로 전환한다.
그때부터 Parking만 reference/speed를 소유하고 실행 ack 및 유효 reference까지 0 속도로 대기한다.

현재 명령은 `/parking/mission_command`의 typed ParkingCommand PREPARE/ACTIVATE/CANCEL이다.
과거 `/parking/gps_command` String 즉시 시작 경로는 MGM 통합 Mission 계약이 아니다.
기존 주차 모듈의 detector/planner/localization/maneuver 알고리즘을 재사용한다.
현재 요청 실행 ack 이후 done만 Mission ID별 완료 기억에 기록한다. 취소는 완료가 아니다.
종료 후 현재 GPS-only/LINE/GPS 조건으로 공통 nav_reselect를 수행한다.

## 안전 및 역사적 재현과의 차이

선택 Reference가 invalid/stale/empty이면 제어권을 유지하고 양·음 속도를 0으로 차단한다.
3차의 empty CLEAR_CONFIRM/초기 (0,0)에 양의 속도가 붙던 기록은 현재 동작이 아니다.
외부 정지/운전자/CAN 및 독립 SAFE_STOP mask는 Mission에서도 유지한다.
Recovery는 enum과 기존 알고리즘이 있으나 config OFF와 rear corridor 인증 생산자 부재로 운용 불가다.
실제 후방 인증 없이 rear_clear bool 단독으로 실행되지 않는다.

## 세션과 이식

FINISH/완료 기억은 explicit start_session에서만 초기화한다. 요청 ID high-water는 유지한다.
비활성 중 이미 확정된 entry는 go 때 다시 합성하지 않는다. reset 틱에는 Mission을 시작하지 않는다.
reset 당시 raw 내부인 Mission Zone은 이후 확인이 끝나더라도 실행을 억제한다. 확인된 exit/reentry가 필요하다.
현재 bus/dump는 v12이며 구 dump는 해당 버전 빌드로 재생한다.
