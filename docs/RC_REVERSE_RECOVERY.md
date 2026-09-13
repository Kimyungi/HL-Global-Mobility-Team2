# Integration v2 무인 RC 자동 후진 — 2026-09-13

사용자가 지정한 1/10 무인 RC 차량의 후방에 사람·장애물이 없는 시험 구성을 적용한다.
`src/adas_mgm/config/params.yaml`의 설정은 다음과 같다.

| 항목 | 값 | 동작 |
|---|---:|---|
| escape_after_cycles | 1000 | E-stop 조건이 연속 1000 제어 tick 지속되면 후진 진입 |
| v_escape | -0.8 m/s | 일반 주행 v_base와 독립된 후진 목표 속도 |
| escape_max_cycles | 162 | 한 번의 후진 명령은 최대 162 제어 tick |
| escape_require_rear_clear | false | 후방 bool/typed CLEAR/freshness를 후진 허가 조건에서 제외 |

10ms 제어 주기에서 대기는 약 10초, 후진 명령은 약 1.62초이며
명령 속도에 시간을 곱한 거리는 **1.296m**다. 실제 이동거리 1.3m를 측정하여
정지하는 제어는 아니다. 제어 주기 지연·추종·정지 동작에 따라 실제 거리는 달라진다.
E-stop이 먼저 해제되면 후진도 일찍 종료된다. 기존 재무장 조건과 대기 시간을
다시 충족하면 반복할 수 있으며 1.296m는 모든 시도를 합한 총 거리 상한이 아니다.

기존 전진 출력 이후의 arming, DRIVE 상태, E-stop 지속, 후진 종료 알고리즘을 사용한다.
이 대기는 같은 장애물이 움직이지 않았는지 추적하는 판정이 아니다.
Mission ACTIVE에서는 탐색·실행 중 모두 일반 회피/E-stop/Recovery가 제외된다.
운전자·CAN 외부 정지, FINISH/disable, 신호 정지와 reference 최종 검사는 유지한다.

후방 CLEAR producer를 새로 만들거나 UNKNOWN을 CLEAR로 바꾸지 않는다.
후방 입력과 진단은 실제 값을 기록한다. `escape_require_rear_clear=true`로 설정하면
legacy rear bool과 fresh typed CLEAR를 모두 요구하는 기존 검사를 다시 적용한다.
이 parameter는 시작 시 YAML에서 읽으며 변경 후 노드를 재시작한다.

일반 v2 통합 런처는 위 YAML을 사용한다. no-estop 런처는 기본 delay를 0으로 강제하며
MBD/bench 구성도 Recovery OFF를 유지한다. `escape_after_cycles:=0`은 명시적 비활성이다.
한라대·용인 런북의 통합 명령은 delay 1000을 사용한다.

병행 Manager에서 후방 요구 parameter와 독립 후진 속도의 의미가 바뀌므로 raw dump는 v22다.
v21 이전 기록은 해당 버전 빌드로 재생한다. 이 변경은 주차 자체의 후방 벽 도착 판정과
별도인 MGM E-stop Recovery 변경이다. 다른 작업창의 주차 코드 수정은 포함하지 않는다.

검증: MGM 빌드 성공, CTest 21/21, 런처 그래프 시험 25/25 통과.
후방 UNKNOWN/BLOCKED opt-out, 1000틱 대기와 162회 후진 출력, 별도 후진 속도,
외부 정지와 Mission 제외, 후방 요구 재활성화를 확인했다. 차량 주행은 실행하지 않았다.
