# PR #108 ESTOP 검토

2026-09-16. 대상: https://github.com/Kimyungi/HL-Global-Mobility-Team2/pull/108
이하 표는 적용 전 검토 기록이다. 이후 사용자 승인으로 현재 상위 ESTOP에 맞춰 통합했다.
[후속 적용 보고서](V2_PR108_INTEGRATION.md)를 현재 계약으로 사용한다.

PR은 main 대상이며 현재 주행 메인은 integration/v2_main이다.
실제 EstopRequest가 유지되면 10초 후 후방 여유·차속 조건을 확인해 -0.3m/s로 후진한다.
후진 제어점은 (-1,0,0,0), 실측 음수 차속의 10ms 적분으로 1m 완료를 판단한다.
5초 내 1m 미달이면 ESTOP 정지를 유지하며 같은 장애물에 대한 후진 반복을 차단한다.

## 현재 v2와의 차이

| 항목 | 현재 v2 | PR #108 |
|---|---|---|
| 진입 입력 | 상위 제어가 전방/좌측/우측 raw LiDAR로 판단 | 독립 EstopRequest 사용 |
| 종료 | 동일 episode의 회복주행 완료 응답 | 1m 후진 완료 또는 estop 입력 해제 |
| 회복 실행 | 별도 실행기 보류, 유효 요청 없으면 정지 | 옛 transition() 내부에 정지/후진 로직 추가 |
| 상태 전달 | MgmState의 상위 ESTOP과 기존 CAN 상태 투영 | TargetRef/CAN state=5 추가 |
| 적용 경로 | base manager + revised v2 | main 기반의 legacy transition 변경 |

PR diff의 transition()은 estop 입력이 사라지면 1m 후진 완료 전에도 즉시 이전 상태로
복귀한다. 따라서 '회복주행 완료 후에만 탈출'이라는 현재 계약과 다르다.
현재 v2는 base manager 분기를 사용하므로 이 legacy transition 변경을 단순히
붙이는 것으로 현재 상위 ESTOP 실행기가 연결되는 것은 아니다.

PR 작성자에 따르면 stack_estop이 rear_clear를 채우지 않아 기본 설정에서는
후진 게이트가 열리지 않는다. CAN state=5는 하위 제어기 수신 처리도 맞춰야 한다.
현재 전방 0.25m/좌우 0.15m·한 센서 연속 3회 진입 계약과 episode 요청 ID를
유지한 채 회복 실행부를 연결하는 별도 통합 작업이 필요하다.

검증 범위: PR 설명·diff와 현재 v2 코드 비교. PR 작성자는 단독 escape_reverse_test
통과를 보고했지만 ROS 전체 빌드·실차 시험은 수행하지 않았다고 명시했다.
이 검토에서는 PR 테스트를 별도로 실행하지 않았다.
