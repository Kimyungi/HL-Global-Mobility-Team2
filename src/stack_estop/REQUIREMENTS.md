# stack_estop — 요구사항

**담당: 박찬미** · 산출물: 긴급 정지 (8/10)

## 역할

돌발 장애물 감지 → 긴급 정지 요구

## 계약 (이것만 지키면 나머지는 자유)

- 입력: 2D LiDAR (최우선 반응 경로 — 가능한 한 짧은 주기, 스켈레톤 기본 50ms).
- 출력: `/perception/estop` (`fma_interfaces/EstopRequest`).
  - `estop`은 모든 주행 스테이트 우선권 표의 **최상위 요구**. MGM이 v_ref=0 (rate limit 우회)으로 반영.
  - **정지는 스테이트가 아니다**: estop 중에도 주행 스테이트는 유지되고, 요구 해제 시 자동 복귀.
    따라서 estop 해제 조건(장애물 소멸 확인)을 신중히 — 해제 즉시 차가 다시 출발한다.
- 금지: 이 스택에서 모터/브리지에 직접 명령 금지. 유일한 출구는 estop 플래그.
- 검증: 감지→토픽 발행 지연 실측 (전체 정지 체인: 감지 → MGM 10ms → TX → dSPACE).

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_estop stack_estop_node`
