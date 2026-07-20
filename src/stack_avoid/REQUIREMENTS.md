# stack_avoid — 요구사항

**담당: 이기돈** · 산출물: 회피 단독 주행 (8/10) — 하위 제어 보수 겸임(김재민에게서 인수)

## 역할

장애물 인지, 회피 가능 판정 재료(TTC·측방), 회피 경로

## 계약 (이것만 지키면 나머지는 자유)

- 입력: 2D LiDAR ×2 (100ms).
- 출력: `/perception/avoid` (`fma_interfaces/AvoidStatus`).
  - `avoidable`(TTC·측방 여유)은 판정 **재료** — lane/waypoint→avoid 전이 결정은 MGM.
  - `ttc`: 장애물이 없으면 반드시 큰 값(1e9). 0으로 두면 MGM이 즉시 정지 바닥을 밟는다.
  - `points[]`: vehicle frame 회피 경로, `maneuver_done`: 복귀 트리거.
  - `v_suggest`: 회피 기하로 결정한 속도 — avoid 스테이트의 종방향은 이 값이 지배.
- 금지: 이 스택에서 정지/모드 결정 금지. "회피 경로 요구" 같은 별도 요구 개념도 없음 — 회피는 스테이트다 (CLAUDE.md §4).

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_avoid stack_avoid_node`
