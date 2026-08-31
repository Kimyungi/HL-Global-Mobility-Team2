# stack_lane — 요구사항

**담당: 이현준** · 산출물: 차선 단독 주행 (8/2)

## 역할

차선 검출(YOLO) → 차선 ref (camera 100ms). 정지선은 stack_traffic의 전용 OAK-D가
주간 흰색·야간 국소 대비·평행 에지 쌍으로 직접 검출한다.

## 계약 (이것만 지키면 나머지는 자유)

- 입력: camera (100ms, OAK-D Pro). 차선 검출은 YOLO 기반.
- 출력 ①: `/perception/lane_path` (`fma_interfaces/LanePath`), 카메라 주기마다.
  - `points[]`: **vehicle frame** ref points — 생성 시점 차량 = (0,0,0). {x, y, yaw, curvature}.
  - `confidence`: 0.0~1.0. lane↔waypoint 전이 판정의 **재료** — 전이 판단 자체는 MGM이 한다.
- **점 개수: 1개** (팀 합의 2026-07-29) — 목표점 하나만 내면 dSPACE 궤적 생성(quintic)이 나머지를 채운다. 차량 전방 ~1m 부근 목표점 권장.
- ~~출력 ②: `/perception/stopline`~~ **폐기 (2026-08-08, PR #21·#28)** — 정지선 검출은
  stack_traffic(김재민)이 자체 OAK-D에서 수행하는 것으로 재배정됨. stack_lane은
  정지선을 발행하지 않는다. 상세: CLAUDE.md §6.
- 금지: v_ref·정지 판단·모드 판단을 이 스택에서 하지 말 것 (CLAUDE.md §5.1).
  정지 요구는 stack_traffic이 만들고 적용은 MGM이 한다.
- 검증 시나리오: 차선 신뢰도가 떨어질 때 confidence가 실제로 떨어지는지
  (MGM 히스테리시스가 이 값에 의존). 정지선 검증은 stack_traffic 문서에서 수행한다.

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_lane stack_lane_node`
