# stack_lane — 요구사항

**담당: 이현준** · 산출물: 차선 단독 주행 (8/2)

## 역할

차선 검출(YOLO) → 차선 ref (camera 100ms) + 정지선 검출 → stack_traffic 전달

## 계약 (이것만 지키면 나머지는 자유)

- 입력: camera (100ms, OAK-D Pro). 차선 검출은 YOLO 기반.
- 출력 ①: `/perception/lane_path` (`fma_interfaces/LanePath`), 카메라 주기마다.
  - `points[]`: **vehicle frame** ref points — 생성 시점 차량 = (0,0,0). {x, y, yaw, curvature}.
  - `confidence`: 0.0~1.0. lane↔waypoint 전이 판정의 **재료** — 전이 판단 자체는 MGM이 한다.
- **점 개수: 1개** (팀 합의 2026-07-29) — 목표점 하나만 내면 dSPACE 궤적 생성(quintic)이 나머지를 채운다. 차량 전방 ~1m 부근 목표점 권장.
- 출력 ②: `/perception/stopline` (`fma_interfaces/StopLine`), 카메라 주기마다 (2026-07-30 결정).
  - 수신자는 MGM이 아니라 **stack_traffic(김재민)** — OAK-D 각도상 신호등이 안 보여
    정지선은 여기서, 신호등 적색은 stack_traffic 웹캠에서 판정해 결합한다.
  - `detected` + `distance`[m, vehicle frame 전방 x]. 미검출 프레임에도 detected=false로 **매 주기 발행**
    (수신 측 stale 판정이 발행 주기에 의존).
  - `header.stamp` 필수 — 수신 측 신선도 판정 기준.
- 금지: v_ref·정지 판단·모드 판단을 이 스택에서 하지 말 것 (CLAUDE.md §5.1). 정지선도 거리 보고만 — 정지 요구는 stack_traffic이, 적용은 MGM이 한다.
- 검증 시나리오: 차선 신뢰도가 떨어질 때 confidence가 실제로 떨어지는지 (MGM 히스테리시스가 이 값에 의존). 정지선 접근~정차 직전까지 distance가 연속적으로 줄어드는지 (특히 **0.5m 이내 근거리에서 시야 이탈 전까지 유지되는지** — stack_traffic 정지 트리거가 0.5m).

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_lane stack_lane_node`
