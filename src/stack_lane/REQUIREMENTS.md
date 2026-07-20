# stack_lane — 요구사항

**담당: 이현준** · 산출물: 차선 단독 주행 (8/10)

## 역할

차선 검출(YOLO) → 차선 ref (camera 100ms)

## 계약 (이것만 지키면 나머지는 자유)

- 입력: camera (100ms). 차선 검출은 YOLO 기반.
- 출력: `/perception/lane_path` (`fma_interfaces/LanePath`), 카메라 주기마다.
  - `points[]`: **vehicle frame** ref points — 생성 시점 차량 = (0,0,0). {x, y, yaw, curvature}.
  - `confidence`: 0.0~1.0. lane↔waypoint 전이 판정의 **재료** — 전이 판단 자체는 MGM이 한다.
- 점 개수는 자유(부족분은 MGM이 마지막 점 복제로 20개 정규화). 차량 전방 최소 ~1m 커버 권장.
- 금지: v_ref·정지 판단·모드 판단을 이 스택에서 하지 말 것 (CLAUDE.md §5.1).
- 검증 시나리오: 차선 신뢰도가 떨어질 때 confidence가 실제로 떨어지는지 (MGM 히스테리시스가 이 값에 의존).

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_lane stack_lane_node`
