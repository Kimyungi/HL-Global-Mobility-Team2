# stack_traffic — 요구사항

**담당: 김재민** · 산출물: 신호등 정지 (8/2) — 하위제어 사전 작업(7/6~7/17) 완료 기반

## 역할

신호등·정지선 인식 → 정지 요구

## 계약 (이것만 지키면 나머지는 자유)

- 입력: camera.
- 출력: `/perception/traffic_stop` (`fma_interfaces/TrafficStop`).
  - `stop_required`는 **요구이지 명령이 아니다**: 적용 여부·우선권은 MGM 스테이트 머신이 결정
    (lane·waypoint에서는 긴급 정지 다음 순위, parking에서는 비활성 — CLAUDE.md §4).
- 청색 전환 시 stop_required=False로 즉시 해제 — 출발 판단도 MGM(요구 소멸 = 기본 속도 복귀).
- 금지: v_ref 직접 산출 금지. 정지 거리 프로파일링이 필요하면 stop_distance로 전달만.

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_traffic stack_traffic_node`
