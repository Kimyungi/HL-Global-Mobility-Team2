# stack_parking — 요구사항

**담당: 손상민** · 산출물: 주차 단독 + MPC 검증 (8/2)

## 역할

주차공간 인식·로컬맵·주차 경로 + MPC/Vehicle MGM(dSPACE)

## 계약 (이것만 지키면 나머지는 자유)

- 입력: 2D LiDAR ×2 (100ms) → 주차 로컬맵.
- 출력: `/perception/parking` (`fma_interfaces/ParkingStatus`).
  - `space_found`: lane→parking 전이 조건의 절반 (나머지 절반은 GPS 주차구간 — stack_gps).
  - `path_blocked`: **동적 침범만**. 정적 경계(콘·연석)는 정지 트리거가 아니라 로컬맵 입력 (CLAUDE.md §4).
  - `points[]`: vehicle frame 주차 경로. 후진 구간 포함 — `v_suggest` 음수로 표현.
- 겸임: dSPACE 측 MPC·Vehicle MGM (quintic 궤적, feasibility, kinematic bicycle 상태 추정).
  PROTOCOL.md(bridge_dspace)가 인터페이스 기준 — dSPACE 모델은 이 문서와 합의 후 변경.
- 금지: 주차 중 회피 로직 만들지 말 것 — parking→avoid 전이가 없는 것이 설계다.

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_parking stack_parking_node`
