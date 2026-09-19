# stack_lane — 요구사항

**담당: 이현준** · 산출물: 차선 단독 주행 (8/2)

## 역할

차선 검출(YOLO) → 차선 ref (camera 100ms). 정지선은 stack_traffic의 전용 OAK-D가
주간 흰색·야간 국소 대비·평행 에지 쌍으로 직접 검출한다.

## 계약 (이것만 지키면 나머지는 자유)

- 입력: camera (100ms, OAK-D Pro). 차선 검출은 YOLO 기반.
- 2026-09-18: 통합 revised_v2는 `zone_gated=true`. 카메라 영상·heartbeat는 상시 유지하고,
  MGM의 GPS 전용·신호등·미션 zone 및 활성 주차·회피 상태, GPS 회피 범위에서 추론·차선 경로 발행을 중단한다.
  일반 구간 복귀 시 이전 추적 이력을 초기화한다. 단독 실행 기본은 기존 상시 추론이다.
- 출력 ①: `/perception/lane_path` (`fma_interfaces/LanePath`), 카메라 주기마다.
  - `points[]`: **vehicle frame** ref points — 생성 시점 차량 = (0,0,0). {x, y, yaw, curvature}.
  - `confidence`: 0.0~1.0. lane↔waypoint 전이 판정의 **재료** — 전이 판단 자체는 MGM이 한다.
- **점 개수: 1개** (2026-09-12 사용자 지정). 차량 원점의 중심선 최근접 투영 station에서
  경로를 따라 **+2.5m**인 점의 x/y/yaw/curvature만 반환한다. x=2.5나 원점 직선거리=2.5가 아니다.
  내부 다점 피팅과 20점 타당성 검사는 유지하며 곡률·신뢰도로 목표 거리를 바꾸지 않는다.
  yaw/curvature는 같은 피팅 곡선의 1·2차 미분으로 구한다. 가시 범위 밖은 해당 곡선의 외삽이다.
  `lookahead_m`은 기존 연속성 검사의 x 기준점, `n_points`는 내부 검사 표본 수이며 반환 수/preview 거리가 아니다.
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
