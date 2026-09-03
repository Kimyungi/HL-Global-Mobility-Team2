# stack_parking — 요구사항

**담당: 손상민** · 산출물: 주차 단독 + MPC 검증 (8/2)

## 역할

주차공간 인식·로컬맵·주차 경로 + MPC/Vehicle MGM(dSPACE)

## 계약 (이것만 지키면 나머지는 자유)

- 입력: `multi_lidar_fusion`의 전·후 보정 endpoint cloud
  (`/lidar/a1/cloud`, `/lidar/a2/cloud`, frame=`base_link`) → 10Hz ICP 자세 추정 +
  주차 로컬맵. 좌·우 LiDAR는 반복 탈락해 주차 SLAM의 필수 입력에서 제외한다.
  endpoint만 누적하며 `base_link`에서 가짜 free-space ray를 만들지 않는다.
- 주차 완료 전용 입력: 후방 원본 LiDAR `/lidar/a2/scan`. 보정 후 후방 벽 지지점이
  0.20m 이하일 때만 완료 정차를 시작한다.
- 출력: `/perception/parking` (`fma_interfaces/ParkingStatus`).
  - `space_found`: lane→parking 전이 조건의 절반 (나머지 절반은 GPS 주차구간 — stack_gps).
  - `path_blocked`: **동적 침범만**. 정적 경계(콘·연석)는 정지 트리거가 아니라 로컬맵 입력 (CLAUDE.md §4).
  - `points[]`: vehicle frame 주차 경로. 후진 구간 포함 — `v_suggest` 음수로 표현.
    **점 개수: 1개** (팀 합의 2026-07-29) — 현재 추종 목표점 하나만. 나머지는 dSPACE 궤적 생성이 채운다.
  - `dx/dy/dyaw/update`: 연속 10Hz LiDAR SLAM pose의 이동량. 이동량은 직전
    vehicle frame이며 MGM이 PARKING TargetRef에 그대로 전달하고 100Hz 사이에는 hold한다.
- `/vehicle/vector`에서는 실속도 `v`만 motion prior와 정지 확인에 사용한다.
  x/y/yaw pose를 직접 쓰지 않는다. yaw prior는 `/perception/imu`의 상대 yaw,
  x/y drift 보정은 RTK FIXED `GpsPath` delta를 사용한다. **dSPACE는 parking
  스테이트 중에도 vehicle vector를 계속 회신한다** (PROTOCOL.md RX).
- 내부 단계는 `SLAM → MAPPING → LOCALIZATION → PARKING`. 계획 확정 뒤 map을
  동결하고 localization 연속 정합 전에는 MGM에 `space_found=true`를 내지 않는다.
- 경로 target은 ICP map에서 계획하되 매 publish마다 현재 SLAM 자세를 이용해
  `base_link`로 변환한다. preview는 1.0m, `points[]`는 항상 최대 1개다.
- 겸임: dSPACE 측 MPC·Vehicle MGM (quintic 궤적, feasibility, kinematic bicycle 상태 추정).
  PROTOCOL.md(bridge_dspace)가 인터페이스 기준 — dSPACE 모델은 이 문서와 합의 후 변경.
  **CAN 수신부 주의: REF_POINT는 헤더의 n_points개만 온다** (현재 모든 소스 1점 — n_points는 확장 대비 필드) — 궤적 생성이 목표점을 지평으로 보간.
- 금지: 주차 중 회피 로직 만들지 말 것 — parking→avoid 전이가 없는 것이 설계다.

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 launch stack_parking parking.launch.py`
- GPS 없는 단독 시험: `ros2 launch stack_parking parking_standalone.launch.py`
  (실제 `stack_gps`와 동시 실행 금지)
