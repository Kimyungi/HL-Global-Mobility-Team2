# stack_gps — 요구사항

**담당: 김윤기 (팀장)** · 산출물: GPS 단독 주행 (8/10) — 베이스 설치·RTK 포함

## 역할

GPS·IMU 융합, RTK, waypoint ref

## 계약 (이것만 지키면 나머지는 자유)

- 입력: GPS(수백 ms) + IMU(~10ms 비주기) 융합 → 위치·헤딩. RTK 베이스 설치 포함.
- 출력: `/perception/gps_path` (`fma_interfaces/GpsPath`).
  - `points[]`: 전역 waypoint를 **vehicle frame으로 변환 완료한** ref points.
  - `accel_zone` / `parking_zone`: 구간 플래그 — 속도를 올리는 판단은 MGM 우선권 표가 한다.
- localization 보정: `/vehicle/vector` (dSPACE 상태 추정 회신, 10ms) 구독하여 GPS 갱신 사이 dead-reckoning 보정.
- 금지: v_ref 결정·모드 판단 금지 (CLAUDE.md §5.1). accel_zone은 요구의 원천일 뿐.
- 검증: RTK fix 상태에서 waypoint 추종 오차, GPS 음영에서 vehicle vector 보정 유지 시간.

## 베이스 스테이션

RTK 베이스(EVK-F9P) 구축·운용 도구와 상세 가이드: `tools/base_station/README.md`
(ROS 무의존 — 좌표 측량 → 베이스 설정 → RTCM TCP 배포 → 로버 주입 순서).

## 공통 규칙 (CLAUDE.md)

- 출력은 `fma_interfaces` 메시지로만. MGM은 이 토픽만 구독한다.
- 경로를 내는 스택은 전부 동일 ref points 포맷 — {x, y, yaw, curvature}, vehicle frame (§5.4).
- 판단 로직(모드 전환·정지 결정·우선권)은 MGM 스테이트 머신에만 존재한다 (§4, §5.1).
- 실행: `ros2 run stack_gps stack_gps_node`
