# stack_gps — 요구사항

**담당: 김윤기 (팀장)** · 산출물: GPS 단독 주행 (8/2) — 베이스 설치·RTK 포함

## 역할

GPS·IMU 융합, RTK, waypoint ref

## 계약 (이것만 지키면 나머지는 자유)

- 입력: GPS(수백 ms) + IMU(~10ms 비주기) 융합 → 위치·헤딩. RTK 베이스 설치 포함.
- 출력: `/perception/gps_path` (`fma_interfaces/GpsPath`). 주차 localization 보조
  출력 `/perception/imu` (`sensor_msgs/Imu`)는 자이로 적분 상대 yaw와 z 각속도만
  10Hz로 싣는다. yaw의 0점은 임의이므로 절대 ENU heading으로 쓰지 않는다.
  - `points[]`: 전역 waypoint를 **vehicle frame으로 변환 완료한** ref points. **점 개수: 1개** (팀 합의 2026-07-29) — 추종 목표 waypoint 하나만.
  - `accel_zone` / `parking_zone`: 구간 플래그 — 속도를 올리는 판단은 MGM 우선권 표가 한다.
  - 트랙 옆 `zones_<이름>.yaml`의 `parking_points`는 위경도와
    `mode: perpendicular|parallel`을 보존한다. 기동 시 현재 트랙의 짧은 인덱스
    구간으로 변환하여 `parking_zone`/`parking_mode`만 발행하며, 주차 진입 판단과
    기동은 MGM/`stack_parking`이 담당한다.
  - 한라대 기준경로 CSV의 `state` 코드는 0=일반, 1=T자, 2=평행,
    3=신호 예상 지점이다. 1·2는 YAML 주차점과 동기화하고, 3은 코스
    메타데이터로만 쓴다. 신호 정지 판단은 `stack_traffic`과 MGM이 담당한다.
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
- 실행: `ros2 run stack_gps stack_gps_node --ros-args -p waypoint_csv:=<기록 CSV> -p rtcm_host:=100.70.198.29`
  (전체 파라미터는 `stack_gps/node.py` 도크스트링 참조 — `rtcm_host` 생략 시 RTCM 주입 없이 수신만)
