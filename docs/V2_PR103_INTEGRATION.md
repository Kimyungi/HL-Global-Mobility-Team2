# 09.16 새 장애물 회피 알고리즘 적용

이 PC의 `integration/v2_main` 로컬 코드 및 `install_v2`에 적용했다.

- PR #103의 waypoint_planner를 현재 회피 경로 생성기로 선택했다.
  횡이동 1m, 진입/복귀 각각 2.5m, 유지 0.7m, 1m preview 및 2프레임 장애물 확인을 사용한다.
- `prepare/drive`: waypoint_avoid=true, avoid_v2_enabled=false. 기존 New_Avoid_v2와
  legacy 회피 producer는 실행하지 않는다. 소스는 과거 재현용으로 보존한다.
- 09.16의 상위 회피 진입/완료/GPS_RETURN 정책, PR #106 주차,
  PR #105 경로 01 → 03 → 04 → 05 → 07 및 zone [3] Traffic/GPS 정책은 유지한다.
- 원 PR의 단일 CSV 가정을 경로별 CSV 전환에 맞췄다. 각 회피 경로는 선택한 첫 CSV의
  GPS 공통 원점으로 변환한다. 일반 GPS의 기존 2.5m preview는 변경하지 않는다.
- MGM AVOID 활성 확인 후 새 회피 경로를 계산한다. 입력 취득시각을 reference_stamp에
  반영하며, 타이머 반복 발행으로 신선도를 갱신하지 않는다.
- 새 v2는 과거 TTC 정지 요청을 사용하지 않으므로, 무효 경로·곡률·입력은 points를
  비워 전달한다. MGM은 AVOID를 유지하면서 정지하고 유효 입력 복구 후 재개한다.
- 회피 제어점은 1m 길이를 유지하며 기존 축소/전환 혼합을 적용하지 않는다.
- dump v33을 유지한다. 이번 교체는 입력을 만드는 모듈 변경이며, 기록되는 코어
  스냅샷 구조와 전이 의미는 바꾸지 않았다. avoid_unblended는 이미 기록되는 파라미터다.
  세션의 avoid_planner_mode.txt 및 avoid_compute_backend.txt로 선택 모듈을 구분한다.

## 검증과 남은 한계

14개 패키지 빌드, 관련 Python 260개, C++ 29개 통과.
가상 ROS 입력으로 새 목표점의 MGM 전달, 무효 경로 정지, 복구 후 진행,
완료 후 GPS_RETURN 전환을 확인했다. 단일 회피 producer 실행 조건도 검증했다.
실제 센서·CAN·차량·MPC 폐루프 주행은 실행하지 않았다.

원 PR의 조향 곡률 검사(enforce_turn_radius=true, 최소 반경 1.15m)는 유지한다.
PR 예제처럼 고정 경로가 이 한계를 넘으면 정지하며, 이 통합이 실제 추종 가능성을
증명한 것은 아니다. 가동을 위해 해당 검사를 임의 해제하지 않았다.


## 후속 요청: 기존 회피 실행 경로 제외

- prepare/drive의 legacy 플래너 선택/복귀 분기를 제거했다.
- waypoint_avoid=false, avoid_v2_enabled=true 및 avoid_planner_mode/avoid_compute_backend
  옵션은 센서·GPS·CAN 시작 전에 오류로 중단한다.
- revised v2 실행 그래프에서 구형 회피 노드를 제거하고 구형 회피 YAML 로딩도 제외했다.
- scripts/v2 vehicle 및 vehicle-no-estop, 두 구형 v2 ROS 런처,
  avoid-drive-test / avoid-zone-drive-test / avoid-lidar-lab 실행을 차단했다.
- 기존 알고리즘 소스는 과거 기록 비교용으로 남겨두며 현재 v2 실행 경로에서는 사용하지 않는다.
- 후속 검증: 런처·구형 진입 차단 테스트 31개 통과. 센서·CAN은 실행하지 않았다.
