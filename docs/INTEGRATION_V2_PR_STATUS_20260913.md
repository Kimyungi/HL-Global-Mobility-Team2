# Integration v2 — 회피·E-stop·신호등 노출 통합 (2026-09-13)

이번 변경의 대상은 `integration/v2_main`이다. 기준은 PR #88 merge commit
`2852a5ce98912f54fe8da5b80c5d0b8c8f24b023`이며 기존 `main`을 변경하지 않는다.
PR #85의 주차 경로, #86의 한라대 GPS, #87의 v2 통합, #88의 정면 LiDAR 방향 수정은
이미 이 기준 브랜치에 포함돼 있다. 이전 이 문서의 main 대상/Draft 설명은 당시 검토 이력이다.

## 통합 동작

- 일반 주행은 `avoidance_enabled=true`, `lidar_estop_enabled=true`다.
  `scripts/v2 vehicle`이 일반 진입점이고 `vehicle-no-estop`은 E-stop 제외 시험용으로 남는다.
  회피와 E-stop은 정면 `/lidar/a1/scan`을 사용하며 PR #88의 장착 방향 해석을 유지한다.
- Mission ACTIVE에서는 GPS 탐색과 실제 주차 기동 전체에 일반 회피와 LiDAR E-stop을
  적용하지 않는다. 종료 틱부터 일반 주행 조건을 다시 평가한다. 운전자/CAN 정지는 유지한다.
- stable 주차 Zone entry에서 ACTIVE/PARKING이 되며 PREPARE 명령으로 모듈을 준비한다.
  ready 이전에는 현재 CSV의 GPS 1점과 v_base로 탐색한다. 이때 신호·GPS reference 정지는
  유지하며 Parking status 부재 자체로 정지하지 않는다. ready 틱에 Parking 제어로 인계하고
  ACTIVATE 실행 ack와 유효 reference가 올 때까지 정지한다.
- 주차 완료 또는 현재 CSV 종점에서 복귀한다. 03 종점까지 미준비이면 ROUTE_END=10으로
  실패를 기록하고 실제 정지와 04의 새 reference 응답 후 자동 주행한다. 추가 go는 필요 없다.
  TargetRef pose delta는 state byte가 아니라 실제 GPS/Parking reference 소스에 맞춘다.
- GPS station의 다음 탐색 반경은 사용자 지정 `abs(v_ref) * sample_time * 1.5 + 0.5m`다.
  유효한 0 속도 명령에서도 0.5m 안에서 갱신한다. 기존 단일 preview station +2.5m와
  90% endpoint snap은 유지한다. raw dump는 이 제어권 변경을 구별하기 위해 v20이다.
- 신호등 카메라는 `oak_exposure_compensation=-2`, 통합 launch는
  `traffic_exposure_compensation=-2`를 기본으로 전달한다. DepthAI 2/3의 RGB 자동 노출
  보정을 사용하며 정수 -9..9만 허용한다. 0은 보정 없는 자동 노출이다.
  startup-only 설정으로 재시작 시 적용하고 차선 카메라는 변경하지 않는다.
- 한라대/용인 런북의 시작 명령에 회피 ON과 신호등 노출 -2를 명시했다.
  기존 속도 고정 및 정지 제어, 신호 거리 seed 1.5m/목표 1.0m, Recovery OFF를 유지한다.

## 검증 결과

격리 checkout `/tmp/fma-v2-safety-exposure-merge`에서 실행했다.
MGM은 별도 `/tmp/fma-v2-safety-exposure-build`에 CMake build했으며
ROS Humble과 기존 v2 메시지/의존성 underlay를 사용했다. 이번 변경에는 메시지 schema 변경이 없다.
Python/launch 검사는 후보 소스와 후보 adas_mgm 설치 prefix를 사용했다.

| 검사 | 결과 |
|---|---|
| 후보 MGM 및 C++ 시험 대상 build | 성공 |
| 관련 Python | **432 passed, 3 skipped** |
| `parking_entry_test` | **134 checks, 0 failures** |
| `parking_entry_ros_smoke.py` | **13 PASS**, localhost domain 178 |
| `parking_search_route_ros_smoke.py` | 업로드된 01→03, 03 탐색 실패→04 자동 주행 **PASS**, domain 179 |
| 후보 전체 CTest | **11/20 통과, 9개 실패** |
| 기준 `2852a5c` 별도 build/CTest | **11/20 통과, 후보와 동일한 실패 목록·assertion** |

기존 실패는 `fixed_speed_test`, `avoidance_disabled_test`, `route_sequence_test`,
`stabilization_test`, `reference_safety_test`, `zone_manager_test`,
`mission_preparation_test`, `mission_zone_search_test`, `manager_state_test`다.
두 build의 실패 assertion 목록 242줄과 각 실패 수가 동일하다. 따라서 전체 회귀 통과를
주장하지 않는다. 공통 fixture의 과거 2점 입력/속도 기대값 등이 남아 있으나, 모든 실패가
fixture 변경만으로 해결된다고 검증한 상태는 아니다.

Python 대상은 `stack_gps/test`, `stack_traffic/test`, `stack_estop/test`,
`stack_avoid/test`, `stack_parking/test`, MGM `test_reference_metadata.py`,
`scripts/test_v2_launch.py`, `scripts/test_v2_front_lidar.py`다.
런처 시험은 노드 실행 없이 일반/시험용 E-stop 분리, 회피 기본 ON/명시 OFF,
노출 -2/0/-3 전달과 차선 카메라 분리를 검사한다.

ROS 시험은 실제 MGM 실행 파일에 모의 입력을 연결한다. 두 번째 시험은 실제 GPS wrapper와
업로드된 한라대 CSV/Zone을 사용하며 GNSS/차속만 합성한다. 센서·CAN bridge를 시작하지 않았다.
작업 폴더의 기존 주행 파일과 설치본은 덮어쓰지 않았으며 별도 checkout으로 PR을 준비했다.

## 남은 현장 확인

이번 시험은 장애물·정지선 검출 성공률, 실제 제동거리, 주차 공간 생성 성공률을 인증하지 않는다.
GPS heading이 반대로 초기화되는 현상은 이번 변경에서 수정하지 않았다.
실차 rosbag/학습용 사진, 통합 RViz의 별도 작업 파일은 이번 변경에 추가하지 않는다.
과거 커밋된 9월 12일 CSV·메타데이터와 로컬 보관 원본은 보존한다.
