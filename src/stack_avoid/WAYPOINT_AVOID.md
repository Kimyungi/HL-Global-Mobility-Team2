> 2026-09-16 이 PC의 v2 통합 설정: `prepare/drive`에서 `waypoint_avoid=true`,
> `avoid_v2_enabled=false`로 이 알고리즘을 기본 사용합니다. 기존 회피 노드는 실행하지 않습니다.
> PR #105의 분리 CSV를 MGM 경로 전환에 맞춰 로드하고, 선택한 첫 CSV의 GPS 원점을 공유합니다.
> MGM AVOID 활성 확인 후 계산하며, 입력 취득시각 기반 reference_stamp를 유지합니다.
> 기존 revised v2의 정지 게이트에 맞춰 무효 경로는 points를 비워 전송합니다.
> 아래 PR 원문의 단일 통합 CSV와 기본 비활성 설명보다 이 설정이 우선합니다.
> 조향 곡률 한계 검사는 유지하며, 원 PR의 예제처럼 경로가 한계를 넘으면 정지합니다.

# Waypoint 고정 경로 회피

사용자 지정 네 제어점을 GPS ENU 좌표계에서 고정하고, 제어기에 **1 m preview
한 점**을 제공한다. 기존 follow-the-gap은 legacy 모드로 남아 있으며 통합 launch의
`waypoint_avoid:=true`가 새 producer와 MGM 설정을 함께 선택한다.

## 좌표 및 station

`stack_gps.path_engine.PathEngine`을 공유한다. 원점은 CSV에서 보존된 첫 lat/lon,
거리 단위는 m, x=east, y=north, yaw=ENU 반시계 라디안이다. 기존 GPS와 같이
최근접 꼭짓점을 찾고 인접 두 선분에 투영하여 수선의 발을 고른다. 그 선분의
누적 길이와 투영 비율로 station을 얻는다. CSV의 세션별 `east_m/north_m` 또는
별도 원점으로 다시 좌표계를 만들지 않는다. `s_m`은 직접 신뢰하지 않고 GPS
주행이 실제 사용하는 ENU polyline의 길이를 사용한다.

CSV의 `yaw_rad`(없으면 `yaw_deg`)는 각 행에서 읽고, 두 station 사이에서는
각도를 wrap하여 보간한다. GPS 주행도 CSV yaw가 있으면 같은 값을 사용한다.
yaw가 없는 구 CSV는 GPS의 기존 접선 추정이 유지되지만 **새 회피 모드는 시작을
거부한다**. yaw와 GPS 경로 방향이 일치하는 CSV가 필요하다.

경로 위치 `r(s)=(E,N)`, yaw `ψ(s)`일 때 횡방향은 `n(s)=(-sinψ,cosψ)`이다.
장애물 station을 `s₀`, 반대편 오프셋을 `d=−sign(d_obs)×1.0`으로 두면:

| 점 | station | 전역 위치 | yaw |
|---|---|---|---|
| (1) | s₀−2.5 | r(s₀−2.5) | ψ(s₀−2.5) |
| (2) | s₀ | r(s₀)+d n(s₀) | ψ(s₀) |
| (3) | s₀+0.7 | r(s₀+0.7)+d n(s₀+0.7) | ψ(s₀+0.7) |
| (4) | s₀+3.2 | r(s₀+3.2) | ψ(s₀+3.2) |

(3)은 (2)를 차량 전방이나 전역 x로 0.7 m 옮긴 점이 아니다. 해당 station의
웨이포인트와 yaw에서 다시 계산한다. 각 구간은 `x(s),y(s)` 3차 Hermite이고,
각 끝점의 station 미분을 `(cosψ,sinψ)`로 둬 위치와 yaw를 보장한다.
곡률 연속성은 보장하지 않는다. 경로 끝을 넘어가는 제어점을 임의로 잘라내지 않는다.

## 라이다 인지

- 기본 입력 `/unified_lidar/cloud` (`PointCloud2`, `base_link`)는 기존
  `lidar_fusion_v2`의 네 라이다 통합 출력이다. 다른 통합 cloud는 `cloud_topic`으로 지정한다.
- 통합 뒤 base_link 원점 기준 반경 3 m 필터 및 차체 반사 제거를 적용한다.
  센서별 외부 보정은 fusion에서 이미 수행되므로 mount offset을 중복 적용하지 않는다.
- GPS 노드의 `map→base_link` TF를 **cloud timestamp에서** 보간 조회하고 전역으로 변환한다.
  TF를 기다리는 cloud 큐를 유지하며, 시간 불일치 때 최신 TF로 억지 변환하지 않는다.
- 무순서 점군에 유클리드 연결 군집화를 적용한다. 기본 3점 이상, 연결거리 0.18 m,
  최대 군집 크기 1 m, 두 프레임 확인이다. station과 횡위치는 관측 표면 점들의
  중심에서 계산한다. 물체의 가려진 뒷면까지 추정하는 알고리즘은 아니다.
- 기본 감지 대역은 `abs(abs(d)−1.0)≤0.25 m`. `obstacle_offsets: [0.5, 1.0]`이면
  ±0.5 m도 포함된다. 같은 편/반대편 모두 실제 검출 부호로 결정한다.
- GPS RTK FIXED, 측정된 헤딩(COG 또는 융합), 신선한 TF/cloud가 필요하다.
  차량 헤딩을 경로 yaw로 대체해 점군을 회전시키지 않는다.

## 고정, 연결 및 preview

한번 검출 확정한 장애물과 경로는 후속 점군의 흔들림·소실로 재생성하지 않는다.
두 번째 장애물이 나타나면 첫 경로 (1)→(2)→(3)은 그대로 보존하고, 첫 (4)를
두 번째 장애물의 (2)로 바꾼다. 두 번째 (1)은 첫 (3)과 **같은 객체/좌표/yaw**다.
두 번째 (3)/(4)는 두 번째 장애물 station 기준으로 만든다. 첫 (3)을 통과하면
두 번째 기동이 active가 된다. 이미 첫 (3)을 지난 뒤 발견해도 허용된 (4)와
이후 연결 구간만 교체한다. 새 곡선에서 1 m 전방 참조점을 얻지 못하면 진행 불가다.
두 번째 station이 첫 (3)보다 앞이면 이 네 점 규칙으로 연결할 수 없어 거부한다.

최종 (4)의 station과 yaw에 수직인 통과선을 모두 지나면 고정 회피 경로를 해제하고,
웨이포인트 위의 1 m preview로 복귀한다. 상태 종료는 별도이며, 회피를 실제 수행한 후
고정 회피 경로가 없고 waypoint 투영점까지 거리 ≤0.10 m, 해당 station CSV yaw와
차량의 wrap한 yaw 오차 ≤20°를 모두 만족할 때만 `maneuver_done`을 낸다.
라이다 무감지, 시간 경과, 일시 정지로 회피 경로를 지우거나 상태를 종료하지 않는다.
장애물 두 개 사이에 기존 (4)를 먼저 지나면 첫 기동을 완료한 뒤 별도 기동을 시작한다.

GPS preview 선택 방식에 맞춰 **차량에서 유클리드 거리 1 m이고 전방인 곡선 위 점**을
선택한다. 곡선 샘플의 선분과 반경 1 m 원의 교점을 보간한다. 따라서 station+1 m와
동일한 뜻은 아니다. preview 이전/이후가 회피 곡선 범위를 벗어나면 원래 웨이포인트를
연결한다. 고정 전역 경로는 변하지 않고, 선택한 한 점만 현재 차량 좌표로 변환된다.

MGM `avoid_fixed_preview=true`에서는 기존 1→20점 축소, 진입 블렌드 및 시간초과
복귀를 적용하지 않는다. preview의 x/y/yaw/curvature를 그대로 전달한다. 이 모드는
`backend=core`를 사용하며 generated v1.88은 미지원이다. 후진 탈출도 통합 launch에서
끄며, 긴급 정지/TTC 바닥은 유지된다.

## CSV state=4 진입과 회피 상태 유지

사용자 제공 CSV의 `state=4`는 MGM 숫자 상태가 아니라 **AVOID 진입 마커**이다.
MGM의 AVOID 상태 번호는 기존 2를 유지한다. `behavior=PARALLEL_PARK`와 충돌하는
행도 사용자가 지정한 state=4를 우선하여 회피 마커로 읽으며, 다른 state 값의 의미는
이번 변경에서 자동 매핑하지 않는다.

선택된 경로 `1→3→4→5→6`은
`src/stack_gps/waypoints/reference_path_1_3_4_5_6_state.csv`에 저장했다.
원본은 reference_paths_01_to_07_split_parking.csv이며 state=4 행은 path_id=4,
idx=77이다. lat/lon과 yaw를 보존하며 연결 경계의 중복 좌표는 기존 loader가 제거한다.
marker도 같은 필터 결과에 정렬하며 중복점 state=4를 보존한다.

GPS는 마커의 최근접 인덱스 도달 또는 전진 통과 때 `GpsPath.avoid_zone`을 발행한다.
샘플 사이에 해당 행을 건너뛰어도 인지한다. fixed-preview MGM은 이 신호 또는
이를 래치한 새 회피 producer의 활성 신호로 LANE/WAYPOINT에서 AVOID로 진입한다.
신호등/주차의 기존 우선순위는 유지한다. 마커가 꺼져도 회피 세션은 유지한다.

- 장애물 감지 전: AVOID 상태에서 원래 waypoint의 1 m preview를 제공한다.
- 장애물 감지 후: 고정 회피 경로를 제공한다.
- 마지막 P4 통과 후: waypoint preview로 복귀하면서 10 cm/20° 조건을 기다린다.
- 완료: WAYPOINT로 전이한다. 같은 마커가 계속 켜져 있어도 즉시 재진입하지 않는다.
- 장애물이 한 번도 나타나지 않으면 자동 완료하지 않는다.
- 입력/preview 무효 또는 기하 검사 실패는 상태 탈출이 아니라 AVOID 안에서 정지한다.

`AvoidStatus.obstacle_detected`는 이 producer에서는 마커로 활성화된 회피 세션을
뜻한다. `avoidable`은 현재 참조점으로 주행 가능한지, `maneuver_done`은 위 복귀
조건을 실제 충족했는지를 뜻한다. legacy producer의 의미와 진입 조건은 유지된다.
완료 신호는 다음 마커까지 유지하고, GPS/TF/cloud 무효 시 또는 MGM stale watchdog에서
완료를 무효화한다.

## 실제 형상 제약 결과

기본 횡오프셋은 1.0 m, 진입/복귀 거리는 각각 2.5 m이다. 직선 단일 회피의
최대 곡률은 `6×1/2.5²=0.96 /m`, 최소 회전반경은 **1.042 m**로 설정된
차량 최소 회전반경 **1.15 m**를 넘는 조향을 요구한다. 장애물 station 간격이 3 m인
반대편 연결에서는 A3→B2의 길이가 2.3 m, 횡이동이 2 m이므로 이상적인 최대 곡률은
`12/2.3²=2.268 /m`이다. 진입/복귀 설정이 장애물 간 연결 거리를 늘리지는 않는다.
MPC 추종 성능은 이 기하 시험에서 검증하지 않으며 순간 최대 곡률도 조향 한계 검사에 포함한다.

**인지 거리:** 최초 확정 검출이 P1(s₀−2.5 m) 이후이면 첫 경로 생성을 거부한다.
갱신한 합성 3 m 라이다 사례에서는 P1 이전에 검출되어 첫 경로가 생성된다.
플롯의 두 장애물은 모두 라이다로 검출되며 사전 입력하지 않는다. 별도의 최초 검출
시점 시험을 JSON의 `live_first_detection`에 기록한다. 차량 자세는 경로 위에 이상적으로
배치하므로 경로 끝 통과 결과가 MPC 추종 성공이나 실차 주행 가능성을 의미하지 않는다.

이 경우에도 전역 경로와 플롯은 생성·보존하지만, `avoidable=false`, `v_suggest=0`,
`ttc=0`을 내며 이미 AVOID 상태라면 MGM이 정지시킨다. 아직 AVOID에 진입하지 않았다면
이 값 자체가 waypoint 모드를 중지시키는 것은 아니며 기존 estop이 별도로 동작한다.
실차 사용 전에는 지연 검출 처리와 연속 장애물 구간의 추종 가능성을 검증해야 한다.
`enforce_turn_radius=false`는 기하 시험용이며 벽/점군 충돌 검사는 계속 적용된다.
조향 제한 수치를 맞추려고 곡률 출력만 잘라내거나 제어점 위치를 바꾸지 않는다.

## 실행 및 검증

이 환경에서는 ROS 2 Humble/실차를 실행하지 않았다. Python 기하 테스트와
ROS에 의존하지 않는 C++ MGM 테스트를 수행했다. 합성 LiDAR 시험은 경로 위에
이상적으로 놓인 차량 자세를 사용하므로 추종 성능이나 제동 성능의 증명이 아니다.

```bash
# GPS와 동일 CSV 및 기존 map→base_link가 실행 중일 때 인지/경로만 확인
ros2 launch stack_avoid waypoint_avoid.launch.py waypoint_csv:=/absolute/route.csv

# 기존 통합 launch에서 새 모드 선택; 실제 운용의 기존 출발 인가 절차 유지
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  waypoint_csv:=/absolute/reference_path_1_3_4_5_6_state.csv waypoint_avoid:=true

# 네 라이다 모드의 출발 점검은 실제 전방 스캔 토픽으로 remap
ros2 run adas_mgm go --ros-args -r /scan:=/lidar/a1/scan

# ±0.5 m 추가 등은 waypoint_avoid.yaml 사본에서 변경 후 전달
# waypoint_avoid_params:=/absolute/custom_waypoint_avoid.yaml

python3 -m unittest discover -s src/stack_avoid/test -p test_waypoint_planner.py -v
python3 -m pytest src/stack_gps/test/test_path_engine.py src/stack_avoid/test/test_waypoint_planner.py
python3 src/stack_avoid/tools/plot_waypoint_avoid.py --output /tmp/waypoint_avoid_plots
colcon build --packages-select stack_gps stack_avoid adas_mgm
colcon test --packages-select stack_gps stack_avoid adas_mgm
```

RViz의 Fixed Frame은 `map`. `/perception/avoid_path_map`은 고정 곡선,
`/perception/avoid_controls_map`은 네 점들의 위치/yaw,
`/perception/avoid_preview_map`은 움직이는 1 m 참조점이다.
`/perception/avoid_diagnostic`에서 조향 한계, 경계 침범, TF/cloud 신선도 문제를 확인한다.
