# stack_parking — 4-LiDAR ICP 주차

기존 `multi_lidar_fusion`의 `/lidar/merged_cloud`를 수정 없이 구독하는 주차 전용
스택이다. `adas_mgm`, `fma_interfaces`, `multi_lidar_fusion`의 코드는 변경하지 않는다.

## 파이프라인

1. `GpsPath.parking_zone` 상승 에지 또는 문자열 명령에서 평행(1자)/직각(T자),
   좌/우를 결정한다.
2. `space_found=false`인 동안 기존 camera lane/GPS waypoint 모드가 직진하고,
   이 스택은 융합 endpoint cloud를 scan-to-local-map ICP로 누적한다.
3. 양쪽 정적 경계가 있는 gap을 3 frame 연속 확인한다. 직각 주차는 gap 내부 후면
   벽 지지점까지 요구한다.
4. 후축 기준 최소 회전반경 1.15m의 원호 경로를 만들고 차량 직사각 footprint로
   정적 충돌 검사를 한다. 계획이 통과한 뒤에만 `space_found=true`를 보낸다.
5. 원호 시작점이 앞이면 전진 접근 경로를 먼저 보낸다. 이미 지난 경우에는 scan
   lane을 직선 후진해 원호 시작점에 합류한다.
6. map-frame 경로에서 약 1m preview를 뽑아 현재 ICP 자세 기준 `base_link`의
   `{x,y,yaw,curvature}` 한 점으로 변환한다. 회전 중 `|v|=0.55m/s`, 마지막
   zero-curvature 도킹 구간만 `0.15m/s`다.
7. 후방 a2 scan의 보정 거리에서 서로 가까운 5개 이상 ray가 0.20m 이하가 되면
   정지한다. 계획 끝까지 갔는데 이 조건이 없으면 자동 완료하지 않고 0속도로 멈춘다.
8. 실제 속도가 정지한 뒤 5초 대기하고, 실제 후진해 온 경로를 역순/전진으로
   재생한다. 끝에서 `done=true`, `space_found=false`를 보내 MGM이 LANE으로 간다.

`path_blocked`는 계획 당시의 정적 벽·콘에는 쓰지 않는다. 계획 snapshot에 없던
새 점이 진행 경로 footprint를 2 frame 연속 침범할 때만 true다.

## 명령

GPS가 주차 형식을 결정할 수 있으면 `/parking/gps_command`에 다음 문자열을 보낸다.

```bash
ros2 topic pub --once /parking/gps_command std_msgs/msg/String \
  "{data: 'start perpendicular right'}"
ros2 topic pub --once /parking/gps_command std_msgs/msg/String \
  "{data: 'start parallel left'}"
```

현재 `GpsPath.msg`에는 `parking_zone` bool만 있고 형식 필드가 없다. 명시 문자열이
없으면 `parking_params.yaml`의 `gps_default_mode`/`gps_default_side`를 사용한다.

GPS 없는 단독 시험은 실제 `stack_gps`를 내린 뒤에만 실행한다. 이 launch는 기존
MGM의 `gps_parking_zone && space_found` 게이트를 통과시키기 위해 test-only
`GpsPath`를 발행한다.

```bash
ros2 launch stack_parking parking_standalone.launch.py start_multi_lidar:=true
ros2 topic pub --once /parking/manual_command std_msgs/msg/String \
  "{data: 'start perpendicular right'}"
# 또는 "start parallel left", "start T자 우측", "start 1자 좌측"
ros2 topic pub --once /parking/manual_command std_msgs/msg/String "{data: cancel}"
```

단독 launch와 실제 `stack_gps`를 함께 실행하면 `/perception/gps_path` publisher가
둘이 되므로 금지한다. SCANNING 중 차량 직진은 기존 lane source를 쓰거나 bench에서
수동 이동한다.

## 실행과 토픽

```bash
# multi_lidar_fusion이 이미 실행 중
ros2 launch stack_parking parking.launch.py

# 4개 driver + fusion도 함께 시작
ros2 launch stack_parking parking.launch.py start_multi_lidar:=true

ros2 topic echo /perception/parking
ros2 topic echo /parking/diagnostics
```

입력:

- `/lidar/merged_cloud` — ICP/endpoint map. 반드시 `multi_lidar_fusion` 출력.
- `/lidar/a2/scan` — 후방 20cm 완료 조건.
- `/vehicle/vector` — ICP initial guess와 정지 확인. 없어도 ICP는 돌지만 기본 안전
  설정에서는 5초 timer를 시작하지 않으므로 출차하지 않는다.

출력:

- `/perception/parking` — 기존 `ParkingStatus` 계약.
- `/parking/slam_pose`, `/parking/slam_scan` — SLAM 단계.
- `/parking/local_map`, `/parking/debug_markers` — mapping/space 단계.
- `/parking/reference_path`, `/parking/active_path` — map 위 경로 단계.
- `/parking/diagnostics` — ICP RMSE/match 수, rear clearance, state/progress.

## RViz 3단계

```bash
rviz2 -d $(ros2 pkg prefix stack_parking)/share/stack_parking/config/parking_1_slam.rviz
rviz2 -d $(ros2 pkg prefix stack_parking)/share/stack_parking/config/parking_2_mapping.rviz
rviz2 -d $(ros2 pkg prefix stack_parking)/share/stack_parking/config/parking_3_reference_path.rviz
```

1. SLAM: 현재 registered scan, 누적 map, ICP vehicle pose.
2. MAPPING: endpoint map, 검출 공간, 최소반경 시작점과 최종 pose.
3. reference path on map: 전체 진입 경로, 현재 gear segment, 1m preview.

## ROS 없는 회귀 시뮬레이션

```bash
PYTHONPATH=src/stack_parking python3 -m unittest discover -s src/stack_parking/test -v
PYTHONPATH=src/stack_parking python3 -m stack_parking.simulation \
  --runs 25 --seed 20260823 --noise 0.012 --dropout 0.08
```

시뮬레이션은 평행/직각 × 좌/우, 전진 staging, 후진, 20cm 정지, 5초 대기,
역경로 출차, 동적 침범 정지/복귀, 곡률 상한과 local preview 변환을 검사한다.

## 실차 전 필수 확인

- `MEASUREMENTS.md`에 rear yaw가 아직 미검증이라고 명시되어 있다. 뒤↔좌/우 pair
  calibration 후 `lidar_mounts.yaml`을 먼저 확정한다.
- `bridge_dspace/DSPACE_LOGGING.md`에는 `/vehicle/vector` 미수신 실측 기록이 있다.
  parking 중 100Hz 회신과 counter 증가를 확인한다.
- dSPACE watchdog 미구현 기록이 있으므로 종료 시 CAN zero guard 없이는 실차를
  띄우지 않는다.
- 실제 후진 조향에서 `v=-0.55m/s`의 부호/곡률 응답과 0.15m/s 직선 도킹 제동거리를
  차륜을 띄운 bench → 저속 폐쇄 구역 순서로 검증한다.
- 후방 raw scan의 normalized 중심이 -90도인지 판을 놓고 확인한다. 5개 지지 ray와
  +0.069m 거리 바이어스는 실제 벽면 bag으로 재검증한다.
