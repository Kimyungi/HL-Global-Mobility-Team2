# 통합 실차 운영 런북 — 2026-09-04

## 차선 + GPS + 회피 + 긴급정지 + 신호등·정지선 + 주차

이 문서는 위 여섯 기능을 **한 번의 통합 launch**로 실행하는 현장 절차다. ROS 2나
현재 로직을 모르는 사람도 위에서부터 명령 블록을 순서대로 복사·붙여넣기 할 수 있게
작성했다.

**통합 launch:** `adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py`

> 실차 CAN 송신, 전진 및 후진이 포함된다. 운전자 1명은 항상 차량의 물리
> 비상정지 스위치에 손을 두고, 감시자 1명은 차량 주변을 통제한다. `go` 실행 전까지
> 물리 비상정지를 해제하지 않는다.

> 이 런북의 신호등 로직은 2026-09-04 `main` 기준이다. 구버전의
> `FINAL_STOP`, CLAHE/Canny 정지선, `traffic_stop_y_ratio` 단독 정지 설명을
> 사용하지 않는다. 현재 MGM의 신호등 진입 조건은
> **`red_active && stopline_detected`**이며, 정지선은 YOLO segmentation 결과다.

> 주차 구간은 코스마다 다르다. 아래 예시 `T_ZONE="[120,140]"`,
> `PARALLEL_ZONE="[260,285]"`는 실제 코스에서 검증된 인덱스로 바꾼 뒤 사용한다.
> 구간을 모르면 주차 통합 주행을 시작하지 않는다.

---

## 0. 동작 범위와 원리

| 기능 | 입력 노드/센서 | MGM 동작 |
|---|---|---|
| 차선 | 차선용 OAK-D → `/perception/lane_path` | `LANE` 경로 추종 |
| GPS | F9P+IMU → `/perception/gps_path` | `WAYPOINT` 경로 추종 |
| 회피 | 전방 a1 LiDAR → `/perception/avoid` | `AVOID` 경로 추종 |
| 긴급정지 | 전방 a1 LiDAR → `/perception/estop` | 모든 상태에서 즉시 `v_ref=0` |
| 신호등·정지선 | 교통용 OAK-D → `/perception/traffic_stop` | 적색+안정 정지선에서 `TRAFFIC` 정지 |
| 주차 | 4-LiDAR+ICP → `/perception/parking` | 지정 GPS 구간에서 `PARKING` 진입·출차 |

`MGM`만 최종 목표 `/adas/target_ref`를 만들고 `bridge_dspace`만 이를 CAN으로
보낸다. 각 인지 노드는 CAN 목표를 직접 만들지 않는다.

신호등은 적색 3/5가 확정된 뒤 정지선 segmentation이 안정적으로 검출될 때
`TRAFFIC`으로 진입한다. 정지선이 화면 아래로 사라지면 MGM이 남은 거리를 시드하고
`/vehicle/vector.v`를 적분해 감속을 계속한다. 같은 신호등 영역의 초록 3/5가
확정되면 진입 전 `LANE` 또는 `WAYPOINT`로 돌아간다. 카메라·노드 고장은 fail-safe
정지한다.

주차는 GPS 지정 구간에서 공간 탐색을 시작한다. `parking_zone=true`와
`space_found=true`가 함께 성립해야 `PARKING`으로 진입한다. 주차 중에는 회피 및
신호등 상태로 전이하지 않지만 긴급정지는 항상 우선한다. 주차 완료 후 5초 정지하고
들어온 경로를 전진으로 되돌아 나온 뒤 주차 진입 전 상태로 복귀한다.

---

## 1. 최초 1회 또는 코드 변경 후 빌드

차량 PC의 새 터미널에서 실행한다. 표준 워크스페이스는 `$HOME/FMA_ws`다.

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/FMA_ws"

# 최신 신호등 계약과 주차 계약이 모두 있는지 확인한다.
grep -q 'bool red_active' src/fma_interfaces/msg/TrafficStop.msg || { echo "red_active 계약 없음"; exit 1; }
grep -q 'bool stopline_detected' src/fma_interfaces/msg/TrafficStop.msg || { echo "stopline_detected 계약 없음"; exit 1; }
grep -q 'uint8 parking_mode' src/fma_interfaces/msg/GpsPath.msg || { echo "parking_mode 계약 없음"; exit 1; }
grep -q 'float64 dx' src/fma_interfaces/msg/ParkingStatus.msg || { echo "주차 pose delta 계약 없음"; exit 1; }

colcon build
source "$HOME/FMA_ws/install/setup.bash"
```

네 `grep` 중 하나라도 실패하거나 `colcon build`가 실패하면 현재 checkout에 신호등
`main` 변경과 주차 변경이 함께 들어 있지 않은 것이다. 실차를 실행하지 말고 통합
브랜치를 먼저 갱신한다. 이 워크스페이스의 실차 표준 빌드는 옵션 없는
`colcon build`다. 기존 build/install과 `--symlink-install` 방식을 섞지 않는다.

빌드 결과의 통합 인자를 확인한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py --show-args | \
  grep -E 'parking_enabled|t_parking_zone_ranges|parallel_parking_zone_ranges|traffic_enabled|traffic_depth_enabled|traffic_yolo'
```

주차 3종과 traffic 관련 인자가 모두 보여야 한다.

---

## 2. 중복 실행 금지

통합 launch가 필요한 드라이버와 인지 노드를 모두 띄운다. 아래 항목을 별도
터미널에서 동시에 실행하지 않는다.

- `stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py`
- `stack_avoid`의 `field_session` 계열 launch
- `adas_mgm/launch/MBD_lane_gps_can.launch.py`
- `stack_parking/launch/parking.launch.py`
- `stack_parking/launch/parking_standalone.launch.py`
- `multi_lidar_fusion/launch/multi_lidar_drivers.launch.py`
- `bridge_dspace/bridge.launch.py`
- `dummy_ref_publisher`

특히 전방 a1 LiDAR를 단일 `/scan` 드라이버와 4-LiDAR 드라이버가 동시에 열면 포트가
충돌한다. 이 런북에서는 `parking_enabled:=true`가 4개 드라이버와 융합, 주차 노드를
함께 시작하며 회피와 긴급정지는 전방 a1 scan을 공유한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 node list
```

이전 시험 노드가 보이면 그 노드를 실행한 터미널에서 `Ctrl-C`로 정상 종료한다.
`kill -9`는 사용하지 않는다.

---

## 3. 차량과 센서 준비

1. 차량을 평탄한 폐쇄 시험 구역에 두고 물리 비상정지를 누른다.
2. 차량 전·후·좌·우를 비우고, 후진과 출차 공간까지 통제한다.
3. 두 OAK-D는 USB2(`HIGH`) 포트에 연결한다. USB3는 GNSS L1을 방해할 수 있다.
4. LiDAR 4대, IMU, 차량 F9P, Kvaser CAN, 라디오를 연결한다.

```text
차선 OAK-D   14442C105157D3D200
신호등 OAK-D 14442C10B167CFD200
```

장착 위치·각도, 카메라 또는 정지선이 바뀌었다면 먼저
`RUNBOOK_full_measurement_20260830.md`로 신호등과 정지선 검출을 다시 확인한다.

---

## 4. 차량 PC 사전점검 (터미널 V0)

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"

ros2 run stack_traffic stack_traffic_ml_preflight --require-xpu
"$HOME/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh" --check
python3 "$HOME/FMA_ws/src/multi_lidar_fusion/tools/check_sensors.py" --no-ros
```

반드시 `ML_RUNTIME_READY`, CAN의 `✔ 점검 통과`와 MTU 72, 센서의
`== 전 항목 통과 ==`를 확인한다. 하나라도 실패하면 이후 명령을 실행하지 않는다.

---

## 5. 코스와 주차 구간 설정 (터미널 V0)

처음 방문한 지역에서 CSV를 아직 만들지 않았다면 먼저
`$HOME/FMA_ws/src/stack_gps/tools/waypoints/README.md`를 위에서부터 수행해 베이스
측량, 웨이포인트 기록, 도보 검증과 주차 구간 인덱스 확정을 끝낸다.

아래는 한라대학교 CSV와 **예시** 주차 인덱스다. 실제 코스에서 검증한 구간으로
숫자를 수정한다. 한 종류의 주차를 하지 않으면 그 값을 `"[0]"`으로 둔다. 두 구간이
겹치면 GPS 노드가 안전하게 기동을 거부한다.

```bash
export FMA_COURSE="$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv"
export FMA_T_ZONE="[120,140]"
export FMA_PARALLEL_ZONE="[260,285]"

test -r "$FMA_COURSE"
python3 -c 'import ast,os; p=ast.literal_eval(os.environ["FMA_T_ZONE"]); q=ast.literal_eval(os.environ["FMA_PARALLEL_ZONE"]); assert p==[0] or len(p)%2==0; assert q==[0] or len(q)%2==0; assert all(isinstance(x,int) and x>=0 for x in p+q); print("course:",os.environ["FMA_COURSE"],"T:",p,"parallel:",q)'
```

원주 코스를 쓸 때는 첫 줄을 다음으로 바꾸고 주차 인덱스도 원주 코스에 맞춘다.

```bash
export FMA_COURSE="$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_wonju_license_20260818_160511.csv"
```

---

## 6. 베이스와 RTCM 시작

### B1 — 베이스 PC의 새 터미널

```bash
source /opt/ros/humble/setup.bash
cd "$HOME/FMA_ws/src/stack_gps/tools/base_station"
python3 rtcm_server.py --radio /dev/ttyRadio
```

### V1 — 차량 PC의 새 터미널

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
python3 "$HOME/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py" \
  --port /dev/ttyRadio --tcp-port 2101
```

V1에 약 10초마다 `RTCM`과 0보다 큰 `B/s`가 보여야 한다. RTK가 안정화되도록
5~10분 기다리면서 다음 단계를 진행한다.

---

## 7. 통합 launch 시작 (차량 PC 터미널 V2)

물리 비상정지가 눌렸는지 다시 확인한다. 아래 구간은 5단계에서 확정한 값으로
바꾼다. 이 명령부터 CAN TX가 활성화되지만 MGM은 `go` 전까지 0속도를 낸다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"

FMA_COURSE="$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv"
FMA_T_ZONE="[120,140]"
FMA_PARALLEL_ZONE="[260,285]"

test -r "$FMA_COURSE" || { echo "코스 CSV 없음"; exit 1; }
test "$FMA_T_ZONE" != "[0]" -o "$FMA_PARALLEL_ZONE" != "[0]" || \
  { echo "주차 구간이 하나도 설정되지 않음"; exit 1; }

ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  waypoint_csv:="$FMA_COURSE" \
  parking_enabled:=true \
  t_parking_zone_ranges:="$FMA_T_ZONE" \
  parallel_parking_zone_ranges:="$FMA_PARALLEL_ZONE" \
  traffic_enabled:=true \
  traffic_depth_enabled:=false \
  traffic_yolo_image_size:=320 \
  traffic_yolo_inference_interval:=2 \
  traffic_red_phase_yolo_inference_interval:=3 \
  traffic_stopline_yolo_image_size:=320
```

통합 주행은 traffic RGB-only다. `stop_distance` depth는 진단값일 뿐 MGM의 진입
조건이 아니므로 `traffic_depth_enabled:=false`를 유지한다. 구버전의
`traffic_stop_y_ratio.txt`와 `traffic_require_stop_gate`는 현재 메인 계약의 출발
필수값이 아니다. V2는 주행 내내 종료하지 않는다.

---

## 8. 출발 전 노드·토픽 확인 (차량 PC 터미널 V3)

launch 후 최소 20초 기다린 다음 실행한다. 각 `timeout`은 자동 종료된다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"

ros2 node list | sort | grep -E 'stack_lane|stack_gps|stack_avoid|stack_estop|stack_traffic|stack_parking|mgm|can_bridge|multi_lidar|fusion'

timeout 12 ros2 topic hz /perception/lane_path
timeout 12 ros2 topic hz /perception/gps_path
timeout 12 ros2 topic hz /perception/avoid
timeout 12 ros2 topic hz /perception/estop
timeout 12 ros2 topic hz /perception/traffic_stop
timeout 12 ros2 topic hz /perception/parking
timeout 12 ros2 topic hz /lidar/a1/scan
timeout 12 ros2 topic hz /lidar/a2/scan
timeout 12 ros2 topic hz /unified_lidar/scan
timeout 12 ros2 topic hz /parking/slam_pose
timeout 12 ros2 topic hz /adas/target_ref
timeout 12 ros2 topic hz /vehicle/vector
```

인지와 scan 계열은 설정에 따라 대체로 5~10 Hz이며 끊기지 않아야 한다.
`/adas/target_ref`와 `/vehicle/vector`는 약 100 Hz여야 한다. 토픽 하나라도
`does not appear to be published`면 출발하지 않는다. `/stack_traffic_node`는 첫
OAK-D 경쟁으로 종료돼도 2초 뒤 respawn하지만 계속 사라졌다 나타나면 출발 금지다.

---

## 9. GPS FIXED와 주차 플래그 확인 (차량 PC 터미널 V4)

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 topic echo /perception/gps_path --field fix_quality | \
while read -r quality; do
  case "$quality" in
    4) echo "$(date '+%H:%M:%S') FIXED (4) ✅" ;;
    5) echo "$(date '+%H:%M:%S') FLOAT (5) — 출발 금지" ;;
    1) echo "$(date '+%H:%M:%S') GPS (1) — 출발 금지" ;;
    0) echo "$(date '+%H:%M:%S') NO FIX (0) — 출발 금지" ;;
  esac
done
```

`FIXED (4)`가 연속으로 나오면 `Ctrl-C`로 끝낸 뒤 주차 구간 밖 출발 위치에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
timeout 3 ros2 topic echo /perception/gps_path --once
```

출발 위치는 `parking_zone: false`, `parking_mode: 0`이어야 한다. 주행 중 T자 구간은
`parking_mode: 1`, 평행 구간은 `parking_mode: 2`가 된다.

---

## 10. 정지 상태에서 신호등·정지선 확인 (차량 PC 터미널 V4)

물리 비상정지를 유지한다. 실제 적색과 정지선을 카메라에 보인 뒤 실행한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 topic echo /perception/traffic_stop
```

적색 3/5와 안정 정지선 3/5가 성립하면 다음을 확인한다.

```text
red_active: true
green_active: false
stopline_detected: true
fail_safe_stop: false
```

같은 신호등을 초록으로 바꾼 뒤 다음을 확인한다.

```text
red_active: false
green_active: true
fail_safe_stop: false
```

확인 후 echo만 `Ctrl-C`로 끝낸다. V2는 종료하지 않는다. V2 로그의 `red_phase`,
`stopline`, `stable`, `red_votes`, `green_votes`, `proc_ms`도 확인한다. 적색에서
정지선이 끝까지 false, 초록에서 red가 해제되지 않음, `fail_safe_stop: true`, 노드
무한 respawn 중 하나라도 있으면 출발하지 않는다. `stop_required`는 구버전 호환 및
fail-safe 필드이며 현재 MGM의 정상 신호등 전이는 위 세 상태 필드로 판정한다.

---

## 11. 긴급정지·주차 중립과 최종 0속도 확인 (차량 PC 터미널 V4)

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
timeout 3 ros2 topic echo /perception/estop --once
timeout 3 ros2 topic echo /perception/parking --once
timeout 3 ros2 topic echo /adas/target_ref --once
```

장애물이 없으면 estop 요청은 false여야 한다. 주차 구간 밖에서는
`space_found: false`, `done: false`가 정상이다. `go` 전 목표 `v_ref`는 0이어야 한다.

---

## 12. 출발 인가 (차량 PC 터미널 V5)

운전자와 감시자가 준비됐을 때만 물리 비상정지를 해제하고 즉시 실행한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 run adas_mgm go --require-traffic
```

모든 항목이 `[OK]`이고 `출발 인가 발행 완료`가 나와야 출발한다. `--force`,
`--skip-lane`, `--skip-avoid`로 오류를 우회하지 않는다.

---

## 13. 주행 중 판정

- 차선 신뢰 구간은 `LANE`, GPS 전용·재합류 구간은 `WAYPOINT`가 된다.
- 회피 조건이면 `AVOID`로 전이하고 완료 뒤 기존 경로로 복귀한다.
- 긴급정지는 LANE/WAYPOINT/AVOID/TRAFFIC/PARKING 어디서나 최우선이다.
- 적색과 안정 정지선이 동시에 참일 때만 `TRAFFIC`으로 들어간다. 횡방향은 진입
  전 LANE/GPS 경로를 유지하고 초록 3/5에서 그 상태로 복귀한다.
- traffic 토픽 0.5초 stale 또는 TRAFFIC 중 vehicle/vector 0.2초 stale이면 정지한다.
- 주차 구간에서도 공간 검출 전에는 기존 LANE/WAYPOINT로 탐색한다.
- `space_found=true`면 `PARKING`으로 바뀐다. `v_suggest<0`은 후진이다.
- 새 장애물이 경로를 침범해 `path_blocked=true`면 정지한다.
- 후방 a2에서 보정 거리 0.20m 이하 ray가 5개 이상이면 정지한다.
- 최종 정지 뒤 5초 대기하고 역경로로 출차한다. `done=true` 뒤 진입 전 상태로
  복귀해야 정상이다.

상태와 주차 진단은 각각 새 터미널에서 볼 수 있다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 run adas_mgm state
```

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 topic echo /parking/diagnostics
```

---

## 14. 위험 시 정지와 정상 종료

위험 시 운전자가 즉시 **물리 비상정지**를 누른다. 차량이 완전히 정지한 뒤 V2에서
`Ctrl-C`를 한 번 눌러 통합 launch를 정상 종료한다.

정상 종료도 다음 순서를 지킨다.

1. 안전한 곳에서 차량을 정지시키고 물리 비상정지를 누른다.
2. V2에서 `Ctrl-C`를 한 번 눌러 `can_zero` 종료 가드가 실행되게 둔다.
3. 차량이 완전히 멈춘 뒤 V1, 마지막으로 B1에서 `Ctrl-C`를 누른다.

`kill -9`, PC 전원 차단, CAN 케이블 분리를 정지 수단으로 사용하지 않는다. 정상
종료를 건너뛰면 마지막 CAN 속도 명령이 유지될 수 있다. Kvaser가 순간 재열거되면
브리지는 자동 재연결하지만 반복 분리는 케이블·커넥터 결함이므로 다음 출발 전에
수리한다.

---

## 15. 시험 후 로그 확인

V2 시작 로그에 출력된 실제 run 디렉터리 이름으로 `<시각>`을 바꾼다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
RUN="$HOME/FMA_ws/drive_logs/run_<시각>"
test -d "$RUN" || { echo "RUN 경로를 다시 확인"; exit 1; }
ros2 bag info "$RUN/rosbag"
ros2 run adas_mgm core_replay "$RUN/mgm_snapshots.bin" "$RUN/replay.csv"
column -s, -t "$RUN/transitions.csv" | cut -c1-180
```

신호등 로그 재생은 터미널 A에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
RUN="$HOME/FMA_ws/drive_logs/run_<시각>"
ros2 bag play "$RUN/rosbag" --topics /rosout /perception/traffic_stop /adas/target_ref
```

동시에 터미널 B에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 topic echo /rosout --field msg | \
  grep --line-buffered -E 'red_phase|stopline|red_votes|green_votes|proc_ms|TRAFFIC'
```

적색+정지선과 TRAFFIC 진입, 최종 `v_ref=0`, 초록 복귀, 주차 구간·공간 검출·
PARKING·후진·정지·5초 대기·출차·`done`, 회피 전이/복귀와 긴급정지를 확인한다.

---

## 참조 문서

- `RUNBOOK_full_measurement_20260830.md` — 신호등·정지선 측정
- `RUNBOOK_lane_gps.md` — 베이스·RTCM·GPS·차선 진단
- `stack_gps/tools/waypoints/README.md` — 처음 가는 지역의 베이스 측량·웨이포인트 기록
- `RUNBOOK_avoid_field_test.md` — 회피 판정과 구간 설정
- `stack_parking/README.md`, `stack_parking/MEASUREMENTS.md` — 주차 로직과 실측 상태
- `stack_traffic/REQUIREMENTS.md` — 현재 메인 신호등 계약
- `bridge_dspace/CAN_BRINGUP.md` — CAN 단계별 검증
- `HANDOVER.md` — 새 PC 설치, udev, USB 배치
