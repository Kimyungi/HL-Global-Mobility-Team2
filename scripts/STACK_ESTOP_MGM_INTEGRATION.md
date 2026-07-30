# Stack E-Stop → MGM 소프트웨어 통합 시험

실제 차량, LiDAR, bridge_dspace, UDP, CAN, dSPACE 및 모터를 실행하지 않는다.

## 터미널 1 — MGM

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch adas_mgm mgm.launch.py
```

`immediate_stop`까지 확인하려면 launch 대신 다음을 사용한다.

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run adas_mgm mgm_node --ros-args \
  --params-file src/adas_mgm/config/params.yaml \
  -p snapshot_dump_path:=/tmp/stack_estop_mgm_snapshots.bin
```

## 터미널 2 — Stack E-Stop

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run stack_estop stack_estop_node
```

## 터미널 3 — MGM 최소 시험 입력

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/test_mgm_inputs.py
```

## 터미널 4 — Rosbag 기록

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p /home/chanmi/ydlidar_tmini_logs/official_stack_estop

ros2 bag record \
  --storage sqlite3 \
  -o /home/chanmi/ydlidar_tmini_logs/official_stack_estop/stack_estop_mgm_integration_01 \
  /scan \
  /perception/lane_path \
  /perception/estop \
  /adas/target_ref
```

## 터미널 5 — 기존 `/scan`만 재생

```bash
source /opt/ros/humble/setup.bash
ros2 bag play \
  /home/chanmi/ydlidar_tmini_logs/motion_bags/final_distance_estop_test \
  --topics /scan
```

재생 종료 후 최소 0.5초 동안 기록을 계속해 scan timeout 결과를 남기고,
그다음 기록 터미널에서 `Ctrl+C`를 누른다.

## 결과 확인

```bash
source /opt/ros/humble/setup.bash
ros2 bag info \
  /home/chanmi/ydlidar_tmini_logs/official_stack_estop/stack_estop_mgm_integration_01
```

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/analyze_estop_mgm_latency.py \
  /home/chanmi/ydlidar_tmini_logs/official_stack_estop/stack_estop_mgm_integration_01
```

Snapshot dump를 사용했다면 MGM 종료 후 다음으로 `immediate_stop`을 확인한다.

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run adas_mgm core_replay \
  /tmp/stack_estop_mgm_snapshots.bin \
  /tmp/stack_estop_mgm_core.csv
grep ',1,0' /tmp/stack_estop_mgm_core.csv
```

## 안전 이슈 시험 1 — MGM 먼저 실행

터미널 1에서 MGM만 먼저 실행하고 다음을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /home/chanmi/HL-Global-Mobility-Team2-1/install/setup.bash
ros2 topic echo /adas/target_ref
```

그 뒤 터미널 2에서 Stack E-Stop을 실행한다. 첫 EstopRequest 전에 MGM이
양수 `v_ref`를 내면 초기 E-Stop 수신 fail-safe가 없는 것이다.

## 안전 이슈 시험 2 — Stack E-Stop 종료

통합 시험 중 `/perception/estop=false`와 양수 `v_ref`를 확인한 뒤
Stack E-Stop 터미널에서 `Ctrl+C`를 누르고 다음을 관찰한다.

```bash
source /opt/ros/humble/setup.bash
source /home/chanmi/HL-Global-Mobility-Team2-1/install/setup.bash
ros2 topic echo /adas/target_ref
```

MGM이 계속 양수 `v_ref`를 내면 EstopRequest stale timeout이 없는 것이다.
