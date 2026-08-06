# 동적 장애물 3m Latch E-Stop 실차 시험

> **안전 경고**
>
> - 실제 CAN 송신 및 차량 구동이 가능합니다.
> - 처음에는 차량 바퀴를 띄운 상태에서 시험합니다.
> - 물리 비상정지를 즉시 사용할 수 있게 준비합니다.
> - `SCAN_TIMEOUT` 또는 `v_ref=0`인데 바퀴가 돌면 즉시 중단합니다.

현재 설정:

- LiDAR: YDLIDAR T-mini Plus
- baudrate: 230400
- lidar_type: 1
- intensity_bit: 8
- dynamic tracking max: 3.0m
- dynamic stop: 1.2m
- static hard stop: 0.7m
- CAN: can0, 1 Mbps

# 1. LiDAR 포트 확인

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1

ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
lsusb | grep -i -E "ydlidar|serial|uart|cp210|ch340"
sudo fuser -v /dev/ttyUSB0

grep -n "ttyUSB" \
  src/stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py
```

# 2. LiDAR 번호가 바뀐 경우

`/dev/ttyUSB0`이 `/dev/ttyUSB1`로 바뀌었으면 `LIDAR_PORT=/dev/ttyUSB1`, 반대이면 `LIDAR_PORT=/dev/ttyUSB0`으로 지정합니다.

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1

LIDAR_PORT=/dev/ttyUSB0

sed -i -E \
  "s#/dev/ttyUSB[0-9]+#$LIDAR_PORT#g" \
  src/stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py

grep -n "ttyUSB" \
  src/stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py

source /opt/ros/humble/setup.bash
source /home/chanmi/ydlidar_ws/install/setup.bash

colcon build \
  --packages-select stack_estop \
  --symlink-install

source install/setup.bash

grep -n "ttyUSB" \
  install/stack_estop/share/stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py
```

# 3. 터미널 1 — CAN 설정 및 CAN 로그

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0

LOG_DIR="$HOME/stack_estop_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "$LOG_DIR" | tee /tmp/stack_estop_log_dir

candump -L can0 2>&1 | tee "$LOG_DIR/candump.log"
```

# 4. 터미널 2 — 실제 코드 실행 및 launch 로그

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source /home/chanmi/ydlidar_ws/install/setup.bash
source install/setup.bash

LOG_DIR=$(cat /tmp/stack_estop_log_dir)

ros2 launch stack_estop \
  REAL_VEHICLE_stack_estop_mgm_can.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  can_interface:=can0 \
  2>&1 | tee "$LOG_DIR/launch.log"
```

로그에서 실제 LiDAR 포트와 다음 노드의 시작 여부를 확인합니다.

- `ydlidar_ros2_driver_node`
- `stack_estop_node`
- `mgm_node`
- `can_bridge_node_REAL_VEHICLE`

# 5. 터미널 3 — ROS bag 기록

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source /home/chanmi/ydlidar_ws/install/setup.bash
source install/setup.bash

LOG_DIR=$(cat /tmp/stack_estop_log_dir)

ros2 bag record \
  -o "$LOG_DIR/dynamic_3m_latch_test_bag" \
  /scan \
  /perception/estop/status \
  /perception/estop \
  /perception/static_estop \
  /perception/dynamic_obstacle_detected \
  /perception/dynamic_estop \
  /adas/target_ref \
  /vehicle/vector \
  /tf \
  /tf_static \
  /rosout \
  /parameter_events
```

# 6. 터미널 4 — 실시간 상태 확인

상태 JSON에는 `state`, `static_nearest_cluster_min_x`, candidate track 정보, `dynamic_tracking_max_distance_m`, `dynamic_stop_distance_m`, `hazard_track_id`, `hazard_latched`, `hazard_clear_count`, `dynamic_estop`, `final_estop`이 포함됩니다.

```bash
source /opt/ros/humble/setup.bash
source /home/chanmi/HL-Global-Mobility-Team2-1/install/setup.bash

while true; do
  clear
  echo "===== /perception/estop/status ====="
  timeout 2 ros2 topic echo --once /perception/estop/status
  echo "===== /perception/estop ====="
  timeout 2 ros2 topic echo --once /perception/estop
  echo "===== /perception/dynamic_estop ====="
  timeout 2 ros2 topic echo --once /perception/dynamic_estop
  echo "===== /adas/target_ref (v_ref 포함) ====="
  timeout 2 ros2 topic echo --once /adas/target_ref
  sleep 1
done
```

# 7. 시험 순서

1. 빈 공간에서 5초 대기
2. 더미를 `x=2.5m`, `y=±0.6m`에 배치
3. `y=±0.2m`까지 한 방향으로 횡이동
4. 통로 안에서 더미 정지
5. `hazard_latched=true` 유지 확인
6. 차량 저속 접근
7. 약 1.2m에서 `dynamic_estop=true`, `final_estop=true`, `v_ref=0` 확인
8. 0.7m `static_estop`보다 먼저 정지해야 성공
9. 더미 제거 후 latch 해제 확인

# 8. 종료 및 로그 확인

종료 순서:

1. rosbag 터미널에서 `Ctrl+C`
2. launch 터미널에서 `Ctrl+C`
3. 상태 모니터 터미널에서 `Ctrl+C`
4. candump 터미널에서 `Ctrl+C`
5. CAN 인터페이스 종료

```bash
sudo ip link set can0 down
```

로그 확인:

```bash
LOG_DIR=$(cat /tmp/stack_estop_log_dir)
echo "$LOG_DIR"
du -sh "$LOG_DIR"
find "$LOG_DIR" -maxdepth 3 -type f | sort
```

최종 로그:

- `launch.log`
- `candump.log`
- `dynamic_3m_latch_test_bag/`
