# Stack E-Stop Reverse Recovery 실차 시험

> 실제 CAN 송신과 차량 구동이 가능한 시험이다. 차량 바퀴를 띄우고 물리 비상정지를 준비한 상태에서만 실행한다. FRONT/REAR scan timeout, `v_ref=0`, CAN 오류가 발생하면 즉시 중단한다.

현재 설정:

- 전방 E-Stop 연속 대기: 10.0초
- 후진 속도: -0.30 m/s
- 후진 고정 시간 제한: 없음
- 후진 종료 후 정지 유지: 0.5초
- FRONT/REAR scan timeout: 0.25초
- E-Stop status stale: 0.50초
- 후진 완료 후: 새로운 유효 AvoidStatus와 MGM AVOID TargetRef가 모두 확인될 때까지 `WAIT_AVOIDANCE`, `v_ref=0`

## 실차 시험 전 — FRONT/REAR LiDAR 안정성 확인

이 launch에는 LiDAR 두 대, static TF, scan gap monitor만 포함된다. MGM, Recovery, CAN bridge는 실행하지 않는다.

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source /home/chanmi/ydlidar_ws/install/setup.bash
source install/setup.bash

LOG_DIR="$HOME/stack_estop_logs/lidar_stability_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

ros2 launch stack_estop lidar_stability_diagnostic.launch.py \
  2>&1 | tee "$LOG_DIR/lidar_stability.log"
```

별도 터미널에서 수신률을 확인한다.

```bash
source /opt/ros/humble/setup.bash
source /home/chanmi/HL-Global-Mobility-Team2-1/install/setup.bash
ros2 topic hz /scan
```

```bash
source /opt/ros/humble/setup.bash
source /home/chanmi/HL-Global-Mobility-Team2-1/install/setup.bash
ros2 topic hz /rear/scan
```

launch 터미널에는 FRONT/REAR별로 다음 값이 매초 JSON으로 출력된다.

- `message_count`
- `current_hz`
- `max_inter_message_gap_sec`
- `gaps_over_0_25_sec`
- `gaps_over_1_0_sec`
- `last_message_age_sec`

종료 후 SDK 오류 횟수를 확인한다.

```bash
grep -c "Failed to get scan" "$LOG_DIR/lidar_stability.log"
grep -c "Timeout count" "$LOG_DIR/lidar_stability.log"
grep -c "Device Failed" "$LOG_DIR/lidar_stability.log"
```

`max_inter_message_gap_sec`가 0.25초를 넘거나 SDK 오류가 발생하면 실차 Recovery 시험을 진행하지 않는다.

## 터미널 1 — CAN 로그

```bash
sudo ip link set can0 down 2>/dev/null
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 txqueuelen 100
sudo ip link set can0 up
ip -details -statistics link show can0

LOG_DIR="$HOME/stack_estop_logs/recovery_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
echo "$LOG_DIR" | tee /tmp/stack_estop_log_dir

candump -L can0 2>&1 | tee "$LOG_DIR/candump.log"
```

## 터미널 2 — 최종 Recovery launch

```bash
cd /home/chanmi/HL-Global-Mobility-Team2-1
source /opt/ros/humble/setup.bash
source install/setup.bash

LOG_DIR=$(cat /tmp/stack_estop_log_dir)

ros2 launch stack_estop \
  REAL_VEHICLE_stack_estop_mgm_can_recovery.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  reverse_actuation_enabled:=true \
  reverse_confirm_token:=I_CONFIRM_REVERSE_RECOVERY_ACTUATION \
  reverse_wait_sec:=10.0 \
  status_stale_timeout_sec:=0.50 \
  can_interface:=can0 \
  2>&1 | tee "$LOG_DIR/launch.log"
```

실행 노드:

- FRONT/REAR YDLIDAR
- `stack_estop_node`
- `stack_avoid_node`
- `mgm_node`
- `reverse_recovery_node`
- `can_bridge_node`

## 터미널 3 — Rosbag 기록

```bash
source /opt/ros/humble/setup.bash
source /home/chanmi/HL-Global-Mobility-Team2-1/install/setup.bash

LOG_DIR=$(cat /tmp/stack_estop_log_dir)

ros2 bag record \
  -o "$LOG_DIR/recovery_final_bag" \
  /scan \
  /rear/scan \
  /perception/estop \
  /perception/estop/status \
  /perception/static_estop \
  /perception/dynamic_obstacle_detected \
  /perception/dynamic_estop \
  /perception/avoid \
  /perception/reverse_recovery/status \
  /adas/target_ref_mgm \
  /adas/target_ref \
  /vehicle/vector \
  /tf \
  /tf_static \
  /rosout \
  /parameter_events
```

## 터미널 4 — 상태 확인

```bash
source /opt/ros/humble/setup.bash
source /home/chanmi/HL-Global-Mobility-Team2-1/install/setup.bash

watch -n 0.2 '
echo "=== RECOVERY ==="
ros2 topic echo /perception/reverse_recovery/status --once
echo "=== AVOID INPUT ==="
ros2 topic echo /perception/avoid --once
echo "=== MGM TARGET ==="
ros2 topic echo /adas/target_ref_mgm --once
echo "=== FINAL TARGET ==="
ros2 topic echo /adas/target_ref --once
'
```

확인할 핵심 값:

- `WAIT_REVERSE_DELAY`: 전방 장애물과 E-Stop이 10초 연속 유지되는지
- `REVERSE_ACTIVE`: 최종 `v_ref=-0.30`
- Rear blocked: 즉시 `v_ref=0`, `WAIT_REAR_CLEAR`
- E-Stop 해제: 즉시 `v_ref=0`, `STOP_AFTER_REVERSE`
- `WAIT_AVOIDANCE`: 아래 fresh 조건이 모두 완료될 때까지 `v_ref=0`
- `avoid_status_fresh=true`
- `avoid_obstacle_detected=true`
- `avoid_avoidable=true`
- `mgm_target_fresh=true`
- `mgm_ref_points_valid=true`
- `avoidance_ready=true`
- `/adas/target_ref_mgm.state=2`: 새로운 MGM AVOID TargetRef 확인
- 이후 `/adas/target_ref`: MGM의 state/ref_points/v_ref가 그대로 전달되는지

## 시험 순서

1. FRONT/REAR `/scan`이 약 10 Hz인지 확인한다.
2. 후방 ROI가 비어 있고 `rear_clear=true`인지 확인한다.
3. 전방 장애물을 배치해 E-Stop을 발생시킨다.
4. `WAIT_REVERSE_DELAY`가 10초 유지되는지 확인한다.
5. `REVERSE_ACTIVE`, `v_ref=-0.30`을 확인한다.
6. 전방 장애물이 E-Stop 해제거리 밖으로 멀어지면 후진이 즉시 멈추는지 확인한다.
7. `STOP_AFTER_REVERSE` 0.5초 후 `WAIT_AVOIDANCE`를 확인한다.
8. 기존의 오래된 `state=2`와 ref points만으로는 `v_ref=0`이 해제되지 않는지 확인한다.
9. 새 AvoidStatus와 새 MGM AVOID TargetRef가 모두 들어와 `avoidance_ready=true`가 되는지 확인한다.
10. 이후 실제 AVOID TargetRef가 `/adas/target_ref`로 그대로 통과하는지 확인한다.

회피 경로가 만들어지지 않으면 `WAIT_AVOIDANCE`에서 정지 유지하는 것이 정상이다.

## 종료 및 로그 확인

종료 순서:

1. Rosbag `Ctrl+C`
2. Recovery launch `Ctrl+C`
3. 상태 모니터 `Ctrl+C`
4. candump `Ctrl+C`
5. CAN 인터페이스 종료

```bash
sudo ip link set can0 down

LOG_DIR=$(cat /tmp/stack_estop_log_dir)
echo "$LOG_DIR"
du -sh "$LOG_DIR"
find "$LOG_DIR" -maxdepth 3 -type f | sort
```

예상 결과 파일:

```text
launch.log
candump.log
recovery_final_bag/
```
