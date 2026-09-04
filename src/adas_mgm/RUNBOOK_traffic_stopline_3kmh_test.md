# 적색 신호·정지선 3 km/h 실차 정지 시험

## 1. 확정 조건

- 실차를 지면에서 주행한다.
- MGM 목표속도는 `0.8333333333 m/s` (`3 km/h`)다.
- 신호등·정지선 위치나 거리를 MGM/GPS에 사전 입력하지 않는다.
- `red_active && stopline_detected`로 TRAFFIC에 진입해 자체 감속한다.
- 인식한 정지선을 넘기 **전 0~1.0 m** 구간에서 완전 정지하면 성공이다.
- 정지선을 넘거나 LiDAR E-stop이 `1.0 m`에서 발동하면 실패다.
- E-stop 발동 후 후진 탈출은 꺼두며, 즉시 시험을 종료한다.
- 초록불 인식·재출발은 이번 시험 범위에서 제외한다.

## 2. 출발 전 필수 조치

1. 차량 진행 구간과 정지선 앞에 사람·차량·장애물이 없는지 확인한다.
2. 조작자는 물리 E-stop을 즉시 누를 수 있는 위치에서 계속 감시한다.
3. 다른 실차 launch, MGM, CAN bridge, stack_estop이 실행 중이 아닌지 확인한다.
4. CAN, LiDAR, 두 OAK-D의 MxID, RTK FIXED, 저장 공간을 확인한다.
5. ML 사전 점검이 통과해야 한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run stack_traffic stack_traffic_ml_preflight
```

`ML_RUNTIME_READY` 이외의 결과에서는 주행하지 않는다.

## 3. 실행

`<코스.csv>`는 현재 베이스 좌표와 짝이 맞는 실제 코스로 바꿘다.

```bash
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  waypoint_csv:=<코스.csv> \
  traffic_enabled:=true \
  v_base:=0.8333333333 \
  estop_on_distance_m:=1.0 \
  estop_off_distance_m:=1.15 \
  dynamic_stop_distance_m:=1.0 \
  zones_file:=/dev/null \
  avoid_zone_only:=true \
  escape_after_cycles:=0 \
  record:=true
```

`zones_file:=/dev/null`로 GPS 구간 파일을 의도적으로 비우고,
`avoid_zone_only:=true`로 이 시험의 평시 AVOID 진입을 막는다.
LiDAR 근접은 회피 성공으로 처리하지 않고 `1.0 m` E-stop 실패 조건으로 남겨야 한다.
`estop_off_distance_m` 1.15 m는 1.0 m 발동 문턱의 히스테리시스 해제값이다.

출발 전 별도 터미널에서 아래를 모두 확인한다.

```bash
ros2 topic hz /scan
ros2 topic hz /perception/traffic_stop
ros2 topic echo /adas/target_ref
ros2 topic echo /vehicle/vector
```

모든 점검이 정상이고 물리 E-stop 담당자가 준비된 후에만 출발을 인가한다.

```bash
ros2 run adas_mgm go
```

## 4. 판정·중단

성공은 아래를 모두 충족해야 한다.

1. 주행 명령 `v_ref` 상한이 약 `0.8333 m/s`다.
2. 적색만 또는 정지선만 보이는 동안 `state != 4`다.
3. 적색과 정지선이 함께 확정되면 `state == 4` (TRAFFIC)다.
4. E-stop 없이 `v_ref == 0` 및 실차속도 `v == 0`으로 수렴한다.
5. 차량 최전단이 정지선을 넘지 않고, 정지선 앞 0~1.0 m에 멈춘다.

아래 중 하나라도 발생하면 물리 E-stop 또는 Ctrl-C로 즉시 종료하고 실패로 기록한다.

- `/perception/estop.estop == true` (특히 1.0 m LiDAR 문턱 발동)
- 정지선 통과
- 신호·정지선 인식 전 비정상 감속/정지
- TRAFFIC 진입 실패
- 조향·경로 이탈 조짐
- 카메라, LiDAR, GPS, MGM, CAN bridge 종료 또는 신선도 경고

Ctrl-C 후 `can_zero` 종료 경로가 목표속도 0을 전송했는지 확인한다.

## 5. 사후 확인

로그는 `~/FMA_ws/drive_logs/run_<시각>/`에 저장된다. 최소한 다음을
확인한다.

- `transitions.csv`: LANE/WAYPOINT → TRAFFIC 전이 시점과 근거
- `mgm_snapshots.bin`: `red_active`, `stopline_detected`, state, v_ref 재생
- `vehicle_vector.csv`: 실차속도 0 수렴
- `rosbag`: `/perception/traffic_stop`, `/perception/estop`, `/adas/target_ref`, `/scan`
- 현장 측정: 정지 후 차량 최전단과 정지선 간 거리

정지 성공 여부와 별개로, 보고서에 적색·정지선 동시 인식 시각,
TRAFFIC 진입 시각, `v_ref=0`, 실차 `v=0`, E-stop 여부를 모두 남긴다.
