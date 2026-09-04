# 실내 직진 적색 신호·정지선 3 km/h 정지 시험

## 확정 조건

- 실내 지면에서 고정 직진 참조(`y=yaw=curvature=0`)만 따라간다.
- GPS·RTK·웨이포인트·차선 카메라·회피 로직은 사용하지 않는다.
- 목표속도는 `0.8333333333 m/s`(3 km/h)다.
- 신호등·정지선 위치와 거리는 MGM에 사전 입력하지 않는다.
- 적색과 현재의 안정 정지선·유효 depth가 함께 검출되면 TRAFFIC에 진입한다.
- 차량 최전단이 인식된 정지선을 넘기 전 0~1.0m에서 멈추면 성공이다.
- 정지선 통과 또는 1.0m LiDAR E-stop 발동은 실패다.
- 초록불·재출발은 시험하지 않으며 E-stop 후 후진 탈출은 비활성이다.

## 출발 전

차량 경로에서 사람과 불필요한 장애물을 치우고, 물리 E-stop 담당자가 차량
옆에서 즉시 개입할 수 있어야 한다. 다른 실차 launch, MGM, CAN bridge,
stack_estop은 모두 종료한다.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run stack_traffic stack_traffic_ml_preflight
```

## 실행

```bash
ros2 launch adas_mgm REAL_VEHICLE_indoor_traffic_stop.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX
```

별도 터미널에서 `/scan`, `/perception/lane_path`, `/perception/traffic_stop`,
`/adas/target_ref`, `/vehicle/vector`가 수신되는지 확인한 다음 출발한다.
주행 전에는 차량을 정지한 채 신호등과 정지선을 카메라에 보여주고,
아래 출력에서 `red_active: true`와 `stopline_detected: true`가 둘 다 실제로
한 번 이상 확인되어야 한다. 둘 중 하나라도 안 나오면 주행하지 않는다.

```bash
ros2 topic echo /perception/traffic_stop
```

```bash
ros2 run adas_mgm go --skip-gps --skip-avoid --require-traffic
```

`--force`는 사용하지 않는다. 출발 직후 `v_ref`가 0.833334m/s를 넘으면 즉시 종료한다.

## 성공·실패 판정

성공 조건은 모두 충족해야 한다.

1. 적색만 또는 정지선만 검출되면 `state != 4`다.
2. 적색과 정지선이 함께 확정되면 `state == 4`다.
3. E-stop 없이 `v_ref == 0`과 실차속도 `v == 0`으로 수렴한다.
4. 차량 최전단이 정지선 전방 0~1.0m에 정지한다.

`/perception/estop.estop == true`, 정지선 통과, TRAFFIC 미진입, 센서·MGM·CAN
노드 종료, 조향 이상 중 하나라도 발생하면 즉시 종료하고 실패로 기록한다.

로그는 `~/FMA_ws/drive_logs/indoor_traffic_<시각>/`에 저장된다.
