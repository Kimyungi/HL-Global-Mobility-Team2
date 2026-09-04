# 실내 직진 적색 신호·정지선 1 m/s 정지 시험

## 확정 조건

- 실내 지면에서 고정 직진 참조(`y=yaw=curvature=0`)만 따라간다.
- GPS·RTK·웨이포인트·차선 카메라·회피 로직은 사용하지 않는다.
- 목표속도는 `1.0 m/s`(3.6 km/h)다.
- 신호등·정지선 위치와 거리는 MGM에 사전 입력하지 않는다.
- 적색과 최근 3/5의 안정 정지선 segmentation이 함께 검출되면 TRAFFIC에 진입한다.
- depth 거리는 진단용이며 무효여도 안정 정지선 검출은 유효하다.
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

`--force`는 사용하지 않는다. 출발 직후 `v_ref`가 1.0m/s를 넘으면 즉시 종료한다.

## 성공·실패 판정

성공 조건은 모두 충족해야 한다.

1. 적색만 또는 정지선만 검출되면 `state != 4`다.
2. 적색과 정지선이 함께 확정되면 `state == 4`다.
3. E-stop 없이 `v_ref == 0`과 실차속도 `v == 0`으로 수렴한다.
4. 차량 최전단이 정지선 전방 0~1.0m에 정지한다.

`/perception/estop.estop == true`, 정지선 통과, TRAFFIC 미진입, 센서·MGM·CAN
노드 종료, 조향 이상 중 하나라도 발생하면 즉시 종료하고 실패로 기록한다.

로그는 `~/FMA_ws/drive_logs/indoor_traffic_<시각>/`에 저장된다.

## 2026-09-04 추론 주기 진단과 반영 사항

실차 PC(`Intel Core Ultra 9 288V`, 논리 CPU 8개, RAM 30GiB)에서 설치된
PyTorch 모델을 `640x360` 입력, `imgsz=640`, CPU 장치로 각각 2회 준비 실행 후
10회 측정했다.

| 모델 | 용도 | 파라미터 | 파일 크기 | CPU 추론 지연(min/median/max) |
|---|---|---:|---:|---:|
| `yolov8n.pt` | 신호등 bbox 검출 | 3,157,200 | 6.25MiB | 98.3/192.1/244.4ms |
| `stopline_yolov8s_seg.pt` | 정지선 segmentation | 11,791,257 | 22.74MiB | 377.3/473.9/625.1ms |

현재 launch의 카메라와 처리 타이머는 모두 10Hz이므로 한 프레임의 처리 예산은
100ms다. 적색이 확정되면 `stack_traffic_node`의 동일 timer callback 안에서
정지선 segmentation을 먼저 실행하고 신호등 detection을 이어서 실행한다. 두
모델의 CPU 중앙 지연 합계만 약 666ms이므로, 후처리를 제외해도 목표 10Hz를
달성할 수 없다.

MGM의 `traffic_stale_timeout_sec`는 0.5초다. 현장 실행 중 아래 경고가
반복됐으며, 모델 추론이 callback과 `/perception/traffic_stop` 발행을 함께
지연시키는 현상과 일치한다.

```text
traffic_stop 신선도 초과 — 정지 요구 강제 (stack_traffic 확인 필요)
```

또한 화면에서 `stopline score=0.32`, `y_max=270px`, `stable=1`이었지만,
OAK-D depth가 `accepted=0`이고 유효 픽셀 비율이 약 0~1%여서 기존 코드는
`stopline_detected=false`를 발행했다.

이 시험 구성에는 다음 수정이 반영됐다.

1. 신호등·정지선 YOLO 입력 크기를 320으로 낮춘다.
2. 신호등 YOLO는 2프레임마다 실행한다.
3. 재출발을 사용하지 않으므로 적색 확정 뒤 신호 페이즈를 고정하고 신호등
   YOLO를 생략한다.
4. `stopline_detected`는 depth가 아니라 최근 3/5 안정 segmentation으로 발행한다.
5. 단발성 segmentation miss는 소실 edge로 취급하지 않는다.
6. depth 스트림은 끄고 RGB-only로 실행한다.
7. 카메라를 열기 전에 두 모델을 한 번씩 warm-up한다. 따라서 첫 주행 프레임의
   100ms 초과 cold-start를 출발 전 기동 단계에서 소모한다.

통합 주행 launch는 초록 재출발이 필요하므로 적색 뒤 신호등 모델을 완전히
멈추지 않는다. 대신 3프레임마다 신호등을 재확인하고 나머지 프레임에만 정지선을
실행한다. 어느 callback에서도 두 YOLO를 함께 실행하지 않는다. 이 설정에서는
정지선 유효 샘플이 약 6.7 Hz라 소실 edge가 단일 정지 시험보다 약 0.1~0.2초
늦어질 수 있다. 통합 실차 시험 전에 정지 위치 로그를 보고
`traffic_ramp_distance_m`과 `traffic_stop_offset_m`을 다시 확인한다.

동일 날짜 개발 PC(`Intel Core Ultra 7 256V`)에서 노드와 같은
`KMP_BLOCKTIME=0`, `OMP_WAIT_POLICY=PASSIVE`, 640x360 검은 입력과
`imgsz=320`으로 warm-up 후 통합 적색 스케줄 120 callback을 재현했다.

| 구간 | callback 수 | median | p95 | max | 100ms 초과 |
|---|---:|---:|---:|---:|---:|
| 전체 | 120 | 45.0ms | 74.4ms | 85.4ms | 0 |
| 신호등 | 40 | 26.2ms | 36.3ms | 42.8ms | 0 |
| 정지선 | 80 | 55.6ms | 75.7ms | 85.4ms | 0 |

이는 모델 callback 예산 검증이며 두 OAK-D, LiDAR, lane XPU, CAN을 모두 띄운
실차 통합 부하 시험을 대신하지 않는다. 실제 통합에서는 `proc_ms`, 토픽 간격,
MGM 지터를 함께 기록한다.

출발 전 정지 상태에서 로그의 `proc_ms`와 토픽 발행 간격을 확인한다. 실제
`/perception/traffic_stop` 발행 간격이 한 번이라도 0.5초 이상이거나 MGM에서
traffic freshness 경고가 발생하면 `go`를 실행하지 않는다. 목표 여유는 최대
발행 간격 0.4초 이하다.
