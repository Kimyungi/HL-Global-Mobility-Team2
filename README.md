# FMA_ws — WHEELTEC 자율주행 시스템

아키텍처·설계 결론은 [CLAUDE.md](CLAUDE.md)가 기준. 통신 바이너리 계약은 [PROTOCOL.md](src/bridge_dspace/PROTOCOL.md).

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/FMA_ws
colcon build --symlink-install
source install/setup.bash
```

## 부트스트래핑 순서 (CLAUDE.md §6)

### ① PC↔dSPACE 왕복 검증 (최우선)

최초 1회 — CAN 자동 셋업 설치 (이후 PCAN을 꽂으면 can0 자동 up, `--vcan`은 루프백용):

```bash
sudo src/bridge_dspace/tools/can_setup/install.sh --vcan
```

PC 단독 루프백 (dSPACE 에뮬레이터 포함, 가상 CAN 사용):

```bash
ros2 launch bridge_dspace loopback_test.launch.py
# 다른 터미널에서:
ros2 topic hz /vehicle/vector     # ≈100 Hz
ros2 topic echo /vehicle/vector   # x·v 증가 = 왕복 성립
```

실기 (dSPACE CAN 연결) — **단계별 검증 절차는 [CAN_BRINGUP.md](src/bridge_dspace/CAN_BRINGUP.md)** (배선·candump·watchdog까지 복붙 가이드):

```bash
ros2 launch bridge_dspace bridge.launch.py can_interface:=can0   # 자동 셋업 설치돼 있으면 꽂기만 하면 됨
ros2 run bridge_dspace dummy_ref_publisher   # 직선 ref, v_ref 0.3 → 바퀴 반응 확인
```

watchdog 검증: dummy_ref_publisher를 죽이고 30ms 후 감속 정지(조향 유지) 확인.

### ② MGM 10ms 루프 + 지터 로깅

```bash
ros2 launch adas_mgm mgm.launch.py
```

- 10초마다 주기 통계(min/mean/max/p99, 지연 최악값) 로그 출력.
- §7 장시간 측정 시 `jitter_csv_path` 파라미터로 CSV 기록 (인지 노드 풀가동 상태에서 수십 분~1시간, 최악값 기준).
- SCHED_FIFO 경고가 뜨면: `sudo setcap cap_sys_nice+ep <mgm_node 경로>` 또는 `/etc/security/limits.conf`에 rtprio 설정.

### ③ 각 스택 병렬 개발

```bash
ros2 run stack_lane stack_lane_node      # 이현준
ros2 run stack_gps stack_gps_node        # 김윤기
ros2 run stack_parking stack_parking_node # 손상민
ros2 run stack_avoid stack_avoid_node    # 이기돈
# 현재 차량: launch에 고정된 교통용 OAK-D MxID 사용
ros2 launch stack_traffic stopline_distance_test.launch.py # 김재민
ros2 run stack_estop stack_estop_node    # 박찬미
```

각 스택 폴더의 `REQUIREMENTS.md`에 담당자별 출력 계약·금지사항 정리. 스켈레톤은 중립값을 퍼블리시하므로 전체 파이프라인(스택 → MGM → 브리지 → dSPACE sim)을 지금 바로 관통 실행할 수 있다.

`stack_traffic` launch는 현재 차량의 교통용 OAK-D MxID를 기본값으로 사용한다.
차량 기본 프로필은 GNSS 간섭과 USB2 대역폭을 고려한
`oak_usb_speed:=high`, `oak_fps:=10`, `oak_depth_enabled:=false`다. 기동
로그에서 `usb_requested=HIGH/usb_actual=HIGH`를 확인해야 한다.
다른 교통용 OAK-D로 시험할 때는 아래 명령으로 확인한 MxID를 명시한다. 실제 차량
MxID 리터럴은 launch 기본값 한 곳만 기준으로 유지한다.

```bash
read -rp "교통용 OAK-D MxID: " FMA_TRAFFIC_OAK_MXID
ros2 launch stack_traffic stopline_distance_test.launch.py \
  oak_mxid:="${FMA_TRAFFIC_OAK_MXID:?교통용 MxID를 입력하세요}"
```

노드를 직접 실행하거나 개인 launch를 쓰는 경우에도 OAK-D가 두 대라면 MxID를
반드시 지정한다.
산업용 PC에서는 카메라 기동 전에 아래 사전점검으로 torch/torchvision NMS,
Ultralytics, DepthAI API를 확인한다. 이 명령은 패키지를 변경하지 않는다.

```bash
ros2 run stack_traffic stack_traffic_ml_preflight
```

`stack_lane`까지 같은 PC에서 실행할 때만 `--require-xpu`를 추가해 lane용 Intel
XPU도 함께 검사한다. 이 옵션은 traffic 자체가 XPU 추론을 사용한다는 뜻이 아니다.
REAL_VEHICLE 통합 launch는 lane OAK도 `usb_speed:=high`, `camera_fps:=10`으로
기동한다. 두 카메라 로그에서 실제 USB가 모두 HIGH인지 확인해야 한다.

또한 `resume_on_green=true`가 패키지 기본값이므로 fresh YOLO 초록 3/5에서 정지
래치가 자동 해제된다. 자동 해제를 원하지 않으면 `resume_on_green:=false`를
명시한다.

## 토픽 맵

| 토픽 | 메시지 | 방향 |
|---|---|---|
| /perception/lane_path | LanePath | stack_lane → MGM |
| /perception/gps_path | GpsPath | stack_gps → MGM |
| /perception/avoid | AvoidStatus | stack_avoid → MGM |
| /perception/parking | ParkingStatus | stack_parking → MGM |
| /perception/traffic_stop | TrafficStop | stack_traffic → MGM |
| /perception/estop | EstopRequest | stack_estop → MGM |
| /adas/target_ref | TargetRef | MGM → bridge (10ms) |
| /vehicle/vector | VehicleVector | bridge → stack_gps 등 (10ms) |
