# FMA_ws — WHEELTEC 자율주행 시스템

아키텍처·설계 결론은 [CLAUDE.md](CLAUDE.md)가 기준. 통신 바이너리 계약은 [PROTOCOL.md](src/bridge_dspace/PROTOCOL.md).

> **처음 세팅하는 PC라면 [HANDOVER.md](HANDOVER.md) 부터.** 새 PC 세팅 순서, 운용 함정
> (카메라 USB3 가 RTK 를 죽이는 건 등), 남긴 미완 작업이 정리돼 있다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/FMA_ws
colcon build          # ⚠ --symlink-install 쓰지 말 것 (아래)
source install/setup.bash
```

> ⚠ **`--symlink-install` 금지.** 이 워크스페이스는 일반 `colcon build` 로 통일한다.
> 두 방식을 섞으면 `stack_gps` 가 `PackageNotFoundError` 로 즉사하는데, **이미 열려 있던
> 터미널에서만** 터져서 원인을 찾기 어렵다. 섞였다면 `rm -rf build install log` 후 재빌드.
> 처음 세팅하는 PC라면 [HANDOVER.md](HANDOVER.md) 를 먼저 볼 것.

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
ros2 topic hz /vehicle/mpc_trajectory  # ≈100 Hz, dSPACE MPC 51점 XY
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
ros2 run stack_traffic stack_traffic_node # 김재민
ros2 run stack_estop stack_estop_node    # 박찬미
```

각 스택 폴더의 `REQUIREMENTS.md`에 담당자별 출력 계약·금지사항 정리. 스켈레톤은 중립값을 퍼블리시하므로 전체 파이프라인(스택 → MGM → 브리지 → dSPACE sim)을 지금 바로 관통 실행할 수 있다.

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
| /vehicle/mpc_trajectory | MpcTrajectory | dSPACE → bridge, 51점 XY (10ms) |
