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

PC 단독 루프백 (dSPACE 에뮬레이터 포함):

```bash
ros2 launch bridge_dspace loopback_test.launch.py
# 다른 터미널에서:
ros2 topic hz /vehicle/vector     # ≈100 Hz
ros2 topic echo /vehicle/vector   # x·v 증가 = 왕복 성립
```

실기 (dSPACE 연결):

```bash
ros2 launch bridge_dspace bridge.launch.py dspace_ip:=192.168.1.10
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
