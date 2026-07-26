# stack_gps 명령어 치트시트 — 복붙용

> 모든 명령은 **그대로 복붙** 가능. 새 터미널은 `.bashrc`에 워크스페이스가
> 등록돼 있어 별도 source 불필요 (안 되면 `source ~/FMA_ws/install/setup.bash`).
> 시리얼 포트(`/dev/ttyRover`)는 **한 번에 한 프로그램만** — 기록 도구와 노드를
> 동시에 켜지 말 것.

## 0. 사전 점검 (노트북, 아무 터미널)

```bash
tailscale status | head -3                  # VPN 연결 확인 (기기 목록 나오면 OK)
ls -l /dev/ttyRover                         # 로버 USB 잡혔는지 (링크 보이면 OK)
nc -vz 100.70.198.29 2101                   # 베이스 RTCM 서버 살아있는지 (succeeded면 OK)
```

## 1. 베이스가 잘 돌고 있는지 확인

**산업용 PC 화면에서** (또는 ssh 접속 후) — `rtcm_server.py` 터미널에
10초마다 `RTCM ~500 B/s` 통계가 찍히고 있으면 정상. `0 B/s ⚠`면 EVK 케이블 확인.

서버가 꺼져 있으면 (산업용 PC에서):

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py
```

노트북에서 간접 확인은 위 `nc -vz` 한 줄이면 된다.

## 2. 웨이포인트 따기 (기록)

**노트북 터미널 1개만** 필요. 로버 안테나 들고 시작 지점에서:

```bash
cd ~/FMA_ws/src/stack_gps/tools/waypoints
python3 record_waypoints.py --host 100.70.198.29 --name track_B
```

- `--name`은 트랙마다 다르게 (track_B, parking_1, ...)
- 상태줄 `FIXED` 확인 후 이동 시작. FLOAT로 떨어지면 자동 일시정지 — 멈춰서 복귀 대기.
- 같은 경로 왕복 금지 — 한 방향 한 번이 한 트랙.
- 끝나면 **안테나 내리기 전에 Ctrl-C** → `waypoints/waypoints_track_B_*.csv` 저장됨.

## 3. 딴 웨이포인트로 실행 (도보 검증)

**터미널 1 — stack_gps 노드** (가장 최근 기록한 CSV 자동 선택):

```bash
ros2 run stack_gps stack_gps_node --ros-args \
    -p waypoint_csv:=$(ls -t $HOME/FMA_ws/src/stack_gps/waypoints/waypoints_*.csv | head -1) \
    -p rtcm_host:=100.70.198.29
```

특정 트랙을 지정하려면 `waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/파일명.csv`.
정상: `[link] 로버 시리얼 연결` → 2초마다 `FIXED age 0.2s RTCM ...B/s idx N 횡오차 0.0Xm`.

**터미널 2 — 정밀 뷰어** (트랙 대비 내 위치·횡오차, 검증은 이걸로):

```bash
cd ~/FMA_ws/src/stack_gps/tools/waypoints
python3 live_view.py
```

**터미널 3 — RViz 위성지도** (선택 — 시연·감각용, 인터넷 필요):

```bash
rviz2 -d ~/FMA_ws/src/stack_gps/tools/waypoints/gps_view.rviz
```

합격 기준: 트랙 위를 걸을 때 횡오차 수 cm 유지, 옆으로 1m 비키면 횡오차 ~100cm.

## 4. 전체 체인 리허설 (차·dSPACE 없이)

GPS → MGM → 브리지까지 노트북 안에서 관통 확인. 위 3번의 터미널 1(노드)을 켠 상태에서:

**터미널 4 — MGM (10ms 루프):**

```bash
ros2 launch adas_mgm mgm.launch.py
```

**터미널 5 — dSPACE 흉내:**

```bash
ros2 run bridge_dspace dspace_sim_node --ros-args -p pc_ip:=127.0.0.1
```

**터미널 6 — UDP 브리지:**

```bash
ros2 run bridge_dspace udp_bridge_node --ros-args -p dspace_ip:=127.0.0.1
```

> ⚠ `loopback_test.launch.py`는 쓰지 말 것 — MGM 없이 브리지만 시험하는
> 구성이라 더미 발행기가 포함돼 있어, MGM과 같은 토픽(`/adas/target_ref`)에
> 동시에 쏘게 된다.

**터미널 7 — 최종 출력 확인:**

```bash
ros2 topic echo /adas/target_ref --once        # ref_points 채워지고 v_ref: 0.5 나오면 성공
ros2 topic echo /vehicle/vector --once         # dSPACE(흉내) 회신까지 오면 왕복 성공
```

차선 스택이 없으므로 MGM은 ~200ms 후 자동으로 waypoint 스테이트가 된다 (정상 동작).

## 5. 실차 (8/2) 

리허설과 동일하되 터미널 5만 교체 — 흉내 대신 진짜 dSPACE로:

```bash
ros2 launch bridge_dspace bridge.launch.py     # dSPACE IP는 launch 파라미터 확인
```

## 자주 막히는 것

| 증상 | 처방 |
|---|---|
| `Package 'stack_gps' not found` | `source ~/FMA_ws/install/setup.bash` |
| 상태줄 `RTCM 0B/s` | 베이스 서버 확인 (`nc -vz 100.70.198.29 2101`) + Tailscale |
| FLOAT에서 안 올라옴 | 하늘 트인 곳으로, 1~2분 대기 (콜드 스타트) |
| `Serial ... Permission/No such file` | 로버 USB 재연결 후 `ls -l /dev/ttyRover` |
| 뷰어 "위치 수신 대기 중" | 노드(터미널 1)가 FIXED인지 확인 |
