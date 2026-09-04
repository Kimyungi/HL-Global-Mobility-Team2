# 통합 실차 운영 런북 (2026-08-30) — 주차 제외, 신호등 실제 정지

**launch:** `adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py`

> Kvaser Leaf v3가 주행 중 순간 재열거되면 CAN 브리지는 프로세스를 유지한 채 새
> `can0` 소켓을 100ms 간격으로 자동 재연결하고 다음 최신 MGM 목표부터 송신한다.
> 브리지 프로세스 자체가 종료돼도 0-command 가드 뒤 1초 후 자동 재기동하므로 현장
> 수동 재시작은 필요하지 않다. 반복 분리는 케이블·커넥터 결함이므로 출발 전에 제거한다.

> **현재 적용 후보: `traffic_stop_y_ratio=0.60`**
> 근거 run: `drive_logs/run_0830_175528`. 1 m/s 접근 중 첫 안정 정지선 검출이
> `y_med=0.600`에서 성립했고 당시 코스 종점까지 경로상 약 5.41 m였다. 다음 안정
> 검출은 `0.687`/약 1.69 m, `0.729`/약 0.66 m여서 검출 누락과 제동 여유를 고려해
> 첫 운영 후보는 0.60으로 잡았다. **실차 출발 전 정지 상태에서 적색+정지선을 보여
> `FINAL_STOP=1`, 이어 초록에서 `FINAL_STOP=0`을 확인해야 최종 확정된다.**
> 2026-08-30 `run_0830_181646`에서 적색과 정지선이 서로 다른 프레임에 검출돼
> 미정지한 뒤, 적색 3/5를 fresh 초록까지 기억하는 `red_phase` 래치를 추가했다.
> 실험 신호등의 초록 좌회전 화살표에서 YOLO bbox가 사라지는 현장 조건 때문에,
> 확정 적색 때 저장한 동일 신호등 bbox 안의 초록색은 모양과 무관하게 3/5에서
> 출발로 인정한다. 저장 영역 밖의 초록색은 출발 근거로 사용하지 않는다.
> 2026-08-30 야간 재시험부터 정지선 후보는 기존 주간 흰색 마스크에 CLAHE 국소 대비와
> 평행 에지 쌍을 추가했다. 에지 쌍 실차 검출은 `y_ratio=0.885~0.978`, 폭 45~80%에서
> `stable=1`, `FINAL_STOP=1`을 확인했다. 순간 `stopline=0`이어도 이미 걸린 정지 래치는
> 초록 3/5 전까지 유지된다. 두 OAK-D 동시 시작 경쟁으로 traffic 노드가 종료되면
> launch가 2초 뒤 자동 재기동한다.

차선 · GPS(waypoint) · 회피 · 돌발 장애물 긴급정지 · 신호등/정지선 정지를 한
구성으로 운용한다. 라이다 주차는 포함하지 않는다. 이 문서는
`RUNBOOK_full_measurement_20260830.md`에서 현장별 신호등 임계값을 얻고 정지·재출발까지
검증한 뒤에만 사용한다.

> **출발 금지 조건:** 검증된 `traffic_stop_y_ratio`가 없거나 0이면 이 런북을 실행하지
> 않는다. 측정 런북으로 돌아간다. 예시 숫자를 다른 장착 상태나 코스에 복사하지 않는다.

## 처음 하는 사람은 여기만 순서대로 실행

전제 조건은 `RUNBOOK_full_measurement_20260830.md`를 완료해 다음 파일이 존재하는 것이다.

```text
$HOME/FMA_ws/traffic_stop_y_ratio.txt
```

아래는 **한라대학교 코스 기준**이다. 원주 운전면허시험장이면 V2 블록의 `COURSE=`
한 줄만 `waypoints_wonju_license_20260818_160511.csv`로 바꾼다.

### 1단계 — 차량을 띄우고 물리 비상정지를 누른 상태로 둔다

주변 사람과 장애물을 치우고 물리 비상정지가 실제로 동작하는지 확인한다. `go`를
실행하기 전까지 물리 비상정지를 해제하지 않는다.

### 2단계 — 차량 PC의 새 터미널에서 사전점검

```bash
source /opt/ros/humble/setup.bash
source $HOME/FMA_ws/install/setup.bash
ros2 run stack_traffic stack_traffic_ml_preflight
$HOME/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh --check
python3 $HOME/FMA_ws/src/multi_lidar_fusion/tools/check_sensors.py --no-ros
python3 -c 'import os; p=os.path.expanduser("~/FMA_ws/traffic_stop_y_ratio.txt"); v=float(open(p).read()); assert 0.0 < v <= 1.10; print("운영 임계값:", v)'
```

각각 `ML_RUNTIME_READY`, `✔ 점검 통과`, `== 전 항목 통과 ==`, `운영 임계값:`이
나와야 한다. 하나라도 실패하면 이후 명령을 실행하지 않는다.

### 3단계 — B1: 베이스 PC에서 실행

```bash
cd $HOME/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py --radio /dev/ttyRadio
```

### 4단계 — V1: 차량 PC의 새 터미널(t1)에서 실행

```bash
python3 $HOME/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
  --port /dev/ttyRadio --tcp-port 2101
```

약 10초마다 `RTCM`과 0보다 큰 `B/s`가 나와야 한다.
RTK가 안정화되도록 5~10분 기다리는 동안 다음 단계를 진행한다.

### 5단계 — V2: 차량 PC의 새 터미널(t2)에서 통합 launch 실행

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
COURSE="$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv"
FMA_TRAFFIC_STOP_Y_RATIO="$(tr -d '[:space:]' < "$HOME/FMA_ws/traffic_stop_y_ratio.txt")"
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  waypoint_csv:="$COURSE" \
  traffic_enabled:=true \
  traffic_require_stop_gate:=true \
  traffic_stop_y_ratio:="$FMA_TRAFFIC_STOP_Y_RATIO"
```

이 명령부터 CAN TX가 활성화된다. 아직 `go`를 실행하지 않는다. 임계값 파일이 잘못되면
launch가 `운영 신호등 정지 게이트가 비활성` 오류로 종료되는 것이 정상 안전 동작이다.

### 5-1단계 — 신호등 노드 자동복구 확인

launch 후 5초 기다린 뒤 새 터미널에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
ros2 node list | grep stack_traffic
```

반드시 `/stack_traffic_node`가 출력돼야 한다. 첫 시작이 실패해도 2초 뒤 자동
respawn한다. 아무 출력이 없으면 출발하지 않는다.

### 5-2단계 — GPS FIXED 확인 (t3)
```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"

ros2 topic echo /perception/gps_path --field fix_quality | \
while read -r quality; do
  case "$quality" in
    4) echo "$(date '+%H:%M:%S')  FIXED (4) ✅" ;;
    5) echo "$(date '+%H:%M:%S')  FLOAT (5) ⚠️" ;;
    1) echo "$(date '+%H:%M:%S')  GPS (1) ❌" ;;
    0) echo "$(date '+%H:%M:%S')  NO FIX (0) ❌" ;;
  esac
done
```

### 6단계 — M: 차량 PC의 새 터미널에서 자동 점검(t4)

```bash
source /opt/ros/humble/setup.bash
source $HOME/FMA_ws/install/setup.bash
timeout 12 ros2 topic hz /scan
timeout 12 ros2 topic hz /perception/lane_path
timeout 12 ros2 topic hz /perception/avoid
timeout 12 ros2 topic hz /perception/traffic_stop
timeout 12 ros2 topic hz /adas/target_ref
timeout 12 ros2 topic hz /vehicle/vector
ros2 param get /stack_traffic_node stopline_stop_y_ratio
```

앞의 네 토픽은 약 10 Hz, 뒤의 두 토픽은 약 100 Hz여야 한다. 마지막 값은 저장했던
0보다 큰 임계값과 같아야 한다.

차량이 정지한 상태에서 먼저 적색을 보여 `red_phase=1`을 만든 다음 정지선을
카메라에 보여 준다. V2 로그에서 `red_phase=1`, `stopline=1`, `y_ok=1`,
`FINAL_STOP=1`을 확인한다. 이어 초록을 보여
`green_votes=3/5`와 `FINAL_STOP=0`을 확인한다. 둘 중 하나라도 확인하지 못하면 출발하지 않는다.

### 7단계 — V3: 출발 직전에 물리 비상정지를 해제하고 실행(t5)

운전자는 계속 물리 비상정지에 손을 둔다.

```bash
source /opt/ros/humble/setup.bash
source $HOME/FMA_ws/install/setup.bash
ros2 run adas_mgm go --require-traffic
```

모든 항목이 `[OK]`이고 `출발 인가 발행 완료`가 나와야 출발한다.

### 8단계 — 즉시 정지와 정상 종료

- 주행 중 위험: 운전자가 **물리 비상정지**를 누른다.
- 소프트웨어 정상 종료: V2에서 `Ctrl-C`를 한 번 누른다.
- 차량이 완전히 멈춘 뒤 V1, 마지막으로 B1에서 `Ctrl-C`를 누른다.
- `kill -9`, PC 전원 차단, CAN 케이블 분리를 정지 방법으로 사용하지 않는다.

---

## 아래는 원리·진단·시험 후 분석 상세

## 0. 포함 범위와 중복 실행 금지

| 시나리오 | 상태 | 입력/결과 |
|---|---|---|
| 차선 주행 | 포함 | `stack_lane` → LANE |
| GPS 주행 | 포함 | `stack_gps` → WAYPOINT |
| 장애물 회피 | 포함 | `stack_avoid` → AVOID |
| 신호등·정지선 정지 | 포함 | `stack_traffic` → `v_ref=0` |
| 돌발 장애물 긴급정지 | 포함 | `stack_estop` → `v_ref=0` |
| 라이다 주차/4-LiDAR 융합 | 제외 | 별도 주차 런북 대상 |

다음 launch와 동시에 실행하지 않는다.

- `stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py`
- `stack_avoid`의 `field_session` 계열
- `adas_mgm/launch/MBD_lane_gps_can.launch.py`
- `multi_lidar_fusion/launch/multi_lidar_drivers.launch.py`

마지막 항목의 전방 `a1`과 이 launch의 `/scan`은 같은 물리 포트다. 함께 띄우면
드라이버가 포트를 중복 점유한다. 이 운영 구성의 안전·회피 입력은 전방 `/scan` 한 개다.

## 1. PC와 센서 사전점검

```bash
ros2 run stack_traffic stack_traffic_ml_preflight
$HOME/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh --check
python3 $HOME/FMA_ws/src/multi_lidar_fusion/tools/check_sensors.py --no-ros
```

- 첫 명령: `ML_RUNTIME_READY`
- CAN: `✔ 점검 통과`, MTU 72(CAN FD)
- 센서: 심링크·IMU·GPS·CAN·OAK-D 두 대 확인
- OAK-D MxID: 차선 `14442C105157D3D200`, 신호등 `14442C10B167CFD200`
- 카메라는 USB2(`HIGH`)로 연결한다. USB3는 GNSS L1을 방해한다.
- `--no-ros`는 의도적이다. 4-LiDAR 토픽은 이 운영 launch의 입력이 아니다.

udev와 새 PC 설치 절차는 `HANDOVER.md` §2.3·§2.5를 따른다.

## 2. 현장 준비와 터미널

베이스 좌표·코스·구간 준비는 `RUNBOOK_lane_gps.md` §0~§2를 따른다. RTK는 B1과
V1을 먼저 켜고 5~10분 워밍업한다.

### B1 — 베이스 PC

```bash
cd $HOME/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py --radio /dev/ttyRadio
```

### V1 — 차량 PC RTCM 중계

```bash
python3 $HOME/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
  --port /dev/ttyRadio --tcp-port 2101
```

10초 로그가 `RTCM ~580 B/s` 근처인지 확인한다.

## 3. 검증된 신호등 임계값 설정

측정 런북에서 결정하고 실제 정지 위치까지 검증한 값을 넣는다.

```bash
export FMA_TRAFFIC_STOP_Y_RATIO=<검증된_0보다_큰_값>
python3 -c 'import os; v=float(os.environ["FMA_TRAFFIC_STOP_Y_RATIO"]); assert 0.0 < v <= 1.10, v; print("traffic gate OK:", v)'
```

검증 명령이 실패하면 출발하지 않는다. 노드의 거리 파라미터명은
`stopline_stop_distance_m`이지만, 현재 통합 launch는 RGB-only라 거리 게이트를
노출하거나 사용하지 않는다.

## 4. V2 — 통합 launch 한 번만 실행

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
COURSE="$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv"
FMA_TRAFFIC_STOP_Y_RATIO="$(tr -d '[:space:]' < "$HOME/FMA_ws/traffic_stop_y_ratio.txt")"
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  waypoint_csv:="$COURSE" \
  traffic_enabled:=true \
  traffic_require_stop_gate:=true \
  traffic_stop_y_ratio:="$FMA_TRAFFIC_STOP_Y_RATIO"
```

`traffic_require_stop_gate:=true`는 신호등 노드가 꺼졌거나 임계값이 0/범위 밖이면
launch 전체를 거부한다. 확인 토큰이 없을 때도 launch 전체가 거부된다. 토큰이 있어도 MGM의 `wait_go=true` 때문에
`go` 전까지 `v_ref=0`이다. 다만 CAN TX는 이미 활성화되므로 물리 비상정지에 손을 둔다.

## 5. 출발 전 필수 게이트

```bash
ros2 topic hz /scan
ros2 topic hz /perception/lane_path
ros2 topic hz /perception/avoid
ros2 topic hz /perception/traffic_stop
ros2 topic hz /adas/target_ref
ros2 topic hz /vehicle/vector
ros2 param get /stack_traffic_node stopline_stop_y_ratio
```

기대값은 `/scan`, lane, avoid, traffic이 약 10 Hz, target_ref와 vehicle/vector가 약
100 Hz다. 마지막 파라미터가 설정한 0보다 큰 값과 같아야 한다.

정지 상태에서 실제 신호등과 정지선을 보여 다음 로그를 확인한다.

```text
red_phase=1 ... stopline=1 stable=1 ... y_ok=1 gate=y ... FINAL_STOP=1
```

이어 초록을 보여 `green_votes=3/5` 이후 `FINAL_STOP=0`으로 해제되는지 확인한다.
카메라 사망, bbox 소실, 정지선 소실은 정지 래치 해제 조건이 아니다.

하나라도 실패하면 `go`를 실행하지 말고 V2를 Ctrl-C로 종료한다.

## 6. V3 — 출발 인가

```bash
ros2 run adas_mgm go --require-traffic
```

GPS 경로+RTK FIXED, lane, 전방 `/scan`, target_ref, avoid와 traffic_stop을 모두
수신해야 출발 인가가 발행된다. `--force`, `--skip-lane`, `--skip-avoid`는 이 통합
운영에서 사용하지 않는다.

## 7. 주행 중 판정과 정지

- 적색 확정과 안정 정지선 검출이 동시에 성립할 때 TRAFFIC으로 전이한다.
  이후 정지선 소실 edge에서 시드한 거리를 `/vehicle/vector.v`로 적분해
  갱신하며 감속하고 초록에서 LANE으로 복귀한다.
- traffic_stop이 0.5초, TRAFFIC 중 vehicle/vector가 0.2초 stale이면 정지를 강제한다.
- 소프트웨어 정지는 **V2 Ctrl-C**다. 정상 종료 경로의 `can_zero`가 목표값 0을 보낸다.
- `kill -9`, PC 전원 차단, CAN 케이블 분리는 정지 수단이 아니다. dSPACE counter
  watchdog이 아직 없으므로 마지막 속도 명령이 유지될 수 있다.

CAN 왕복은 출발 전에 다음으로 확인할 수 있다.

```bash
python3 $HOME/FMA_ws/src/stack_avoid/tools/can_log.py \
  --iface can0 --duration 30 --out /tmp/can.log
```

FD 프레임만 존재하고 TX 프레임/헤더가 2.00이어야 한다.

## 8. 시험 후 분석

V2가 출력한 정확한 run 경로를 사용한다.

```bash
RUN=$HOME/FMA_ws/drive_logs/run_<시각>
ros2 bag info "$RUN/rosbag"
ros2 run adas_mgm core_replay "$RUN/mgm_snapshots.bin" "$RUN/replay.csv"
column -s, -t "$RUN/transitions.csv" | cut -c1-160
```

신호등 로그는 터미널 A에서 `ros2 bag play "$RUN/rosbag" --topics /rosout`, 터미널
B에서 다음 명령을 동시에 실행해 확인한다.

```bash
ros2 topic echo /rosout --field msg | \
  grep --line-buffered -E 'FINAL_STOP|red_votes|green_votes|y_ratio='
```

`/perception/traffic_stop`, `FINAL_STOP=1`, 초록 해제, MGM의 최종 `v_ref=0` 시점이
일관되는지 확인한다.

## 참조

- `RUNBOOK_full_measurement_20260830.md` — 임계값 측정 전용
- `RUNBOOK_lane_gps.md` — 베이스·RTCM·일반 GPS/차선 진단
- `RUNBOOK_avoid_field_test.md` — 회피 판정·튜닝·구간 기록
- `HANDOVER.md` — PC 설치, udev, USB 전원/허브 함정
- `stack_traffic/REQUIREMENTS.md` — 투표·정지선·정지 래치 계약
- `bridge_dspace/CAN_BRINGUP.md` — CAN 단계별 검증
