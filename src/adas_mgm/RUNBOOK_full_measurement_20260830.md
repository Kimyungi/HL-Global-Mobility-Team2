# 통합 실차 측정 런북 (2026-08-30) — 주차 제외, 신호등 정지 임계값 수집

**launch: `adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py` + `traffic_enabled:=true`**

> 이 문서는 **측정 전용**이다. 기본 `traffic_stop_y_ratio:=0.0`에서는 신호등이
> 차를 세우지 않는다. 검증된 임계값으로 실제 정지까지 시험하려면
> `RUNBOOK_full_operation_20260830.md`를 사용한다.

## 처음 하는 사람은 여기만 순서대로 실행

아래는 **한라대학교 코스 기준**이다. 원주 운전면허시험장이면 V2 블록의 `COURSE=`
한 줄만 다음 파일로 바꾼다.

```bash
COURSE=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_wonju_license_20260818_160511.csv
```

명령 앞의 `B1`, `V1` 등은 터미널 이름이다. 코드 블록 안의 내용만 복사한다.
명령 실행 중에는 해당 터미널을 닫지 않는다.

### 1단계 — 차량을 띄우고 물리 비상정지를 누른 상태로 둔다

- 구동 바퀴가 땅에 닿아 있다면 주변 사람과 장애물을 치운다.
- 물리 비상정지가 실제로 차량을 멈추는지 먼저 확인한다.
- 아래 절차에서 `go`를 실행하기 전까지 물리 비상정지를 해제하지 않는다.

### 2단계 — 차량 PC의 새 터미널에서 사전점검

```bash
source /opt/ros/humble/setup.bash
source $HOME/FMA_ws/install/setup.bash
ros2 run stack_traffic stack_traffic_ml_preflight
$HOME/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh --check
python3 $HOME/FMA_ws/src/multi_lidar_fusion/tools/check_sensors.py --no-ros
```

`ML_RUNTIME_READY`, CAN의 `✔ 점검 통과`, 센서의 `== 전 항목 통과 ==`가 모두 나와야
한다. 하나라도 실패하면 이후 명령을 실행하지 말고 이 문서의 상세 진단을 본다.

### 3단계 — B1: 베이스 PC에서 실행

```bash
cd $HOME/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py --radio /dev/ttyRadio
```

이 터미널은 그대로 둔다.

### 4단계 — V1: 차량 PC의 새 터미널에서 실행

```bash
python3 $HOME/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
  --port /dev/ttyRadio --tcp-port 2101
```

약 10초마다 `RTCM`과 0보다 큰 `B/s`가 나오면 정상이다. 이 터미널도 그대로 둔다.
RTK가 안정화되도록 5~10분 기다리는 동안 다음 단계를 진행한다.

### 5단계 — V2: 차량 PC의 새 터미널에서 통합 launch 실행

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
COURSE="$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv"
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  waypoint_csv:="$COURSE" \
  traffic_enabled:=true \
  traffic_stop_y_ratio:=0.0
```

이 명령부터 CAN TX가 활성화된다. 아직 `go`를 실행하지 않는다. 콘솔에서
`출발 대기 중`과 신호등 카메라의 `usb_actual=HIGH`를 확인한다.

### 6단계 — M: 차량 PC의 새 터미널에서 자동 점검

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

앞의 네 토픽은 약 10 Hz, 뒤의 두 토픽은 약 100 Hz여야 한다. 마지막 값은 반드시
`0.0`이어야 한다. `timeout` 종료 코드가 표시되는 것은 정상이며, 각 명령 출력에
`average rate`가 없으면 실패다.

### 7단계 — V3: 출발 직전에 물리 비상정지를 해제하고 실행

운전자는 계속 물리 비상정지에 손을 둔다.

```bash
source /opt/ros/humble/setup.bash
source $HOME/FMA_ws/install/setup.bash
ros2 run adas_mgm go --require-traffic
```

모든 항목이 `[OK]`이고 마지막에 `출발 인가 발행 완료`가 나와야 출발한다.
`[FAIL]`이 하나라도 나오면 출발하지 않는다.

> 이 측정 주행에서는 빨간 신호등과 정지선이 보여도 자동으로 서지 않는다. 운전자가
> 물리 비상정지로 안전하게 정차해야 한다.

### 8단계 — 측정 종료

먼저 V2에서 `Ctrl-C`를 한 번 누른다. `can_zero`가 실행되고 차량이 완전히 멈춘 것을
확인한 뒤 V1, 마지막으로 B1에서 `Ctrl-C`를 누른다. `kill -9`나 PC 전원 차단으로
멈추지 않는다.

V2 콘솔에 출력된 `~/FMA_ws/drive_logs/run_...` 경로를 기록하고, 아래 상세 절차의
§6에서 `y_ratio`를 추출한다. 정지 시작 위치로 검증할 값을 정한 뒤 다음 블록을
그대로 실행하고, 질문이 나오면 측정한 숫자를 입력한다.

```bash
read -r -p '측정한 traffic_stop_y_ratio 입력 (0보다 크고 1.10 이하): ' FMA_TRAFFIC_STOP_Y_RATIO
export FMA_TRAFFIC_STOP_Y_RATIO
python3 -c 'import os; v=float(os.environ["FMA_TRAFFIC_STOP_Y_RATIO"]); assert 0.0 < v <= 1.10, v' && \
  printf '%s\n' "$FMA_TRAFFIC_STOP_Y_RATIO" > $HOME/FMA_ws/traffic_stop_y_ratio.txt
python3 -c 'import os; p=os.path.expanduser("~/FMA_ws/traffic_stop_y_ratio.txt"); v=float(open(p).read()); print("저장된 임계값:", v)'
```

이제 `RUNBOOK_full_operation_20260830.md`의 초보자 절차로 이동한다.

---

## 아래는 원리·튜닝·문제 해결 상세

차선 · GPS(waypoint) · 회피 · 긴급정지와 **신호등/정지선 측정**을 launch 하나로 띄운다.
`RUNBOOK_lane_gps.md`(신호등 없는 구성)의 상위집합이며, 신호등이 붙으면서 달라지는
것만 이 문서가 따로 다룬다. 판단 코어는 **운영 C++** 이다 — 생성 C(MBD) 검증은
`RUNBOOK_mbd_lane_gps.md` 쪽이고 **두 launch 를 동시에 띄우지 말 것**.

> **동시 실행 금지**: `stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py`,
> `stack_avoid` 의 field_session 계열, `MBD_lane_gps_can.launch.py`.
> estop·mgm·bridge·scan 이 중복되면 CAN TX 가 두 배로 나가고 무엇이 명령했는지
> 사후에 못 가른다.

---

## 0. 시나리오 커버리지 — 무엇이 돌고 무엇이 안 도나

CLAUDE.md §1 의 시나리오 6종 기준:

| 시나리오 | 이 런북 | 담당 스택 |
|---|---|---|
| 차선 주행 | ✅ | `stack_lane` → LANE |
| GPS(waypoint) 주행 | ✅ | `stack_gps` → WAYPOINT |
| 장애물 회피 | ✅ | `stack_avoid` → AVOID |
| 신호등·정지선 인지·임계값 측정 | ✅ **정지 요구는 비활성** | `stack_traffic` 측정 로그 |
| 신호등·정지선 실제 정지 | ❌ | `RUNBOOK_full_operation_20260830.md` |
| 돌발 장애물 긴급 정지 | ✅ | `stack_estop` → v_ref 0 |
| **라이다 주차** | ❌ | `stack_parking` — 아래 참조 |

**주차를 뺀 이유는 "안 켰다"가 아니라 "켤 것이 없다"이다.** 이 launch 에 parking
producer 가 없어 `/perception/parking` 발행자가 0 이고, PARKING 스테이트로 가는
전이 조건(`GPS 주차구간 AND 주차공간 인식`)이 성립하지 않는다. 4-LiDAR ICP 파이프라인은
아직 PR #45(draft)로 main 밖에 있다(P0 6건 중 후방 각도 1건만 해소).

**신호등 정지는 스테이트가 아니다** (CLAUDE.md §4 원칙). 운영 런북에서 게이트를
활성화한 경우 적색+정지선은 `v_ref=0`으로만 반영되고 스테이트는 LANE/WAYPOINT
그대로 유지된다. 이 측정 런북에서는 게이트가 꺼져 있어 신호등으로 `v_ref=0`이 되지 않는다.

---

## 1. 이 PC 준비 — 최초 1회 (현장 아님)

네 가지가 다 초록이어야 현장에 나간다. 하나라도 빠지면 **증상이 엉뚱한 곳에서 터진다.**

> **경로는 전부 `$HOME/FMA_ws/...` 절대경로로 적는다.** 이 문서 초판은 `src/...` 상대
> 경로로 적어 두어 **워크스페이스 루트 밖에서 치면 셸이 "그런 파일이나 디렉터리가
> 없습니다" 로 거부한다** (2026-08-30 현장에서 실제로 걸렸다). 스크립트가 없는 게
> 아니라 **cwd 가 다른 것**이고, 그렇게 말한 것도 스크립트가 아니라 셸이다 —
> `/bin/bash: 줄 1: src/...: 그런 파일이나 디렉터리가 없습니다`. 현장에서는 터미널
> 5개가 각자 다른 위치에서 열리므로 cwd 를 전제하지 않는다.

### 1-1. 신호등 실행 의존성 — `torchvision` · `ultralytics`

신호등만 `ultralytics`(→`torchvision`)를 쓴다. 차선은 안 쓰므로 **차선이 멀쩡한데
신호등만 죽는** 형태로 온다.

```bash
ros2 run stack_traffic stack_traffic_ml_preflight
```

`ML_RUNTIME_READY` (exit 0) 가 아니면 설치 절차는 **HANDOVER §2.3** 에 있다. 요점만:
torch 와 torchvision 은 **같은 채널·같은 세대**여야 하고(2.12 ↔ 0.27), 둘 다
`--no-deps` 로 넣는다. 빼면 `numpy`·`Pillow`·`opencv-python` 이 딸려 들어와 ROS 쪽
`cv2 4.5.4` 를 가린다.

> 이 사전점검은 **설치·삭제·업데이트를 하지 않는다.** 무엇이 어긋났는지만 말한다.

### 1-2. CAN FD

```bash
$HOME/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh --check   # sudo 불필요
```

`✔ 점검 통과` 가 아니면 `sudo $HOME/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh` 한 번.
**MTU 16 = classic 이면 브리지가 dSPACE 의 64B 프레임을 못 받는다** — 값의 단일
진실 원천은 `can_setup/can_params.sh` 하나이고, `install.sh` 가 udev·systemd·networkd
세 경로를 거기서 생성한다(손으로 복사하면 한쪽만 낡는 사고가 반복됐다).

### 1-3. udev 규칙

`HANDOVER §2.5`. **OAK-D 규칙(`80-movidius.rules`)이 빠지면 `lsusb` 엔 카메라 2대가
보이는데 depthai 는 0대를 열거한다** — 신호등·차선이 동시에 죽는다.

### 1-4. 센서 배치 전수 점검

```bash
python3 $HOME/FMA_ws/src/multi_lidar_fusion/tools/check_sensors.py --no-ros
```

"무엇이 살아 있나"가 아니라 **확정 배치와 같은가**를 본다. 심링크 7종·IMU·GPS·CAN·
OAK-D 2대·허브 분리까지 한 번에 본다. 4-LiDAR `/lidar/*/scan` 검사는 주차/융합용
별도 드라이버가 떠 있어야 하며, 이 통합 launch의 전방 `/scan`과 같은 물리 포트를
중복 점유할 수 있으므로 이 런북의 사전점검에서는 실행하지 않는다.

---

## 2. 현장 준비 — 베이스 · 코스 · 구간

`RUNBOOK_lane_gps.md` §0~§2 와 동일하다. 여기 옮겨 적지 않는다.

- 베이스 좌표: `stack_gps/tools/base_station/BASE_SURVEY.md` · `BASE_MOVE.md`
- 어느 코스가 어느 베이스 것인가: `BASE_LOCATIONS.md`
- 구간 찍기(`정지 지점`·`회피 허용`·`GPS 전용`): `RUNBOOK_avoid_field_test.md` §2-1
- **RTK 워밍업 5~10분** — B1·V1 을 먼저 켜고 나머지 준비를 한다

**신호등 때문에 추가로 필요한 현장 조건:**

- [ ] 시연 신호등이 **적색=정지 / 초록=재출발** 타입인가 (2026-08-09 팀장 확정).
      `resume_on_green` 이 실차 표준이고 launch 가 그렇게 띄운다.
- [ ] 정지선이 노면에 있는가 — 적색만으로는 안 선다(§0 판정식).
- [ ] 신호등 카메라(`14442C10B167CFD200`)가 **상단 시야**로 물려 있는가.
- [ ] 야간에는 기존 흰색 외에 CLAHE 국소 대비와 평행 에지 쌍이 동작하지만, 정지선이
  카메라 화면 밖이면 어떤 조건도 검출할 수 없다. 하단 ROI 안에 실제 선이 보이는가.
      차선용(`14442C105157D3D200`)과 바뀌면 둘 다 못 쓴다.

---

## 3. 터미널 구성

```
터미널 5개: B1(베이스) + V1(RTCM 중계) + V2(launch) + M(state 모니터) + V3(go)
```

B1·V1·M·V3 는 `RUNBOOK_lane_gps.md` 와 **완전히 같다.** V2 만 인자가 늘어난다.

### V1 [차량 PC]

```bash
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101
```

### V2 [차량 PC] — 두 단계

**launch 는 한 번만 띄운다.** 두 단계를 가르는 것은 launch 인자가 아니라
`ros2 run adas_mgm go --require-traffic` 다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
COURSE="$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv"
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
  waypoint_csv:="$COURSE" \
  traffic_enabled:=true \
  traffic_stop_y_ratio:=0.0
```

**한 번만 띄운다** — launch 를 두 번 띄우면
estop·mgm·bridge·scan 이 중복되고 CAN TX 가 두 배로 나간다(§0 동시 실행 금지).

- **① 정지 점검** — 기동 직후 `go` 를 주기 전까지. §4 의 확인을 여기서 한다.
- **② 측정 주행** — ①을 통과한 뒤 `ros2 run adas_mgm go --require-traffic`.

> **⚠ 2026-08-30 정정 — 토큰을 빼고 띄우는 "CAN 없는 점검"은 존재하지 않는다.**
> `REAL_VEHICLE_CONFIRM` 없이 띄우면 정지 점검이 되는 게 아니라 **launch 가 통째로
> 거부된다** (`REAL VEHICLE launch refused`). 이 게이트는 2026-08-11 `9dc3693` 부터
> 있었고, 이 런북 초판(2026-08-29)이 적어 둔 무토큰 명령은 **한 번도 실행된 적이 없다.**
>
> **`go` 전이 안전한 근거는 인자가 아니라 코드다.** 이 launch 는 `wait_go: True` 를
> 강제로 켜고(launch 678행), MGM 은 `/operator/go` 를 받기 전까지 매 틱
> `estop = true` 로 덮는다(`mgm_node.cpp:419`). estop 이면 v_ref 0 이므로
> (CLAUDE.md §4 "정지는 스테이트가 아니다") **바퀴는 안 움직인다.** 콘솔에 5초마다
> `출발 대기 중 — 점검 완료 후 ros2 run adas_mgm go 로 출발` 이 뜨는 것이 그 증거다.
>
> 이 방식이 무토큰 방식보다 오히려 낫다: CAN TX 가 실제로 나가므로 §5 의 **CAN 왕복
> 확인(`can_log.py`)을 출발 전에** 끝낼 수 있다. 대신 **물리 비상정지에 손을 올린
> 상태로** 점검할 것 — 버스는 이미 살아 있다.

> `usb_speed:=high camera_fps:=10` 은 **붙이지 않는다** — 2026-08-24(`8c251cb`)부터
> launch 기본값이다. 손으로 붙이는 걸 한 번 잊으면 OAK-D 가 USB3 로 열거돼 GNSS L1 을
> 덮고, 위성 수·HDOP·RTCM 이 전부 정상으로 보이는 채 **FIXED 만 안 잡힌다**
> (C/N0 39 → 22dB). 확인은 `--show-args | grep -A3 usb_speed`.

### 신호등 인자 (이 런북에서만 쓰는 것)

| 인자 | 기본값 | 언제 바꾸나 |
|---|---|---|
| `traffic_enabled` | **`false`** | 이 런북은 `true`. 끄면 거동이 신호등 통합 전과 **완전히 동일**하다 |
| `traffic_mxid` | `14442C10B167CFD200` | 다른 카메라로 시험할 때만 |
| `traffic_width` / `traffic_height` | `640` / `360` | 신호등이 멀어 안 잡힐 때 `1280`/`720` — 단 아래 대역폭 주의 |
| `traffic_stop_y_ratio` | **`0.0` = 측정 전용** | 값을 정한 뒤(§6) 그 값으로 |

> **⚠ `1280x720` 으로 올릴 때 — 두 카메라가 USB2 대역폭을 나눠 쓴다.**
> 비압축 BGR 3B/px 기준 차선 1280x720@10 = 27.65 MB/s, 신호등도 같으면 합계
> **55.3 MB/s** 로 USB2 실효(~40MB/s)를 넘는다. 640x360 이면 6.91 → 합계 34.6 MB/s.
> `stack_traffic` 의 대역폭 검사는 **카메라 한 대씩만** 보므로 각각은 통과하지만
> 합계는 못 본다. 올렸으면 **양쪽 fps 를 반드시 실측할 것**(§5 표).

---

## 4. ① 출발 전 측정 체인 점검 (`go` 전) — 필수 게이트

`go` 를 주기 전까지 MGM 이 estop 을 물고 있어 v_ref 0 이다 — 바퀴가 안 움직인다
(근거는 §3 의 정정 상자). 이 런북에서는 신호등 정지 게이트가 꺼져 있으므로
**신호등이 차를 세우지 않는 것이 정상**이다. 대신 아래 토픽과 판정 재료가 모두
정상인지 확인하지 못하면 측정 주행으로 넘어가지 말 것.

기동 직후 콘솔에서:

```
[stack_traffic_node] traffic_red_binary ROS 2 started | model=... camera=oak:640x360@10/rgb-only/
    mxid=14442C10B167CFD200/usb_requested=HIGH/usb_actual=HIGH ...
```

`usb_actual` 이 `HIGH` 가 아니면 fail-closed 로 노드가 죽는다(정상 동작). `mxid` 가
차선용이면 배선이 바뀐 것이다.

| 확인 | 명령 | 기대 |
|---|---|---|
| MGM 체인 관통 | `ros2 topic hz /adas/target_ref` | ~100 Hz |
| 차선 | `ros2 topic hz /perception/lane_path` | ~10 Hz |
| **신호등** | `ros2 topic hz /perception/traffic_stop` | **~10 Hz** |
| 회피 | `ros2 topic hz /perception/avoid` | ~10 Hz |
| 전방 안전 라이다 | `ros2 topic hz /scan` | ~10 Hz |
| 카메라를 손으로 가림 | `M` 터미널 | 0.5s 뒤 `→ gps` 전이 |
| 콘을 놓아 본다 | `M` 터미널 | `→ AVOID` |

**신호등 단독 확인** — 노드 로그 한 줄에 판정 재료가 전부 있다:

```
frame=000310 | yolo_run=1 yolo=1 yolo_ms=25.0 conf=0.83 bbox_src=yolo
  | red_raw=1 red_votes=3/5 red_active=1 green_raw=0 green_votes=0/5 green_active=0
  | stopline=1 stable=1 y_ratio=0.940 y_thr=0.000 y_ok=0 gate=off
  | FINAL_STOP=0 | proc_ms=48.8 fps=10.0
```

- `red_votes 3/5` 가 차야 `red_active=1` — 한 프레임 적색으로는 안 선다
- `gate=off` 는 **측정 전용**(`traffic_stop_y_ratio=0.0`)이라는 뜻이다. 이 상태에서는
  `FINAL_STOP` 이 **영원히 0** 이다 — 고장이 아니다(§6)
- ⚠ **`go` 점검 5종에 신호등은 없다**(gps·scan·ref·lane·avoid). 신호등이 죽어 있어도
  `go` 는 통과한다 — 위 `topic hz` 를 사람이 봐야 한다

---

## 5. ② 측정 주행

```
[launch] ⚠ 실주행 모드 — CAN TX 나갑니다. 물리 비상정지에 손 올릴 것
```

- 운전자는 **물리 비상정지에 손 올리고** 대기
- 소프트웨어 정지 = **V2 Ctrl-C** (종료 경로가 `can_zero` 로 목표값 0 을 보낸다)
- 출발 인가: `ros2 run adas_mgm go --require-traffic` (매 출발마다)

### 판정 기준값 — 2026-08-29 Xanadu-book5 실측

이 값에서 크게 벗어나면 무언가 어긋난 것이다.

| 항목 | 기준값 | 비고 |
|---|---|---|
| 전방 안전 라이다 `/scan` | ~10 Hz | 통합 launch가 직접 기동하는 입력 |
| `/perception/lane_path` | 9.86 Hz | |
| `/perception/traffic_stop` | 10.07 Hz | |
| `/adas/target_ref` | ~100 Hz | |
| `/vehicle/vector` | 99.999 Hz | dSPACE RX 살아 있을 때 |
| `stack_traffic` CPU | **~184 %** | 487% 면 OpenMP 스핀 수정이 안 들어간 빌드다 |
| `stack_lane` CPU | ~55 % | XPU 35.8 ms/frame |
| load average | **~1.8** | 12.0 이면 위와 같은 원인 |
| MGM 지터 `late max` | **~0.68 ms** | §7 판정: 최악지연 × 2 ≪ watchdog 30ms |

> **CPU 가 487% 로 나오면** `stack_traffic/stack_traffic/omp_runtime.py` 가 있는
> 빌드인지 본다. Intel OpenMP 가 작업 후 200ms 스핀하는 것이 10fps 주기(100ms)보다
> 길어 워커 7개가 영영 안 자던 문제다. 그 스핀이 **MGM 10ms 루프 지터를 3.34ms 로
> 밀어냈다** — 수정 후 0.68ms 로, 신호등을 끄고 돌 때(2.26ms)보다도 좋다.

### CAN 왕복 확인

```bash
python3 $HOME/FMA_ws/src/stack_avoid/tools/can_log.py --iface can0 --duration 30 --out /tmp/can.log
```

```
와이어 포맷: FD 9000          ← 단독이어야 한다. classic 이 섞이면 한쪽이 미전환
0x100 1.00/주기 · 0x101 1.00/주기 · 0x200 1.00/주기
TX 프레임/헤더 = 2.00          ← v5. 21.00 이면 옛 v3 코드가 도는 것
```

---

## 6. 신호등 임계값 정하기 — 첫 세션은 **측정 전용**

`traffic_stop_y_ratio` 와 노드 파라미터 `stopline_stop_distance_m` 이 **둘 다 0 이면 노드는 정지
요구를 만들지 않는다.** 측정만 한다. 이것이 기본값인 이유:

현장값 `0.98` 은 **옛 ROI · 고정 장착 · 0.28 m/s 이하**에서만 검증됐다. 카메라 장착
높이·각도, ROI, 주행 속도가 바뀌면 재보정 대상이다 — 이 런북은 `v_base` 1.0 m/s 로
달리므로 그 조건 밖이다.

**절차:**

1. `traffic_stop_y_ratio:=0.0`(기본) 으로 코스를 돈다. 신호등 앞을 **세우지 않고**
   천천히 통과한다.
2. rosbag의 `/rosout`에서 `y_ratio`를 추출한다. 터미널 A에서 bag을 재생하고 터미널
   B에서 메시지를 저장한다. `stable=1`이고 세우고 싶은 지점의 값을 본다.

   ```bash
   # 터미널 A
   RUN=$HOME/FMA_ws/drive_logs/run_<시각>
   ros2 bag play "$RUN/rosbag" --topics /rosout

   # 터미널 B (A를 실행하기 전에 먼저 대기시켜도 됨)
   ros2 topic echo /rosout --field msg | \
     grep --line-buffered -oE 'y_ratio=[0-9.]+' | tee "$RUN/traffic_y_ratio.txt"
   ```

3. **정차 위치의 값을 그대로 쓰지 않는다.** 그 위치에서야 감속을 시작하므로 지나친다.
   조금 이른 값(작은 y_ratio)을 고른다.
4. 선택한 값과 근거 run을 기록한다.
5. 실제 정지·재출발 검증은 `RUNBOOK_full_operation_20260830.md`를 따른다.

**정지 우선권** (CLAUDE.md §4): `긴급정지 > 신호등 정지 > 트랙 종점 > 역방향 > 지정
지점 정지 > 가속구간 > 기본 속도`. 신호등이 걸려도 estop 이 이긴다.

---

## 7. 신호등을 켜면 달라지는 것 — 운영상 3가지

**① MGM 의 traffic watchdog 이 깨어난다** (§5.7 ③).
`/perception/traffic_stop` 은 **수신 이력이 있은 뒤에만** 감시된다. 즉 신호등을
한 번 띄운 뒤 그 노드가 죽으면 0.5s 뒤 `stop_required=true` 로 보정되어 **차가 선다**
(estop 아닌 일반 감속). 안 띄우면 감시 자체가 잠들어 있어 지금까지와 동일하다.
→ 주행 중 신호등 노드만 죽이는 실험은 하지 말 것. 차가 선다.

**② 카메라가 2대가 된다.** USB2 대역폭을 나눠 쓴다(대역폭은 §3 의 계산). 전류는
2026-08-29 에 **카메라 전원을 별도 라인으로 분리**해서 갈라 놓았으므로 허브 하나로
운용한다 — `check_sensors.py` 의 허브 항목은 그날부터 경고다. 다만 카메라를 다시
버스 전원으로 물리면 RPLiDAR 가 `health OK` 인데 `/scan` 0Hz 가 되는 함정이 그대로
돌아온다(HANDOVER §3.7).

**③ CPU 여유가 준다.** §5 표의 값이 기준이다. 나머지 스택까지 붙였을 때 모자라면
`taskset -c 4,5,6` 으로 신호등을 코어에 묶는 카드가 있다(487→140% 실측). 지금은
load 1.8 이라 필요 없다.

---

## 8. 지금 막혀 있는 것 — 나가기 전에 알고 갈 것

| 항목 | 상태 | 영향 |
|---|---|---|
| **dSPACE 상태값 전부 0** | 이슈 #50 | `/vehicle/vector` 의 `x·y·yaw·v·str` 이 0. **counter 는 정상**(우리 헤더 counter 의 에코로 확인, 2026-08-29). 명령 대 실현 분석이 안 된다 |
| **dSPACE counter watchdog 미구현** | 이슈 #49 | PC 송신이 끊겨도 dSPACE 가 마지막 v_ref 를 유지한다. **V2 Ctrl-C 종료 경로(`can_zero`)를 반드시 탈 것** — 프로세스를 kill 하면 안 된다 |
| **참조점 20 → 1** | main `aa9753a`, 알고 내린 결정 | avoid 의 20점 보간이 만들던 곡률 이득이 구조적으로 사라졌다. **회피 거동 실차 재확인 필수** |
| `EstopRequest.rear_clear` 미구현 | 이기돈 | 후진 탈출이 기본 설정에서 자연히 잠긴다(`escape_after_cycles=0`) |
| 신호등 2카메라 동시 실차 | 2026-08-29 책상에서 확인 | 주행 중 동시 동작은 이 런북이 처음이다 |

---

## 9. 문제별 진단 — 신호등 관련만

`RUNBOOK_lane_gps.md` §7 이 나머지를 다룬다.

| 증상 | 확인 | 조치 |
|---|---|---|
| `stack_traffic` 기동 즉시 죽음 · `torchvision` 오류 | `ros2 run stack_traffic stack_traffic_ml_preflight` | §1-1. torch 와 짝이 맞는 torchvision |
| `AttributeError: XLinkOut` | depthai 버전 | v3 는 XLinkOut 이 없다. PR #53 이 든 빌드인지 확인 |
| 카메라를 못 염 · MxID 불일치 | 콘솔의 `mxid=` | 배선 확인. 두 카메라가 바뀌면 차선도 같이 죽는다 |
| `usb_actual` 이 `SUPER` | fail-closed 로 노드가 죽음 | 정상 동작. USB2 포트로 옮긴다 |
| `/perception/traffic_stop` 0 Hz | 노드 살아 있나 | 위 3가지 순서로 |
| `FINAL_STOP` 이 영원히 0 | 로그에 `gate=off` | **측정 전용 모드다.** §6 으로 값을 정한다 |
| 신호등이 있는데 `yolo=0` | `conf` · 거리 | 해상도를 `1280x720` 로(대역폭 주의, §3) |
| 적색인데 안 섬 | `red_votes` · `stopline` · `y_ok` | 셋이 다 서야 선다. 정지선이 안 보이면 `stopline=0` |
| 재시작 직후 신호등 로그가 없음 | `ros2 node list \| grep stack_traffic` | OAK 동시 초기화 실패는 2초 자동 respawn. 계속 없으면 출발 금지 |
| 초록인데 재출발 안 함 | `green_votes 3/5` | **fresh YOLO 초록만** 해제한다. bbox 를 놓치면 안 풀린다 |
| 갑자기 감속 정지 + traffic 로그 없음 | §7 ①의 watchdog | 신호등 노드가 죽었다. V2 재시작 |
| `stack_traffic` CPU 487% | `omp_runtime.py` 있는 빌드인가 | §5 |

---

## 10. 시험 후

`RUNBOOK_lane_gps.md` · `RUNBOOK_mbd_lane_gps.md` §7 과 동일하다. 신호등이 남기는 것:

```bash
RUN=~/FMA_ws/drive_logs/run_<시각>
ros2 bag info $RUN/rosbag | grep traffic_stop      # 신호등 요구 이력
ros2 run adas_mgm core_replay $RUN/mgm_snapshots.bin $RUN/replay.csv
column -s, -t $RUN/transitions.csv | cut -c1-160
```

`/perception/traffic_stop` 은 처음부터 bag 기록 목록에 있었다(미탑재면 비어 있게
기록됨). 이제 실제로 찬다. `mgm_snapshots.bin` 에도 `traffic_stop_required` 가 매 틱
들어가므로 **"왜 그때 섰나"를 재생으로 되짚을 수 있다.**

---

## 참조

- `RUNBOOK_full_operation_20260830.md` — 검증된 임계값으로 신호등 정지까지 수행
- `RUNBOOK_lane_gps.md` — 신호등 없는 같은 구성 (베이스·RTCM·go·일반 진단의 원본)
- `RUNBOOK_avoid_field_test.md` — 회피 판정 기준·튜닝 노브·구간 찍기 §2-1
- `RUNBOOK_mbd_lane_gps.md` — 생성 C(MBD) 검증. **동시 실행 금지**
- `HANDOVER.md` §2.3 — 신호등 실행 의존성 설치 절차 / §2.5 udev / §3.7 허브
- `stack_traffic/REQUIREMENTS.md` — 판정식·투표·정지선 게이트
- `bridge_dspace/CAN_BRINGUP.md` — CAN 단계별 검증 (RX 3단계 / TX 4단계 / 왕복 5단계)
- CLAUDE.md §4 — 스테이트 전이·우선권 / §5.7 — 인지 신선도 watchdog 6종 / §6 — 카메라
