# MBD(생성 C) MGM 실차 시험 런북 — 차선 + GPS 2상태

**launch: `adas_mgm/launch/MBD_lane_gps_can.launch.py`**

김재민이 Simulink 로 만든 `ADAS_MGR2` v1.68 생성 C 를 `mgm_step()` 자리에 끼워
(CLAUDE.md §5.5 이중 트랙) 레퍼런스 C++ 코어와 **같은 차·같은 코스**에서 굴려 보는
절차. 베이스 설치·RTCM 중계·출발 인가 같은 공통 절차는 `RUNBOOK_lane_gps.md` 를
그대로 따르고, 이 문서는 **MBD 시험에만 추가되는 것**을 담는다.

> **운영 런치(`REAL_VEHICLE_lane_gps_can.launch.py`)는 건드리지 않는다.** 그쪽은
> 지정 구간 3종까지 물려 있는 4상태 운영 구성이고, 이 파일은 별개다. 두 launch 를
> 동시에 띄우지 말 것 (estop·mgm·bridge 중복).

---

## 0. 무엇이 바뀌고 무엇이 안 바뀌나

바뀌는 것은 **판단 코어 하나뿐이다.** 인지 스택·ref 포맷·`bridge_dspace`·CAN 프레임·
dSPACE 는 전부 그대로다. 생성 C 도 `(ref_points, v_ref, flags)` 까지만 내놓는다 —
**CAN 은 MBD 모델의 몫이 아니다.** 양자화·프레임 분할은 계속 `bridge_dspace` 가 한다
(그래서 김재민의 PR 에서 `bridge_dspace` diff 는 0 이다).

### v1.68 이 갖고 있지 않은 것 — 시험 전에 반드시 알고 갈 것

| 없는 것 | 이 시험에서 어떻게 되나 |
|---|---|
| AVOID · PARKING 스테이트 | **stack_avoid 를 아예 안 띄운다.** 장애물은 회피가 아니라 stack_estop 정지로만 대응 |
| TTC 안전 바닥 · narrow_gap 감속 | 없음 (AVOID 가 없으므로 애초에 경로가 없다) |
| 종점(at_end) 래치 | 종점에 닿으면 **영구 fail-stop 래치** → v_ref 0. 다시 달리려면 launch 재시작 |
| 역방향 래치 | 차가 트랙을 등지면 영구 fail-stop 래치 (정지 자체는 정상 동작) |
| 지정 구간 3종 (gps_only_zone · stop_zone · avoid_zone, 2026-08-18) | **전부 무시된다.** 언덕 지정 정차 안 함, GPS 전용 구간에서도 차선 신뢰도가 높으면 LANE 으로 감 |

`DecisionBackend` 가 이 입력들을 매 틱 감시하다가 하나라도 들어오면 **영구 fail-stop
래치**(v_ref 0, 비어 있지 않은 ref)를 걸고 `[ERROR] decision backend fault latched` 를
찍는다. 즉 "몰래 다르게 굴러가는" 일은 구조적으로 없다 — 대신 **원인을 없애고 노드를
재시작해야** 다시 움직인다.

그래서 이 시험에서 **LANE ↔ WAYPOINT 전이는 차선 신뢰도 히스테리시스
(0.35 / 0.70, 50틱) + 재합류 게이트(cross ≤ 0.5m) 로만** 일어난다.

---

## 1. 빌드 — opt-in 이 필요하다 (1회)

생성 C 는 기본 빌드에 링크되지 않는다. CMake 옵션을 켜야 `mgm_node` 안에 들어간다.

```bash
cd ~/FMA_ws
colcon build --packages-up-to adas_mgm \
    --cmake-args -DADAS_MGM_ENABLE_GENERATED_BACKEND=ON -DBUILD_TESTING=ON
colcon test --packages-select adas_mgm --event-handlers console_direct+   # 4/4 통과여야 정상
```

- **CMake cache 는 값을 기억한다.** 한 번 ON 으로 빌드한 뒤 옵션을 생략해도 OFF 로
  안 돌아간다. 되돌리려면 `-DADAS_MGM_ENABLE_GENERATED_BACKEND=OFF` 를 **명시**할 것.
- ON 으로 빌드해도 **운영 launch 는 그대로 C++ 코어로 돈다.** `backend` 기본값이
  `core` 이고, 이 값은 startup-only·read-only 파라미터라 주행 중 못 바꾼다.
  생성 backend 는 `backend:=generated` + `generated_backend_acknowledge_limited_scope:=true`
  **둘 다** 있어야 켜지고, 하나라도 빠지면 core 로 몰래 폴백하지 않고 **기동 실패**한다.

---

## 2. 현장 준비 — 베이스 측량부터 코스 기록까지

**시험 장소: 한라대학교.** 직전 회피 시험(`RUNBOOK_avoid_field_test.md`)은 **원주
운전면허시험장**이었으므로 베이스를 옮겨 온다. 이번엔 한라대 좌표를 **새로 측량**한다.

### 먼저 알아 둘 것 — 좌표는 지워지지 않는다

| | 어디에 |
|---|---|
| **좌표 숫자** | `stack_gps/tools/base_station/BASE_LOCATIONS.md` — **지워지지 않는다.** 지점을 늘려 가며 쌓는 표 |
| **지금 로드된 좌표** | F9P 플래시 — **한 벌만.** 다른 지점 값을 쓰면 덮인다 |

표가 **서가**고 플래시가 **지금 펴 놓은 책 한 권**이다. 원주에서 한라대로 옮긴다고
원주 좌표가 사라지는 게 아니라, 나중에 원주로 돌아가면 표의 그 줄을 `setup_base.py` 에
다시 넣으면 그대로 복원된다.

> ⚠ **단, 표에 없는 값이 플래시에 들어 있으면 그건 진짜로 사라진다.**
> 그래서 덮기 전에 항상 §2-0 을 먼저 한다.

### 2-0. 덮기 전 백업 — 지금 들어 있는 좌표 확인 (베이스 PC)

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 read_base_position.py
```

출력된 좌표를 `BASE_LOCATIONS.md` 표와 대조한다. 원주 값
(`37.300314764 / 127.979451327 / 224.2647`)이면 이미 표에 있으니 그냥 진행.
**표에 없는 값이 나오면 출력 그대로 표에 행을 추가한 뒤** 진행할 것.

### 2-1. 안테나 고정

베이스 안테나를 세울 자리에 **최종 위치로 고정**한다. 측량 후 옮기면 좌표가 무효다.

- 하늘 시야가 트인 곳. 벽·처마 밑 금지
- **삼각대 높이까지 이번 자리 그대로 유지** — 다음에 재현해야 하므로 어디에 어떻게
  세웠는지 사진을 찍어 두고 §2-5 에 적는다

### 2-2. 베이스 모드 해제 ← 이걸 빼먹으면 측량이 영원히 안 된다

```bash
python3 setup_base.py --disable
```

EVK 가 베이스 모드(`fixType=5`)면 **측위를 안 해서 샘플이 한 개도 안 쌓인다.**
직전 현장(원주)에서 베이스로 쓰던 그 수신기이므로 반드시 먼저 푼다.

- `ublox_gps` ROS 노드(`start_rtk.sh`)가 떠 있으면 끌 것 — UART1 포트가 겹친다

### 2-3. [터미널 A] NGII VRS 보정 주입 (인터넷 필요)

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
export NGII_USER=kyg100800 NGII_PASS=ngii
python3 ntrip_inject.py --lat 37.3038 --lon 127.9073
```

- `--lat/--lon` 은 **현장 개략 좌표**다 (VRS 가 이 위치 기준으로 보정을 만든다).
  위 값이 한라대다 — 원주로 갈 땐 `--lat 37.3003 --lon 127.9795` 로 바꿀 것.
- 비밀번호 `ngii` 는 계정별 값이 아니라 **전 사용자 공통 고정값**이다(NGII 공식 FAQ).
  401 이 떠도 비번을 의심하지 말 것.
- **계정당 동시접속 1개.** 다른 PC 에서 같은 ID 로 붙어 있으면 실패한다.
- 접속만 먼저 확인하려면(F9P 불필요): `python3 ntrip_check.py kyg100800`
- 캐스터는 **RTS1** 이 기본이다 — RTS2 는 정상 계정도 401 로 거부한다(캐스터 측 문제).

### 2-4. [터미널 B] 10분 측량

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 measure_base_position.py --duration 600
```

- `carrSoln=FIXED` 가 떠야 샘플이 쌓인다. FLOAT 에 머물면 하늘 시야·NGII 접속 확인
- 끝나면 **위도 / 경도 / 타원체고**와 각 표준편차가 출력된다.
  표준편차가 몇 cm 를 넘으면 다시 (경고가 뜬다)
- 출력 맨 아래에 `setup_base.py` 실행 커맨드가 그대로 찍힌다 — 그걸 쓰면 된다

### 2-5. 좌표 등록 ← 여기서 안 적으면 다음에 못 쓴다

`BASE_LOCATIONS.md` 표에 행을 추가한다. **측량 직후 바로.**

| 채울 칸 | 예 |
|---|---|
| 위치 ID | `halla_20260819` |
| 장소 | 한라대학교 |
| 안테나 설치 | (어디에 어떻게 세웠는지 — 다음에 재현할 사람이 읽는다) |
| 위도/경도/타원체고 | §2-4 출력 그대로 |
| 측량일 / 상태 | 2026-08-19 / 현재 사용 |

### 2-6. 베이스 모드 설정 (플래시 저장)

```bash
python3 setup_base.py --lat <2-4 위도> --lon <2-4 경도> --height <2-4 타원체고>
```

`TMODE3=FIXED` + 항법 1Hz + UART2 를 RTCM3 전용 출력으로 전환하고 **플래시에 저장**한다.
이후엔 전원만 넣으면 베이스로 동작한다.

- 검증: 스크립트가 `fixType=5 (TIME — 베이스 정상)` 을 확인해 준다

### 2-7. [B1, 베이스 PC] RTCM 송출 — 운용 내내 켜 둠

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py --radio /dev/ttyRadio
```

정상 판정: 10초마다 `RTCM ~500 B/s`. `0 B/s ⚠` 가 계속되면 거의 항상 케이블·포트 문제.

### 2-8. [V1, 차량 PC] 라디오 → 로컬 TCP 중계 — 운용 내내 켜 둠

```bash
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101
```

### 2-9. 로버 RTK FIXED 확인

```bash
python3 ~/FMA_ws/src/stack_gps/tools/rtk_probe.py --seconds 120
```

**C/N0 를 볼 것.** 위성 수·HDOP 는 정상인데 RTK 만 무너지는 게 OAK-D USB3 간섭의
증상이다(39dB → 22dB). 그래서 이 시험의 launch 는 항상 `usb_speed:=high camera_fps:=10`
을 붙인다 (CLAUDE.md §6).

### 2-10. 코스 기록

베이스를 새로 측량했으므로 **코스도 새로 기록해야 한다** — 옛 한라대 코스
(`straight_1_20260811` 등)는 그때 베이스 좌표 기준이라 그대로는 못 쓴다.

```bash
# stack_gps 노드(V2)는 꺼둘 것 — FST 포트를 한 프로세스만 쓸 수 있다
cd ~/FMA_ws/src/stack_gps/tools/waypoints
python3 record_waypoints.py --host 127.0.0.1 --name mbd_1 --spacing 0.3
```

요령: FIXED 확인 후 출발 / 시작점 3초 정지 / 주행보다 느리게, 조향 부드럽게 /
**급커브는 조향 70~80%만** (풀조향으로 기록하면 추종 보정 여유가 0 이 되어 커브
바깥으로 이탈한다) / 폐곡선이면 시작·끝 3cm 이내면 합격 / "FIX 아님" 경고가 떴던
run 은 버리고 다시.

```bash
python3 live_view.py --csv ../../waypoints/waypoints_mbd_1_*.csv   # 품질 눈검사
```

- ⚠ 구간 파일(`zones_*.yaml`)은 **찍어도 이 시험에선 무시된다** (v1.68 미구현).
  운영 런치용으로 같이 찍어 두는 건 상관없다 — launch 가 개수를 세어 경고를 찍는다.
- `BASE_LOCATIONS.md` 의 "위치 ↔ 코스 대응" 표에 새 코스를 추가할 것.

---

## 2.5 위치를 옮길 때마다 할 일 (이미 측량한 지점끼리)

한 번 등록된 지점끼리 오갈 때는 **재측량하지 않는다.** 표의 숫자를 그대로 다시 넣는
것이 정답이다 — 같은 자리에서 재측량해도 값이 cm 단위로 달라져 그만큼 코스가 밀린다.

| # | 할 일 |
|---|---|
| 1 | 안테나·삼각대를 그 지점의 **등록된 자리·높이**로 설치 (표의 "안테나 설치" 칸) |
| 2 | `python3 read_base_position.py` — 지금 들어 있는 값이 표에 있는지 확인 |
| 3 | `python3 setup_base.py --lat <그 지점 위도> --lon <경도> --height <타원체고>` |
| 4 | `python3 rtcm_server.py --radio /dev/ttyRadio` (B1) · `--port /dev/ttyRadio --tcp-port 2101` (V1) |
| 5 | launch 의 `waypoint_csv:=` 를 **그 지점에서 기록한 코스**로 지정 |

**안 해도 되는 것**: 재측량 · NGII 접속(인터넷 불필요) · 로버 설정 변경 ·
dSPACE/CAN 쪽 아무것도.

```bash
# 원주 운전면허시험장으로 돌아갈 때
python3 setup_base.py --lat 37.300314764 --lon 127.979451327 --height 224.2647
#   코스: waypoints_straight_1_20260818_160511.csv (지정 구간 3종 포함)

# 한라대 8/1 지점 (삼각대 자리를 재현할 수 있을 때)
python3 setup_base.py --lat 37.303841799 --lon 127.907284433 --height 183.9014
#   코스: waypoints_straight_1_20260811_193556.csv 등
```

⚠ **틀린 짝을 쓰면 조용히 실패한다** — RTK FIXED 는 멀쩡히 뜨는데 위치만 통째로
밀린다. 코스 CSV 첫 줄의 lat/lon 으로 장소를 구분할 수 있다
(한라대 `37.3041/127.9075`, 원주 `37.3006/127.9791`).

---

## 3. ① 정지 상태 전이 확인 — CAN 없음 (필수 게이트, 5분)

**바퀴가 안 움직인다.** `bridge_dspace` 가 아예 안 뜨므로 `/adas/target_ref` 를 아무도
읽지 않는다. 여기서 스테이트가 안 변하면 주행으로 넘어가지 말 것.

```bash
# V2 [차량 PC]
ros2 launch adas_mgm MBD_lane_gps_can.launch.py \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<새 코스>.csv \
    usb_speed:=high camera_fps:=10
```

기동 직후 콘솔에서 **이 두 줄**을 확인한다:

```
[launch] backend = generated (ADAS_MGR2 v1.68) — LANE/WAYPOINT 2상태만
[launch] bench 모드 — bridge_dspace 미기동, 바퀴 안 움직입니다
[INFO] [mgm_node]: decision backend=generated (ADAS_MGR2 v1.68 LANE/WAYPOINT bench only)
```

세 번째 줄이 `decision backend=core` 면 **빌드 opt-in 이 안 된 것이 아니라** 파라미터가
안 먹은 것이다 (opt-in 이 없으면 노드가 기동 실패한다).

```bash
# M [차량 PC] — 시험 내내 켜 둔다. 전이 이력이 곧 시험 기록
ros2 run adas_mgm state

# V3 — 출발 인가. ★ --skip-avoid 필수 (stack_avoid 를 안 띄운다)
ros2 run adas_mgm go --skip-avoid
```

| 확인 | 기대 |
|---|---|
| `ros2 topic hz /adas/target_ref` | 100 Hz |
| 카메라를 손으로 가림 | 차선 신뢰도 ↓ → 0.5s 뒤 `→ gps` 전이 |
| 다시 열어 줌 | 신뢰도 ↑ → 0.5s 뒤 `→ 차선` 복귀 |
| `[ERROR] decision backend fault latched` | **안 떠야 한다.** 뜨면 §5 |

> 차를 손으로 밀어 트랙 밖으로 빼면 재합류 게이트(cross ≤ 0.5m)가 걸려 차선으로
> 안 돌아오는 것도 여기서 확인할 수 있다.

---

## 4. ② 실주행

①을 통과한 뒤에만. 확인 토큰을 주면 `bridge_dspace` + 종료 시 `can_zero` 가드가 붙는다.

```bash
ros2 launch adas_mgm MBD_lane_gps_can.launch.py \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/<새 코스>.csv \
    usb_speed:=high camera_fps:=10
```

```
[launch] ⚠ 실주행 모드 — CAN TX 나갑니다. 물리 비상정지에 손 올릴 것
```

- 운전자는 **물리 비상정지에 손 올리고** 대기 (field_session 관례).
- 소프트웨어 정지 = **V2 Ctrl-C** — 종료 시 `can_zero` 가 dSPACE 목표값 0 을 송신한다
  (dSPACE watchdog 미구현, CLAUDE.md §3 ⚠).
- 속도는 운영과 같은 `v_base` 1.0 m/s 다 (params.yaml). 첫 run 은 낮춰서 보고 싶으면
  `--ros-args` 가 아니라 params.yaml 을 고칠 것 — **운영 런치와 값이 갈리면 back-to-back
  비교가 무의미해진다.**
- **트랙 종점에 닿으면 서고 안 움직인다** (fail-stop 래치). 정상이다. 다음 바퀴를
  돌리려면 launch 를 재시작한다. 이게 이 시험의 가장 큰 운용상 불편이다.

---

## 5. `fault latched` 가 떴을 때

로그에 이유가 그대로 찍힌다. 원인을 없애고 **launch 재시작** (파라미터로는 못 푼다).

| 메시지 | 원인 | 조치 |
|---|---|---|
| `the production gps_at_end latch is unsupported` | 트랙 종점 도달 | 정상. 재시작 |
| `the production wrong-way latch is unsupported` | 차가 트랙을 등짐 | 차를 트랙 방향으로 놓고 재시작 |
| `AVOID input is unsupported` | stack_avoid 가 떠 있음 | 이 launch 는 안 띄운다 — 운영 런치가 같이 떠 있는지 확인 |
| `PARKING input is unsupported` | stack_parking 이 떠 있음 | 같음 |
| `generated v_ref exceeds the configured maximum` | 모델 출력이 `max(v_base, v_accel_zone)` 초과 | **모델 버그다.** 덤프 들고 김재민에게 |
| `generated output did not honor E-stop` | estop 인데 v_ref ≠ 0 | **모델 버그다.** 최우선 보고 |

아래 두 개가 뜨면 실주행 중단하고 김재민에게 넘긴다 — 안전 계약 위반이다.

---

## 6. 시험 후 — 레퍼런스 코어와 back-to-back (§5.5)

이 시험의 진짜 산출물은 주행 성공/실패가 아니라 **두 구현이 같은 입력에 같은 판단을
했는가**다. `parity_replay` 가 run 폴더의 `mgm_snapshots.bin` 을 **두 구현에 동시에**
재생해 바로 답을 낸다.

```bash
RUN=~/FMA_ws/drive_logs/run_mbd_<시각>
ls $RUN            # rosbag/  mgm_snapshots.bin  mgm_jitter.csv  lateral.csv

ros2 run adas_mgm parity_replay $RUN/mgm_snapshots.bin $RUN/parity_diff.csv
```

```
═══ back-to-back 재생 (CLAUDE.md §5.5) ═══
재생      : 12480 틱 (124.8 s)
비교 대상 : 12480 틱 / 범위 밖 0 틱 (0.0%)

스테이트 전이 (범위 밖 틱 포함 — 실제로 흘러간 이력 그대로)
  레퍼런스 : LANE@0.00s → WAYPOINT@34.21s → LANE@51.20s
  생성     : LANE@0.00s → WAYPOINT@34.21s → LANE@51.20s
  → 틱 단위까지 일치 (전이 3회 / 3회)

필드별 불일치 (범위 안 틱만, 허용오차 3e-5)
  state 0 / path_source 0 / immediate_stop 0 / v_ref 0 / n_points 0 / ref_points 0

판정: 완전 일치
```

- **전이 시각이 틱 단위로 같은지**가 이 시험의 핵심 질문이다. 한 틱만 어긋나도
  히스테리시스 카운팅이 다르다는 뜻이라 위 표에 그대로 드러난다.
- 불일치가 있으면 `parity_diff.csv` 에 틱·필드·양쪽 값이 남는다. 첫 불일치 틱을
  `rosbag` 의 같은 시각과 맞춰 보면 어떤 입력에서 갈렸는지 나온다.
- 어느 쪽이 맞는지는 CLAUDE.md §4 가 정한다 — 스펙의 단일 소스는 문서이고 두 구현
  모두 거기서 파생한다.
- **"범위 밖" 틱은 예상된 차이다** (§0 표: AVOID·PARKING·종점·역방향). 판정에서 자동
  제외되며, 전 구간이 범위 밖이면 "비교 불가"로 나온다.
- 종료 코드: `0` 일치 / `1` 차이 있음 / `2` 비교 불가·재생 실패.

> 레퍼런스 코어만 재생해 CSV 로 보고 싶으면 기존 `core_replay` 가 그대로 있다.

---

## 7. 사전 검증 기록 (2026-08-19, 실차 전)

실차에 나가기 전 벤치에서 확인한 것:

| 항목 | 결과 |
|---|---|
| 최신 main 위 병합 빌드 | 성공 (`-DADAS_MGM_ENABLE_GENERATED_BACKEND=ON -DBUILD_TESTING=ON`) |
| ctest | 4/4 통과 — 패리티 900틱 `mismatches=0` 포함 |
| ROS wrapper 전이 (합성 lane/gps) | core `['LANE','WAYPOINT','LANE']` = generated **일치** |
| `wait_go` 게이트 | 인가 전 v_ref 0 / 인가 후 1.0 — **기동 중 fault 래치 없음** |
| `at_end` | 두 backend 모두 v_ref 0. generated 는 fault 래치 + ERROR 로그 |
| `parity_replay` (2상태 덤프 1162틱) | 전이 5회 **틱 단위까지 일치**, 필드 불일치 0 → `완전 일치` |

**실차 미검증** — 위는 전부 합성 입력이다. 실제 카메라 신뢰도 잡음·RTK 품질 변동에서
어떻게 되는지가 이 시험의 목적이다.

---

## 참조

- `RUNBOOK_lane_gps.md` — 베이스·RTCM·출발 인가 등 공통 절차
- `README.md` §"실험용 generated backend" — 빌드 옵션 상세 (김재민)
- `src/generated/adas_mgr2/README.md` — 생성본 출처·라이선스 고지
- CLAUDE.md §5.5 — 이중 트랙 개발 전략
