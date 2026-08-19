# MBD(생성 C) MGM 실차 시험 런북 — 차선 + GPS 2상태

**launch: `adas_mgm/launch/MBD_lane_gps_can.launch.py`**

김재민이 Simulink 로 만든 `ADAS_MGR2` v1.68 생성 C 를 `mgm_step()` 자리에 끼워
(CLAUDE.md §5.5 이중 트랙) 레퍼런스 C++ 코어와 **같은 차·같은 코스**에서 굴려 보는
절차. **이 문서 하나로 시험이 돌아간다** — 터미널 명령은 전부 복붙 가능하게 §3 에
모아 두었다. 베이스 좌표 측량·지점 이동처럼 시험 이전의 준비는
`stack_gps/tools/base_station/` 문서로 갈라 두었다 (§2).

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

## 2. 현장 준비 — 베이스와 코스

**시험 장소: 한라대학교.** 직전 회피 시험(`RUNBOOK_avoid_field_test.md`)은 **원주
운전면허시험장**이었으므로 베이스를 옮겨 온다. 이번엔 한라대 좌표를 **새로 측량**한다.

베이스 절차는 `stack_gps/tools/base_station/` 에 있다 — 이 런북에 옮겨 적지 않는다.

| 하려는 것 | 문서 |
|---|---|
| **한라대 좌표를 새로 측량** (이번에 할 일) | [`BASE_SURVEY.md`](../stack_gps/tools/base_station/BASE_SURVEY.md) |
| 이미 등록된 지점끼리 옮기기 (원주 복귀 등) | [`BASE_MOVE.md`](../stack_gps/tools/base_station/BASE_MOVE.md) |
| 어느 좌표·어느 코스가 등록돼 있나 | [`BASE_LOCATIONS.md`](../stack_gps/tools/base_station/BASE_LOCATIONS.md) |

시험 시작 전 갖춰져야 할 것:

- [x] 한라대 좌표 측량 완료 + `BASE_LOCATIONS.md` 에 등록 (`halla_20260819`)
- [ ] 그 좌표로 `setup_base.py` 실행 (`fixType=5` 확인)
- [x] **새 코스 CSV** — `waypoints_halla_univ_20260819_182657.csv` (303점, 전 구간 RTK FIXED, 2026-08-19 기록)
- [ ] 로버 RTK **FIXED** 확인 (`rtk_probe.py`, C/N0 39dB 이상)
- [ ] `-DADAS_MGM_ENABLE_GENERATED_BACKEND=ON` 빌드 (§1)

> ⚠ 구간 파일(`zones_*.yaml`)은 **찍어도 이 시험에선 무시된다** (v1.68 미구현).
> 운영 런치용으로 같이 찍어 두는 건 상관없다 — launch 가 개수를 세어 경고를 찍는다.

---

## 3. 터미널 구성

```
터미널 5개: B1(베이스) + V1(RTCM 중계) + V2(launch) + M(state 모니터) + V3(go)
```

| 이름 | 어디서 | 역할 | 언제 |
|---|---|---|---|
| **B1** | 베이스 PC | RTCM 보정 송출 (라디오) | 현장 도착 직후 ~ 철수 |
| **V1** | 차량 PC | 라디오 → 로컬 TCP 중계 | 현장 도착 직후 ~ 철수 |
| **V2** | 차량 PC | MBD launch (인지 + MGM [+ CAN]) | 시험 시작 ~ 종료 |
| **M** | 차량 PC | 스테이트 모니터 | **시험 내내** — 전이 이력이 곧 시험 기록 |
| **V3** | 차량 PC | 출발 인가 | 매 출발마다 |

### B1 [베이스 PC]

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py --radio /dev/ttyRadio
```

정상: 10초마다 `RTCM ~500 B/s`. `0 B/s ⚠` 는 거의 항상 케이블·포트 문제.

### V1 [차량 PC]

```bash
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101
```

### V2 [차량 PC] — 두 단계

**① 정지 상태 전이 확인 (CAN 없음)** — 바퀴가 안 움직인다.

```bash
ros2 launch adas_mgm MBD_lane_gps_can.launch.py \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv \
    usb_speed:=high camera_fps:=10
```

**② 실주행** — ①을 통과한 뒤에만. 토큰을 주면 `bridge_dspace` + `can_zero` 가드가 붙는다.

```bash
ros2 launch adas_mgm MBD_lane_gps_can.launch.py \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv \
    usb_speed:=high camera_fps:=10
```

`usb_speed:=high camera_fps:=10` 은 **항상 붙인다** — OAK-D 를 USB2 에 묶어 GPS 간섭을
막는다 (CLAUDE.md §6, USB3 면 C/N0 가 39 → 22dB 로 무너진다).

### M [차량 PC]

```bash
ros2 run adas_mgm state
```

### V3 [차량 PC]

```bash
ros2 run adas_mgm go --skip-avoid
```

★ **`--skip-avoid` 필수** — 이 시험은 `stack_avoid` 를 안 띄우므로 회피 점검을
건너뛰어야 인가가 난다.

### 종료

**V2 에서 Ctrl-C.** 종료 경로를 타야 `can_zero` 가 dSPACE 목표값 0 을 송신한다
(dSPACE watchdog 미구현, CLAUDE.md §3 ⚠).

---

## 4. ① 정지 상태 전이 확인 — 필수 게이트 (5분)

V2 를 **CAN 없이**(토큰 없이) 띄운 상태. `bridge_dspace` 가 아예 안 뜨므로
`/adas/target_ref` 를 아무도 읽지 않는다. **여기서 스테이트가 안 변하면 주행으로
넘어가지 말 것.**

기동 직후 콘솔에서 이 세 줄을 확인한다:

```
[launch] backend = generated (ADAS_MGR2 v1.68) — LANE/WAYPOINT 2상태만
[launch] bench 모드 — bridge_dspace 미기동, 바퀴 안 움직입니다
[INFO] [mgm_node]: decision backend=generated (ADAS_MGR2 v1.68 LANE/WAYPOINT bench only)
```

세 번째 줄이 `decision backend=core` 면 빌드 opt-in 문제가 아니라 **파라미터가 안 먹은
것**이다 (opt-in 이 없으면 노드가 아예 기동 실패한다).

M 과 V3 를 띄운 뒤:

| 확인 | 기대 |
|---|---|
| `ros2 topic hz /adas/target_ref` | 100 Hz |
| 카메라를 손으로 가림 | 차선 신뢰도 ↓ → 0.5s 뒤 `→ gps` 전이 |
| 다시 열어 줌 | 신뢰도 ↑ → 0.5s 뒤 `→ 차선` 복귀 |
| `[ERROR] decision backend fault latched` | **안 떠야 한다.** 뜨면 §6 |

**전이가 일어나면 V2 콘솔에 이유가 한 줄로 뜬다** — 같은 줄이 `transitions.csv` 에도
쌓인다.

```
전이 LANE → WAYPOINT @6.97s | lane→waypoint: 차선 신뢰도 < lane_conf_exit 가 n_cycles 연속
  | lane_conf=0.200 lane_low_cnt=49/50 lane_high_cnt=0/50 cross_track=0.100 gps_n=20
    gps_only_zone=0 at_end=0 estop=0 traffic_stop=0 obstacle=0 avoidable=0 ...
```

- `lane_low_cnt`/`lane_high_cnt` 는 **생성 모델 내부 카운터**(`ADAS_MGR2_DW`)를 그대로
  읽은 값이다 — 레퍼런스 코어와 같은 자리에서 같은 이름으로 나온다
- 규칙 이름 뒤에 **`★ 스펙 불일치`** 가 붙으면 §4 조건이 성립하지 않았는데 전이한
  것이다. **이게 이 시험이 찾는 것** — 보이면 그 틱 번호와 `transitions.csv` 를 들고
  김재민에게 넘긴다
- 카운터는 **전이 직전 틱** 값이다(전이가 나면 코어가 리셋하므로 이후 값은 0)

> 차를 손으로 밀어 트랙 밖으로 빼면 재합류 게이트(cross ≤ 0.5m)가 걸려 차선으로
> 안 돌아오는 것도 여기서 확인할 수 있다.

---

## 5. ② 실주행

V2 를 토큰과 함께 다시 띄운다 (§3). 콘솔에 이 줄이 떠야 한다:

```
[launch] ⚠ 실주행 모드 — CAN TX 나갑니다. 물리 비상정지에 손 올릴 것
```

- 운전자는 **물리 비상정지에 손 올리고** 대기 (field_session 관례)
- 소프트웨어 정지 = **V2 Ctrl-C**
- 속도는 운영과 같은 `v_base` 1.0 m/s 다 (params.yaml). 첫 run 을 낮춰 보고 싶으면
  `--ros-args` 가 아니라 params.yaml 을 고칠 것 — **운영 런치와 값이 갈리면
  back-to-back 비교가 무의미해진다**
- **트랙 종점에 닿으면 서고 안 움직인다** (fail-stop 래치). 정상이다. 다음 바퀴를
  돌리려면 V2 를 재시작한다 — 이 시험의 가장 큰 운용상 불편이다

---

## 6. `fault latched` 가 떴을 때

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

## 7. 시험 후 — 레퍼런스 코어와 back-to-back (§5.5)

이 시험의 진짜 산출물은 주행 성공/실패가 아니라 **두 구현이 같은 입력에 같은 판단을
했는가**다. `parity_replay` 가 run 폴더의 `mgm_snapshots.bin` 을 **두 구현에 동시에**
재생해 바로 답을 낸다.

```bash
RUN=~/FMA_ws/drive_logs/run_mbd_<시각>
ls $RUN     # rosbag/  transitions.csv  mgm_snapshots.bin  mgm_jitter.csv  lateral.csv

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

`transitions.csv` 는 그 run 에서 **실제로 무슨 이유로 바뀌었는지**의 기록이다.
`parity_replay` 가 "두 구현이 같은가"를 보고, 이쪽은 "왜 바뀌었나"를 본다 — 둘을
같이 보면 차이가 났을 때 원인 틱으로 바로 갈 수 있다.

```bash
column -s, -t $RUN/transitions.csv | cut -c1-160     # 눈으로 훑기
awk -F, 'NR>1 && $6==0' $RUN/transitions.csv         # 스펙 불일치만
```

> 레퍼런스 코어만 재생해 CSV 로 보고 싶으면 기존 `core_replay` 가 그대로 있다.

---

## 8. 사전 검증 기록 (2026-08-19, 실차 전)

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

- [`base_station/BASE_SURVEY.md`](../stack_gps/tools/base_station/BASE_SURVEY.md) — 베이스 좌표 측량
- [`base_station/BASE_MOVE.md`](../stack_gps/tools/base_station/BASE_MOVE.md) — 지점 이동
- [`base_station/BASE_LOCATIONS.md`](../stack_gps/tools/base_station/BASE_LOCATIONS.md) — 좌표·코스 레지스트리
- `stack_gps/DRIVE_GUIDE.md` §A2 — 코스 기록 상세
- `README.md` §"실험용 generated backend" — 빌드 옵션 상세 (김재민)
- `src/generated/adas_mgr2/README.md` — 생성본 출처·라이선스 고지
- CLAUDE.md §5.5 — 이중 트랙 개발 전략
