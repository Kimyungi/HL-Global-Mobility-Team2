# MBD(생성 C) MGM 실차 시험 런북 — ADAS_MGR2 v1.88 4상태

**launch: `adas_mgm/launch/MBD_lane_gps_can.launch.py`**

김재민이 Simulink 로 만든 `ADAS_MGR2` v1.88 생성 C 를 `mgm_step()` 자리에 끼워
(CLAUDE.md §5.5 이중 트랙) 레퍼런스 C++ 코어와 **같은 차·같은 코스**에서 굴려 보는
절차. **이 문서 하나로 시험이 돌아간다** — 터미널 명령은 전부 복붙 가능하게 §3 에
모아 두었다. 베이스 좌표 측량·지점 이동처럼 시험 이전의 준비는
`stack_gps/tools/base_station/` 문서로 갈라 두었다 (§2).

> **운영 런치(`REAL_VEHICLE_lane_gps_can.launch.py`)는 건드리지 않는다.** 그쪽은
> 후진 탈출까지 포함한 운영 C++ 코어 구성이고, 이 파일은 생성 C 검증 전용이다. 두 launch 를
> 동시에 띄우지 말 것 (estop·mgm·bridge 중복).

---

## 0. 무엇이 바뀌고 무엇이 안 바뀌나

바뀌는 것은 **판단 코어 하나뿐이다.** 인지 스택·ref 포맷·`bridge_dspace`·CAN 프레임·
dSPACE 는 전부 그대로다. 생성 C 도 `(ref_points, v_ref, flags)` 까지만 내놓는다 —
**CAN 은 MBD 모델의 몫이 아니다.** 양자화·프레임 분할은 계속 `bridge_dspace` 가 한다
(그래서 김재민의 PR 에서 `bridge_dspace` diff 는 0 이다).

### v1.88 범위와 최신 main의 차이

v1.88에는 다음이 들어 있다.

- `LANE / WAYPOINT / AVOID / PARKING` 4상태
- TTC 즉시 정지, 좁은 회피로 감속, AVOID 속도·시간 상한과 복귀 hold
- 종점·역방향 래치
- 지정 정지, 회피 허용, GPS 전용 구간
- PARKING의 음수 속도

모델 생성 직후 main에 추가된 **후진 탈출(rear escape)**만 없다. 따라서 생성 backend는
`escape_after_cycles=0`에서만 기동한다. 이 런치도 값을 0으로 고정한다. 후진 탈출까지
비교하려면 `estop_rear_clear`, `escape_*`, `MGM_SRC_ESCAPE`를 모델에 넣어 재생성해야 한다.
PARKING은 오프라인 패리티에는 포함되지만 이 런치에 parking producer가 없어 실차 항목은
아니다.

---

## 1. 빌드 — opt-in 이 필요하다 (1회)

생성 C 는 기본 빌드에 링크되지 않는다. CMake 옵션을 켜야 `mgm_node` 안에 들어간다.

```bash
cd ~/FMA_ws
colcon build --packages-up-to adas_mgm \
    --cmake-args -DADAS_MGM_ENABLE_GENERATED_BACKEND=ON -DBUILD_TESTING=ON
colcon test --packages-select adas_mgm --event-handlers console_direct+   # 전부 통과해야 정상
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

- [ ] 한라대 좌표 측량 완료 + `BASE_LOCATIONS.md` 에 등록
- [ ] 그 좌표로 `setup_base.py` 실행 (`fixType=5` 확인)
- [x] **새 코스 CSV** — `waypoints_halla_univ_20260819_182657.csv` (303점, 전 구간 RTK FIXED, 2026-08-19 기록)
- [ ] 로버 RTK **FIXED** 확인 (`rtk_probe.py`, C/N0 39dB 이상)
- [ ] `-DADAS_MGM_ENABLE_GENERATED_BACKEND=ON` 빌드 (§1)
- [ ] (`avoid_zone_only:=true` 로 띄울 때만) 회피 구간을 찍었나 — 아래 참조

> 구간 파일(`zones_*.yaml`)의 정지·회피·GPS 전용 구간은 이제 v1.88 입력으로 전달된다
> (v1.68 때처럼 무시되지 않는다).
>
> **`avoid_zone_only` 기본값은 `false` 다 — 어디서나 회피한다** (CLAUDE.md §4 의 기본).
> 그냥 띄우면 회피 구간을 안 찍어도 AVOID 가 정상 동작한다.
>
> ⚠ 회피를 **지정 구간에만** 쓰려는 코스(원주 운전면허시험장 등)에서만
> `avoid_zone_only:=true` 를 붙이고, **그때는 구간을 반드시 찍을 것.** 켜 놓고 구간이
> 없으면 회피가 전면 차단되고 장애물 앞에서 estop 으로만 선다 — 2026-08-25 한라대에서
> 실제로 그렇게 됐다(§8 참조). launch 가 그 조합을 기동 시 큰 경고로 찍는다.

### 2-1. 구간 찍기 — MBD 에서도 명령이 똑같다

v1.88 은 지정 구간 3종을 **직접 입력으로 받는다**(v1.68 이 무시하던 것). 그래서 찍는
방법도 운영과 완전히 동일하다 — 절차 원본은
[`RUNBOOK_avoid_field_test.md` §2-1](RUNBOOK_avoid_field_test.md) 이고, 여기 옮겨
적지 않는다. **V2(launch)를 켜 둔 채** 새 터미널에서:

```bash
# 지정 정지 지점 (점 하나)
ros2 run stack_gps mark_zone stop --note "언덕 오르막"

# 회피 허용 구간 (시작·끝 짝)
ros2 run stack_gps mark_zone avoid_start
ros2 run stack_gps mark_zone avoid_end

# GPS 전용 구간 (시작·끝 짝)
ros2 run stack_gps mark_zone gps_only_start
ros2 run stack_gps mark_zone gps_only_end
```

- **CAN 토큰 없이 띄운 V2(①)로 찍는 것이 안전하다.** `mark_zone` 은 구독만 하고,
  bench 라도 `stack_gps_node` 는 똑같이 돈다. 조이스틱으로 그 자리에 정차한 뒤 찍는다.
- 트랙 CSV 옆 `zones_<코스>.yaml` 한 파일에 쌓이고, **MBD 와 운영 런치가 그 파일을
  공유한다** — 한 번 찍으면 양쪽에 다 적용된다.
- RTK **FIXED(quality=4)** 표본만 쓴다. 30개(약 3초) 중앙값이라 FIXED 가 아니면 안 찍힌다.
- ⚠ **찍은 뒤 V2 를 재시작할 것.** `stack_gps` 는 구간 파일을 **기동 시 한 번만** 읽는다
  (`_setup_zones`). 재시작하면 콘솔에
  `구간 파일 로드: ... (정지 N · 회피 N · GPS전용 N)` 이 뜬다 — 그 숫자로 확인한다.

**⚠ `gps_only_zone`(구간) 과 `gps_only:=true`(런치 인자)는 다른 것이다.**

| | 무엇 | 범위 | 되돌리기 |
|---|---|---|---|
| `gps_only_zone` | 구간 파일에 찍은 **장소** | 그 구간 안에서만 WAYPOINT 고정, 벗어나면 정상 히스테리시스 | 구간 파일에서 지운다 |
| `gps_only:=true` | 런치 인자 — `lane_conf_exit`·`lane_conf_return` 을 **둘 다 2.0** 으로 덮어씀 | **run 전체.** 신뢰도 최대가 1.0 이라 LANE 전이가 구조적으로 불가능 | 인자를 뺀다 |

후자를 모르고 붙이면 `lane_path` 가 정상 수신되는데도 run 내내 gps 라
**인지 고장으로 오진하기 쉽다**(2026-08-15 run_0815_162102 실측). 구간을 쓰려는
거라면 `gps_only` 는 붙이지 말 것.

**동작은 레퍼런스 코어와 같은 것을 확인했다** (2026-08-25, back-to-back 1200틱):
차선 신뢰도를 0.9(복귀 임계 0.70 위)로 **계속 높게** 둔 채 구간만 켰다 껐다 했을 때 —

| | 레퍼런스 | 생성 v1.88 |
|---|---|---|
| 구간 진입 → WAYPOINT | t=200 (즉시, 히스테리시스 없음) | t=200 |
| 구간 안 (600틱) | 신뢰도 0.9 여도 WAYPOINT 유지 | 동일 |
| 구간 이탈 → LANE | t=849 (**50틱 새로 채운 뒤**) | t=849 |
| 필드 불일치 | — | **0 / 1200틱** |

이탈 직후 0틱 만에 LANE 이 되면 CLAUDE.md §4 위반이다(`lane_high_cnt` 를 구간 안에서
0 으로 묶는 규칙). 생성 C 도 그 규칙을 갖고 있다.

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
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv
```

ros2 launch adas_mgm MBD_lane_gps_can.launch.py \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv \
    ydlidar_params:=$HOME/ydlidar_ros2_ws/src/ydlidar_ros2_driver/params/Tmini-Plus-SH.yaml


**② 실주행** — ①을 통과한 뒤에만. 토큰을 주면 `bridge_dspace` + `can_zero` 가드가 붙는다.

```bash
ros2 launch adas_mgm MBD_lane_gps_can.launch.py \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv
```

> **`usb_speed:=high camera_fps:=10` 을 더 붙이지 않는다** — 2026-08-24(커밋
> `8c251cb`)부터 그게 **launch 기본값**이다. OAK-D 를 USB2 에 묶어 GPS 간섭을 막는
> 설정인데(CLAUDE.md §6, USB3 면 C/N0 가 39 → 22dB 로 무너진다), 손으로 붙이는 걸
> 한 번 잊으면 위성 수·HDOP·RTCM 이 전부 정상으로 보이는 채 FIXED 만 안 잡혀
> 원인을 찾기 어려워 안전한 쪽을 기본으로 뒤집었다. 확인은
> `--show-args | grep -A3 usb_speed` 로 한다.
> USB3 가 정말 필요하면 그때만 명시한다: `usb_speed:=super camera_fps:=30`.

### M [차량 PC]

① CAN 없는 bench:

```bash
ros2 run adas_mgm state --ros-args \
    -r /adas/target_ref:=/bench/adas/target_ref
```

② CAN 실주행:

```bash
ros2 run adas_mgm state
```

### V3 [차량 PC]

① CAN 없는 bench — MGM 출력이 `/bench/adas/target_ref` 로 격리되므로
④ target_ref 점검에 remap 을 준다.

```bash
ros2 run adas_mgm go --ros-args \
    -r /adas/target_ref:=/bench/adas/target_ref
```

② CAN 실주행 — 운영 토픽 그대로다.

```bash
ros2 run adas_mgm go
```

- `stack_avoid` 도 기동되므로 **`--skip-avoid` 를 쓰지 않는다.** 의도적으로 차선
  노드만 끄는 시험이라면 `--skip-lane` 만 추가한다.
- ★ **bench 에서 `--force` 를 쓰지 말 것.** `--force` 는 점검 5종을 **전부** 버린다 —
  RTK FIXED·차선·라이다까지 안 보고 인가한다. bench 에서 걸리는 건 target_ref
  하나뿐이고 그건 위 remap 으로 정확히 해결된다. §4 가 "필수 게이트"인 이유가
  그 점검들이다.

### 종료

**V2 에서 Ctrl-C.** 종료 경로를 타야 `can_zero` 가 dSPACE 목표값 0 을 송신한다
(dSPACE watchdog 미구현, CLAUDE.md §3 ⚠).

---

## 4. ① 정지 상태 전이 확인 — 필수 게이트 (5분)

V2 를 **CAN 없이**(토큰 없이) 띄운 상태. `bridge_dspace`는 기동하지 않고,
MGM 출력도 `/bench/adas/target_ref`로 강제 remap된다. 다른 launch에서 이전
bridge가 남아 있어도 운영 `/adas/target_ref`를 받을 수 없다. **여기서
스테이트가 안 변하면 주행으로 넘어가지 말 것.**

기동 직후 콘솔에서 이 세 줄을 확인한다:

```
[launch] backend = generated (ADAS_MGR2 v1.88) — 4상태, rear escape 비활성
[launch] bench 모드 — MGM 출력을 /bench/adas/target_ref로 격리
[INFO] [mgm_node]: decision backend=generated (ADAS_MGR2 v1.88 four-state; rear escape disabled)
```

세 번째 줄이 `decision backend=core` 면 빌드 opt-in 문제가 아니라 **파라미터가 안 먹은
것**이다 (opt-in 이 없으면 노드가 아예 기동 실패한다).

M 과 V3 를 띄운 뒤:

| 확인 | 기대 |
|---|---|
| `ros2 topic hz /bench/adas/target_ref` | 100 Hz |
| 카메라를 손으로 가림 | 차선 신뢰도 ↓ → 0.5s 뒤 `→ gps` 전이 |
| 다시 열어 줌 | 신뢰도 ↑ → 0.5s 뒤 `→ 차선` 복귀 |
| 차 앞에 콘을 놓아 본다 (**회피 구간 안에서** — §2 함정) | `→ AVOID`, 완료 또는 `avoid_max_cycles` 에서 `→ WAYPOINT` |
| 정지 구간 입력 | 일반 감속으로 0 도달 후 설정 시간 정차·재출발 |
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
- **트랙 종점에 닿으면 at_end 래치로 정지**한다. 해제는 실제 E-stop의
  `estop_latch_release` 계약을 따른다. watchdog 보정값으로는 래치가 풀리지 않는다.
- rear escape는 이 모델에 없으므로 장시간 E-stop이 유지돼도 자동 후진하지 않는다.
- **dSPACE RX 가 살아 있는지 콘솔에서 확인할 것** (2026-08-25 신설). `can_bridge_node`
  가 5초마다 `tx=... rx=... cycles` 를 찍는다. `rx` 가 0 이면 경고로 올라온다:

  ```
  [WARN] [can_bridge_node]: tx=500 cycles rx=0 — dSPACE RX 무수신 (can0). ...
  ```

  RX 는 2026-08-25 이전 **전 구간에서 0건**이었다(옛 bag 의 `/vehicle/vector` Count: 0).
  토픽·기록 설정은 처음부터 있었고 dSPACE 송신만 없었다. 이제 들어오면
  `vehicle_vector.csv` 에 100Hz 로 쌓이고 bag 의 `/vehicle/vector` 도 함께 찬다.
  **rx 가 0 이어도 주행 자체는 된다** — 그래서 지금까지 아무도 못 알아챘다.

---

## 6. `fault latched` 가 떴을 때

로그에 이유가 그대로 찍힌다. 원인을 없애고 **launch 재시작** (파라미터로는 못 푼다).

아래는 `decision_backend.cpp` 의 문자열 그대로다 — 로그에서 grep 하면 바로 걸린다.

| 메시지 | 원인 | 조치 |
|---|---|---|
| `generated path source does not match the four-state output` | 스테이트와 선택 경로 소스가 어긋남 | **모델/어댑터 버그.** 덤프 보존 |
| `generated \|v_ref\| exceeds all configured and requested speeds` | 설정·요청 속도 어느 것보다도 큰 v_ref | **모델 버그다.** 덤프 보존 |
| `generated output did not honor E-stop` | estop 인데 v_ref ≠ 0 | **모델 버그다. 최우선 보고** |
| `generated AVOID output did not honor the TTC stop threshold` | TTC < `ttc_stop` 인데 즉시 정지 안 함 | **모델 버그다. 최우선 보고** |
| `generated immediate-stop output has nonzero v_ref` | 즉시 정지 플래그와 v_ref 모순 | **모델 버그다.** 덤프 보존 |
| `generated negative v_ref is not a monotonic PARKING exit ramp` | PARKING 밖인데 음수 v_ref | **모델 버그다.** 덤프 보존 |
| `generated output contains a non-finite reference point` | ref 에 NaN/inf | **모델 버그다.** 덤프 보존 |
| `generated backend input contains an invalid path` / `... a non-finite decision value` | 인지 스택이 깨진 값을 보냄 | **인지 쪽 문제다.** 어느 스택인지 bag 으로 확인 |
| `generated backend requires a non-negative AVOID speed suggestion` | `stack_avoid` 가 음수 v_suggest | 같음 (stack_avoid 확인) |

굵게 표시한 셋(**E-stop · TTC · 최우선 보고**)이 뜨면 **실주행을 중단**하고 김재민에게
넘긴다 — 안전 계약 위반이다.

**기동 자체가 실패하는 경우는 fault 래치가 아니다** (노드가 아예 안 뜬다):

| 메시지 | 원인 |
|---|---|
| `backend=generated requires escape_after_cycles=0 because ADAS_MGR2 v1.88 has no rear-escape input or state` | `escape_after_cycles` 가 0 이 아님. 이 launch 는 0 을 고정하므로, 뜬다면 `params.yaml` 이나 다른 launch 를 쓰고 있는 것 |
| `backend=generated requires generated_backend_acknowledge_limited_scope=true` | 확인 파라미터 누락 (core 로 몰래 폴백하지 않는다) |
| `rear escape is unsupported by ADAS_MGR2 v1.88` | 위와 같은 원인이 매 틱 감시에서 잡힌 것 |

---

## 7. 시험 후 — 레퍼런스 코어와 back-to-back (§5.5)

이 시험의 진짜 산출물은 주행 성공/실패가 아니라 **두 구현이 같은 입력에 같은 판단을
했는가**다. `parity_replay` 가 run 폴더의 `mgm_snapshots.bin` 을 **두 구현에 동시에**
재생해 바로 답을 낸다.

```bash
RUN=~/FMA_ws/drive_logs/run_mbd_<시각>
ls $RUN     # rosbag/  transitions.csv  mgm_snapshots.bin  mgm_jitter.csv  lateral.csv
            # vehicle_vector.csv  ← dSPACE RX 피드백 (②실주행에서만)

ros2 run adas_mgm parity_replay $RUN/mgm_snapshots.bin $RUN/parity_diff.csv
```

```
═══ back-to-back 재생 (CLAUDE.md §5.5) ═══
재생      : 12480 틱 (124.8 s)
비교 대상 : 12480 틱 (rear escape 비활성)

스테이트 전이
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
- v1.88 범위인 LANE·WAYPOINT·AVOID·PARKING·지정 구간·종점·역방향은 전 틱을
  비교한다. `escape_after_cycles!=0`인 덤프만 명시적으로 비교 불가다.
- 종료 코드: `0` 일치 / `1` 차이 있음 / `2` 비교 불가·재생 실패.
- ⚠ **덤프는 기록한 빌드와 같은 ABI 여야 한다.** `CoreSnapshot` 에 필드가 붙으면 옛
  덤프는 `덤프 헤더 불일치` 로 거절된다 — 2026-08-24 후진 탈출로 `estop_rear_clear`
  가 붙었으므로 **그 이전 run(예: `run_mbd_0819_*`)은 지금 빌드로 재생되지 않는다.**
  이번 시험 덤프는 같은 빌드로 뜨므로 문제없다. 옛 run 을 다시 보려면 그 시점
  커밋으로 `parity_replay` 를 빌드해야 한다.

`vehicle_vector.csv` 는 **dSPACE 가 실제로 무엇을 했는지**의 기록이다 —
`{stamp_s, counter, x, y, yaw, v, str}`. PC 가 시킨 것(`transitions.csv`·덤프의 v_ref·
ref)과 나란히 놓으면 **명령 대 실현**을 볼 수 있다. CLAUDE.md §3 의 조향 실현율
(`실제δ/명령δ`) 논의가 전부 이 비교다. `counter` 로 dSPACE 측 자체 로그와도
틱 단위 정합이 된다 (`bag_index = counter − off`, `tools/dspace_merge.py`).

```bash
head -3 $RUN/vehicle_vector.csv
python3 -c "import csv,sys; r=list(csv.DictReader(open(sys.argv[1]))); \
  print(f'{len(r)}행, v {min(float(x[\"v\"]) for x in r):.2f}~{max(float(x[\"v\"]) for x in r):.2f} m/s')" \
  $RUN/vehicle_vector.csv
```

⚠ 파일이 **비어 있으면**(헤더만) RX 가 안 들어온 것이다 — §5 의 `rx=0` 경고를 볼 것.
bench(①)에는 `bridge_dspace` 자체가 안 떠서 이 파일이 아예 생기지 않는다.

`transitions.csv` 는 그 run 에서 **실제로 무슨 이유로 바뀌었는지**의 기록이다.
`parity_replay` 가 "두 구현이 같은가"를 보고, 이쪽은 "왜 바뀌었나"를 본다 — 둘을
같이 보면 차이가 났을 때 원인 틱으로 바로 갈 수 있다.

```bash
column -s, -t $RUN/transitions.csv | cut -c1-160     # 눈으로 훑기
awk -F, 'NR>1 && $6==0' $RUN/transitions.csv         # 스펙 불일치만
```

> 레퍼런스 코어만 재생해 CSV 로 보고 싶으면 기존 `core_replay` 가 그대로 있다.

---

## 8. 사전 검증 기록

실차에 나가기 전 벤치에서 확인한 것:

**김재민 PR #46 이 보고한 것:**

| 항목 | 결과 |
|---|---|
| 최신 main 위 병합 빌드 | 성공 (`-DADAS_MGM_ENABLE_GENERATED_BACKEND=ON -DBUILD_TESTING=ON`) |
| 생성 C 단독 Linux x86-64 GCC 빌드 | 경고를 오류로 처리해 통과 |
| 합성 랜덤 back-to-back | escape 비활성 50,000틱, 4상태 포함, 불일치 0 |
| 4상태 결정론 패리티 | LANE/WAYPOINT/AVOID/PARKING 및 구간·래치·속도 우선권 포함 |
| rear escape | **미지원** — `escape_after_cycles=0`만 허용 |

**이쪽 PC 에서 다시 확인한 것 (2026-08-25, 병합 전):**

| 항목 | 결과 |
|---|---|
| `colcon build` ON + `BUILD_TESTING=ON` | 성공 |
| `colcon test` | **7/7 통과** |
| `generated_lane_waypoint_parity_test` | `ticks=900 mismatches=0` |
| `generated_four_state_parity_test` | `ticks=383 mismatches=0 assertions=0` |
| 운영 빌드(OFF) 에 생성 심볼 누출 | `nm` 결과 **0개** — 운영 경로는 영향 없음 |
| `--show-args` | `usb_speed=high` · `camera_fps=10` · `avoid_zone_only=false` 기본값 확인 |
| `go` remap (가짜 발행자로 재현) | remap 주면 인가, 안 주면 target_ref FAIL — §3 V3 절차대로 동작 |

### 실차 1회차 — 2026-08-25 한라대 (`run_mbd_0825_162752`, 39.4s)

| 항목 | 결과 |
|---|---|
| LANE ↔ WAYPOINT 전이 | 정상 2회 (0.49s LANE→WAYP, 22.51s WAYP→LANE), 둘 다 `spec_match=1` |
| AVOID | **한 번도 안 걸림** — 원인은 모델이 아니라 launch 설정(아래) |
| dSPACE RX | **0건** — `vehicle_vector.csv` 헤더만, bag `/vehicle/vector` Count 0 |

**AVOID 미진입 원인 (모델 결함 아님).** `avoid_zone_only=1` 로 떠 있었는데 한라대
코스에는 구간 파일이 없어 `gps_avoid_zone` 이 0% 였다. 인지는 정상이었다 —
`avoid_obstacle_detected` 35.9%, t=25.28s 에 `avoidable` 참 + `avoid_path.n=1` 로
**1.49초 동안 회피 가능**이라고 말했는데 게이트가 막았고, 그대로 직진하다
t=27.07s 에 estop 으로 섰다.

→ 2026-08-25 에 `avoid_zone_only` 기본값을 `false` 로 되돌렸다 (원주 전용 선택이
전 코스 기본이 돼 있던 것). 지금은 그냥 띄우면 어디서나 회피한다.

**같은 덤프를 두 설정으로 재생해 확인**(`core_replay`) — 게이트만 끄면 그날 그 자리에서
회피한다. 인지 입력은 한 글자도 안 바뀌었다:

```
게이트 켬(그날)   AVOID    0틱   LANE@0.00 → WAYP@0.49 → LANE@22.51
게이트 끔(복구)   AVOID 1200틱   LANE@0.00 → WAYP@0.49 → LANE@22.51 → AVOID@25.28 → WAYP@37.28
```

1200틱에서 나온 것은 `avoid_max_cycles`(12s) 상한이다 — 장애물이 계속 잡혀 있어
`maneuver_done` 이 안 섰다는 뜻이니, 실차에서 회피를 다시 볼 때 확인할 항목이다.

**아직 실차 미검증**: AVOID·PARKING 스테이트, 지정 구간 3종의 실차 동작,
back-to-back parity(`parity_replay`).

---

## 참조

- [`base_station/BASE_SURVEY.md`](../stack_gps/tools/base_station/BASE_SURVEY.md) — 베이스 좌표 측량
- [`base_station/BASE_MOVE.md`](../stack_gps/tools/base_station/BASE_MOVE.md) — 지점 이동
- [`base_station/BASE_LOCATIONS.md`](../stack_gps/tools/base_station/BASE_LOCATIONS.md) — 좌표·코스 레지스트리
- `stack_gps/DRIVE_GUIDE.md` §A2 — 코스 기록 상세
- `README.md` §"실험용 generated backend" — 빌드 옵션 상세 (김재민)
- `src/generated/adas_mgr2/README.md` — 생성본 출처·라이선스 고지
- CLAUDE.md §5.5 — 이중 트랙 개발 전략
