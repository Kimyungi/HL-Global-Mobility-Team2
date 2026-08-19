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

## 2. 현장 준비 — 베이스 재측량 + 코스 재기록

장소가 바뀌었으면 순서가 있다. **베이스를 옮기면 코스도 다시 기록해야 한다**
(웨이포인트는 베이스 기준 상대 좌표계로 의미를 가진다 — `stack_gps/DRIVE_GUIDE.md §A2`).

1. 베이스 안테나 설치 → 측량 (`RUNBOOK_lane_gps.md §1`, 수렴 약 7분)
2. B1(베이스 RTCM 송출) · V1(라디오 → 로컬 TCP 중계) 기동 — `RUNBOOK_lane_gps.md §1·§2`
3. 로버 RTK **FIXED** 확인 — `python3 ~/FMA_ws/src/stack_gps/tools/rtk_probe.py --seconds 120`
   - **C/N0 를 볼 것.** 위성 수·HDOP 는 정상인데 RTK 만 무너지는 게 OAK-D USB3
     간섭의 증상이다 (39dB → 22dB). 그래서 아래 launch 는 항상
     `usb_speed:=high camera_fps:=10` 을 붙인다 (CLAUDE.md §6).
4. 코스 재기록 → 새 `waypoints_*.csv`
   - ⚠ 구간 파일(`zones_*.yaml`)은 **찍어도 이 시험에선 무시된다.** 운영 런치용으로
     같이 찍어 두는 건 상관없다 (launch 가 개수를 세어 경고를 찍어 준다).

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

이 시험의 진짜 산출물은 주행 성공/실패가 아니라 **두 구현의 차이**다.
run 폴더의 `mgm_snapshots.bin` 을 레퍼런스 C++ 코어에 재생해 같은 입력에 같은 출력이
나오는지 본다.

```bash
RUN=~/FMA_ws/drive_logs/run_mbd_<시각>
ls $RUN            # rosbag/  mgm_snapshots.bin  mgm_jitter.csv  lateral.csv

# 같은 스냅샷을 레퍼런스 C++ 코어에 재생 → CSV
ros2 run adas_mgm core_replay $RUN/mgm_snapshots.bin $RUN/core_replay.csv
```

- 스테이트 전이 시퀀스·v_ref·ref points 를 비교한다.
- **차이가 나면 그게 곧 결과다** — 어느 쪽이 맞는지는 CLAUDE.md §4 가 정한다
  (스펙의 단일 소스는 문서이고 두 구현 모두 거기서 파생한다).
- 지정 구간·AVOID 관련 차이는 **예상된 차이**다 (§0 표). 그 밖의 차이만 보고 대상.

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

**실차 미검증** — 위는 전부 합성 입력이다. 실제 카메라 신뢰도 잡음·RTK 품질 변동에서
어떻게 되는지가 이 시험의 목적이다.

---

## 참조

- `RUNBOOK_lane_gps.md` — 베이스·RTCM·출발 인가 등 공통 절차
- `README.md` §"실험용 generated backend" — 빌드 옵션 상세 (김재민)
- `src/generated/adas_mgr2/README.md` — 생성본 출처·라이선스 고지
- CLAUDE.md §5.5 — 이중 트랙 개발 전략
