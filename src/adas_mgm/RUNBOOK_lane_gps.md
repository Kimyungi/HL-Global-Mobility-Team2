# 차선+GPS+회피 통합 주행 런북

**launch: `adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py`** — lane ↔ waypoint ↔ avoid 자동 전이 (CLAUDE.md §4)

이 launch 하나가 ydlidar + stack_estop + **stack_avoid(2026-08-12 통합)** + stack_gps + stack_lane +
adas_mgm + bridge_dspace + rosbag 로깅을 전부 띄운다. GPS 단독 주행의 DRIVE_GUIDE.md(V2~V6 터미널 5개)를
대체하는 구성이며, **`stack_estop/launch/REAL_VEHICLE_stack_estop_mgm_can.launch.py` ·
stack_avoid의 field_session 계열과 동시 실행 금지** (estop/mgm/bridge/scan 중복).

**회피 동작 (CLAUDE.md §4, 2026-08-12):** 주행 중 전방 장애물 감지 + 회피 성립(TTC·측방 여유)
→ AVOID 스테이트로 자동 전이해 회피 목표점 추종 → 통과 완료(maneuver_done) → **waypoint로 복귀**
(GPS 트랙 재합류) → 차선 신뢰도 회복 시 lane 자동 재전이. 열림이 없으면(narrow) 감속(v_narrow),
TTC < 임계면 즉시 정지. `ros2 run adas_mgm state`에 `AVOID(회피)`로 표시된다.

```
터미널 4개: B1(베이스) + V1(RTCM 중계) + V2(통합 launch) + V3(출발 인가)
```

| 이름 | 어디서 | 역할 | 켜는 때 | 끄는 때 |
|---|---|---|---|---|
| **B1** | 베이스 PC | RTCM 보정 송출 (라디오) | 현장 도착 직후 | 철수할 때 |
| **V1** | 차량 PC | 라디오 → 로컬 TCP 중계 | 현장 도착 직후 | 철수할 때 |
| **V2** | 차량 PC | 통합 launch (인지+판단+CAN) | 주행 준비 시 | 세우고 싶을 때 (Ctrl-C) |
| **V3** | 차량 PC | `go` 출발 인가 (점검 4종) | 매 출발 직전 | 인가 후 자동 종료 |

---

## 0. 출발 전 점검 (현장 도착 시 1회)

```bash
# CAN — udev가 자동으로 1Mbps up. UP인지 만 확인
ip link show can0          # state UP 이면 OK. 없으면 PCAN USB 재삽입

# 장치 노드 — 라디오·로버 심볼릭 링크 확인
ls -l /dev/ttyRadio /dev/ttyRover

# dSPACE 회신 확인 (dSPACE 켜진 뒤)
candump -n 3 can0          # 0x200/0x201/0x202 프레임 보이면 OK
```

⚠ **USB 허브 주의 (2026-08-11~12 실측):** 허브가 간헐적으로 전체 재열거를 일으켜
라디오 노드가 바뀌고(V1 서버 죽음) PCAN이 순간 끊긴다. 증상이 반복되면
**PCAN부터 PC 직결 포트로 이동.** 라디오 끊김 여부는 V1의 B/s 로그로 감시:
`RTCM 없음`이 뜨면 V1을 재시작한다 (심볼릭 링크가 새 노드를 따라가므로 명령은 동일).

⚠ **RTK 워밍업:** 보정 주입 시작 후 첫 FIXED까지 **5~10분** 걸릴 수 있다
(2026-08-12 실측 약 7분 — 베이스 1층 설치 환경). B1·V1을 먼저 켜고 다른 준비를 하면 된다.

## 1. B1 [베이스 PC] — 보정 송출

```bash
cd ~/FMA_ws/src/stack_gps/tools/base_station
python3 rtcm_server.py --radio /dev/ttyRadio
```

안테나는 확정 좌표(측량 지점)에. 베이스를 옮겼으면 재측량 + 코스 재기록 (DRIVE_GUIDE §A2).

## 2. V1 [차량 PC] — 라디오 → 로컬 TCP 중계

```bash
python3 ~/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py \
    --port /dev/ttyRadio --tcp-port 2101
```

10초마다 `RTCM ~580 B/s` 로그가 정상. `0 B/s ⚠ RTCM 없음`이면 → B1 가동 여부 →
허브 끊김(위 §0) 순으로 확인.

## 3. V2 [차량 PC] — 통합 launch

```bash
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_straight_1_20260811_193556.csv
```

- `waypoint_csv`는 **필수** — 코스에 맞는 CSV로 교체 (S자: `waypoints_straight_1_20260806_191643.csv`).
- 실제 CAN TX가 나가므로 확인 토큰 없이는 거부된다.
- **`wait_go: true`로 떠서 인가 전까지 정지 대기** — launch가 떴다고 차가 바로 움직이지 않는다.
- 로그는 run마다 `~/FMA_ws/drive_logs/run_<시각>/`에 자동 저장
  (rosbag + mgm_snapshots.bin + mgm_jitter.csv + lateral.csv). rosbag만 끄려면 `record:=false`.

자주 쓰는 선택 인자:

| 인자 | 기본값 | 언제 바꾸나 |
|---|---|---|
| `lane_device` | `xpu` (인텔 iGPU, 172ms/frame) | XPU 초기화 실패 시 `cpu`로 폴백 |
| `ref_point0_lookahead_m` | `1.8` (오실레이션 잠정 최적) | 차선 추종 재튜닝 시 |
| `rtcm_host` | `127.0.0.1` | V1을 다른 호스트에서 돌릴 때 |
| `can_interface` | `can0` | — |

전제 파일 (없으면 launch가 거부/실패):
- `~/FMA_ws/src/stack_lane/config/homography.json` — 실측 캘리브레이션 (2026-08-11, RMS 4cm)
- `~/FMA_ws/src/stack_lane/models/yolopv2.pt` — gitignore 대상(156MB), 새 PC엔 수동 다운로드

## 4. 확인 (임시 터미널 — 닫아도 됨)

```bash
ros2 topic echo /vehicle/vector --once                          # dSPACE 회신 = CAN 왕복 OK
ros2 topic echo /perception/gps_path --once | grep fix_quality  # 4 = RTK FIXED
ros2 topic hz /perception/lane_path                             # ~5.8Hz (XPU 기준)
ros2 topic hz /adas/target_ref                                  # ~100Hz = MGM 체인 관통
```

**현재 스테이트 모니터** — 차선/gps 어느 쪽으로 달리는지 이름으로 표시 (주행 내내 켜두면 전이 이력이 그대로 로그가 된다):

```bash
ros2 run adas_mgm state
# [21:40:03]   차선  (v_ref 0.60 m/s)
# [21:40:11] → gps   (v_ref 0.60 m/s)     ← 차선 신뢰도 하락으로 waypoint 전이
# 전이 없어도 매 초 보고 싶으면: ros2 run adas_mgm state --all
```

v_ref가 0이면 정지 요구(긴급/신호등 등)가 이기고 있다는 뜻 — 스테이트는 그대로 유지된다 (CLAUDE.md §4).

## 5. V3 [차량 PC] — 출발 인가 (매 출발마다)

```bash
ros2 run adas_mgm go
```

점검 5종(① gps_path 수신+RTK FIXED ② lane_path 수신 ③ scan 수신 ④ target_ref 수신
⑤ avoid 수신) 통과 시 `/operator/go` 발행 → 차가 출발한다.

- GPS 단독 시험 등 차선 생략: `ros2 run adas_mgm go --skip-lane`
- 회피 점검 생략(구 launch 등): `--skip-avoid`
- 비상용 전체 생략: `--force` (실주행에선 쓰지 말 것)

## 6. 정지 / 재출발 / 철수

| 하고 싶은 것 | 방법 |
|---|---|
| 차 세우기 | **V2 Ctrl-C** — 종료 시 `can_zero`가 dSPACE 목표값 0을 송신해 세운다 (2026-08-12 가드 추가. **주의: dSPACE 자체 counter watchdog은 아직 미구현**(2026-08-09 실측) — 가드 이전 구성에선 Ctrl-C 후에도 마지막 v_ref로 계속 굴렀다) |
| 다시 주행 | V2 재실행 → §4 확인 → V3 `go` |
| 철수 | V2 → V1 → B1 순서로 Ctrl-C |

돌발 장애물 앞 1.2m 내 진입 시 stack_estop이 자동 긴급 정지 (launch에 포함, `dynamic_stop_distance_m:=1.20`).

## 7. 문제별 빠른 진단

| 증상 | 확인 | 조치 |
|---|---|---|
| V1 `RTCM 없음` | B1 가동? → `ls -l /dev/ttyRadio` | B1 켜기 / V1 재시작 (허브 끊김이면 §0) |
| fix_quality 5(FLOAT)에서 안 올라감 | V1 B/s 정상? 위성 시야? | 워밍업 5~10분 대기, 안테나 시야 확보 |
| fix_quality 0 + 빈 points | FST 안테나·케이블 | MGM watchdog이 estop 보정 (안전 동작 정상) |
| `go`에서 lane FAIL | 차선 카메라 MxID(`14442C105157D3D200`) 연결? | 카메라 재삽입, `lane_device:=cpu` 폴백 시험 |
| `/vehicle/vector` 안 옴 | dSPACE 전원? `candump can0`? | dSPACE 기동 확인, PCAN 재삽입(§0) |
| `go`에서 avoid FAIL | `ros2 topic hz /perception/avoid` (~10Hz) | stack_avoid 로그 확인 — 라이다(/scan) 죽으면 avoid도 침묵 |
| AVOID 중 정지 + "avoid 신선도 초과" 로그 | stack_avoid 사망 (MGM watchdog 정상 동작) | V2 재시작 |
| 차선 인식 5.8Hz 미만 | V2 로그에 XPU 에러? | `lane_device:=cpu`로 재실행 |
| launch가 homography 에러로 거부 | 파일 존재? | placeholder 실주행 금지 — 캘리브레이션 먼저 (stack_lane CALIBRATION_GUIDE.md) |

## 참조

- **회피 통합 실차 시험 (첫 검증 절차·판정 기준·튜닝 노브): `RUNBOOK_avoid_field_test.md`**
- GPS 단독 주행·코스 기록·현장 검증 절차: `src/stack_gps/DRIVE_GUIDE.md` (PART A)
- 차선 캘리브레이션: `src/stack_lane/CALIBRATION_GUIDE.md`
- 스테이트 전이·우선권: CLAUDE.md §4, `docs/state_machine_detail.drawio`
