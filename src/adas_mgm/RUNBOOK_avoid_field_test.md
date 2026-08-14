# 회피 통합 실차 시험 런북 (차선+GPS+회피)

**launch: `adas_mgm/launch/REAL_VEHICLE_lane_gps_can.launch.py`** — 2026-08-12 통합된
avoid 스테이트의 **첫 실차 검증** 절차. 기본 주행 절차는 `RUNBOOK_lane_gps.md`를 그대로
따르고, 이 문서는 회피 시험에 **추가되는 것**만 담는다.

> 전제: 통합 코드(2026-08-12)가 빌드된 상태. avoidable 판정·maneuver_done 클리어런스·
> v_suggest 계수는 sim 검증만 된 **초기값**이다 — 이 시험이 그 값을 확정한다.
> **이기돈 동석 권장** (stack_avoid 판정 계수의 현장 확인).

**기대 동작 (CLAUDE.md §4):** 주행 중 전방 장애물 감지 + 회피 성립 → `AVOID(회피)` 전이
→ 회피 목표점 추종으로 비켜 감 → 통과 완료(maneuver_done) → **WAYPOINT 복귀**(GPS 트랙
재합류) → 차선 신뢰도 회복 시 LANE 재전이.

```
터미널 5개: B1(베이스) + V1(RTCM 중계) + V2(통합 launch) + M(state 모니터) + V3(go)
```

| 이름 | 어디서 | 역할 | 비고 |
|---|---|---|---|
| B1·V1·V2·V3 | — | RUNBOOK_lane_gps.md와 동일 | V2 인자만 §2 참조 |
| **M** | 차량 PC | `ros2 run adas_mgm state` | **시험 내내 켜 둔다** — 전이 이력이 곧 시험 기록 |

---

## 0. 콘(장애물) 배치 — 기하가 절반이다

| 항목 | 값 | 근거 |
|---|---|---|
| 장애물 | 콘 1개 (라이다 스캔 높이 6.5cm에 걸리는 물체) | lidar_mount.z_m |
| 위치 (종방향) | **트랙 중간 지점** | 출발 과도·종점 래치 구간 회피 |
| 위치 (횡방향) | **경로 정중앙 ±0.2m 이내** | 감지 통로 반폭 0.46m (차폭/2+측방여유) — 벗어나면 장애물로 안 봄 |
| 콘 양옆 열림 | **각 1.2m 이상** (벽·콘 없음) | 회피 오프셋 상한 1.0m + 여유 |
| 예상 AVOID 진입 | 콘 앞 ~3.0m | avoid.detect_range_m |
| 예상 통과 이격 | 콘 옆 ~0.55m | 콘반경0.1+차폭/2+lateral_margin 0.15 |
| 예상 복귀(done) | 감지 소실 후 **7~8초** | (마지막 감지거리+전장 0.85+clear_margin 0.3)÷0.5m/s. 3m에서 감지→2.4m쯤 소실이면 3.55m÷0.5 ≈ 7.1s (2026-08-14 상한 1.0→3.0 상향 후) |

**안전 바닥 (회피가 실패해도):** stack_estop 정적 0.7m / 동적 1.2m 정지 + MGM TTC<0.8s
즉시 정지. 운전자는 **물리 비상정지에 손 올리고** 대기 (field_session 관례).
소프트웨어 정지 = **V2 Ctrl-C** — 종료 시 can_zero가 dSPACE 목표값 0을 송신한다 (2026-08-12 가드).

## 1. 정지 인지 확인 — 차 출발 전 (2분, 필수 게이트)

V2까지 띄운 상태(`wait_go` 정지 대기 — go는 아직)에서 콘을 차 전방 2~3m 경로 위에 놓고:

```bash
ros2 topic echo /perception/avoid --once
```

| 확인 | 기대값 |
|---|---|
| obstacle_detected | true |
| avoidable | **true** ← 이게 false면 주행 시작 금지 |
| ttc | 3.5~6 (거리÷0.5) |
| points | 1개, y 부호 = 열린 쪽 |
| v_suggest | **0.5** (= 목표속도. 회피 중 감속 금지 — 조향 하한 0.5 m/s, 이기돈 실측) |

콘을 좌/우로 옮기면 `points[0].y` 부호가 따라 바뀌는지, 치우면 detected=false로 돌아오는지 확인.
avoidable이 안 서면 → 콘이 통로(±0.46m) 밖이거나 ttc<1.5 (§5 표).

## 2. 단계 1 — 첫 회피 주행 (gps_only, 변수 축소)

첫 회피는 차선 전이를 빼고 **avoid↔waypoint만** 검증한다 (야간 차선 오검출 변수 제거):

```bash
# V2
ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
    REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
    waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_straight_1_20260811_193556.csv \
    gps_only:=true usb_speed:=high camera_fps:=10
# M (내내 켜 둠)
ros2 run adas_mgm state
# V3 — 점검 5종(avoid 포함) 통과 시 출발
ros2 run adas_mgm go
```

> **`usb_speed:=high camera_fps:=10`은 GPS를 위한 필수 인자다 (2026-08-14).**
> 빼면 OAK-D가 USB3(SuperSpeed)로 열거되고 그 방사 잡음이 GNSS L1을 덮어
> **RTK FIXED가 아예 안 잡힌다** (같은 위치에서 C/N0 39dB↔22dB). USB2는 대역폭이
> ~40MB/s라 `camera_fps:=10`을 반드시 동반해야 한다 (CLAUDE.md §6).
> 기동 후 `stack_lane` 콘솔에 `USB 링크 속도 제한: HIGH` 경고가 떠야 적용된 것이다.
>
> **카메라를 아예 빼려면** `gps_only:=true` 대신 `lane_enabled:=false`를 쓰고
> 출발은 `ros2 run adas_mgm go --skip-lane`. `gps_only`는 LANE **전이**만 막을 뿐
> stack_lane은 그대로 떠서 사후 분석용 데이터를 남긴다 — 진짜 변수 축소는 `lane_enabled`다.

**기대 전이 로그 (M 터미널):**

```
[hh:mm:ss]   gps    (v_ref 0.50)
[hh:mm:ss] → 회피   (v_ref 0.50)      ← 콘 앞 ~3m (회피 중 감속 금지 — CLAUDE.md §3)
[hh:mm:ss] → gps    (v_ref 0.50)      ← 감지 소실 7~8초 뒤 (maneuver_done)
[hh:mm:ss]   gps    (v_ref 0.00)      ← 트랙 종점
```

복귀 후에는 **최소 3초간 gps에 머물러야** 한다 (`avoid_return_hold_cycles` 300틱,
2026-08-14 추가). 한 틱 만에 lane으로 튀면 그 수정이 안 먹은 것이다 — 재빌드 확인.

**즉시 중단 기준 (V2 Ctrl-C 또는 물리 비상정지):**
- 콘 옆을 지나기 **전에** `→ gps` 복귀 (조기 done — 콘을 향해 재수렴한다)
- AVOID에서 콘 반대쪽이 아닌 **콘 쪽으로 조향**
- 이격이 눈짐작 0.3m 미만으로 스침

## 3. 단계 2 — 성공 시 변형 2가지

1. **full 통합** — `gps_only`만 빼고 재실행 (**`usb_speed`·`camera_fps`는 유지**):

   ```bash
   ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py \
       REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX \
       waypoint_csv:=$HOME/FMA_ws/src/stack_gps/waypoints/waypoints_straight_1_20260811_193556.csv \
       usb_speed:=high camera_fps:=10
   ```

   차선 주행 중 콘 → AVOID → **WAYPOINT 복귀(최소 3초 유지)** → 차선 신뢰도 회복 시
   LANE 재전이까지. 2026-08-12 복귀 정책 개정(CLAUDE.md §4)과 2026-08-14 히스테리시스
   수정의 실차 확인이다.
2. **열림 없음(narrow)**: 콘 3개로 양옆을 막고 접근 → narrow_gap 감속(v_narrow 0.2) →
   estop 정지 확인. 회피 시도 없이 감속·정지해야 정상.

## 4. 합격 판정

| # | 항목 | 기준 |
|---|---|---|
| 1 | AVOID 진입 | 콘 앞 2.5~3.5m에서 전이, 조향 방향 = 열린 쪽 |
| 2 | 통과 이격 | 0.4m 이상 (목표 0.55m) |
| 3 | 복귀 타이밍 | 콘을 **차 뒤로 완전히 보낸 뒤** 복귀 — 조기 done 없음. 복귀 직후 콘이 다시 감지되면(ttc<1) **측면 충돌 직전**이므로 즉시 중단 |
| 4 | 복귀 궤적 | GPS 트랙으로 부드럽게 재합류 (블렌드 — 급조향 없음) |
| 5 | 종점 정지 | 회피와 무관하게 정상 작동 |
| 6 | **복귀 후 gps 체류** | **3초 이상** — 한 틱 만에 lane으로 튀면 불합격 (2026-08-14 수정 확인) |
| 7 | **RTK 유지** | run 내내 `fix_quality=4` — `lateral.csv`의 quality 열로 사후 확인 |

7개 전부 합격 → 확정 파라미터를 `stack_avoid/config/params.yaml`에 반영 + 커밋/PR.

## 5. 증상 → 튜닝 노브 (주행 중 터미널에서, 재시작 불필요)

| 증상 | 원인 후보 | 노브 (`ros2 param set /stack_avoid_node ...`) |
|---|---|---|
| **AVOID 진입은 되는데 조향 없이 직진 → estop** | run_0812_234253에서 실측 — ref 1점 + v_ref 0.44(조향 하한 0.5 미달, 이기돈 검증)의 중첩 | **수정 완료** (① 조립 20점 보간 ② avoid 중 감속 제거 — CLAUDE.md §3 재개정). 재발 시 AVOID 중 `ros2 topic echo /adas/target_ref --once`로 ref_points=20 & v_ref≥0.5 확인 |
| **막힌 쪽으로 조향** (열린 쪽 반대) | 라이다 드라이버 reversion/inverted 불일치 → 스캔 좌우 거울상 (run_0813_001140에서 규명 — 전방·거리·estop은 정상이라 회피에서만 드러남) | **수정 완료** (통합 launch가 false/false 고정). 드라이버 params를 바꿀 땐 이 두 값 유지 필수 |
| 콘 사이 빈틈을 목표로 찍음 | 통과 최소폭(차폭+2×여유=0.92m) 이상인 틈은 설계상 유효한 통로 + depth_band(0.6m) 밖 콘은 못 봄 | 틈을 막거나(콘 밀착·판자), `avoid.lateral_margin_m`↑로 최소폭 상향 (0.25 → 1.12m), `avoid.depth_band_m`↑ |
| AVOID 진입 안 함 | 콘이 통로 밖 | 콘을 경로 중앙으로 (±0.46m) — 노브보다 먼저 |
| 〃 | 감지 거리 부족 | `avoid.detect_range_m` 3.0→4.0 |
| 〃 (avoidable=false) | ttc<1.5 | `avoid.ttc_stop_s` 확인 — 낮추는 건 신중히 |
| **조기 done → 복귀 중 측면 충돌** | 클리어런스 부족. 감지 소실은 통과가 아니라 조향으로 장애물이 통로를 측방 이탈한 것 (2026-08-14 실차 2회) | `avoid.clear_gap_max_m` 확인(3.0). 더 필요하면 `avoid.clear_margin_m` 0.3→0.8 |
| 복귀가 너무 늦음 | 클리어런스 과대 | `avoid.clear_gap_max_m` ↓ (3.0 기본). 단 **낮추면 조기 복귀=측면 충돌 위험** — 2026-08-14에 1.0이었다가 실차 2회 스침 |
| **통과 대기 중 조향 표류 → 복귀 후 발산/유턴** | 통과 단계 빈 경로 → MGM이 직전 목표 감쇠 hold → 퇴화 ref (run_0813_003037: 이탈 3.9m 루프 후 역방향 가드 정지) | **수정 완료** — 통과 단계에 전방 직진 유지점 (1.5, 0) 발행 |
| 이격 부족하게 스침 | 편측 여유 부족 | `avoid.lateral_margin_m` 0.15→0.25 (이격 +0.1m) |
| 열림 있는데 narrow 정지 | 오프셋 상한 | `avoid.offset_max_m` 1.0→1.3 |
| 목표점이 프레임마다 튐 | rate limit | `avoid.target_rate_limit_mps` 3.0→2.0 |
| AVOID 중 갑자기 정지 + "avoid 신선도 초과" 로그 | stack_avoid 사망 (watchdog 정상 동작) | V2 재시작 |
| **RTK FIXED가 아예 안 잡힘 (DGPS/FLOAT 고착)** | 카메라 USB3 방사 잡음 — 위성 수·HDOP·RTCM은 정상값 그대로라 상태줄로는 안 보인다 (2026-08-14 규명) | `usb_speed:=high camera_fps:=10` 인자 확인. 그래도 안 되면 launch 끄고 `python3 ~/FMA_ws/src/stack_gps/tools/rtk_probe.py --seconds 120` 로 **C/N0** 측정 — 38dB 미만이면 안테나를 하늘 트인 쪽으로 |
| **avoid 복귀가 한 틱 만에 lane으로** | 히스테리시스 카운터 미리셋 (2026-08-14 수정) | 재빌드 여부 확인. 체류를 더 원하면 `avoid_return_hold_cycles`↑ (MGM 파라미터, 300틱=3s) |
| lane↔gps 전이가 잦음 | 차선 confidence가 임계 밴드 안에서 진동 (평균 0.47 실측) | `lane_conf_return`↑ / `n_cycles`↑. 확정 전 `core_replay`로 기존 run을 재생해 비교할 것 |

⚠ `param set`으로 바꾼 값은 **재시작하면 사라진다** — 확정값은 params.yaml에 반영해야 다음 세션에 살아남는다.

## 6. 시험 후

- rosbag·스냅샷은 `~/FMA_ws/drive_logs/run_<시각>/`에 자동 — Claude에게 "회피 run 분석"을
  요청하면 진입 시점·회피 기하·done 타이밍·전이 연속성을 뽑아준다.
- 확정 파라미터 → `stack_avoid/config/params.yaml` 반영 → 커밋/PR (이기돈 멘션).
- maneuver_done·v_suggest 계수가 초기값과 크게 다르면 `stack_avoid/stack_avoid/node.py`의
  해당 주석(★ 이기돈 검증 필요)도 실측값 근거로 갱신할 것.

## 참조

- 기본 주행 절차·RTK·문제 진단: `RUNBOOK_lane_gps.md`
- 회피 단독 시험·계수 배경: `src/stack_avoid/FIELD_TEST_PLAN.md`, `MEASUREMENTS.md`
- 스테이트 전이·우선권: CLAUDE.md §4 / 입력 watchdog: CLAUDE.md §5.7 ⑤
