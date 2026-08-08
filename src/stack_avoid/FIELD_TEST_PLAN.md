# stack_avoid 실차 측정 계획 — stage2 실측 ①②③ + 경계시험 ⓐⓑⓒ

담당: 이기돈 (ⓐⓑⓒ는 박찬미 합동) · 작성 2026-08-07

한 번의 실차 세션에서 stage2에 필요한 값을 전부 딴다. 모든 항목이 같은 구성
(`field_session.launch.py`)에서 돌고, `mode`만 바꾼다. 어떤 mode든 stack_estop이 함께
뜨고 명령 노드는 estop 게이트를 통과하므로 **안전 바닥은 세션 내내 동일**하다.

분석은 전부 `tools/analyze_field_bag.py` 하나로 끝난다 — 현장에서는 구간 라벨만
잘 남기면 된다.

---

## 0. 준비물 · 사전 확인

| | 항목 | 비고 |
|---|---|---|
| □ | 콘 (회피 대상) | ③ⓐ용. 라이다 스캔 높이 0.065m에서 보이는 크기여야 함 |
| □ | 줄자 | ③의 1m·2m·3m 배치, ②의 코스 길이 |
| □ | 연석 또는 긴 판재 | ⓒ 연속 경계용 |
| □ | 스탠드 (바퀴 들기) | ①용. 모든 mode의 첫 확인에도 사용 |
| □ | **조이스틱 전원 ON** | ★ 8/6에 이게 꺼져서 액추에이션이 죽어 `str`이 고정됐다. 측정 전부 무효가 된 원인 |
| □ | 물리 비상정지 | 손 닿는 곳 |
| □ | can0 UP (1 Mbps) | 실행 스크립트가 자동 확인 |
| □ | **dSPACE watchdog 확인** | 손상민. **미확인이면 ⓑ 보류** — CAN 끊김 시 정지가 실증되지 않았다 |
| □ | 직선 구간 20m 이상 | ② 전용. 아래 소요 참조 |

**세션 내내 다른 터미널에 구간 표시기를 띄워둘 것:**
```bash
ros2 run stack_avoid mark
```
`/test/event`가 bag에 없으면 ③ⓐⓑⓒ는 **사후 분석이 불가능**하다(어느 30초가 무슨
시험이었는지 복원할 수 없음). ①②는 스텝 자체로 구간이 갈리므로 라벨이 없어도 된다.

---

## 1. 순서 — 안전한 것부터

### ③ 감지 신뢰 거리 (차 안 움직임) — 약 15분

```bash
MODE=perception bash src/stack_avoid/tools/run_field_session.sh
```

콘을 정면 **3m → 2m → 1m** 순으로 놓고 각 위치에서 **60초 이상** 정지 상태로 관찰.
위치를 옮길 때마다 `mark` 터미널에서 `3`, `2`, `1` 입력.

- **합격**: 감지율 100%, gap 표준편차가 작을 것(수 cm)
- 3m에서 감지율이 낮으면 `detect_range_m: 3.0`이 실질 상한이 아니라는 뜻 →
  avoidable 임계 계산이 감지 범위를 넘어설 수 있으므로 반드시 기록

### ① 조향 응답 시간 (★스탠드) — 약 15분

```bash
MODE=step VREF=0.3 bash src/stack_avoid/tools/run_field_session.sh
```

바퀴를 들고 실행. 기본 시퀀스는 오프셋 `[0.46, -0.46, 0.30, -0.30]` × 3회,
유지 3s / 정렬 3s → **약 72초 후 자동 정지**.

> ★ `v_ref=0`으로는 측정할 수 없다. dSPACE MPC 지평 = 0.2 × v_ref 이므로 v_ref=0이면
> 지평이 붕괴해 조향이 반응하지 않는다. 바퀴를 들었으니 v_ref를 줘도 차는 안 움직인다 —
> ①을 스탠드에서 하는 이유가 이것이다.

- **합격**: 모든 스텝에서 `str`이 반응하고, dead time이 스텝마다 비슷할 것
- `str`이 통째로 고정이면 → 조이스틱 전원 확인. 분석 스크립트가 자동 경고한다

**8/6 로그 기준 예상값** (수동 조작이 섞인 오염된 데이터라 참고용):
dead 0.111s · 63% 0.330s · 95% 0.451s (중앙값)

### ② 측방 이동 곡선 (지상) — 약 20분

```bash
MODE=step VREF=0.3 HOLD=6.0 REPEATS=2 OFFSETS="[0.46, -0.46]" bash src/stack_avoid/tools/run_field_session.sh
MODE=step VREF=0.5 HOLD=6.0 REPEATS=2 OFFSETS="[0.46, -0.46]" bash src/stack_avoid/tools/run_field_session.sh
```

좌우 교대라 차가 위빙하며 대체로 직선을 유지한다.

| 속도 | 시퀀스 길이 | 필요 주행 거리(여유 포함) |
|---|---|---|
| 0.3 m/s | 약 39s | 약 12m + 여유 |
| 0.5 m/s | 약 39s | 약 20m + 여유 |

- **측정값**: 스텝 시점 기준 |측방변위| 0.30m·0.46m 도달까지의 **전진 거리**
- **기하 이상치**(조향 지연 제외, R=1.15m): 0.30m→**0.775m** / 0.46m→**0.920m**.
  실측이 이보다 얼마나 큰지가 곧 조향 지연분이며, 그게 avoidable 공식에 들어간다
- ⚠ 반드시 지상에서. dSPACE 추측항법은 바퀴가 떠 있어도 v를 적분하므로 스탠드에서도
  숫자는 나오지만 의미가 없다 (분석 스크립트가 경고하지만 판별은 못 한다)

### ⓞ 조향 게인 스윕 (★스탠드) — 약 15분

8/6에 회피 목표점은 맞는데 **실차 조향이 너무 작았다.** 원인은 `avoid_to_ref`가 목표점을
그대로 보내지 않고 호 위 lookahead 점으로 바꿔 보내기 때문:

```
회피 목표점      (2.76, 0.46)      ← 장애물 gap 2.0m, 측방 0.46m
실제 송신 ref    (0.400, 0.0094)   ← y가 목표의 2%, κ는 물리 한계의 13.5%
```

**회피 기하(목표점)는 그대로 두고** 조향만 키울 레버가 둘인데, dSPACE가 어느 쪽에
반응하는지 실증되지 않았다. 이 스윕이 그걸 확정한다.

| 레버 | 무엇이 바뀌나 | 범위 |
|---|---|---|
| `lookahead_m` | 호 위 어느 점을 보낼지. ref y가 커짐 (목표점 불변) | 0.4 → 2.8 (호 길이 넘으면 목표점 자체) |
| `curvature_gain` | κ만 배수. 송신 위치(x,y,yaw) 완전 불변 | 1.0 → 7.4 (7.4배가 물리 한계) |

```bash
# 터미널 1 — 세션 (스탠드에서, 정면 2m 앞에 콘)
MODE=avoid VREF=0.2 bash src/stack_avoid/tools/run_field_session.sh

# 터미널 2 — 한 번에 한 레버만
python3 src/stack_avoid/tools/gain_sweep.py --param lookahead_m    --values 0.4 1.2 2.0 2.8 --dwell 15
python3 src/stack_avoid/tools/gain_sweep.py --param curvature_gain --values 1.0 3.0 5.0 7.0 --dwell 15
```

- 스윕 도구가 구간 라벨을 자동으로 남기고, **끝나면 원래 값으로 되돌린다**(중단해도)
- 분석기가 구간별 "명령 κ·ref y vs 실제 |str|"을 표로 낸다 —
  **값이 커질 때 |str|이 같이 커지는 쪽이 실제로 듣는 레버**
- 물리 한계: κ 0.870 1/m = 최소회전반경 1.15m = 등가 조향 27.3°. 그 이상은 포화
- ★ 두 레버를 동시에 흔들지 말 것. 어느 쪽이 들었는지 알 수 없게 된다
- ★ 콘이 있어야 회피 목표점이 나온다. 없으면 clear라 직진 명령만 나감

**이 결과가 확정되면** avoidable 공식의 전제(회피가 실제로 성립하는가)가 검증되고,
②의 측방 이동 곡선도 제대로 된 조향 크기에서 재야 의미가 있다 —
**②보다 먼저 할 것.**

### ⓐ 3m 콘 회피 — estop 미발동 확인 — 약 15분

```bash
MODE=avoid VREF=0.2 bash src/stack_avoid/tools/run_field_session.sh
```
`mark`에서 `a` 입력 후 3m 전방 콘을 향해 접근.

- **합격**: 회피 목표점이 나오고 비켜 가는 동안 **estop 미발동**
- 발동하면 → 사유를 볼 것. `동적`이면 자차 운동 때문에 정지한 콘이 상대운동으로
  보인 오탐일 수 있다 → `DYNAMIC=false`로 한 번 더 돌려 정적 기준만으로 재확인
- ★ 이 mode는 `straight_when_clear=true` — 장애물이 없으면 **차가 계속 전진한다**.
  통제된 공간에서만

### ⓒ 연속 경계 접근 — 약 10분

`mark`에서 `c` 입력 후 연석/판재에 접근.

- **합격**: 회피 목표점이 나오지 않고(`narrow_gap`) 정지
- 정지 사유가 `narrow_gap(하네스)`인지 `ESTOP`인지 둘 다인지 기록 —
  분석 스크립트가 구간별로 갈라준다

### ⓑ 1m 급투입 — estop 발동 확인 — 약 10분  ⚠ **watchdog 확인 후**

`mark`에서 `b` 입력 후, 주행 중인 차 앞 1m에 콘을 투입.

- **합격**: estop 발동, `정적` 기준
- ⚠ **dSPACE watchdog이 미확인이면 이 항목은 보류.** 의도적으로 estop 조건을
  만드는 시험인데, CAN 끊김 시 정지가 실증되지 않은 상태다 (8/6에 CAN TX를 끊어도
  차가 계속 움직였음). 손상민 확인 선행

---

## 2. 사후 분석

세션마다 bag이 `~/avoid_logs/field_<mode>_<시각>/bag`에 남는다.

```bash
python3 src/stack_avoid/tools/analyze_field_bag.py ~/avoid_logs/field_step_20260807_.../bag
```

①②③ⓐⓑⓒ 중 그 bag에 있는 항목만 뽑아 출력한다. 결과를 `MEASUREMENTS.md`에 옮겨 적고,
그 값으로 avoidable 임계를 확정한다:

```
avoidable = (통과 가능 gap 존재)  AND  ttc >= (0.70 + 측방이동거리 + 조향지연 × v) / v
                                          ↑ 찬미      ↑ ②           ↑ ①
```

---

## 3. 실패 대비

| 증상 | 원인 | 대응 |
|---|---|---|
| `str`이 고정 | 조이스틱 전원 off (8/6 사례) | 전원 확인 후 재측정 |
| 조향 무반응인데 전원은 정상 | `v_ref=0` → MPC 지평 붕괴 | v_ref > 0으로 |
| 스텝 대부분이 분석에서 제외됨 | 수동 조작 혼입 | step_injector 단독으로 재측정 |
| ③ⓐⓑⓒ 분석 불가 | `/test/event` 없음 | `mark` 터미널을 띄우고 재측정 |
| `/adas/target_ref` 이중 발행 | mgm_node·dummy_ref 잔존 | 실행 스크립트가 사전 차단하지만, 세션 중 띄우지 말 것 |

## 4. 사전 점검 (하드웨어 없이)

현장에 나가기 전 4가지를 확인한다. 하나라도 깨지면 세션이 통째로 무효가 될 수 있는 것들.

**① 안전 게이트 — 5케이스**
```bash
ros2 run stack_avoid avoid_to_ref --ros-args -p straight_when_clear:=true   # 터미널 1
python3 src/stack_avoid/tools/test_estop_gate.py                            # 터미널 2
```

**② 스텝 주입기 — 4케이스** (계단 생성 + estop 최상위)
```bash
ros2 run stack_avoid step_injector --ros-args -p hold_s:=1.0 -p settle_s:=1.0  # 터미널 1
python3 src/stack_avoid/tools/test_step_injector.py                            # 터미널 2
```

**③ 구간 표시기** — 이게 안 돌면 ③ⓐⓑⓒ 분석이 통째로 불가능하다
```bash
ros2 topic echo /test/event                        # 터미널 1
ros2 run stack_avoid mark                          # 터미널 2 — 3, a 등 입력해 수신 확인
```

**④ 분석기 — 합성 세션으로 왕복 확인**
```bash
ros2 bag record -o /tmp/synbag /perception/avoid /perception/estop \
    /perception/static_estop /perception/dynamic_estop /test/event \
    /adas/target_ref /vehicle/vector                          # 터미널 1
python3 src/stack_avoid/tools/synth_session.py                # 터미널 2 (20초)
# 터미널 1 Ctrl+C 후
python3 src/stack_avoid/tools/analyze_field_bag.py /tmp/synbag
```
기대 출력 — 이 값이 안 나오면 분석기가 고장난 것:

| 구간 | 감지율 | gap |
|---|---|---|
| cone 3m | 100.0% | 3.000 |
| cone 1m | 50.0% | 1.000 |

```
ⓐ  estop 미발동 · 회피목표점 있었음
ⓑ  estop 발동 (정적) · 회피목표점 없었음
ⓒ  estop 발동 (정적) · 회피목표점 없었음 · narrow_gap
```
