# 회피 조향 스윕 시험 가이드 (2026-08-09)

> ⚠ **이 문서는 2026-08-09 세션의 기록이다. 그대로 재실행할 수 없다.**
> 본문의 `ros2 param set /avoid_to_ref ray_pull|send_target_as_is ...` 는 해당 경로가
> 2026-08-11 에 제거되어 **거부된다**(MEASUREMENTS V절). 송신 경로는 이제 방향보존 당김
> 하나뿐이고, 점 개수만 `ray_n_points` 로 바꿀 수 있다. 시험 결과와 판정 근거는 그대로
> 유효하니 기록으로만 참고할 것.

**담당: 이기돈** · 목적: **송신점 거리**와 **ref 점 개수** 중 무엇이 dSPACE 조향을 키우는지 확정

```
LiDAR → stack_avoid(초록점) → avoid_to_ref(송신점) → bridge(CAN) → dSPACE MPC → 조향
```

- **스탠드(바퀴 듦)에서 한다.** 구간마다 30초씩 기하를 고정해야 하는데 지상에선 불가능하다.
- 콘은 **앞범퍼에서 2.60m** — 8/9 H 시험과 같아야 비교가 된다. 줄자로 잴 것.
- 설계 근거·판정 배경은 `MEASUREMENTS.md` I절. 이 문서는 **현장에서 칠 것만** 담는다.

---

## 터미널 지도 — 이 문서의 모든 명령은 아래 이름으로 부른다

| 이름 | 역할 | 켜는 때 | 끄는 때 |
|---|---|---|---|
| **T1** | 기동 (라이다·인지·estop·RViz·**bag 기록**) | 시작할 때 | 다 끝나고 (마지막에서 두 번째) |
| **T2** | CAN 로그 (버스에 실제로 나간 프레임) | T1 다음 | 다 끝나고 (마지막) |
| **T3** | 시험 실행 (스윕) | 시험할 때마다 | 시험 끝나면 자동 종료 |
| (임시) | 확인용 `ros2 topic echo` 등 | 필요할 때 | 확인 후 닫아도 됨 |

모든 터미널 첫 줄은 이것으로 시작한다:

```bash
cd ~/FMA_ws && source ~/FMA_ws/install/setup.bash
```

> ydlidar 드라이버·SDK 는 FMA_ws 에 벤더링됨 (2026-08-09) — FMA_ws 만 source 하면 된다.
> `~/ydlidar_ros2_ws` 는 더 이상 source 하지 말 것 (같은 패키지가 둘 잡히면 혼선).

---

# PART A — 시작 전 확인 (임시 터미널, 1분)

```bash
ip link show can0 | head -1              # state UP 이어야 함
pgrep -af "avoid_to_ref|dummy_ref|mgm_node"   # 아무것도 안 나와야 함
```

- `state UP` 이 아니면: `sudo /usr/local/bin/can_up.sh can0`
  (평소엔 PCAN 꽂으면 udev 가 자동으로 올린다 — 문서 맨 아래 참조)
- 프로세스가 나오면: `pkill -9 -f "avoid_to_ref|dummy_ref|mgm_node" && ros2 daemon stop`
  두 노드가 같이 `/adas/target_ref` 를 쏘면 측정이 통째로 무효다.

```bash
cd ~/FMA_ws && colcon build --packages-select stack_avoid --symlink-install
```

---

# PART B — 기동

**순서가 중요하다: T1 → T2 → 확인.** T1 이 모든 노드를 띄우므로,
T1 없이 확인 명령을 치면 `Node not found` 가 난다.

## T1 [기동 + 로깅]  ← 가장 먼저, 켜둔 채로 둔다

```bash
cd ~/FMA_ws && source ~/FMA_ws/install/setup.bash
ros2 launch stack_avoid field_session.launch.py \
    mode:=avoid v_ref:=0.2 dynamic:=false \
    bag_dir:=$HOME/avoid_logs/nsweep_20260809_b
```

로그에 이 줄이 떠야 새 코드다 — 없으면 PART A 빌드부터 다시:

```
송신점=방향보존 당김 0.90m × 1점 @0.32m
```

> `bag_dir` 는 세션마다 새 이름이어야 한다 (이미 있는 폴더에는 기록이 안 된다).
> 1차 세션이 `nsweep_20260809` 를 썼으므로 이번은 `_b`. 다음은 `_c` 식으로.

## T2 [CAN 로그]

```bash
cd ~/FMA_ws
python3 src/stack_avoid/tools/can_log.py --out $HOME/avoid_logs/nsweep_20260809_b_can.log
```

## 출발 전 확인 3줄 (임시 터미널 — 닫아도 됨)

> ⚠ **T1 이 떠 있어야 한다.** 아래 세 줄은 전부 T1 이 띄운 노드에게 묻는 것이다.
> T1 없이 치면 이렇게 나온다 — 고장이 아니라 순서가 틀린 것:
> ```
> WARNING: topic [/vehicle/vector] does not appear to be published yet
> Node not found
> ```
> `ros2 node list` 로 `/avoid_to_ref` · `/can_bridge_node` 가 보이는지 먼저 확인할 것.

```bash
ros2 topic echo /vehicle/vector --once      # v 가 0.19~0.21 → 배터리 살아 있음 ★
ros2 topic echo /adas/target_ref --once     # ref_points 1개, 점 (x≈0.88, y≈0.17)
ros2 param dump /avoid_to_ref > $HOME/avoid_logs/nsweep_20260809_b_params.yaml
```

**`v` 가 0 이면 여기서 멈춘다.** 배터리 없이 잰 8/8 측정을 전량 폐기한 적이 있다.

RViz 에서 초록점(인지)과 주황점(송신)이 **원점에서 같은 직선 위**에 있으면 정상이다.

---

# PART C — 시험 (T3 에서 순서대로)

**현재 남은 시험은 C4(속도 스윕) 하나다.** C1~C3 은 8/9 완료 — 결과는
`MEASUREMENTS.md` J절 (점 개수 무영향 · 거리 +41% 포화 · 기준선 재현).
재실행할 필요 없음. C4 만 하고 PART D 로 넘어갈 것.

모든 시험은 `/test/event` 에 구간 라벨을 남기고 끝나면 원래 값으로 되돌린다.
**T1·T2 는 계속 켜둔다.**

## ~~C1. 점 개수~~ ✅ 완료 (8/9, J-1: 1~20개 전부 str −2.91° — 무영향 확정)

```bash
cd ~/FMA_ws && source ~/FMA_ws/install/setup.bash
python3 src/stack_avoid/tools/gain_sweep.py \
    --param ray_n_points --values 1 2 3 5 8 12 16 20 1 --dwell 30
```

점 0 의 위치·방향은 전 구간 고정이고 개수만 바뀐다. 마지막 `1` 은 세션 드리프트 확인용.

## ~~C2. 송신점 거리~~ ✅ 완료 (8/9, J-2: +41% 후 0.9m 포화)

```bash
python3 src/stack_avoid/tools/gain_sweep.py \
    --param ref_lookahead_m --values 0.4 0.9 1.4 2.0 --dwell 30
```

## ~~C3. 기준선~~ ✅ 완료 (8/9, J-2: 3.29m → str −2.35°)

```bash
ros2 param set /avoid_to_ref ray_pull false
ros2 param set /avoid_to_ref send_target_as_is true
ros2 topic pub --times 3 --rate 2 /test/event std_msgs/msg/String "{data: 'sweep baseline'}"
sleep 40
ros2 topic pub --times 3 --rate 2 /test/event std_msgs/msg/String "{data: 'sweep end'}"
ros2 param set /avoid_to_ref send_target_as_is false
ros2 param set /avoid_to_ref ray_pull true
```

## C4. ★ 속도 스윕 — 지금 할 시험 (3분)

dSPACE 조향 게인이 속도에 비례한다는 것이 K 절의 결론이다 (GPS 실측: v<0.45 게인
0.22~0.24, v≥0.45 게인 0.94). 콘·방향각은 그대로 두고 속도만 흔든다:

```bash
python3 src/stack_avoid/tools/gain_sweep.py \
    --param target_speed_mps --values 0.2 0.3 0.4 0.5 0.2 --dwell 30
```

- 예측: str 이 0.2 에서 ~2.9°, **0.45 부근에서 8~10° 로 급증**하면 가설 확정.
- 스탠드(바퀴 듦)이므로 0.5 도 안전하다.

**`--dwell 30` 을 줄이지 말 것** — dSPACE 조향은 정착에 수십 초 걸린다. 12~15초는 정착 전 값이다.

---

# PART D — 종료

순서대로:

1. **T2** `Ctrl-C` → 출력되는 `TX 프레임/헤더` 값을 메모 (20점 구간이면 21.00)
2. **T1** `Ctrl-C` → bag 마감까지 몇 초 기다린다
   → 로그에 `can_zero: ... 30회 송신 완료` 가 나오는지 확인 (dSPACE 목표값 0 복귀. 안 보이면 문서 맨 아래 수동 절차)
3. 임시 터미널에서 확인:

```bash
ls -la $HOME/avoid_logs/ | grep nsweep
pgrep -af "avoid_to_ref|ros2 bag" || echo "정리 완료"
```

산출물 3개:

```
~/avoid_logs/nsweep_20260809_b/        ← bag (명령·응답·라벨·스캔·estop·로그 전부)
~/avoid_logs/nsweep_20260809_b_can.log   ← 버스에 실제로 나간 프레임
~/avoid_logs/nsweep_20260809_b_params.yaml
```

---

# PART E — 분석

```bash
cd ~/FMA_ws && source ~/FMA_ws/install/setup.bash
python3 src/stack_avoid/tools/analyze_field_bag.py $HOME/avoid_logs/nsweep_20260809_b
```

"조향 스윕" 표가 나온다 (각 구간 **뒤쪽 절반** = 정착분):

```
            구간   n    점0 x    점0 y    방향°       κ    str°   명령등가°   실행률   고유값
  ray_n_points=1   1   0.884   +0.169   10.80  +0.416    ...
 ray_n_points=20  20   0.884   +0.169   10.80  +0.416    ...
```

- **`점0 x·y·방향` 이 전 구간 같아야** 점 개수 비교가 유효하다. 다르면 콘이 움직인 것.
- **`고유값` 이 1~2** 면 그 구간 str 이 얼어붙은 것 = 무효 구간.
- 비교는 **`str°`** 로 한다.

## 판정

| 결과 | 의미 | 다음 |
|---|---|---|
| C1 에서 n 과 무관하게 동일 | 점 개수는 레버가 아니다 | C2 결과로 판단 |
| C1 에서 특정 n 이상 급증 | dSPACE 가 뒤 점들도 쓴다 | 손상민에게 모델 재확인 요청 |
| C3(2.3°) < C2 의 0.9m 구간 | **거리가 레버다** | 거리 확정 후 **지상 회피 주행** |
| 전부 2.3°대 | PC 측 레버 소진 | dSPACE 구조 문제로 이관 |

부호는 별개 문제다 — 크기로 비교하고, 바퀴가 초록점 쪽으로 도는지는 눈으로 본다.

세션 폴더 경로를 알려주면 결과가 `MEASUREMENTS.md` J절로 정리된다.

---

# 트러블슈팅

| 증상 | 조치 |
|---|---|
| T1 이 `PackageNotFoundError` | FMA_ws 빌드 안 됨 — PART A 의 colcon build 실행 |
| 기동 배너에 `0.90m × 20점` 없음 | 옛 코드 — PART A 빌드 후 재기동 |
| `v` 가 0 | 배터리·구동 전원. 이 상태 측정은 전량 무효 |
| `str` 이 세션 내내 한 값 | 액추에이션 사망 — 그 구간 버린다 |
| estop 이 계속 걸림 (`v_ref=0`) | `dynamic:=false` 로 띄웠는지 확인. 이미 래치됐으면 **노드 재시작** (param set 으로 안 풀림) |
| `/scan` 이 멈춤 | 라이다 USB 재연결 후 드라이버가 죽은 포트를 잡음 — 드라이버 재시작 |
| 스윕이 `파라미터 없음` | `ros2 node list` 로 `/avoid_to_ref` 확인 |
| 구간 라벨이 안 남음 | T1 기동 후 5초 뒤에 스윕 시작 |
| `bag record` 가 폴더 있다고 거부 | `bag_dir` 이름 변경 |

---

# CAN 자동 설정 — ✅ 설치 완료 (2026-08-09)

```
/usr/local/bin/can_up.sh
/etc/udev/rules.d/70-can-auto.rules
```

**PCAN 을 뺐다 꽂으면 can0 이 자동으로 1 Mbps 로 올라온다.** 손으로 올릴 일 없음.
(검증: can0 down → udev `add` 이벤트 → `state UP · bitrate 1000000 · restart-ms 100`,
시스템 로그에 `can_up[…]: can0 up @ 1Mbps` 기록 확인)

자동으로 안 올라오는 것 같으면 수동 실행:

```bash
sudo /usr/local/bin/can_up.sh can0
ip -d link show can0        # bitrate 1000000, state ERROR-ACTIVE 확인
```

> 이미 UP 인 인터페이스에 `sudo ip link set can0 up type can bitrate ...` 를 치면
> `RTNETLINK answers: Device or resource busy` 가 뜬다. **고장이 아니다** — 살아 있는
> 인터페이스는 재설정이 안 되는 것뿐이다. 바꾸려면 `can_up.sh` 처럼 down 부터 해야 한다.

---

# dSPACE watchdog 없음 — 종료 시 0 복귀는 **자동**이다

PC 가 송신을 멈춰도 dSPACE 는 **마지막 목표값을 무기한 유지한다** (2026-08-09 실측).
CLAUDE.md §3 의 "30ms 미갱신 → v_ref=0" 이 동작하지 않는다. 8/9 오전에 어제 스윕
이후 PC 가 아무것도 안 보내는 상태로 dSPACE 가 `v=0.20` 을 들고 있었다.

**대응: `can_zero` 가드가 launch 에 포함돼 있다.** T1 을 `Ctrl-C` 하면 SocketCAN 에
직접 `v_ref=0 · ref_point 0` 을 30회 쓰고 종료한다. 브리지가 먼저 죽어도 동작한다.
T1 로그에 이 줄이 나오면 성공이다:

```
[can_zero-1] can_zero: can0 에 v_ref=0 · ref_point 0 을 30회 송신 완료
```

## 자동 복귀가 안 된 경우 (kill -9 로 죽였거나 위 줄이 안 보일 때)

```bash
ros2 run stack_avoid can_zero --once
```

확인:

```bash
python3 ~/FMA_ws/src/stack_avoid/tools/can_log.py --duration 3 --out /tmp/chk.log
python3 - <<'EOF'
import struct
v=[struct.unpack('<ff',bytes.fromhex(l.split('#')[1]))[1]
   for l in open('/tmp/chk.log') if ' 201#' in l]
print(f"dSPACE v = {sorted(v)[len(v)//2]:+.4f} m/s  →  {'OK' if abs(sorted(v)[len(v)//2])<0.01 else '★아직 0 아님'}")
EOF
```
