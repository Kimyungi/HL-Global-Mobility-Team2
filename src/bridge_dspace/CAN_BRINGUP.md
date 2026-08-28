# PC ↔ dSPACE CAN 통신 검증 가이드

실차 PC(산업용 PC)에서 dSPACE와 CAN이 실제로 붙는지 단계별로 확인하는 절차.
**각 단계의 명령을 그대로 복사–붙여넣기** 하면 된다. 프로토콜 내용 자체는 `PROTOCOL.md`가 기준.

> 전제: 이 PC에는 ROS 2 Humble·워크스페이스 빌드·CAN 자동 셋업이 이미 설치돼 있다 (2026-07-29 완료).
> 새 터미널을 열면 ROS 환경이 자동 소싱된다. 안 되면: `source ~/FMA_ws/install/setup.bash`

> ## ★ 2026-08-28 — 어댑터 PCAN → **Kvaser Leaf v3**, 와이어 포맷 classic → **CAN FD**
>
> **논리 프로토콜은 안 바뀌었다.** ID 맵·8바이트 페이로드 레이아웃·양자화 스케일·
> 커밋(latch) 규칙·watchdog 은 그대로다. 그래서 아래 **4~6단계의 성공 판정 기준이 전부
> 그대로 재사용된다** — 바뀐 것은 1~3단계(어댑터 인식·배선·dSPACE 설정)뿐이다.
>
> **2026-08-28 오후 추가 — v5(PR #52)로 재전환했다.** dSPACE 가 먼저 64바이트 계약으로
> 넘어가 있어 PC 를 맞췄다. 이 문서의 **판정 기준 중 길이 관련 항목이 바뀐다**:
> - `0x101`·`0x200` 은 **64바이트**, `0x100` 헤더만 8바이트
> - RX 는 **`0x200` 한 프레임** (`0x201`/`0x202` 없음)
> - 주기당 TX = **2프레임** (구 21프레임) → `TX 프레임/헤더 = 2.00`
> - `can_dump.py` 가 `MPC_TARGET` / `VEH_FB` 로 해석해 준다
> - 브리지 기동 로그에 `계약 v5 (PR #52 64B)` 가 찍히면 정상
>
> ⚠ **"dSPACE 만 FD 로 바꾸면 되는 것"이 아니다.** PC 가 classic 수신만 하고 있으면
> 커널이 FD 프레임을 **에러 없이 통째로 버린다** — 배선도 candump 도 멀쩡한데
> `/vehicle/vector` 만 0Hz 가 된다. 그래서 PC 코드·인터페이스 설정이 함께 바뀌었다.

---

## 0단계 — PC 단독 사전 점검 (dSPACE 없이, 5분)

케이블 꽂기 전에 PC 쪽 소프트웨어가 멀쩡한지 확인. 가상 CAN(vcan0)으로 전체 체인을 돌린다.

```bash
ros2 launch bridge_dspace loopback_test.launch.py
```

**다른 터미널에서:**

```bash
ros2 topic hz /vehicle/vector
```

✅ **성공 판정**: `average rate: 99.9~100.0` 부근. 확인 후 두 터미널 모두 `Ctrl-C`.

와이어 포맷까지 눈으로 보려면:

```bash
python3 ~/FMA_ws/src/bridge_dspace/tools/can_dump.py --iface vcan0
```

✅ 각 줄 앞에 **`FD/BRS`** 가 찍히면 FD 로 돌고 있는 것이다 (`STD` = classic).

| 실패 증상 | 처방 |
|---|---|
| `CAN FD 활성화 실패: vcan0 (MTU 16, FD 는 72 필요)` | 예전 vcan0 이 남아 있는 것. `sudo ~/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh --vcan` 재실행 (또는 `sudo ip link set vcan0 down && sudo ip link set vcan0 mtu 72 && sudo ip link set vcan0 up`) |
| `vcan0` 관련 다른 에러 | `sudo ~/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh --vcan` |
| `Package 'bridge_dspace' not found` | `source ~/FMA_ws/install/setup.bash` 후 재시도 |

> 노드가 조용히 classic 으로 강등되지 않고 **죽는 것은 의도**다. 실기와 다른 포맷으로
> 통과하면 루프백이 검증해야 할 것을 안 검증한 셈이 된다.

**CAN 인터페이스 없이 포맷 계약만 확인** (하드웨어 도착 전에도 가능):

```bash
cd ~/FMA_ws && colcon test --packages-select bridge_dspace --ctest-args -R socketcan_fd_test
```

✅ `socketcan_fd_test: OK — FD 와 classic 의 페이로드가 동일하다`

---

## 1단계 — Kvaser Leaf v3 인식 확인

Kvaser Leaf v3(USB)를 PC에 꽂는다. 자동 셋업이 설치돼 있으므로 꽂기만 하면 CAN FD로 올라온다.

```bash
dmesg | tail -20 | grep -i kvaser
ip -br link show can0
```

✅ **성공 판정**: `kvaser_usb` 가 장치를 잡았다는 줄 + `can0  UP` (또는 `UNKNOWN <NOARP,UP,LOWER_UP>`)

**FD로 올라왔는지 확인 — MTU 하나면 된다** (PC 코드도 이 값으로 FD 가능 여부를 판정한다):

```bash
cat /sys/class/net/can0/mtu
```

✅ **성공 판정**: `72` (= CAN FD). `16` 이면 classic 전용이라 브리지가 기동에서 죽는다.

비트레이트 상세:

```bash
ip -details link show can0
```

✅ **성공 판정**: `bitrate 1000000` + `dbitrate 2000000` + 플래그에 `<FD>`

| 실패 증상 | 처방 |
|---|---|
| `Device "can0" does not exist`, dmesg에 kvaser 없음 | 재삽입. 그래도 없으면 **커널이 Leaf v3를 모르는 것** — 아래 "Leaf v3 커널 지원" 참조 |
| `can0 DOWN` | 자동 셋업 미설치 — `sudo ~/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh` |
| MTU가 `16` | FD 설정이 안 먹었다. install.sh 재실행 후 재삽입. networkd 잔재 확인(아래) |
| bitrate/dbitrate가 표준값이 아님 | 위 install.sh 재실행 후 재삽입 |
| 값이 **재삽입할 때마다** 되돌아감 | systemd-networkd의 `/etc/systemd/network/*.network` 잔재가 udev 설정을 덮어쓰는 것 (실사례 2026-08-03, journalctl에 up→down→up 두 번 찍힘). install.sh 재실행이 표준 `80-can0.network`(FD 1M/2M)로 교체해 준다. **`FDMode=yes` 가 빠진 파일이 남아 있으면 MTU가 16으로 고정된다** |
| `ip link` 가 `Invalid argument` | 컨트롤러가 sample-point 0.8을 정확히 못 맞추는 경우. `can_up.sh`가 자동으로 샘플포인트 없이 재시도한다(`journalctl -t can_up` 확인) — 그 경로로 올라왔으면 **dSPACE와 샘플포인트가 어긋날 수 있으니 손상민과 대조** |

### Leaf v3 커널 지원 (Ubuntu 22.04 주의)

Leaf v3는 비교적 최근에 메인라인 `kvaser_usb`에 추가됐다. **22.04 기본 커널(5.15)에는
없을 수 있다.**

```bash
uname -r
modinfo kvaser_usb | grep -c alias      # 지원 장치 수
```

안 잡히면 HWE 커널로 올린다: `sudo apt install linux-generic-hwe-22.04` 후 재부팅.

> ⚠ **Kvaser 자체 배포 `linuxcan` 드라이버를 설치하지 말 것.** SocketCAN이 아닌
> `/dev/kvaser*` 로 잡혀서 이 스택 전체(브리지·can_zero·can_dump·candump)가 못 쓴다.
> 메인라인 `kvaser_usb`를 쓴다.

---

## 2단계 — 물리 배선

Kvaser Leaf v3 D-sub9 ↔ dSPACE(MicroLabBox) CAN 포트:

| 신호 | D-sub9 핀 | 상대측 |
|---|---|---|
| CAN_H | **7** | dSPACE CAN_H |
| CAN_L | **2** | dSPACE CAN_L |
| GND | 3 | dSPACE GND (권장) |

- **핀아웃은 CiA 303-1 표준이라 PCAN과 동일하다** — 기존 케이블을 그대로 쓴다.
- **종단저항 120Ω × 2** — 버스 양 끝단에 하나씩 (PC 쪽 1개 + dSPACE 쪽 1개).
- ⚠ **Leaf v3의 내장 종단 유무를 데이터시트로 확인할 것.** 내장 종단이 있는데 기존
  외부 120Ω을 그대로 두면 **합성 40Ω**이 되어 FD 데이터 구간에서 먼저 무너진다.
- 확인(전원 꺼진 상태): 멀티미터로 CAN_H–CAN_L 사이 저항 측정 → **약 60Ω**이면 정상
  (120Ω이면 종단이 한쪽뿐, 40Ω이면 셋, ∞이면 없음).
- ⚠ **FD는 배선 품질에 classic보다 훨씬 민감하다.** 데이터 구간이 2 Mbps로 돌기 때문에
  스텁(분기선) 길이·종단이 classic 1 Mbps에서 문제없던 수준이어도 드러날 수 있다.
  스텁은 짧게(수 cm), 트위스트 페어 유지.

---

## 3단계 — dSPACE → PC 수신 확인 (한 방향씩!)

**dSPACE 쪽(손상민)**: 모델에서 `0x200`~`0x202` 3프레임을 10ms 주기로 송신 시작
(값은 아무거나, 예: x=1.0).

**★ 설정 필수 확인 — FD는 아래 5개가 전부 맞아야 붙는다** (PROTOCOL.md §공통의 표가 정본):

| # | 항목 | 값 |
|---|---|---|
| 1 | nominal bitrate (= baud rate) | **1 MBaud** |
| 2 | data bitrate | **2 MBaud** |
| 3 | **BRS** (bit rate switch) | **on** |
| 4 | ISO CAN FD vs non-ISO(Bosch) | **ISO** |
| 5 | sample point (nominal / data) | **80% / 80%** |

추가로 v2에서부터 그대로인 것: **11-bit 표준 ID / byte order Intel(little-endian) /
메시지 길이(DLC) 정확히 8바이트**.

> ⚠ **DLC를 8로 맞출 것.** CAN FD는 9·10·11바이트 길이가 없어서(8 다음이 12) RTI가
> 12나 16으로 올려붙이는 경우가 있다. 그러면 PC가
> `bad CAN frame: id=0x200 len=12 (8 이어야 함, FD 프레임)` 경고를 내고 프레임을 버린다.

**PC 쪽:**

```bash
candump -td can0
```

✅ **성공 판정**: `0x200`, `0x201`, `0x202`가 각각 ~10ms 간격으로 계속 찍힘.
FD 프레임은 candump에서 `200##1...` 처럼 **`#` 이 두 개**로 나온다 (classic은 `200#...`).

값까지 해석해서 보려면 (공학 단위 + 포맷 표시):

```bash
python3 ~/FMA_ws/src/bridge_dspace/tools/can_dump.py --iface can0
```

✅ `FD/BRS  0x200  [...]  POSE x=1.000 ...` 처럼 나오면 **포맷·byte order 모두 정상**.

| 실패 증상 | 처방 |
|---|---|
| candump 완전 침묵 | 배선(2단계) 재확인 → 아래 "버스 에러 확인" |
| candump에는 보이는데 브리지 `/vehicle/vector`가 0Hz | 브리지 로그의 `bad CAN frame` 확인 — DLC가 8이 아닐 가능성 |
| 프레임은 오는데 값이 이상함 | dSPACE RTI byte order가 Motorola로 돼 있을 가능성 — Intel로 변경 |
| ID가 다르게 찍힘 | dSPACE 모델의 ID 설정을 PROTOCOL.md 표와 대조 |
| `STD`로 찍힘 (FD를 기대했는데) | dSPACE가 아직 classic 송신 중. **PC는 양쪽을 다 받으므로 이 상태로도 동작한다** — dSPACE 설정만 마저 넘기면 된다 |
| `FD` 인데 `BRS`가 없음 | dSPACE의 BRS가 off. 붙긴 하지만 속도 이득이 0이다 |

**버스 에러 확인** (양쪽 어느 단계든 문제 시):

```bash
ip -details -statistics link show can0
```

- `ERROR-PASSIVE` / `BUS-OFF` 상태, 또는 `bus-error` 카운트 증가 → **비트레이트·샘플포인트
  불일치, ISO/non-ISO 불일치, 또는 종단저항 문제**가 대부분.
- **FD 전용 절반 실패**: nominal은 맞는데 data bitrate/샘플포인트만 어긋나면 프레임이
  간헐적으로만 깨진다. 이때는 **BRS를 잠깐 꺼서 갈라 본다** — BRS off로 에러가 사라지면
  원인은 데이터 구간(#2·#5), 그대로면 nominal 쪽(#1·#5)이다.
- bus-off는 100ms 후 자동 복구되도록 설정돼 있으나, 근본 원인(배선/설정)을 잡아야 한다.

---

## 3b단계 — 포맷 A/B 대조 (안 붙을 때만)

"FD 때문인가, 아니면 원래부터 안 되는 건가"를 가르는 절차. 양쪽을 **동시에** classic으로
되돌려 v2 상태를 재현한다.

```bash
# PC를 classic 송신으로 (인터페이스 설정은 그대로 둬도 된다 — FD 인터페이스는
# classic 프레임을 그대로 실어 보낸다)
ros2 launch bridge_dspace bridge.launch.py can_fd:=false
```

dSPACE 쪽도 classic으로 되돌린다.

- **classic에서 붙고 FD에서 안 붙는다** → 위 5개 파라미터 불일치 또는 배선 품질(2단계).
- **classic에서도 안 붙는다** → FD와 무관한 문제. 배선·ID·종단부터.

BRS만 떼어 보려면 `can_fd_brs:=false` (양쪽 동시에).

---

## 4단계 — PC → dSPACE 송신 확인

**PC 쪽 (터미널 2개):**

```bash
ros2 launch bridge_dspace bridge.launch.py
```

```bash
ros2 run bridge_dspace dummy_ref_publisher
```

**모니터 (터미널 3):**

```bash
candump -td can0
```

✅ **성공 판정 (PC측)**: 매 10ms마다 `0x101`(64B 참조점) → `0x100`(8B 헤더) 순서로 찍힘.
✅ **성공 판정 (dSPACE측, 손상민)**: RTI CAN 수신에서 `0x100` counter가 매 주기 +1,
v_ref = 300 (=0.3 m/s), `0x101` x = 500 (=0.5 m).

> ⚠ candump에 내 송신 프레임이 **안 보이면** dSPACE가 ACK를 안 하는 것 (SocketCAN은
> 전송 성공한 프레임만 에코). 3단계의 "버스 에러 확인"으로.

| dSPACE 측이 꼭 지킬 것 | 근거 |
|---|---|
| point 프레임은 버퍼링만, **`0x100` 수신 시점에 n_points개 latch** | PROTOCOL.md 커밋 규칙 |
| n_points는 헤더에서 읽기 (v5 실운용 1점 — 확장 대비 가변 필드 유지) | PROTOCOL.md TX 절 |
| watchdog: `0x100` counter 30ms 미갱신 → v_ref=0, 조향 유지 | CLAUDE.md §3 |
| 메시지 길이(DLC) = `0x100` **8B** / `0x101`·`0x200` **64B** (중간값 금지) | PROTOCOL.md §공통 |

---

## 5단계 — 전체 왕복 + 실차 거동

3·4단계가 동시에 돌아가는 상태에서 (dSPACE는 실제 Vehicle MGM 모델 가동):

```bash
ros2 topic hz /vehicle/vector
```

✅ **성공 판정**: ~100 Hz. 이 시점에 **바퀴가 0.3 m/s로 굴러야 한다** (dummy ref = 직선 0.3 m/s).

```bash
ros2 topic echo /vehicle/vector
```

✅ dSPACE 상태 추정값 {x, y, yaw, v, str}이 실시간으로 갱신되고, v가 0.3 부근.

**부하 확인 (FD 전환 효과 측정 — 선택):**

```bash
python3 ~/FMA_ws/src/stack_avoid/tools/can_log.py --iface can0 --duration 60 --out /tmp/can_fd.log
```

✅ 요약의 **"와이어 포맷"** 줄이 `FD <숫자>` 단독이면 양쪽 다 넘어온 것.
`classic`과 `FD`가 **섞여 나오면 한쪽이 아직 전환 안 된 것**이다.
✅ `TX 프레임/헤더 = 2.00` (v5: 참조점 1 + 헤더). 21.00 이 나오면 v3 코드가 도는 것이다.

---

## 6단계 — watchdog 실차 검증 (안전 기능, 반드시 확인)

바퀴가 도는 상태에서 **송신을 죽인다**:

```bash
pkill -f dummy_ref_publisher
```

✅ **성공 판정**: ~30ms 내에 dSPACE가 v_ref=0 처리 → **바퀴 감속 정지, 조향은 그대로**
(급조향하면 안 됨). `/vehicle/vector`의 v가 0으로 떨어지는 것으로도 확인 가능.

재시작:

```bash
ros2 run bridge_dspace dummy_ref_publisher
```

✅ 바퀴가 다시 0.3 m/s로 — 별도 리셋 없이 복구되어야 정상.

> ⚠ **이 watchdog은 dSPACE에 아직 없다** (2026-08-09 실측, J-6 — CLAUDE.md §3).
> 구현 전까지는 종료 시 `can_zero`가 목표값 0 복귀를 보장한다. `can_zero`는 인터페이스
> MTU로 FD/classic을 **자동 판정**하므로 이관 후에도 인자를 붙일 필요가 없다.

---

## 7단계 — (옵션) MGM 연계

dummy 대신 실제 Decision 체인으로:

```bash
pkill -f dummy_ref_publisher
ros2 launch adas_mgm mgm.launch.py
```

이후는 각 스택 연동 — `~/FMA_ws/src/stack_gps/COMMANDS.md`의 실차 절차 참조.

---

## 자주 쓰는 진단 명령 모음

```bash
cat /sys/class/net/can0/mtu                                  # 72 = FD ✔ / 16 = classic ✘
ip -br link show can0                                        # 인터페이스 상태
ip -details link show can0                                   # bitrate·dbitrate·<FD>·샘플포인트
ip -details -statistics link show can0                       # 에러 카운터·버스 상태
candump -td can0                                             # 원시 프레임 (FD는 ID## 로 표시)
python3 ~/FMA_ws/src/bridge_dspace/tools/can_dump.py --iface can0   # 공학 단위 + STD/FD/FD-BRS
python3 ~/FMA_ws/src/stack_avoid/tools/can_log.py --iface can0 --duration 60  # 포맷별 프레임 수
cansend can0 100##10100000000000000                          # 수동 FD 프레임 1개 (can-utils)
cansend can0 100#0100000000000000                            # 수동 classic 프레임 1개
python3 ~/FMA_ws/src/bridge_dspace/tools/protocol_selftest.py       # 프로토콜 로직 셀프테스트
journalctl -t can_up -n 20                                   # 인터페이스 자동 up 로그
```

## 자동 활성화 (설치하면 이 문서의 수동 bringup 불필요)

어댑터를 꽂기만 하면 udev가 CAN FD(1M/2M) + 자동 BUS-OFF 복구(restart-ms 100)로 올려준다:

```bash
cd ~/FMA_ws/src/bridge_dspace/tools
sudo cp can_up.sh /usr/local/bin/ && sudo chmod +x /usr/local/bin/can_up.sh
sudo cp 70-can-auto.rules /etc/udev/rules.d/
sudo udevadm control --reload
# 이미 꽂혀 있으면 1회: sudo /usr/local/bin/can_up.sh can0
```

확인: `ip -details link show can0` → `state ERROR-ACTIVE` + `bitrate 1000000` + `dbitrate 2000000` + `<FD>`,
그리고 `cat /sys/class/net/can0/mtu` → `72`.

참고: dSPACE 미연결(버스에 혼자) 상태에서 송신하면 BUS-OFF가 정상이며,
restart-ms 덕에 상대가 나타나면 자동 복구된다. (2026-08-01, 차량 PC 적용 완료)

> ★ 비트레이트·샘플포인트는 **두 곳에 중복**돼 있다 — `tools/can_up.sh`(udev·서비스 경로)와
> `tools/can_setup/80-can0.network`(systemd-networkd 경로). networkd가 udev를 이기므로
> **바꿀 때 반드시 둘 다** 고칠 것 (2026-08-03 500k 잔재 실사례).
