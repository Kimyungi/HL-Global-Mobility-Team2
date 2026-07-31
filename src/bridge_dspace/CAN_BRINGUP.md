# PC ↔ dSPACE CAN 통신 검증 가이드

실차 PC(산업용 PC)에서 dSPACE와 CAN이 실제로 붙는지 단계별로 확인하는 절차.
**각 단계의 명령을 그대로 복사–붙여넣기** 하면 된다. 프로토콜 내용 자체는 `PROTOCOL.md`가 기준.

> 전제: 이 PC에는 ROS 2 Humble·워크스페이스 빌드·CAN 자동 셋업이 이미 설치돼 있다 (2026-07-29 완료).
> 새 터미널을 열면 ROS 환경이 자동 소싱된다. 안 되면: `source ~/FMA_ws/install/setup.bash`

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

| 실패 증상 | 처방 |
|---|---|
| `vcan0` 관련 에러 | `sudo ~/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh --vcan` |
| `Package 'bridge_dspace' not found` | `source ~/FMA_ws/install/setup.bash` 후 재시도 |

---

## 1단계 — PCAN 인식 확인

PCAN(USB)을 PC에 꽂는다. 자동 셋업이 설치돼 있으므로 꽂기만 하면 1 Mbps로 올라온다.

```bash
ip -br link show can0
```

✅ **성공 판정**: `can0  UP` (또는 `UNKNOWN <NOARP,UP,LOWER_UP>`)

비트레이트 확인:

```bash
ip -details link show can0 | grep bitrate
```

✅ **성공 판정**: `bitrate 1000000`

| 실패 증상 | 처방 |
|---|---|
| `Device "can0" does not exist` | PCAN 재삽입. 그래도 없으면 `dmesg | tail -20`에서 `peak_usb` 인식 확인 |
| `can0 DOWN` | 자동 셋업 미설치 — `sudo ~/FMA_ws/src/bridge_dspace/tools/can_setup/install.sh` |
| bitrate가 1000000이 아님 | 위 install.sh 재실행 후 PCAN 재삽입 |

---

## 2단계 — 물리 배선

PCAN D-sub9 ↔ dSPACE(MicroLabBox) CAN 포트:

| 신호 | PCAN D-sub9 핀 | 상대측 |
|---|---|---|
| CAN_H | **7** | dSPACE CAN_H |
| CAN_L | **2** | dSPACE CAN_L |
| GND | 3 | dSPACE GND (권장) |

- **종단저항 120Ω × 2** — 버스 양 끝단에 하나씩 (PCAN 쪽 1개 + dSPACE 쪽 1개).
  둘 다 내장 종단이 꺼져 있으면 외부 저항 필요.
- 확인(전원 꺼진 상태): 멀티미터로 CAN_H–CAN_L 사이 저항 측정 → **약 60Ω**이면 정상
  (120Ω이면 종단이 한쪽뿐, ∞이면 없음).

---

## 3단계 — dSPACE → PC 수신 확인 (한 방향씩!)

**dSPACE 쪽(손상민)**: 모델에서 `0x200`~`0x202` 3프레임을 10ms 주기로 송신 시작
(값은 아무거나, 예: x=1.0). 설정 필수 확인 — **baud rate 1 MBaud(= bitrate 1 Mbps, 같은 말) / 11-bit 표준 ID / byte order Intel(little-endian)**.

**PC 쪽:**

```bash
 candump -td can0
```

✅ **성공 판정**: `0x200`, `0x201`, `0x202`가 각각 ~10ms 간격으로 계속 찍힘.

값까지 해석해서 보려면 (공학 단위 출력):

```bash
python3 ~/FMA_ws/src/bridge_dspace/tools/can_dump.py --iface can0
```

✅ `POSE x=1.000 ...` 처럼 dSPACE가 넣은 값 그대로 나오면 **byte order까지 정상**.

| 실패 증상 | 처방 |
|---|---|
| candump 완전 침묵 | 배선(2단계) 재확인 → 아래 "버스 에러 확인" |
| 프레임은 오는데 값이 이상함 | dSPACE RTI CAN byte order가 Motorola로 돼 있을 가능성 — Intel로 변경 |
| ID가 다르게 찍힘 | dSPACE 모델의 ID 설정을 PROTOCOL.md 표와 대조 |

**버스 에러 확인** (양쪽 어느 단계든 문제 시):

```bash
ip -details -statistics link show can0
```

- `ERROR-PASSIVE` / `BUS-OFF` 상태, 또는 `bus-error` 카운트 증가 → **bitrate 불일치 또는 종단저항 문제**가 대부분.
- bus-off는 100ms 후 자동 복구되도록 설정돼 있으나, 근본 원인(배선/설정)을 잡아야 한다.

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

✅ **성공 판정 (PC측)**: 매 10ms마다 `0x101`(점 1개) → `0x100`(헤더) 순서로 찍힘.
✅ **성공 판정 (dSPACE측, 손상민)**: RTI CAN 수신에서 `0x100` counter가 매 주기 +1,
v_ref = 300 (=0.3 m/s), `0x101` x = 500 (=0.5 m).

> ⚠ candump에 내 송신 프레임이 **안 보이면** dSPACE가 ACK를 안 하는 것 (SocketCAN은
> 전송 성공한 프레임만 에코). 3단계의 "버스 에러 확인"으로.

| dSPACE 측이 꼭 지킬 것 | 근거 |
|---|---|
| point 프레임은 버퍼링만, **`0x100` 수신 시점에 n_points개 latch** | PROTOCOL.md 커밋 규칙 |
| n_points는 헤더에서 읽기 (현재 모든 소스 1 — 확장 대비 가변 필드, 20개 아님!) | 점은 전 스테이트 1개 |
| watchdog: `0x100` counter 30ms 미갱신 → v_ref=0, 조향 유지 | CLAUDE.md §3 |

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
ip -br link show can0                                        # 인터페이스 상태
ip -details -statistics link show can0                       # 에러 카운터·버스 상태
candump -td can0                                             # 원시 프레임 + 시간차
python3 ~/FMA_ws/src/bridge_dspace/tools/can_dump.py --iface can0   # 공학 단위 해석
cansend can0 100#0100000000000000                            # 수동 프레임 1개 송신 (can-utils)
python3 ~/FMA_ws/src/bridge_dspace/tools/protocol_selftest.py       # 프로토콜 로직 셀프테스트
```
