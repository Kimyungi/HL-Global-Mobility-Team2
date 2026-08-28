# PC ↔ dSPACE CAN 프로토콜 v5 (CAN FD, 64바이트)

CLAUDE.md §3의 바이너리 구현. dSPACE 측(RTI CAN FD 블록셋)과 **반드시 이 문서 기준으로 합의**할 것 (담당: 손상민).
변경 시 이 문서 → `src/can_protocol.hpp` 구조체 → dSPACE 모델 순서로 갱신.

> v1(Ethernet UDP)은 dSPACE 측 이더넷 불가로 폐기. 논리 계약(§3의 ref_points/v_ref/flags/vehicle_vector,
> watchdog)은 동일하고 **물리 계층과 프레임 분할만** 변경됨.
>
> **v2 → v3 (2026-08-28 오전): 어댑터 PCAN → Kvaser Leaf v3, 와이어 포맷 classic → CAN FD.**
> 페이로드는 8바이트 그대로였다 (1단계 이관).
>
> **v3 → v5 (2026-08-28 오후, PR #52 손상민): 64바이트 페이로드 + 참조점 1개.**
> dSPACE 가 먼저 이 계약으로 넘어가 있어 PC 를 맞췄다(팀장 결정). 실측으로 확인:
> `0x200` 이 64B 로 도착하고 `<6d QQ>` 로 정확히 파싱된다.
> - `0x100` TARGET_HEADER 는 **8바이트 그대로** (v2 부터 불변)
> - `0x101` 이 64B 가 되고 **참조점 1개**만 싣는다 — `0x102`~`0x114` 소멸
> - `0x200` 하나가 v3 의 `0x200`/`0x201`/`0x202` 를 대체 → **커밋(latch) 규칙 소멸**
> - `str_ref`(MPC 명령 조향각) 신규 — 실현율을 직접 측정할 수 있게 된다
> - 양자화 없음: 전부 IEEE-754 **float64**
>
> ⚠ **참조점 1개는 측정된 위험을 알고 내린 결정이다** — 아래 "TX" 절의 경고 참조.

## 공통

- **CAN FD (ISO), 11-bit 표준 ID. 페이로드 길이는 프레임마다 다르다:**
  | ID | 길이 | 비고 |
  |---|---|---|
  | `0x100` TARGET_HEADER | **8 B** | v2 부터 불변 |
  | `0x101` MPC_TARGET | **64 B** | float64 ×7 + uint64 |
  | `0x200` VEH_FEEDBACK | **64 B** | float64 ×6 + uint64 ×2 |
  - **길이가 계약을 가른다.** PC 는 `0x200` 을 len=64 면 v5, len=8 이면 v3 로 해석한다
    (dSPACE 가 되돌아가도 링크가 죽지 않게 하는 폴백). 그 외 길이는 경고 후 폐기.
  - FD 는 9~11 바이트 길이가 없다(8 다음이 12). DLC 를 정확히 8 또는 64 로 맞출 것.
- **비트레이트 — nominal 1 Mbps / data 2 Mbps, BRS on**
  - dSPACE 설정 화면에는 **"baud rate"로 표기됨. CAN에서 baud rate = bitrate (같은 값)**
  - nominal 1 Mbps 는 v2 와 동일 — 기존 배선·종단이 이 속도로 검증돼 있다
  - data 2 Mbps: 8바이트 페이로드에서는 더 올려도 얻는 게 없다(아래 버스 부하). 배선
    품질 요구만 커지므로 보수적으로 잡았다
- 페이로드 내 byte order: **little-endian (Intel format)** — dSPACE RTI CAN 블록에서 Intel로 설정할 것
- float = IEEE 754 single (4 bytes), int16 = 2's complement
- PC 측: **Kvaser Leaf v3 (USB)** — 메인라인 `kvaser_usb` 드라이버가 표준 SocketCAN(`can0`)으로 잡는다.
  ⚠ Kvaser 자체 배포 `linuxcan` 드라이버를 설치하지 말 것 — SocketCAN 이 아닌 `/dev/kvaser*`
  로 잡혀 이 스택 전체가 못 쓴다. 루프백 테스트는 가상 CAN (`vcan0`, **MTU 72**)

### FD 파라미터 — dSPACE 와 합의할 5개 (★ 전부 일치해야 링크가 붙는다)

classic 시절엔 "1 Mbps / Intel" 두 줄이면 끝이었다. FD 는 아래가 전부 맞아야 한다.
하나라도 어긋나면 ERROR-PASSIVE / BUS-OFF 로 떨어진다.

| # | 항목 | 팀 표준값 | PC 측 설정 위치 | 어긋나면 |
|---|---|---|---|---|
| 1 | nominal bitrate | **1 Mbps** | `tools/can_up.sh` `NOM_BITRATE` | 즉시 BUS-OFF |
| 2 | data bitrate | **2 Mbps** | `tools/can_up.sh` `DATA_BITRATE` | 데이터 구간에서만 에러 — 간헐 유실로 보인다 |
| 3 | **BRS** (bit rate switch) | **on** | 노드 파라미터 `can_fd_brs` (기본 true) | 붙긴 하는데 **속도 이득이 0** (전 구간 nominal). "FD 로 바꿨는데 왜 안 빨라지지"의 단골 원인 |
| 4 | ISO CAN FD vs non-ISO(Bosch) | **ISO** | `ip link ... fd on` (기본 ISO. non-ISO 면 `fd-non-iso on`) | CRC 불일치 — **전 프레임 에러** |
| 5 | sample point (nominal / data) | **80% / 80%** | `tools/can_up.sh` `*_SAMPLE_POINT` | 케이블이 길어질수록 간헐 에러. 짧은 배선에선 안 드러나다가 실차에서 터진다 |

> PC 측 1·2·5 는 `tools/can_up.sh` 한 파일에 모여 있고, systemd-networkd 경로
> (`tools/can_setup/80-can0.network`)에도 **같은 값이 중복**돼 있다 — networkd 가
> udev 를 이기기 때문이다(2026-08-03 실사례). **바꿀 때 두 파일을 함께 고칠 것.**

### 와이어 포맷 전환 스위치 (A/B 진단용)

- **PC TX**: `can_fd` 파라미터 (기본 `true`). `ros2 launch bridge_dspace bridge.launch.py can_fd:=false`
  로 classic 프레임 송신으로 되돌린다 — 인터페이스 설정은 그대로 둬도 된다
  (FD 인터페이스는 classic 프레임을 그대로 실어 보낸다).
- **PC RX**: 스위치가 없다. 인터페이스가 FD 면 **항상** classic·FD 를 모두 받는다.
  RX 를 파라미터로 묶으면 dSPACE 만 먼저 FD 로 넘어간 순간 커널이 프레임을 **에러 없이
  통째로 버려** "배선은 멀쩡한데 무수신"이 된다 — 2026-08-25 RX 0건 사고와 같은 침묵이다.
- `can_zero`(종료 시 목표값 0 복귀 가드)는 **인터페이스 MTU 로 자동 판정**한다. 안전
  경로에 인자를 요구하면 한 번 빠뜨렸을 때 dSPACE 가 마지막 v_ref 를 그대로 문다.

## CAN ID 맵

| ID | 방향 | 이름 | 주기 |
|---|---|---|---|
| `0x100` | PC → dSPACE | TARGET_HEADER, 8 B (커밋 프레임, **watchdog 입력**) | 10ms |
| `0x101` | PC → dSPACE | MPC_TARGET, 64 B (참조점 1개) | 10ms |
| `0x200` | dSPACE → PC | VEH_FEEDBACK, 64 B (**단일 프레임 — 커밋 규칙 없음**) | 10ms |
| ~~`0x102`~`0x114`~~ | — | v3 잔재. v5 에서 미사용 (ID 는 예약 유지) | — |
| ~~`0x201`,`0x202`~~ | — | v3 잔재. RX 폴백 해석에만 쓰인다 | — |

- `0x000`~`0x0FF`: 예약 (향후 긴급/진단용). `0x300` 이상: 하위 제어 내부용으로 자유 — 단 이 문서에 등록 후 사용.

## TX — PC → dSPACE, 매 10ms, n_points + 1 프레임 (가변)

**유효 점만 송신한다** — REF_POINT_0 … REF_POINT_(n_points−1) → 마지막에 TARGET_HEADER.
dSPACE는 point 프레임을 버퍼에 쌓다가 **TARGET_HEADER 수신 시점에 n_points개 세트를 원자적으로
latch**한다 (프레임 간 반쯤 갱신된 세트를 MPC가 읽는 것 방지). 점이 sparse해도 되는 이유:
dSPACE 궤적 생성(quintic)이 목표점(들)로부터 MPC 지평(200ms/N=20) 궤적을 만들기 때문.

**점 수: 1점** (v5, 팀장 결정 2026-08-28). 주기당 TX = **2 프레임** (`0x101` → `0x100`).
MGM 이 20점을 만들어도 브리지는 **첫 점**만 싣는다 (v3 의 REF_POINT_0 와 같은 점).

> ## ⚠ 1점은 측정된 위험을 알고 내린 결정이다 — 실차 재확인 필수
>
> CLAUDE.md §3 ② 는 "실운용에서 검증된 포맷은 20점뿐", 1점 무반응 8/8 실측을 기록하고 있다.
> 더 구체적으로, **20점 보간이 첫 점을 목표의 1/20(≈7.5cm)에 두어 κ 를 10~20배 부풀리는 것은
> 결함이 아니라 dSPACE 곡률 실현율 10~55% 를 메우는 보상**이었다. 2026-08-15 에 그 첫 점을
> 1.2m 로 "정상화"했다가 같은 회피 목표(|y|≈0.29m)에서:
>
> | | 변경 전 | 변경 후 |
> |---|---|---|
> | 헤딩 | +13.0° | +4.3° |
> | `str` | 0.089 | 0.047 |
> | 횡변위 | 0.26 m | 0.08 m |
>
> → **콘 회피 실패·estop** (run_0815_153633, 당일 즉시 복구).
>
> 1점에서는 그 보상이 구조적으로 존재할 수 없다. **회피 거동을 실차에서 반드시 재확인할 것.**
> `dx`/`dy`/`dyaw` 가 그 정보를 대신 나르도록 정의되면 재검토 대상이다 (현재 의미 미정).
> 되돌리려면 `can_protocol.hpp` 의 `kNumPoints` 와 이 절을 함께 되돌린다.

주의: 조향이 제대로 반응하려면 **v_ref ≥ 0.5 m/s** 필요 (CLAUDE.md §3 ①).

### MPC_TARGET (`0x101`) — 64 bytes

| offset | 형식 | 필드 | 단위 |
|---|---|---|---|
| 0 | f64 | x | m |
| 8 | f64 | y | m |
| 16 | f64 | yaw | rad |
| 24 | f64 | curvature | 1/m |
| 32 | f64 | dx | **의미 미정 — 0 송신** |
| 40 | f64 | dy | **의미 미정 — 0 송신** |
| 48 | f64 | dyaw | **의미 미정 — 0 송신** |
| 56 | u64 | update | **의미 미정 — 0 송신** |

- vehicle frame (생성 시점 차량 = 0,0,0). **양자화 없음** — v3 의 int16 스케일은 v5 에서 안 쓴다.
- `dx`/`dy`/`dyaw`/`update` 는 PR #52 에서 의미가 정해지지 않았다. 팀장 결정(2026-08-28)에
  따라 **0 으로 채운다.** 의미가 정해지면 이 표와 `can_protocol.hpp` 를 함께 갱신할 것.

### TARGET_HEADER (`0x100`) — 8 bytes

| offset | 형식 | 필드 | 설명 |
|---|---|---|---|
| 0 | u16 | counter | 송신마다 +1 (wrap). **watchdog 판정 입력** — 30ms(3주기) 미갱신 시 v_ref=0, 조향 유지 |
| 2 | u8 | state | 0=lane, 1=waypoint, 2=avoid, 3=parking |
| 3 | u8 | n_points | 유효 포인트 수 (1~20) — 이번 주기에 송신된 REF_POINT 프레임 수 |
| 4 | i16 | v_ref | 1 mm/s LSB. [±32.767 m/s] 최종 목표 속도. 정지 = 0. **음수 = 후진** (2026-08-24: MGM §4 후진 탈출이 처음으로 음수를 낸다 — 그전까지 PC는 0 이상만 보냈다. dSPACE MPC·하위 PI 가 음수 목표속도를 그대로 후진으로 처리한다는 팀 확인을 받았으나 **실차 재확인 권장**) |
| 6 | u16 | reserved | 0 |

- **브리지는 수신한 TargetRef를 즉시 송신한다 (자체 재송신 없음).** MGM이 죽으면 송신도 멈춰야
  dSPACE watchdog이 동작한다 — 브리지에 keep-alive를 넣지 말 것.
- **watchdog은 TARGET_HEADER의 counter만 본다.** point 프레임 수신 여부는 판정에 쓰지 않는다.

### watchdog 상세 (dSPACE 측 구현 규정 — 담당: 손상민)

- **판정 기준은 "counter 값 변화"이지 프레임 수신이 아니다.** 헤더가 도착해도 counter가
  직전과 같으면 생존 신호로 치지 않는다 (타이머 리셋 없음).
- **발동 동작: v_ref = 0 강제 (감속 정지), str_ref는 직전 값 유지** — 0(중립)으로 풀지 말 것.
  코너에서 통신 두절 시 조향을 풀면 감속 중 트랙을 이탈한다 (급조향 금지).
- **부팅 초기 상태 = fault에서 시작.** 기동 시 "마지막 counter 갱신 시각"을 부팅 시각으로
  초기화 — 첫 헤더가 타임아웃 내에 안 오면 자연히 fault(v_ref=0) 대기. "첫 수신 전에는
  watchdog 비활성"으로 구현하지 말 것 (PC 없이 부팅한 차가 잔류 목표값으로 움직이는 것 방지).
- **복구는 자동.** counter가 다시 갱신되면 즉시 정상 추종으로 복귀 (별도 해제 절차·래치 없음).
- **타임아웃은 튜너블 파라미터로.** 기본 30ms(3주기) — CLAUDE.md §7 실시간성 검증 결과에
  따라 조정될 수 있으므로(예: 50ms) dSPACE 모델에 하드코딩하지 말 것.
- 레퍼런스 동작: `dspace_sim_node` (위 규정 전부 구현, `watchdog_timeout_ms` 파라미터) —
  PC 단독 루프백으로 실기 구현과 동작 대조 가능.

## RX — dSPACE → PC, 매 10ms **1프레임**

**커밋(latch) 규칙이 없다.** 한 프레임이 곧 한 주기 세트이므로 PC 는 `0x200` 수신 즉시
`/vehicle/vector` 를 퍼블리시한다. v3 의 "POSE→VEL→COMMIT 순서" 규정은 v5 에서 소멸했다.

**모든 스테이트에서 상시 송신 — parking 중에도 끊지 말 것.** 주차 스택(stack_parking)의
로컬맵·경로 추종이 vehicle vector를 입력으로 쓴다.

### VEH_FEEDBACK (`0x200`) — 64 bytes

| offset | 형식 | 필드 | 단위 |
|---|---|---|---|
| 0 | f64 | x | m |
| 8 | f64 | y | m |
| 16 | f64 | yaw | rad |
| 24 | f64 | v | m/s |
| 32 | f64 | str | rad — **실제** 조향각 |
| 40 | f64 | str_ref | rad — **MPC 명령** 조향각 (v5 신규) |
| 48 | u64 | counter | dSPACE 송신 카운터 (매 주기 +1) |
| 56 | u64 | reserved | 0 |

- **`str_ref` 는 실현율을 직접 재게 해 준다.** CLAUDE.md §3 의 `명령δ / PC 기하δ`(43~59%)는
  지금까지 간접 추정이었다. `str_ref`(명령)와 `str`(실제)가 함께 오면
  `실제δ/명령δ` 와 `명령δ/PC 기하δ` 를 한 로그에서 분리할 수 있다.
- **⚠ 미해결 (2026-08-28 실측):** dSPACE 의 `counter` 가 **매 주기 +1 이 아니다** —
  500 프레임(5초) 동안 값이 고정돼 있었다. `dspace_merge.py` 의 틱 정합
  (`bag_index = counter − off`)이 이 값에 의존하므로 손상민 확인 필요.
  상태값(x/y/yaw/v/str)도 아직 전부 0 이다.

### v3 폴백 (RX 전용)

PC 는 `0x200` 이 len=8 로 오면 v3 로 해석하고 `0x201`/`0x202` 세트를 기다린다.
dSPACE 가 되돌아가도 링크가 죽지 않게 하는 장치이며, **길이로 구분되므로 모호하지 않다.**
TX 에는 폴백이 없다 — 페이로드 계약은 양쪽이 같아야 하므로 A/B 진단 가치가 없다.

## 버스 부하

> ⚠ 이 절의 v2 수치("5프레임 ≈ 7%")는 n_points=1 가정이라 **2026-08-15 이후로 틀린 값**이었다.
> 실운용 20점 기준으로 아래와 같이 정정한다 (2026-08-28).

주기당 프레임: **TX 2**(MPC_TARGET + TARGET_HEADER) **+ RX 1 = 3 프레임 / 10ms.**

| 구성 | 프레임 수 | 10ms 틱당 | 부하 |
|---|---|---|---|
| classic 1 Mbps (v2, 20점) | 24 | ~3.24 ms | **~32%** |
| CAN FD 1M/2M, 8B ×20점 (v3) | 24 | ~1.8 ms | **~18%** |
| **CAN FD 1M/2M, 64B ×1점 (v5, 현행)** | **3** | **~0.33 ms** | **~3%** |

부하가 32% → 3% 로 떨어진 것은 대부분 **프레임 수가 24 → 3 이 된 덕**이다.
다만 그 감소의 큰 부분은 점 수를 20 → 1 로 줄인 결과이므로, 위 TX 절의 경고와 함께 읽을 것 —
**부하는 목표가 아니었고, 여유가 생긴 것이 점 수 축소를 정당화하지 않는다.**

## PC 측 CAN 인터페이스 설정

**최초 1회 자동 셋업 설치 (권장)** — 이후로는 어댑터를 꽂기만 하면 can0이 CAN FD 로 자동 up:

```bash
sudo src/bridge_dspace/tools/can_setup/install.sh          # 실차 PC
sudo src/bridge_dspace/tools/can_setup/install.sh --vcan   # 개발 머신 (+vcan0 상시 생성)
```

> ⚠ systemd-networkd가 활성인 머신에서는 `/etc/systemd/network/`의 can0 설정이
> udev·서비스보다 **나중에 적용되어 이긴다**. install.sh가 팀 표준
> `80-can0.network`(CAN FD 1M/2M)를 함께 설치해 이 경로도 맞춘다 — 다른 비트레이트의
> can0 `.network` 파일을 수동으로 만들지 말 것 (2026-08-03 500k 잔재로 ERROR-PASSIVE 실사례).
> **`FDMode=yes` 가 빠지면 MTU 가 16 으로 남아** 브리지가 기동에서 죽는다 (조용히
> classic 으로 강등되지 않는 것은 의도다).

수동 설정 (자동 셋업 미설치 시):

```bash
# 실기 (Kvaser Leaf v3, CAN FD 1M/2M)
sudo ip link set can0 down
sudo ip link set can0 type can \
     bitrate 1000000 sample-point 0.8 \
     dbitrate 2000000 dsample-point 0.8 \
     fd on restart-ms 100
sudo ip link set can0 up

# 루프백 테스트용 가상 CAN — MTU 72 여야 FD 프레임이 실린다
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 mtu 72
sudo ip link set vcan0 up
```

**FD 로 올라왔는지 확인 — MTU 하나면 된다** (PC 코드도 이 값으로 판정한다):

```bash
cat /sys/class/net/can0/mtu     # 72 = CAN FD ✔ / 16 = classic 전용 ✘
ip -details link show can0      # bitrate·dbitrate·<FD> 플래그
```

## 루프백 검증 절차 (부트스트래핑 ①)

> 실기(dSPACE 연결) 단계별 검증은 **`CAN_BRINGUP.md`** — 배선·수신·송신·왕복·watchdog 순서의 복붙 가이드.

1. PC 단독 (dSPACE 없이): 위 vcan0 설정(**MTU 72**) 후 `ros2 launch bridge_dspace loopback_test.launch.py`
   — dummy_ref_publisher → can_bridge → **dspace_sim_node**(dSPACE 에뮬레이터, watchdog 동작 포함) → vehicle vector 회신 → `/vehicle/vector` 토픽 확인.
2. 실기: dspace_sim_node 대신 실제 dSPACE (`can_interface:=can0`). 더미 ref(직선, v_ref 0.3)로 바퀴 반응 + vehicle vector 회신 확인.
3. watchdog 검증: dummy_ref_publisher를 죽이고 30ms 후 dSPACE가 v_ref=0 처리하는지 확인.
4. 저수준 디버그: `candump can0` (can-utils) 또는 `python3 tools/can_dump.py` (본 프로토콜 해석 출력).
   `can_dump.py` 는 각 줄에 **STD / FD / FD-BRS** 를 찍으므로 와이어 포맷 전환을 눈으로 확인할 수 있다.
5. 와이어 포맷 계약 단위시험 (CAN 인터페이스 불필요): `colcon test --packages-select bridge_dspace`
   — `socketcan_fd_test` 가 "FD 로 보내도 페이로드 바이트가 classic 과 동일"을 고정한다.
