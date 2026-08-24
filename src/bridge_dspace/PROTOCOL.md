# PC ↔ dSPACE CAN 프로토콜 v2

CLAUDE.md §3의 바이너리 구현. dSPACE 측(RTI CAN 블록셋)과 **반드시 이 문서 기준으로 합의**할 것 (담당: 손상민).
변경 시 이 문서 → `src/can_protocol.hpp` 구조체 → dSPACE 모델 순서로 갱신.

> v1(Ethernet UDP)은 dSPACE 측 이더넷 불가로 폐기. 논리 계약(§3의 ref_points/v_ref/flags/vehicle_vector,
> watchdog)은 동일하고 **물리 계층과 프레임 분할만** 변경됨.

## 공통

- **CAN 2.0A, 11-bit 표준 ID, 페이로드 8 bytes 고정** (CAN FD 아님)
- **Bitrate: 1 Mbps** (권장 기본값) — dSPACE 설정 화면에는 **"baud rate"로 표기됨. CAN에서 baud rate = bitrate (같은 값, 1 MBaud 선택)**. sample point는 기본값 유지. 500 kbps도 동작하나 양쪽 일치 필수
- 페이로드 내 byte order: **little-endian (Intel format)** — dSPACE RTI CAN 블록에서 Intel로 설정할 것
- float = IEEE 754 single (4 bytes), int16 = 2's complement
- PC 측: **PCAN(USB) 어댑터** — peak_usb 드라이버가 커널 기본 포함이라 표준 SocketCAN(`can0`)으로 잡힘. 루프백 테스트는 가상 CAN (`vcan0`)

## CAN ID 맵

| ID | 방향 | 이름 | 주기 |
|---|---|---|---|
| `0x100` | PC → dSPACE | TARGET_HEADER (커밋 프레임, **watchdog 입력**) | 10ms |
| `0x101`~`0x114` | PC → dSPACE | REF_POINT_0 ~ REF_POINT_19 (ID = 0x101 + index) | 10ms |
| `0x200` | dSPACE → PC | VEH_POSE | 10ms |
| `0x201` | dSPACE → PC | VEH_VEL | 10ms |
| `0x202` | dSPACE → PC | VEH_COMMIT (커밋 프레임) | 10ms |

- `0x000`~`0x0FF`: 예약 (향후 긴급/진단용). `0x300` 이상: 하위 제어 내부용으로 자유 — 단 이 문서에 등록 후 사용.

## TX — PC → dSPACE, 매 10ms, n_points + 1 프레임 (가변)

**유효 점만 송신한다** — REF_POINT_0 … REF_POINT_(n_points−1) → 마지막에 TARGET_HEADER.
dSPACE는 point 프레임을 버퍼에 쌓다가 **TARGET_HEADER 수신 시점에 n_points개 세트를 원자적으로
latch**한다 (프레임 간 반쯤 갱신된 세트를 MPC가 읽는 것 방지). 점이 sparse해도 되는 이유:
dSPACE 궤적 생성(quintic)이 목표점(들)로부터 MPC 지평(200ms/N=20) 궤적을 만들기 때문.

**소스별 점 수 (팀 합의, 2026-07-29 / 2026-08-10 재확정): 모든 스테이트 1점** — 단 avoid는 2점까지 허용.
n_points = 1(avoid 최대 2), 주기당 TX 2~3프레임(`0x101`[+`0x102`] + `0x100`). n_points는 확장 대비 가변 필드로 유지.
주의: 조향이 제대로 반응하려면 **v_ref ≥ 0.4 m/s** 필요 (2026-08-10 실측 — 저속에서 str 무반응은 점 수 문제가 아님).

ID `0x101`~`0x114`는 최대 20점 폭으로 예약 — 나중에 점 수를 늘려도 ID 맵은 불변.

### REF_POINT_i (`0x101 + i`, i = 0…19) — 8 bytes

| offset | 형식 | 필드 | 스케일 (LSB) | 범위 |
|---|---|---|---|---|
| 0 | i16 | x | 1 mm | ±32.767 m |
| 2 | i16 | y | 1 mm | ±32.767 m |
| 4 | i16 | yaw | 1e-4 rad | ±3.2767 rad (±π 커버) |
| 6 | i16 | curvature | 5e-4 1/m | ±16.38 1/m (최소 회전반경 6.1 cm) |

- vehicle frame (생성 시점 차량 = 0,0,0). 목표점은 차량 전방 수 m 이내 →
  1mm 분해능·±32m 범위로 충분.
- `i ≥ n_points`인 REF_POINT 프레임은 **송신하지 않는다** (수신 측도 무시할 것).

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

## RX — dSPACE → PC, 매 10ms 3프레임

**송신 순서: VEH_POSE → VEH_VEL → 마지막에 VEH_COMMIT.** PC는 VEH_COMMIT 수신 시점에
`/vehicle/vector` 1회 퍼블리시 (한 주기 세트 = 한 메시지).

**모든 스테이트에서 상시 송신 — parking 중에도 끊지 말 것.** 주차 스택(stack_parking)의
로컬맵·경로 추종이 vehicle vector를 입력으로 쓴다.

상태 추정값은 localization 보정에 쓰이고 odom 누적 범위가 커질 수 있어 **양자화 없이 f32 유지**.

### VEH_POSE (`0x200`) — 8 bytes

| offset | 형식 | 필드 |
|---|---|---|
| 0 | f32 | x [m] |
| 4 | f32 | y [m] |

### VEH_VEL (`0x201`) — 8 bytes

| offset | 형식 | 필드 |
|---|---|---|
| 0 | f32 | yaw [rad] |
| 4 | f32 | v [m/s] |

### VEH_COMMIT (`0x202`) — 8 bytes

| offset | 형식 | 필드 |
|---|---|---|
| 0 | f32 | str [rad] 조향각 |
| 4 | u16 | counter (dSPACE 송신 카운터, wrap) |
| 6 | u16 | reserved |

## 버스 부하

프레임당 최악 ~135 bits (11-bit ID, 8B 데이터, stuffing 포함).

전 스테이트 동일: 주기당 TX 2 + RX 3 = **5프레임 ≈ 68 kbit/s** → 1 Mbps에서 ~7%, 500 kbps에서 ~14%.
어느 쪽이든 여유 충분 — 기본값은 1 Mbps로 하되 dSPACE 측 설정과 일치만 시키면 됨.

## PC 측 CAN 인터페이스 설정

**최초 1회 자동 셋업 설치 (권장)** — 이후로는 PCAN을 꽂기만 하면 can0이 1 Mbps로 자동 up:

```bash
sudo src/bridge_dspace/tools/can_setup/install.sh          # 실차 PC
sudo src/bridge_dspace/tools/can_setup/install.sh --vcan   # 개발 머신 (+vcan0 상시 생성)
```

> ⚠ systemd-networkd가 활성인 머신에서는 `/etc/systemd/network/`의 can0 설정이
> udev·서비스보다 **나중에 적용되어 이긴다**. install.sh가 팀 표준
> `80-can0.network`(1 Mbps)를 함께 설치해 이 경로도 맞춘다 — 다른 비트레이트의
> can0 `.network` 파일을 수동으로 만들지 말 것 (2026-08-03 500k 잔재로 ERROR-PASSIVE 실사례).

수동 설정 (자동 셋업 미설치 시):

```bash
# 실기 (PCAN, 1 Mbps)
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100

# 루프백 테스트용 가상 CAN
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

## 루프백 검증 절차 (부트스트래핑 ①)

> 실기(dSPACE 연결) 단계별 검증은 **`CAN_BRINGUP.md`** — 배선·수신·송신·왕복·watchdog 순서의 복붙 가이드.

1. PC 단독 (dSPACE 없이): 위 vcan0 설정 후 `ros2 launch bridge_dspace loopback_test.launch.py`
   — dummy_ref_publisher → can_bridge → **dspace_sim_node**(dSPACE 에뮬레이터, watchdog 동작 포함) → vehicle vector 회신 → `/vehicle/vector` 토픽 확인.
2. 실기: dspace_sim_node 대신 실제 dSPACE (`can_interface:=can0`). 더미 ref(직선, v_ref 0.3)로 바퀴 반응 + vehicle vector 회신 확인.
3. watchdog 검증: dummy_ref_publisher를 죽이고 30ms 후 dSPACE가 v_ref=0 처리하는지 확인.
4. 저수준 디버그: `candump can0` (can-utils) 또는 `python3 tools/can_dump.py` (본 프로토콜 해석 출력).
