# PC ↔ dSPACE CAN 프로토콜 v2

CLAUDE.md §3의 바이너리 구현. dSPACE 측(RTI CAN 블록셋)과 **반드시 이 문서 기준으로 합의**할 것 (담당: 손상민).
변경 시 이 문서 → `src/can_protocol.hpp` 구조체 → dSPACE 모델 순서로 갱신.

> v1(Ethernet UDP)은 dSPACE 측 이더넷 불가로 폐기. 논리 계약(§3의 ref_points/v_ref/flags/vehicle_vector,
> watchdog)은 동일하고 **물리 계층과 프레임 분할만** 변경됨.

## 공통

- **CAN 2.0A, 11-bit 표준 ID, 페이로드 8 bytes 고정** (CAN FD 아님)
- **Bitrate: 1 Mbps** (권장 기본값). 500 kbps도 동작하나 버스 부하 ~65%로 여유 없음 — 아래 부하 계산 참조
- 페이로드 내 byte order: **little-endian (Intel format)** — dSPACE RTI CAN 블록에서 Intel로 설정할 것
- float = IEEE 754 single (4 bytes), int16 = 2's complement
- PC 측: Linux SocketCAN (`can0`), 루프백 테스트는 가상 CAN (`vcan0`)

## CAN ID 맵

| ID | 방향 | 이름 | 주기 |
|---|---|---|---|
| `0x100` | PC → dSPACE | TARGET_HEADER (커밋 프레임, **watchdog 입력**) | 10ms |
| `0x101`~`0x114` | PC → dSPACE | REF_POINT_0 ~ REF_POINT_19 (ID = 0x101 + index) | 10ms |
| `0x200` | dSPACE → PC | VEH_POSE | 10ms |
| `0x201` | dSPACE → PC | VEH_VEL | 10ms |
| `0x202` | dSPACE → PC | VEH_COMMIT (커밋 프레임) | 10ms |

- `0x000`~`0x0FF`: 예약 (향후 긴급/진단용). `0x300` 이상: 하위 제어 내부용으로 자유 — 단 이 문서에 등록 후 사용.

## TX — PC → dSPACE, 매 10ms 21프레임

**송신 순서: REF_POINT_0 … REF_POINT_19 → 마지막에 TARGET_HEADER.**
dSPACE는 point 프레임을 버퍼에 쌓다가 **TARGET_HEADER 수신 시점에 20점 세트를 원자적으로 latch**한다
(프레임 간 반쯤 갱신된 세트를 MPC가 읽는 것 방지).

### REF_POINT_i (`0x101 + i`, i = 0…19) — 8 bytes

| offset | 형식 | 필드 | 스케일 (LSB) | 범위 |
|---|---|---|---|---|
| 0 | i16 | x | 1 mm | ±32.767 m |
| 2 | i16 | y | 1 mm | ±32.767 m |
| 4 | i16 | yaw | 1e-4 rad | ±3.2767 rad (±π 커버) |
| 6 | i16 | curvature | 5e-4 1/m | ±16.38 1/m (최소 회전반경 6.1 cm) |

- vehicle frame (생성 시점 차량 = 0,0,0). MPC 지평 200ms × 최대 속도에서 point 거리는 수 m 이내 →
  1mm 분해능·±32m 범위로 충분.
- n_points 미만 슬롯은 마지막 점 복제 (v1과 동일).

### TARGET_HEADER (`0x100`) — 8 bytes

| offset | 형식 | 필드 | 설명 |
|---|---|---|---|
| 0 | u16 | counter | 송신마다 +1 (wrap). **watchdog 판정 입력** — 30ms(3주기) 미갱신 시 v_ref=0, 조향 유지 |
| 2 | u8 | state | 0=lane, 1=waypoint, 2=avoid, 3=parking |
| 3 | u8 | n_points | 유효 포인트 수 (≤ 20) |
| 4 | i16 | v_ref | 1 mm/s LSB. [±32.767 m/s] 최종 목표 속도. 정지 = 0 |
| 6 | u16 | reserved | 0 |

- N=20은 MPC 예측 지평(200ms / Ts 10ms)과 일치.
- **브리지는 수신한 TargetRef를 즉시 송신한다 (자체 재송신 없음).** MGM이 죽으면 송신도 멈춰야
  dSPACE watchdog이 동작한다 — 브리지에 keep-alive를 넣지 말 것.
- **watchdog은 TARGET_HEADER의 counter만 본다.** point 프레임 수신 여부는 판정에 쓰지 않는다.

## RX — dSPACE → PC, 매 10ms 3프레임

**송신 순서: VEH_POSE → VEH_VEL → 마지막에 VEH_COMMIT.** PC는 VEH_COMMIT 수신 시점에
`/vehicle/vector` 1회 퍼블리시 (한 주기 세트 = 한 메시지).

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

프레임당 최악 ~135 bits (11-bit ID, 8B 데이터, stuffing 포함). 주기당 TX 21 + RX 3 = 24프레임 → 2,400 프레임/s ≈ **324 kbit/s**.

| bitrate | 부하 |
|---|---|
| 1 Mbps | ~32% ✅ 권장 |
| 500 kbps | ~65% ⚠ 동작은 하나 다른 노드 추가 여유 없음 |

## PC 측 CAN 인터페이스 설정

```bash
# 실기 (USB-CAN 어댑터 등, 1 Mbps)
sudo ip link set can0 up type can bitrate 1000000

# 루프백 테스트용 가상 CAN
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
```

## 루프백 검증 절차 (부트스트래핑 ①)

1. PC 단독 (dSPACE 없이): 위 vcan0 설정 후 `ros2 launch bridge_dspace loopback_test.launch.py`
   — dummy_ref_publisher → can_bridge → **dspace_sim_node**(dSPACE 에뮬레이터, watchdog 동작 포함) → vehicle vector 회신 → `/vehicle/vector` 토픽 확인.
2. 실기: dspace_sim_node 대신 실제 dSPACE (`can_interface:=can0`). 더미 ref(직선, v_ref 0.3)로 바퀴 반응 + vehicle vector 회신 확인.
3. watchdog 검증: dummy_ref_publisher를 죽이고 30ms 후 dSPACE가 v_ref=0 처리하는지 확인.
4. 저수준 디버그: `candump can0` (can-utils) 또는 `python3 tools/can_dump.py` (본 프로토콜 해석 출력).
