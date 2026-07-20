# PC ↔ dSPACE UDP 프로토콜 v1

CLAUDE.md §3의 바이너리 구현. dSPACE 측(Simulink UDP 블록)과 **반드시 이 문서 기준으로 합의**할 것.
변경 시 이 문서 → `src/udp_bridge_node.cpp`의 `packet.hpp` 구조체 → dSPACE 모델 순서로 갱신.

## 공통

- Byte order: **little-endian** (x86 PC ↔ dSPACE DS1202 동일)
- 정렬: 1-byte packed (패딩 없음)
- float = IEEE 754 single (4 bytes)
- 포트 (기본값, launch 파라미터로 변경 가능):
  - TX (PC → dSPACE): dSPACE `50001`
  - RX (dSPACE → PC): PC `50002`

## TX 프레임 — PC → dSPACE, 10ms, 336 bytes

| offset | 크기 | 필드 | 설명 |
|---|---|---|---|
| 0 | u32 | magic | `0x464D4131` ("FMA1") — 프레임 검증 |
| 4 | u32 | counter | 송신마다 +1. **watchdog 판정 입력** — 30ms(3주기) 미갱신 시 v_ref=0, 조향 유지 |
| 8 | u8 | state | 0=lane, 1=waypoint, 2=avoid, 3=parking |
| 9 | u8 | n_points | 유효 포인트 수 (≤ 20) |
| 10 | u16 | reserved | 0 |
| 12 | f32 | v_ref | [m/s] 최종 목표 속도. 정지 = 0 |
| 16 | f32×4×20 | ref_points[20] | {x, y, yaw, curvature} × 20, vehicle frame. n_points 미만은 마지막 점 복제 |

- N=20은 MPC 예측 지평(200ms / Ts 10ms)과 일치.
- **브리지는 수신한 TargetRef를 즉시 송신한다 (자체 재송신 없음).** MGM이 죽으면 송신도 멈춰야 dSPACE watchdog이 동작한다 — 브리지에 keep-alive를 넣지 말 것.

## RX 프레임 — dSPACE → PC, 10ms, 32 bytes

| offset | 크기 | 필드 | 설명 |
|---|---|---|---|
| 0 | u32 | magic | `0x464D4132` ("FMA2") |
| 4 | u32 | counter | dSPACE 송신 카운터 |
| 8 | f32 | x | 상태 추정 (kinematic bicycle model) |
| 12 | f32 | y | |
| 16 | f32 | yaw | [rad] |
| 20 | f32 | v | [m/s] |
| 24 | f32 | str | [rad] 조향각 |
| 28 | u32 | reserved | 0 |

## 루프백 검증 절차 (부트스트래핑 ①)

1. PC 단독 (dSPACE 없이): `ros2 launch bridge_dspace loopback_test.launch.py`
   — dummy_ref_publisher → udp_bridge → **dspace_sim_node**(dSPACE 에뮬레이터, watchdog 동작 포함) → vehicle vector 회신 → `/vehicle/vector` 토픽 확인.
2. 실기: dspace_sim_node 대신 실제 dSPACE. 더미 ref(직선, v_ref 0.3)로 바퀴 반응 + vehicle vector 회신 확인.
3. watchdog 검증: dummy_ref_publisher를 죽이고 30ms 후 dSPACE가 v_ref=0 처리하는지 확인.
