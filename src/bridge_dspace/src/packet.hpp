// PC↔dSPACE UDP 프레임 정의 — 단일 진실 원천은 PROTOCOL.md.
// 이 구조체를 바꾸면 PROTOCOL.md와 dSPACE 모델도 함께 갱신할 것.
#ifndef BRIDGE_DSPACE__PACKET_HPP_
#define BRIDGE_DSPACE__PACKET_HPP_

#include <cstdint>

namespace bridge_dspace
{

constexpr uint32_t kMagicTx = 0x464D4131;  // "FMA1"
constexpr uint32_t kMagicRx = 0x464D4132;  // "FMA2"
constexpr int kNumPoints = 20;             // MPC 예측 지평 200ms / Ts 10ms

#pragma pack(push, 1)

struct RefPointWire
{
  float x;
  float y;
  float yaw;
  float curvature;
};

struct TxFrame  // PC → dSPACE, 336 bytes
{
  uint32_t magic;
  uint32_t counter;   // watchdog 판정 입력
  uint8_t state;      // 0=lane 1=waypoint 2=avoid 3=parking
  uint8_t n_points;
  uint16_t reserved;
  float v_ref;
  RefPointWire points[kNumPoints];
};

struct RxFrame  // dSPACE → PC, 32 bytes
{
  uint32_t magic;
  uint32_t counter;
  float x;
  float y;
  float yaw;
  float v;
  float str;
  uint32_t reserved;
};

#pragma pack(pop)

static_assert(sizeof(TxFrame) == 336, "TxFrame must match PROTOCOL.md");
static_assert(sizeof(RxFrame) == 32, "RxFrame must match PROTOCOL.md");

}  // namespace bridge_dspace

#endif  // BRIDGE_DSPACE__PACKET_HPP_
