// PC↔dSPACE CAN 프레임 정의 — 단일 진실 원천은 PROTOCOL.md.
// 이 파일을 바꾸면 PROTOCOL.md와 dSPACE 모델(RTI CAN)도 함께 갱신할 것.
#ifndef BRIDGE_DSPACE__CAN_PROTOCOL_HPP_
#define BRIDGE_DSPACE__CAN_PROTOCOL_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace bridge_dspace
{

constexpr int kNumRefPoints = 3;
constexpr int kNumTrajectoryPoints = 51;  // 10ms 주기로 전송되는 MPC XY 궤적 한 세트

// CAN ID 맵 (PROTOCOL.md)
constexpr uint32_t kIdTargetHeader = 0x100;  // 커밋 프레임 — watchdog 입력 (counter)
constexpr uint32_t kIdRefPointBase = 0x101;  // 0x101 + i, i = 0..2
constexpr uint32_t kIdVehPose = 0x200;
constexpr uint32_t kIdVehVel = 0x201;
constexpr uint32_t kIdVehCommit = 0x202;     // 커밋 프레임 — 수신 시 /vehicle/vector 퍼블리시
constexpr uint32_t kIdTrajectoryPointBase = 0x203;  // 0x203 + i, i = 0..50
constexpr uint32_t kIdTrajectoryPointLast =
  kIdTrajectoryPointBase + kNumTrajectoryPoints - 1;  // 0x235, 궤적 세트 완료

// 양자화 스케일 (LSB) — TX만 int16, RX는 f32 무손실
constexpr double kPosScale = 1e-3;   // [m]   ±32.767 m
constexpr double kYawScale = 1e-4;   // [rad] ±3.2767 rad
constexpr double kCurvScale = 5e-4;  // [1/m] ±16.38 1/m
constexpr double kVelScale = 1e-3;   // [m/s] ±32.767 m/s

inline int16_t quantize(double value, double scale)
{
  return static_cast<int16_t>(std::lround(
    std::clamp(value / scale, -32767.0, 32767.0)));
}

inline double dequantize(int16_t raw, double scale)
{
  return raw * scale;
}

// 페이로드 레이아웃 — 모두 8 bytes, little-endian (x86·DS1202 동일)
#pragma pack(push, 1)

struct RefPointPayload  // 0x101 + i
{
  int16_t x;          // kPosScale
  int16_t y;          // kPosScale
  int16_t yaw;        // kYawScale
  int16_t curvature;  // kCurvScale
};

struct TargetHeaderPayload  // 0x100 — point 프레임들 뒤에 마지막으로 송신 (latch 트리거)
{
  uint16_t counter;   // watchdog 판정 입력, wrap
  uint8_t state;      // 0=lane 1=waypoint 2=avoid 3=parking
  uint8_t n_points;
  int16_t v_ref;      // kVelScale
  uint16_t reserved;
};

struct VehPosePayload  // 0x200
{
  float x;
  float y;
};

struct VehVelPayload  // 0x201
{
  float yaw;
  float v;
};

struct VehCommitPayload  // 0x202
{
  float str;
  uint16_t counter;
  uint16_t reserved;
};

struct TrajectoryPointXyPayload  // 0x203 + i
{
  float x;
  float y;
};

#pragma pack(pop)

static_assert(sizeof(RefPointPayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(TargetHeaderPayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(VehPosePayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(VehVelPayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(VehCommitPayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(TrajectoryPointXyPayload) == 8, "must match PROTOCOL.md");

}  // namespace bridge_dspace

#endif  // BRIDGE_DSPACE__CAN_PROTOCOL_HPP_
