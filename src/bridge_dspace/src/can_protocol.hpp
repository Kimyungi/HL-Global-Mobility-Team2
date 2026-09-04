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

// ★ v5 (2026-08-28, PR #52) — 참조점 1개. 팀장 결정.
// ⚠ 이 값을 1 로 내린 것은 **측정된 위험을 알고 내린 결정**이다. CLAUDE.md §3 ② 참조:
//   20점 보간이 첫 점을 목표의 1/20(≈7.5cm)에 두어 κ 를 10~20배 부풀리는 것은 결함이
//   아니라 dSPACE 곡률 실현율 10~55% 를 메우는 **보상**이었다. 2026-08-15 에 그 첫 점을
//   1.2m 로 "정상화"했다가 헤딩 13.0°→4.3°, str 0.089→0.047 로 반토막 나
//   **콘 회피 실패·estop** 했다(run_0815_153633, 당일 복구).
//   1점에서는 그 보상이 구조적으로 존재할 수 없으므로, 회피 거동을 실차에서
//   반드시 재확인할 것. dx/dy/dyaw 가 그 정보를 대신 나르게 되면 재검토 대상이다.
constexpr int kNumPoints = 1;

// CAN ID 맵 (PROTOCOL.md)
constexpr uint32_t kIdTargetHeader = 0x100;  // 커밋 프레임 — watchdog 입력 (counter)
constexpr uint32_t kIdRefPointBase = 0x101;  // 0x101 + i, i = 0..19
constexpr uint32_t kIdVehFeedback = 0x200;   // v5 — 64B 단일 프레임. 수신 즉시 퍼블리시
// v3 (8B × 3프레임) 잔재 — RX 폴백 전용. dSPACE 가 되돌아가도 받을 수 있게 남긴다.
constexpr uint32_t kIdVehPose = 0x200;
constexpr uint32_t kIdVehVel = 0x201;
constexpr uint32_t kIdVehCommit = 0x202;

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
  uint8_t state;      // 0=lane 1=waypoint 2=avoid 3=parking 4=traffic
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

// ─────────────────────────── v5 (PR #52) 64바이트 페이로드 ───────────────────────────
// 스케일 없는 IEEE-754 float64. v3 의 int16 양자화(1mm/1e-4rad/5e-4 1/m)는 여기서 안 쓴다.

struct MpcTargetFdPayload  // 0x101 MPC_TARGET_FD — 64 B
{
  double x;          // [m]
  double y;          // [m]
  double yaw;        // [rad]
  double curvature;  // [1/m]
  // Consecutive valid localization poses: translation in the previous
  // vehicle frame, wrapped yaw difference, and source sample sequence.
  // LANE/WAYPOINT/TRAFFIC use GNSS; PARKING may use LiDAR SLAM.
  double dx;
  double dy;
  double dyaw;
  uint64_t update;
};

inline bool stateUsesPoseDelta(uint8_t state)
{
  // TargetRef constants: 0=LANE, 1=WAYPOINT, 3=PARKING, 4=TRAFFIC.
  return state == 0U || state == 1U || state == 3U || state == 4U;
}

class PoseDeltaUpdateGate
{
public:
  bool shouldApply(uint8_t state, uint64_t update)
  {
    if (!stateUsesPoseDelta(state)) {
      return false;
    }
    if (initialized_ && state == last_state_ && update == last_update_) {
      return false;
    }
    initialized_ = true;
    last_state_ = state;
    last_update_ = update;
    return true;
  }

private:
  bool initialized_{false};
  uint8_t last_state_{0};
  uint64_t last_update_{0};
};

struct VehFeedbackFdPayload  // 0x200 VEH_FEEDBACK_FD — 64 B
{
  double x;        // [m]
  double y;        // [m]
  double yaw;      // [rad]
  double v;        // [m/s]
  double str;      // [rad] 실제 조향각
  double str_ref;  // [rad] MPC 명령 조향각 — v5 신규.
                   // CLAUDE.md §3 의 실현율 `명령δ / PC 기하δ`(43~59%)를 간접 추정이 아니라
                   // 직접 측정할 수 있게 해 준다.
  uint64_t counter;   // PC 0x100 헤더 counter 의 **에코** (dSPACE 자체 카운터 아님).
                      // PC 송신 중이면 매 주기 +1, PC 가 멈추면 고정된다 — 그래서
                      // 이 값만으론 dSPACE 사망과 PC 무송신을 구분 못 한다.
                      // 같은 시점 (PC TX counter − 이 값) = 왕복 틱 수 (실측 2틱/20ms).
  uint64_t reserved;
};

#pragma pack(pop)

static_assert(sizeof(MpcTargetFdPayload) == 64, "0x101 은 64바이트여야 한다 (PR #52)");
static_assert(sizeof(VehFeedbackFdPayload) == 64, "0x200 은 64바이트여야 한다 (PR #52)");
static_assert(sizeof(TargetHeaderPayload) == 8, "0x100 은 v3 와 동일한 8바이트");

static_assert(sizeof(RefPointPayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(TargetHeaderPayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(VehPosePayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(VehVelPayload) == 8, "must match PROTOCOL.md");
static_assert(sizeof(VehCommitPayload) == 8, "must match PROTOCOL.md");

}  // namespace bridge_dspace

#endif  // BRIDGE_DSPACE__CAN_PROTOCOL_HPP_
