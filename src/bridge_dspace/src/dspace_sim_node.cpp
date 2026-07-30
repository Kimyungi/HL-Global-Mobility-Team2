// dspace_sim_node — dSPACE 에뮬레이터 (PC 단독 루프백 검증용).
// 실제 dSPACE의 최소 동작을 재현: REF_POINT 버퍼링 → TARGET_HEADER에서 latch →
// watchdog(30ms) → kinematic bicycle 적분(10ms) → VEH_* 3프레임 회신.
// ROS 토픽 인터페이스 없음 — 순수 CAN (실기와 같은 조건, vcan0 사용).
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "can_protocol.hpp"
#include "socketcan.hpp"

using namespace bridge_dspace;
using Clock = std::chrono::steady_clock;

class DspaceSimNode : public rclcpp::Node
{
public:
  DspaceSimNode()
  : Node("dspace_sim_node")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "vcan0");
    wheelbase_ = declare_parameter<double>("wheelbase", 0.32);      // WHEELTEC 근사
    timeout_ms_ = declare_parameter<int>("watchdog_timeout_ms", 30);

    // 수신은 PC→dSPACE ID만 (0x100 헤더 + 0x101..0x114 포인트)
    const std::vector<can_filter> rx_filters = {
      {kIdTargetHeader, CAN_SFF_MASK},
      // 0x101..0x114를 하나의 mask 필터로: 0x100~0x11F 대역에서 헤더 제외 상위 매칭
      {kIdRefPointBase, static_cast<canid_t>(CAN_SFF_MASK & ~0x01F)},
    };
    sock_ = openCanSocket(can_interface_, rx_filters);

    last_rx_ = Clock::now();  // 부팅 직후는 정상 취급 — 30ms 내 첫 헤더 미도착 시 자연히 타임아웃
    rx_thread_ = std::thread([this] {rxLoop();});
    // 10ms 태스크 — 실 dSPACE의 Vehicle MGM 주기에 대응
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {step();});
    RCLCPP_INFO(get_logger(), "dSPACE sim: %s, watchdog %dms",
      can_interface_.c_str(), timeout_ms_);
  }

  ~DspaceSimNode() override
  {
    running_ = false;
    if (rx_thread_.joinable()) {rx_thread_.join();}
    ::close(sock_);
  }

private:
  void rxLoop()
  {
    can_frame f{};
    while (running_ && rclcpp::ok()) {
      const ssize_t len = ::read(sock_, &f, sizeof(f));
      if (len != sizeof(can_frame) || f.can_dlc != 8) {continue;}

      if (f.can_id >= kIdRefPointBase &&
        f.can_id < kIdRefPointBase + kNumPoints)
      {
        // point 프레임은 스테이징 버퍼에만 — latch는 헤더 수신 시점 (PROTOCOL.md)
        std::memcpy(&staged_points_[f.can_id - kIdRefPointBase], f.data,
          sizeof(RefPointPayload));
        continue;
      }
      if (f.can_id != kIdTargetHeader) {continue;}

      TargetHeaderPayload hdr{};
      std::memcpy(&hdr, f.data, sizeof(hdr));
      std::lock_guard<std::mutex> lk(mtx_);
      if (hdr.counter != last_counter_) {   // counter 갱신 = 링크 생존 (watchdog 입력)
        last_counter_ = hdr.counter;
        last_rx_ = Clock::now();
      }
      v_ref_ = dequantize(hdr.v_ref, kVelScale);
      // 조향 목표: 첫 ref point의 곡률로 근사 (실기는 quintic+MPC — 여기선 스모크 수준)
      const double curv = dequantize(staged_points_[0].curvature, kCurvScale);
      str_ref_ = std::atan(wheelbase_ * curv);
    }
  }

  void step()
  {
    double v_cmd, str_cmd;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
        Clock::now() - last_rx_).count();
      const bool timed_out = age > timeout_ms_;
      if (timed_out && !timeout_latched_) {
        RCLCPP_WARN(get_logger(), "watchdog TIMEOUT (%ldms) → v_ref=0, 조향 유지", age);
      }
      timeout_latched_ = timed_out;
      v_cmd = timed_out ? 0.0 : v_ref_;   // 감속 정지, 급조향 금지 → str 직전 값 유지
      str_cmd = str_ref_;
    }

    // kinematic bicycle model, dt = 10ms (1차 지연으로 구동계 근사)
    constexpr double dt = 0.01;
    v_ += (v_cmd - v_) * 0.2;
    str_ += (str_cmd - str_) * 0.3;
    x_ += v_ * std::cos(yaw_) * dt;
    y_ += v_ * std::sin(yaw_) * dt;
    yaw_ += v_ / wheelbase_ * std::tan(str_) * dt;

    // 회신 순서: POSE → VEL → COMMIT (PC는 COMMIT에서 퍼블리시)
    VehPosePayload pose{static_cast<float>(x_), static_cast<float>(y_)};
    VehVelPayload vel{static_cast<float>(yaw_), static_cast<float>(v_)};
    VehCommitPayload commit{static_cast<float>(str_), ++tx_counter_, 0};
    sendCanFrame(sock_, kIdVehPose, pose);
    sendCanFrame(sock_, kIdVehVel, vel);
    sendCanFrame(sock_, kIdVehCommit, commit);
  }

  std::string can_interface_;
  double wheelbase_{};
  int timeout_ms_{};
  int sock_{-1};
  std::mutex mtx_;
  RefPointPayload staged_points_[kNumPoints]{};
  uint16_t last_counter_{0}, tx_counter_{0};
  Clock::time_point last_rx_;
  bool timeout_latched_{false};
  double v_ref_{0.0}, str_ref_{0.0};
  double x_{0.0}, y_{0.0}, yaw_{0.0}, v_{0.0}, str_{0.0};
  std::atomic<bool> running_{true};
  std::thread rx_thread_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DspaceSimNode>());
  rclcpp::shutdown();
  return 0;
}
