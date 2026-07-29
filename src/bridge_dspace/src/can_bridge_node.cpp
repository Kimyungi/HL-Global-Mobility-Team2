// can_bridge_node — CLAUDE.md §3 계약의 실행부 (SocketCAN).
// TX: /adas/target_ref 수신 즉시 REF_POINT×20 → TARGET_HEADER 순서로 송신 (자체 재송신 없음 —
//     MGM이 죽으면 송신이 멈춰야 dSPACE watchdog이 동작한다).
// RX: 수신 스레드에서 VEH_POSE/VEH_VEL을 모으고 VEH_COMMIT 시점에 /vehicle/vector 퍼블리시.
#include <atomic>
#include <cerrno>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "fma_interfaces/msg/target_ref.hpp"
#include "fma_interfaces/msg/vehicle_vector.hpp"
#include "can_protocol.hpp"
#include "socketcan.hpp"

using fma_interfaces::msg::TargetRef;
using fma_interfaces::msg::VehicleVector;

namespace bridge_dspace
{

class CanBridgeNode : public rclcpp::Node
{
public:
  CanBridgeNode()
  : Node("can_bridge_node")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");

    // 수신은 dSPACE→PC ID만 (자기 송신 프레임·잡음 배제)
    const std::vector<can_filter> rx_filters = {
      {kIdVehPose, CAN_SFF_MASK},
      {kIdVehVel, CAN_SFF_MASK},
      {kIdVehCommit, CAN_SFF_MASK},
    };
    sock_ = openCanSocket(can_interface_, rx_filters);

    vv_pub_ = create_publisher<VehicleVector>("/vehicle/vector", rclcpp::SensorDataQoS());
    ref_sub_ = create_subscription<TargetRef>(
      "/adas/target_ref", rclcpp::QoS(1),
      [this](TargetRef::ConstSharedPtr msg) {sendFrames(*msg);});

    rx_thread_ = std::thread([this] {rxLoop();});

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this] {
        RCLCPP_INFO(
          get_logger(), "tx=%lu cycles rx=%lu cycles (%s)",
          tx_count_.load(), rx_count_.load(), can_interface_.c_str());
      });

    RCLCPP_INFO(
      get_logger(), "bridge up — %s, TX 0x%03X+0x%03X..0x%03X, RX 0x%03X..0x%03X",
      can_interface_.c_str(), kIdTargetHeader, kIdRefPointBase,
      kIdRefPointBase + kNumPoints - 1, kIdVehPose, kIdVehCommit);
  }

  ~CanBridgeNode() override
  {
    running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    ::close(sock_);
  }

private:
  void sendFrames(const TargetRef & msg)
  {
    const size_t n = std::min<size_t>(msg.ref_points.size(), kNumPoints);
    if (n == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "empty ref_points — skip");
      return;
    }
    bool ok = true;
    for (size_t i = 0; i < static_cast<size_t>(kNumPoints); ++i) {
      // n_points 미만 슬롯은 마지막 점 복제 (PROTOCOL.md)
      const auto & p = msg.ref_points[std::min(i, n - 1)];
      RefPointPayload pt{
        quantize(p.x, kPosScale),
        quantize(p.y, kPosScale),
        quantize(p.yaw, kYawScale),
        quantize(p.curvature, kCurvScale)};
      ok &= sendCanFrame(sock_, kIdRefPointBase + i, pt);
    }
    // 헤더는 반드시 마지막 — dSPACE는 이 프레임에서 20점 세트를 latch
    TargetHeaderPayload hdr{};
    hdr.counter = ++tx_counter_;
    hdr.state = msg.state;
    hdr.n_points = static_cast<uint8_t>(n);
    hdr.v_ref = quantize(msg.v_ref, kVelScale);
    ok &= sendCanFrame(sock_, kIdTargetHeader, hdr);

    if (ok) {
      ++tx_count_;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CAN write failed: %s",
        std::strerror(errno));
    }
  }

  void rxLoop()
  {
    can_frame f{};
    VehPosePayload pose{};
    VehVelPayload vel{};
    while (running_ && rclcpp::ok()) {
      const ssize_t len = ::read(sock_, &f, sizeof(f));
      if (len < 0) {
        continue;  // timeout — 종료 플래그 재확인
      }
      if (len != sizeof(can_frame) || f.can_dlc != 8) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "bad CAN frame: len=%zd id=0x%03X dlc=%u", len, f.can_id, f.can_dlc);
        continue;
      }
      switch (f.can_id) {
        case kIdVehPose:
          std::memcpy(&pose, f.data, sizeof(pose));
          break;
        case kIdVehVel:
          std::memcpy(&vel, f.data, sizeof(vel));
          break;
        case kIdVehCommit: {
          // 커밋 프레임 = 한 주기 세트 완성 → 퍼블리시
          VehCommitPayload commit{};
          std::memcpy(&commit, f.data, sizeof(commit));
          VehicleVector vv;
          vv.header.stamp = now();
          vv.header.frame_id = "odom";
          vv.x = pose.x;
          vv.y = pose.y;
          vv.yaw = vel.yaw;
          vv.v = vel.v;
          vv.str = commit.str;
          vv.counter = commit.counter;
          vv_pub_->publish(vv);
          ++rx_count_;
          break;
        }
        default:
          break;
      }
    }
  }

  std::string can_interface_;
  int sock_{-1};
  uint16_t tx_counter_{0};
  std::atomic<bool> running_{true};
  std::atomic<uint64_t> tx_count_{0}, rx_count_{0};
  std::thread rx_thread_;
  rclcpp::Publisher<VehicleVector>::SharedPtr vv_pub_;
  rclcpp::Subscription<TargetRef>::SharedPtr ref_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

}  // namespace bridge_dspace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bridge_dspace::CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
