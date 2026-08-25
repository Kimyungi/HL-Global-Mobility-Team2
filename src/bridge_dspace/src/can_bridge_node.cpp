// can_bridge_node — CLAUDE.md §3 계약의 실행부 (SocketCAN).
// TX: /adas/target_ref 수신 즉시 REF_POINT×20 → TARGET_HEADER 순서로 송신 (자체 재송신 없음 —
//     MGM이 죽으면 송신이 멈춰야 dSPACE watchdog이 동작한다).
// RX: 수신 스레드에서 VEH_POSE/VEH_VEL을 모으고 VEH_COMMIT 시점에 /vehicle/vector 퍼블리시.
//     `vehicle_csv_path`를 주면 같은 값을 CSV로도 남긴다 (빈 값 = 끔). 토픽은 이미
//     rosbag RECORD_TOPICS에 들어 있지만, 실차 분석은 run 폴더의 CSV(lateral·transitions·
//     jitter)를 먼저 보므로 dSPACE 피드백도 같은 자리에 같은 포맷(epoch 초)으로 둔다.
//     ⚠ RX는 2026-08-25 이전 전 구간에서 **0건**이었다(bag Count: 0). 토픽·기록 설정은
//     처음부터 있었고 dSPACE 송신만 없었다 — 그래서 5초 통계에서 rx=0을 **경고로**
//     올린다. INFO 한 줄로는 프로젝트 내내 아무도 못 알아챘다.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "fma_interfaces/msg/target_ref.hpp"
#include "fma_interfaces/msg/vehicle_vector.hpp"
#include "can_protocol.hpp"
#include "vehicle_csv.hpp"
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

    // dSPACE RX 피드백 CSV — 빈 값이면 끔. lateral.csv 와 같은 epoch 초 타임스탬프라
    // dspace_merge.py / 분석 스크립트가 두 파일을 그대로 겹칠 수 있다.
    vehicle_csv_path_ = declare_parameter<std::string>("vehicle_csv_path", "");
    if (!vehicle_csv_path_.empty()) {
      vehicle_csv_.open(vehicle_csv_path_, std::ios::trunc);
      if (vehicle_csv_.is_open()) {
        vehicle_csv_ << vehicleCsvHeader();
        vehicle_csv_.flush();
        RCLCPP_INFO(get_logger(), "dSPACE RX CSV: %s", vehicle_csv_path_.c_str());
      } else {
        RCLCPP_WARN(
          get_logger(), "dSPACE RX CSV를 열 수 없음: %s", vehicle_csv_path_.c_str());
      }
    }

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
        const uint64_t tx = tx_count_.load(), rx = rx_count_.load();
        // TX 가 나가는데 RX 가 한 건도 없으면 dSPACE 송신이 없는 것이다. 이 상태로도
        // 주행은 되지만 TTC 자차속도가 계속 폴백값을 쓰고(§stack_avoid) vehicle vector
        // 로깅이 통째로 비므로, INFO 가 아니라 경고로 올린다.
        if (tx > 0 && rx == 0) {
          RCLCPP_WARN(
            get_logger(),
            "tx=%lu cycles rx=0 — dSPACE RX 무수신 (%s). "
            "0x%03X~0x%03X 가 안 들어온다: dSPACE 송신 설정·배선·비트레이트 확인",
            tx, can_interface_.c_str(), kIdVehPose, kIdVehCommit);
        } else {
          RCLCPP_INFO(
            get_logger(), "tx=%lu cycles rx=%lu cycles (%s)",
            tx, rx, can_interface_.c_str());
        }
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
    if (vehicle_csv_.is_open()) {
      vehicle_csv_.flush();          // join 뒤라 기록자가 없다
      vehicle_csv_.close();
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
    // 유효 점만 송신 — 현재 모든 소스 1점 (PROTOCOL.md). dSPACE는
    // 헤더의 n_points로 몇 개가 왔는지 알고, 궤적 생성(quintic)이 나머지를 채운다.
    for (size_t i = 0; i < n; ++i) {
      const auto & p = msg.ref_points[i];
      RefPointPayload pt{
        quantize(p.x, kPosScale),
        quantize(p.y, kPosScale),
        quantize(p.yaw, kYawScale),
        quantize(p.curvature, kCurvScale)};
      ok &= sendCanFrame(sock_, kIdRefPointBase + i, pt);
    }
    // 헤더는 반드시 마지막 — dSPACE는 이 프레임에서 n_points개 세트를 latch
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
          // CSV 는 이 rx 스레드가 유일한 기록자이고, 소멸자가 스레드를 join 한 뒤
          // 닫으므로 잠금이 필요 없다. 타임스탬프는 header.stamp 와 같은 값이라
          // bag 과 CSV 를 틱 단위로 겹칠 수 있다.
          if (vehicle_csv_.is_open()) {
            writeVehicleCsvRow(vehicle_csv_, vv);
            if ((rx_count_.load() % 100) == 0) {
              vehicle_csv_.flush();   // 1초마다 — 전원이 끊겨도 직전 1초만 잃는다
            }
          }
          break;
        }
        default:
          break;
      }
    }
  }

  std::string can_interface_;
  std::string vehicle_csv_path_;
  std::ofstream vehicle_csv_;      // dSPACE RX 피드백 (기록자 = rx 스레드 하나)
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
