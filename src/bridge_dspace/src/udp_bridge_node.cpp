// udp_bridge_node — CLAUDE.md §3 계약의 실행부.
// TX: /adas/target_ref 수신 즉시 패킹·송신 (자체 재송신 없음 — MGM이 죽으면 송신이 멈춰야
//     dSPACE watchdog이 동작한다).
// RX: 수신 스레드에서 vehicle vector를 받아 /vehicle/vector로 퍼블리시.
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "fma_interfaces/msg/target_ref.hpp"
#include "fma_interfaces/msg/vehicle_vector.hpp"
#include "packet.hpp"

using fma_interfaces::msg::TargetRef;
using fma_interfaces::msg::VehicleVector;

namespace bridge_dspace
{

class UdpBridgeNode : public rclcpp::Node
{
public:
  UdpBridgeNode()
  : Node("udp_bridge_node")
  {
    dspace_ip_ = declare_parameter<std::string>("dspace_ip", "192.168.1.10");
    tx_port_ = static_cast<uint16_t>(declare_parameter<int>("tx_port", 50001));
    rx_port_ = static_cast<uint16_t>(declare_parameter<int>("rx_port", 50002));

    tx_sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (tx_sock_ < 0) {
      throw std::runtime_error("TX socket creation failed");
    }
    std::memset(&tx_addr_, 0, sizeof(tx_addr_));
    tx_addr_.sin_family = AF_INET;
    tx_addr_.sin_port = htons(tx_port_);
    if (::inet_pton(AF_INET, dspace_ip_.c_str(), &tx_addr_.sin_addr) != 1) {
      throw std::runtime_error("invalid dspace_ip: " + dspace_ip_);
    }

    rx_sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (rx_sock_ < 0) {
      throw std::runtime_error("RX socket creation failed");
    }
    int reuse = 1;
    ::setsockopt(rx_sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    timeval tv{0, 100000};  // 100ms — 종료 플래그 확인 주기
    ::setsockopt(rx_sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in rx_addr{};
    rx_addr.sin_family = AF_INET;
    rx_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rx_addr.sin_port = htons(rx_port_);
    if (::bind(rx_sock_, reinterpret_cast<sockaddr *>(&rx_addr), sizeof(rx_addr)) < 0) {
      throw std::runtime_error("RX bind failed on port " + std::to_string(rx_port_));
    }

    vv_pub_ = create_publisher<VehicleVector>("/vehicle/vector", rclcpp::SensorDataQoS());
    ref_sub_ = create_subscription<TargetRef>(
      "/adas/target_ref", rclcpp::QoS(1),
      [this](TargetRef::ConstSharedPtr msg) {sendFrame(*msg);});

    rx_thread_ = std::thread([this] {rxLoop();});

    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this] {
        RCLCPP_INFO(
          get_logger(), "tx=%lu rx=%lu (dspace %s:%u / bind :%u)",
          tx_count_.load(), rx_count_.load(), dspace_ip_.c_str(), tx_port_, rx_port_);
      });

    RCLCPP_INFO(
      get_logger(), "bridge up — TX %s:%u, RX :%u, frame %zu/%zu bytes",
      dspace_ip_.c_str(), tx_port_, rx_port_, sizeof(TxFrame), sizeof(RxFrame));
  }

  ~UdpBridgeNode() override
  {
    running_ = false;
    if (rx_thread_.joinable()) {
      rx_thread_.join();
    }
    ::close(tx_sock_);
    ::close(rx_sock_);
  }

private:
  void sendFrame(const TargetRef & msg)
  {
    TxFrame f{};
    f.magic = kMagicTx;
    f.counter = ++tx_counter_;
    f.state = msg.state;
    f.v_ref = msg.v_ref;
    const size_t n = std::min<size_t>(msg.ref_points.size(), kNumPoints);
    f.n_points = static_cast<uint8_t>(n);
    for (size_t i = 0; i < static_cast<size_t>(kNumPoints) && n > 0; ++i) {
      // n_points 미만 슬롯은 마지막 점 복제 (PROTOCOL.md)
      const auto & p = msg.ref_points[std::min(i, n - 1)];
      f.points[i] = {p.x, p.y, p.yaw, p.curvature};
    }
    const ssize_t sent = ::sendto(
      tx_sock_, &f, sizeof(f), 0,
      reinterpret_cast<const sockaddr *>(&tx_addr_), sizeof(tx_addr_));
    if (sent == sizeof(f)) {
      ++tx_count_;
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "sendto failed: %s",
        std::strerror(errno));
    }
  }

  void rxLoop()
  {
    RxFrame f{};
    while (running_ && rclcpp::ok()) {
      const ssize_t len = ::recv(rx_sock_, &f, sizeof(f), 0);
      if (len < 0) {
        continue;  // timeout — 종료 플래그 재확인
      }
      if (len != sizeof(RxFrame) || f.magic != kMagicRx) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "bad RX frame: len=%zd magic=0x%08X", len, f.magic);
        continue;
      }
      VehicleVector vv;
      vv.header.stamp = now();
      vv.header.frame_id = "odom";
      vv.x = f.x;
      vv.y = f.y;
      vv.yaw = f.yaw;
      vv.v = f.v;
      vv.str = f.str;
      vv.counter = f.counter;
      vv_pub_->publish(vv);
      ++rx_count_;
    }
  }

  std::string dspace_ip_;
  uint16_t tx_port_{}, rx_port_{};
  int tx_sock_{-1}, rx_sock_{-1};
  sockaddr_in tx_addr_{};
  uint32_t tx_counter_{0};
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
  rclcpp::spin(std::make_shared<bridge_dspace::UdpBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
