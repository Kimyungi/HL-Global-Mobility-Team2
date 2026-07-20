// dspace_sim_node — dSPACE 에뮬레이터 (PC 단독 루프백 검증용).
// 실제 dSPACE의 최소 동작을 재현: TX 프레임 수신 → watchdog(30ms) → kinematic bicycle
// 적분(10ms) → RX 프레임 회신. ROS 인터페이스 없음 — 순수 UDP (실기와 같은 조건).
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "packet.hpp"

using namespace bridge_dspace;
using Clock = std::chrono::steady_clock;

class DspaceSimNode : public rclcpp::Node
{
public:
  DspaceSimNode()
  : Node("dspace_sim_node")
  {
    listen_port_ = static_cast<uint16_t>(declare_parameter<int>("listen_port", 50001));
    pc_ip_ = declare_parameter<std::string>("pc_ip", "127.0.0.1");
    pc_port_ = static_cast<uint16_t>(declare_parameter<int>("pc_port", 50002));
    wheelbase_ = declare_parameter<double>("wheelbase", 0.32);      // WHEELTEC 근사
    timeout_ms_ = declare_parameter<int>("watchdog_timeout_ms", 30);

    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    int reuse = 1;
    ::setsockopt(sock_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
    timeval tv{0, 100000};
    ::setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(listen_port_);
    if (::bind(sock_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
      throw std::runtime_error("bind failed");
    }
    std::memset(&pc_addr_, 0, sizeof(pc_addr_));
    pc_addr_.sin_family = AF_INET;
    pc_addr_.sin_port = htons(pc_port_);
    ::inet_pton(AF_INET, pc_ip_.c_str(), &pc_addr_.sin_addr);

    last_rx_ = Clock::now();  // 부팅 직후는 정상 취급 — 30ms 내 첫 패킷 미도착 시 자연히 타임아웃
    rx_thread_ = std::thread([this] {rxLoop();});
    // 10ms 태스크 — 실 dSPACE의 Vehicle MGM 주기에 대응
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {step();});
    RCLCPP_INFO(get_logger(), "dSPACE sim: listen :%u → reply %s:%u, watchdog %dms",
      listen_port_, pc_ip_.c_str(), pc_port_, timeout_ms_);
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
    TxFrame f{};
    while (running_ && rclcpp::ok()) {
      const ssize_t len = ::recv(sock_, &f, sizeof(f), 0);
      if (len != sizeof(TxFrame) || f.magic != kMagicTx) {continue;}
      std::lock_guard<std::mutex> lk(mtx_);
      if (f.counter != last_counter_) {   // counter 갱신 = 링크 생존 (watchdog 입력)
        last_counter_ = f.counter;
        last_rx_ = Clock::now();
      }
      v_ref_ = f.v_ref;
      // 조향 목표: 첫 ref point의 곡률로 근사 (실기는 quintic+MPC — 여기선 스모크 수준)
      str_ref_ = std::atan(wheelbase_ * f.points[0].curvature);
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

    RxFrame r{};
    r.magic = kMagicRx;
    r.counter = ++tx_counter_;
    r.x = static_cast<float>(x_);
    r.y = static_cast<float>(y_);
    r.yaw = static_cast<float>(yaw_);
    r.v = static_cast<float>(v_);
    r.str = static_cast<float>(str_);
    ::sendto(sock_, &r, sizeof(r), 0,
      reinterpret_cast<const sockaddr *>(&pc_addr_), sizeof(pc_addr_));
  }

  uint16_t listen_port_{}, pc_port_{};
  std::string pc_ip_;
  double wheelbase_{};
  int timeout_ms_{};
  int sock_{-1};
  sockaddr_in pc_addr_{};
  std::mutex mtx_;
  uint32_t last_counter_{0}, tx_counter_{0};
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
