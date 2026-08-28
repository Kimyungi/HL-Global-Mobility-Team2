// dspace_sim_node — dSPACE 에뮬레이터 (PC 단독 루프백 검증용).
// ★ v5 (2026-08-28, PR #52): MPC_TARGET(0x101, 64B) 버퍼링 → TARGET_HEADER(0x100)에서
// latch → watchdog(30ms) → kinematic bicycle 적분(10ms) → VEH_FEEDBACK(0x200, 64B) **1프레임** 회신.
// v3 의 3프레임 커밋 규칙이 사라졌으므로 여기서도 한 프레임만 보낸다.
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
    // 실기와 같은 와이어 포맷으로 돌린다 (PROTOCOL.md §공통). vcan0 는 MTU 72 필요 —
    // install.sh --vcan 이 설정한다. can_fd:=false 로 classic 대조도 가능.
    can_fd_ = declare_parameter<bool>("can_fd", true);
    can_fd_brs_ = declare_parameter<bool>("can_fd_brs", true);

    // 수신은 PC→dSPACE ID만 (0x100 헤더 + 0x101..0x114 포인트)
    const std::vector<can_filter> rx_filters = {
      {kIdTargetHeader, CAN_SFF_MASK},
      // 0x101..0x114를 하나의 mask 필터로: 0x100~0x11F 대역에서 헤더 제외 상위 매칭
      {kIdRefPointBase, static_cast<canid_t>(CAN_SFF_MASK & ~0x01F)},
    };
    const bool iface_fd = canIfaceSupportsFd(can_interface_);
    if (can_fd_ && !iface_fd) {
      throw std::runtime_error(
              "can_fd:=true 인데 " + can_interface_ + " 가 classic 전용이다 (MTU " +
              std::to_string(canIfaceMtu(can_interface_)) + "). vcan 이면: "
              "sudo ip link set " + can_interface_ + " down && sudo ip link set " +
              can_interface_ + " mtu 72 && sudo ip link set " + can_interface_ + " up "
              "(또는 sudo src/bridge_dspace/tools/can_setup/install.sh --vcan)");
    }
    sock_ = openCanSocket(can_interface_, rx_filters, 100, iface_fd);

    last_rx_ = Clock::now();  // 부팅 직후는 정상 취급 — 30ms 내 첫 헤더 미도착 시 자연히 타임아웃
    rx_thread_ = std::thread([this] {rxLoop();});
    // 10ms 태스크 — 실 dSPACE의 Vehicle MGM 주기에 대응
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {step();});
    RCLCPP_INFO(get_logger(), "dSPACE sim: %s, watchdog %dms, TX %s",
      can_interface_.c_str(), timeout_ms_,
      can_fd_ ? (can_fd_brs_ ? "CAN FD (BRS on)" : "CAN FD (BRS off)") : "classic CAN 2.0A");
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
    CanRxFrame f{};
    while (running_ && rclcpp::ok()) {
      // classic·FD 를 가리지 않고 받는다 (실기 브리지와 동일 — socketcan.hpp 주석)
      // 0x101 은 64B(v5), 0x100 헤더는 8B — 길이가 프레임 종류를 가른다
      if (!readCanFrame(sock_, f)) {continue;}

      if (f.id >= kIdRefPointBase &&
        f.id < kIdRefPointBase + kNumPoints)
      {
        // point 프레임은 스테이징 버퍼에만 — latch는 헤더 수신 시점 (PROTOCOL.md)
        if (f.len == sizeof(MpcTargetFdPayload)) {
          std::memcpy(&staged_points_[f.id - kIdRefPointBase], f.data,
            sizeof(MpcTargetFdPayload));
        }
        continue;
      }
      if (f.id != kIdTargetHeader || f.len != sizeof(TargetHeaderPayload)) {continue;}

      TargetHeaderPayload hdr{};
      std::memcpy(&hdr, f.data, sizeof(hdr));
      std::lock_guard<std::mutex> lk(mtx_);
      if (hdr.counter != last_counter_) {   // counter 갱신 = 링크 생존 (watchdog 입력)
        last_counter_ = hdr.counter;
        last_rx_ = Clock::now();
      }
      v_ref_ = dequantize(hdr.v_ref, kVelScale);
      // 조향 목표: 첫 ref point의 곡률로 근사 (실기는 quintic+MPC — 여기선 스모크 수준).
      // v5 는 float64 원값이라 역양자화가 없다.
      str_ref_ = std::atan(wheelbase_ * staged_points_[0].curvature);
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

    // v5: 한 프레임에 전부 (커밋 규칙 없음). str_ref 는 MPC 명령, str 은 실제 조향.
    VehFeedbackFdPayload fb{};
    fb.x = x_; fb.y = y_; fb.yaw = yaw_; fb.v = v_;
    fb.str = str_; fb.str_ref = str_cmd; fb.counter = ++tx_counter_;
    sendCanFrame(sock_, kIdVehFeedback, fb, can_fd_, can_fd_brs_);
  }

  std::string can_interface_;
  double wheelbase_{};
  int timeout_ms_{};
  bool can_fd_{true}, can_fd_brs_{true};
  int sock_{-1};
  std::mutex mtx_;
  MpcTargetFdPayload staged_points_[kNumPoints]{};
  uint16_t last_counter_{0};
  uint64_t tx_counter_{0};
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
