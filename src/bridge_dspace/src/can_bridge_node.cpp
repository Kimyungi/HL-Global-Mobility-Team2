// can_bridge_node — CLAUDE.md §3 계약의 실행부 (SocketCAN).
// ★ v5 (2026-08-28, PR #52): CAN FD 64바이트 페이로드. dSPACE 가 먼저 이 포맷으로 넘어가
//   있어서 PC 를 맞춘다(팀장 결정). 참조점은 1개다 — 그 위험은 can_protocol.hpp 주석 참조.
// TX: /adas/target_ref 수신 즉시 MPC_TARGET(0x101, 64B) → TARGET_HEADER(0x100, 8B) 순서로
//     송신 (자체 재송신 없음 — MGM이 죽으면 송신이 멈춰야 dSPACE watchdog이 동작한다).
//     헤더가 마지막인 것은 v3 와 같다: dSPACE 는 헤더에서 latch 한다.
// RX: v5 는 0x200 한 프레임(64B)이라 **수신 즉시 퍼블리시**한다 — 세트를 모을 필요가 없어
//     커밋 프레임 규칙이 사라졌다. v3(8B × 0x200/0x201/0x202)도 계속 받는다: dSPACE 가
//     되돌아가도 링크가 죽지 않게 하는 폴백이며, 길이로 구분되므로 모호하지 않다.
//     `vehicle_csv_path`를 주면 같은 값을 CSV로도 남긴다 (빈 값 = 끔). 토픽은 이미
//     rosbag RECORD_TOPICS에 들어 있지만, 실차 분석은 run 폴더의 CSV(lateral·transitions·
//     jitter)를 먼저 보므로 dSPACE 피드백도 같은 자리에 같은 포맷(epoch 초)으로 둔다.
//     ⚠ RX는 2026-08-25 이전 전 구간에서 **0건**이었다(bag Count: 0). 토픽·기록 설정은
//     처음부터 있었고 dSPACE 송신만 없었다 — 그래서 5초 통계에서 rx=0을 **경고로**
//     올린다. INFO 한 줄로는 프로젝트 내내 아무도 못 알아챘다.
// TX 도 같은 침묵이 가능하다 (2026-08-28 실측): write() 는 프레임을 **커널 큐에 넣는
//     것**까지만 성공을 뜻한다. 상대가 ACK 하지 않으면 와이어에 한 프레임도 못 나가는데
//     write() 는 계속 성공한다 — dSPACE 가 아직 classic 인 버스에 FD 프레임을 보냈을 때
//     30/30 write 성공 · tx_packets 0 · bus-errors +16 · ERROR-PASSIVE 였다. 그래서
//     커널이 세는 실제 송신 수(sysfs tx_packets)를 매 5초 대조한다.
// USB 재열거: Kvaser가 순간 분리되면 옛 SocketCAN fd는 ENODEV/ENXIO/ENETDOWN이 된다.
//     노드는 종료하지 않고 fd만 폐기한 뒤 100ms마다 같은 이름의 새 인터페이스를 찾아
//     재바인드한다. 복구 뒤 별도 저장 명령을 재생하지 않고 다음 MGM 최신 목표부터 보낸다.
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

// 커널이 **실제로 버스에 올린** 프레임 수. write() 성공과 별개다 — 상대가 ACK 하지
// 않으면 write 는 성공해도 이 값은 늘지 않는다. vcan 도 정상 카운트하므로 루프백에서
// 오탐이 나지 않는다(2026-08-28 확인: vcan0·can0 모두 수용 30/30, 거부 0/30).
constexpr uint64_t kTxPacketsUnavailable = ~0ULL;

inline uint64_t readIfaceTxPackets(const std::string & ifname)
{
  std::ifstream f("/sys/class/net/" + ifname + "/statistics/tx_packets");
  uint64_t v = 0;
  return (f >> v) ? v : kTxPacketsUnavailable;
}

class CanBridgeNode : public rclcpp::Node
{
public:
  CanBridgeNode()
  : Node("can_bridge_node")
  {
    can_interface_ = declare_parameter<std::string>("can_interface", "can0");

    // CAN FD (PROTOCOL.md §공통 — 2026-08-28 Kvaser Leaf v3 이관과 함께 팀 표준).
    // 프레임 레이아웃·ID·스케일은 classic 과 동일하다. 바뀌는 건 와이어 포맷뿐.
    //   can_fd:=false  → classic 프레임으로 송신 (A/B 진단용. 인터페이스 설정은 그대로
    //                    둬도 된다 — FD 인터페이스는 classic 프레임을 그대로 실어 보낸다)
    //   can_fd_brs     → 데이터 구간 비트레이트 전환. dSPACE 설정과 **반드시 일치**.
    //                    끄면 FD 프레임이지만 전 구간 nominal 이라 속도 이득이 0이다.
    can_fd_ = declare_parameter<bool>("can_fd", true);
    can_fd_brs_ = declare_parameter<bool>("can_fd_brs", true);

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

    // RX 는 인터페이스가 FD 를 지원하면 **무조건** FD 수신을 켠다 (can_fd 와 무관).
    // 그래야 dSPACE 만 먼저 FD 로 넘어간 과도기에도 프레임이 보인다 — socketcan.hpp 주석 참조.
    const bool iface_fd = canIfaceSupportsFd(can_interface_);
    if (can_fd_ && !iface_fd) {
      throw std::runtime_error(
              "can_fd:=true 인데 " + can_interface_ +
              " 가 classic 전용이다 (MTU " + std::to_string(canIfaceMtu(can_interface_)) +
              "). 인터페이스를 FD 로 올리거나(sudo /usr/local/bin/can_up.sh " +
              can_interface_ + ") can_fd:=false 로 실행할 것");
    }
    std::atomic_store(&sock_, openSocket(iface_fd));

    vv_pub_ = create_publisher<VehicleVector>("/vehicle/vector", rclcpp::SensorDataQoS());
    ref_sub_ = create_subscription<TargetRef>(
      "/adas/target_ref", rclcpp::QoS(1),
      [this](TargetRef::ConstSharedPtr msg) {sendFrames(*msg);});

    rx_thread_ = std::thread([this] {rxLoop();});

    last_wire_tx_ = readIfaceTxPackets(can_interface_);
    stats_timer_ = create_wall_timer(
      std::chrono::seconds(5), [this] {
        const uint64_t tx = tx_count_.load(), rx = rx_count_.load();
        checkWireTx();
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
    reconnect_timer_ = create_wall_timer(
      std::chrono::milliseconds(100), [this] {reconnectIfNeeded();});

    RCLCPP_INFO(
      get_logger(),
      "bridge up — %s, TX %s v5(64B, %d점), RX v5+v3(%s), 0x%03X+0x%03X → 0x%03X",
      can_interface_.c_str(),
      can_fd_ ? (can_fd_brs_ ? "CAN FD (BRS on)" : "CAN FD (BRS off)") : "classic CAN 2.0A",
      kNumPoints, iface_fd ? "iface FD" : "iface classic 전용",
      kIdTargetHeader, kIdRefPointBase, kIdVehFeedback);
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
    std::atomic_store(&sock_, SocketHandle{});
  }

private:
  using SocketHandle = std::shared_ptr<int>;

  SocketHandle openSocket(bool iface_fd)
  {
    const std::vector<can_filter> rx_filters = {
      {kIdVehPose, CAN_SFF_MASK},
      {kIdVehVel, CAN_SFF_MASK},
      {kIdVehCommit, CAN_SFF_MASK},
    };
    const int fd = openCanSocket(can_interface_, rx_filters, 100, iface_fd);
    return SocketHandle(new int(fd), [](int * value) {
      if (value != nullptr) {
        ::close(*value);
        delete value;
      }
    });
  }

  void invalidateSocket(const SocketHandle & failed_socket, int error_number, const char * path)
  {
    if (!isCanReconnectError(error_number)) {
      return;
    }
    SocketHandle expected = failed_socket;
    if (std::atomic_compare_exchange_strong(&sock_, &expected, SocketHandle{})) {
      RCLCPP_ERROR(
        get_logger(),
        "CAN %s 단절 감지: %s (%d) — %s 재열거 후 소켓을 자동 재연결합니다",
        path, std::strerror(error_number), error_number, can_interface_.c_str());
    }
  }

  void reconnectIfNeeded()
  {
    if (std::atomic_load(&sock_)) {
      return;
    }
    const int mtu = canIfaceMtu(can_interface_);
    const bool iface_fd = mtu >= static_cast<int>(CANFD_MTU);
    if (mtu < 0 || (can_fd_ && !iface_fd)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "CAN 자동 복구 대기 — %s가 아직 없거나 FD 준비 전입니다 (MTU=%d)",
        can_interface_.c_str(), mtu);
      return;
    }
    try {
      SocketHandle replacement = openSocket(iface_fd);
      SocketHandle expected;
      if (!std::atomic_compare_exchange_strong(&sock_, &expected, replacement)) {
        return;
      }
      last_wire_tx_ = readIfaceTxPackets(can_interface_);
      last_tx_frames_ = tx_frames_.load();
      ++reconnect_count_;
      RCLCPP_WARN(
        get_logger(),
        "CAN 소켓 자동 재연결 성공 #%lu — 다음 MGM 주기부터 최신 목표를 자동 송신합니다",
        reconnect_count_.load());
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "CAN 자동 재연결 대기: %s", e.what());
    }
  }

  void sendFrames(const TargetRef & msg)
  {
    const size_t n = std::min<size_t>(msg.ref_points.size(), kNumPoints);
    if (n == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "empty ref_points — skip");
      return;
    }
    const SocketHandle socket = std::atomic_load(&sock_);
    if (!socket) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "CAN 재열거 중 — 목표 송신을 보류합니다");
      return;
    }
    // v5: 0x101 한 프레임에 참조점 1개(float64 ×4). MGM 이 20점을 만들어도 **첫 점**만
    // 싣는다 — v3 의 REF_POINT_0 와 같은 점이라 의미가 바뀌지 않는다.
    // dx/dy/dyaw/update 는 PR #52 에서 의미 미정이라 0 으로 채운다(팀장 결정).
    for (size_t i = 0; i < n; ++i) {
      const auto & p = msg.ref_points[i];
      MpcTargetFdPayload pt{};
      pt.x = p.x;
      pt.y = p.y;
      pt.yaw = p.yaw;
      pt.curvature = p.curvature;
      if (!sendCanFrame(*socket, kIdRefPointBase + i, pt, can_fd_, can_fd_brs_)) {
        const int error_number = errno;
        tx_frames_ += i + 1;
        invalidateSocket(socket, error_number, "TX");
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000, "CAN write failed: %s",
          std::strerror(error_number));
        return;  // 헤더를 보내지 않아 dSPACE가 불완전 세트를 latch하지 않게 한다.
      }
    }
    // 헤더는 반드시 마지막 — dSPACE는 이 프레임에서 n_points개 세트를 latch
    TargetHeaderPayload hdr{};
    hdr.counter = ++tx_counter_;
    hdr.state = msg.state;
    hdr.n_points = static_cast<uint8_t>(n);
    hdr.v_ref = quantize(msg.v_ref, kVelScale);
    if (!sendCanFrame(*socket, kIdTargetHeader, hdr, can_fd_, can_fd_brs_)) {
      const int error_number = errno;
      tx_frames_ += n + 1;
      invalidateSocket(socket, error_number, "TX");
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "CAN write failed: %s",
        std::strerror(error_number));
      return;
    }

    tx_frames_ += n + 1;    // REF_POINT n개 + 헤더 1 — 와이어 대조 기준
    ++tx_count_;
  }

  // write() 는 성공했는데 프레임이 버스에 안 나가는 침묵을 잡는다.
  // 가장 흔한 원인은 **양쪽 와이어 포맷 불일치**(우리 FD ↔ 상대 classic)이고,
  // 그 다음이 비트레이트·샘플포인트 불일치다. 둘 다 조용히 진행되므로 경고로 올린다.
  void checkWireTx()
  {
    if (!std::atomic_load(&sock_)) {
      // 재열거로 sysfs 카운터가 0부터 다시 시작할 수 있다. 끊긴 동안 옛 값과 새 값을
      // 빼면 unsigned underflow로 거대한 송신량이 만들어지므로 재연결 측에서 기준을 리셋한다.
      return;
    }
    const uint64_t wire = readIfaceTxPackets(can_interface_);
    const uint64_t frames = tx_frames_.load();
    if (wire == kTxPacketsUnavailable || last_wire_tx_ == kTxPacketsUnavailable) {
      last_wire_tx_ = wire;
      last_tx_frames_ = frames;
      return;   // sysfs 를 못 읽는 환경 — 대조를 건너뛴다 (판단 아님)
    }
    const uint64_t d_frames = frames - last_tx_frames_;
    const uint64_t d_wire = wire - last_wire_tx_;
    last_wire_tx_ = wire;
    last_tx_frames_ = frames;

    if (d_frames == 0) {
      return;   // 보낸 게 없으면 대조할 것도 없다
    }
    if (d_wire == 0) {
      RCLCPP_ERROR(
        get_logger(),
        "★ CAN TX 가 버스에 안 나간다 — write %lu 프레임 성공, 실제 송신 0 (%s, TX %s). "
        "dSPACE 가 우리 프레임을 ACK 하지 않는다: 와이어 포맷 불일치(상대가 classic 이면 "
        "can_fd:=false)나 비트레이트 불일치를 의심할 것. "
        "'ip -details -statistics link show %s' 로 bus-errors·ERROR-PASSIVE 확인",
        d_frames, can_interface_.c_str(),
        can_fd_ ? "CAN FD" : "classic", can_interface_.c_str());
    } else if (d_wire * 2 < d_frames) {
      RCLCPP_WARN(
        get_logger(),
        "CAN TX 유실 — write %lu 프레임 중 실제 송신 %lu (%s). "
        "데이터 구간 비트레이트·샘플포인트 불일치 또는 배선 품질을 의심할 것",
        d_frames, d_wire, can_interface_.c_str());
    }
  }

  void rxLoop()
  {
    CanRxFrame f{};
    VehPosePayload pose{};
    VehVelPayload vel{};
    while (running_ && rclcpp::ok()) {
      const SocketHandle socket = std::atomic_load(&sock_);
      if (!socket) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }
      ssize_t raw_len = 0;
      if (!readCanFrame(*socket, f, &raw_len)) {
        const int error_number = errno;
        if (raw_len < 0) {
          invalidateSocket(socket, error_number, "RX");
        }
        continue;  // timeout — 종료 플래그 재확인
      }
      // 길이가 포맷을 가른다: 64 = v5(PR #52), 8 = v3. 그 외는 설정 오류다.
      if (f.len != 64 && f.len != 8) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "bad CAN frame: id=0x%03X len=%u (64=v5 / 8=v3 만 유효, %s 프레임) — "
          "dSPACE 메시지 길이(DLC) 확인", f.id, f.len, f.fd ? "FD" : "classic");
        continue;
      }
      // 상대가 어느 포맷·계약으로 보내고 있는지 한 번은 남긴다.
      if (!rx_fmt_logged_) {
        rx_fmt_logged_ = true;
        RCLCPP_INFO(get_logger(), "dSPACE RX 포맷: %s%s, 계약 %s (len=%u)",
          f.fd ? "CAN FD" : "classic CAN 2.0A", (f.fd && f.brs) ? " (BRS)" : "",
          f.len == 64 ? "v5 (PR #52 64B)" : "v3 (8B ×3)", f.len);
      }

      // ── v5: 0x200 한 프레임이 한 주기 세트다 → 즉시 퍼블리시
      if (f.id == kIdVehFeedback && f.len == 64) {
        VehFeedbackFdPayload fb{};
        std::memcpy(&fb, f.data, sizeof(fb));
        VehicleVector vv;
        vv.header.stamp = now();
        vv.header.frame_id = "odom";
        vv.x = static_cast<float>(fb.x);
        vv.y = static_cast<float>(fb.y);
        vv.yaw = static_cast<float>(fb.yaw);
        vv.v = static_cast<float>(fb.v);
        vv.str = static_cast<float>(fb.str);
        vv.str_ref = static_cast<float>(fb.str_ref);
        vv.counter = static_cast<uint32_t>(fb.counter);
        publishVehicle(vv);
        continue;
      }

      // ── v3 폴백: 8B × 3프레임, VEH_COMMIT 에서 세트 완성
      switch (f.id) {
        case kIdVehPose:
          std::memcpy(&pose, f.data, sizeof(pose));
          break;
        case kIdVehVel:
          std::memcpy(&vel, f.data, sizeof(vel));
          break;
        case kIdVehCommit: {
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
          vv.str_ref = 0.0f;   // v3 에는 없는 신호
          vv.counter = commit.counter;
          publishVehicle(vv);
          break;
        }
        default:
          break;
      }
    }
  }

  // v5·v3 공통 퍼블리시 경로. CSV 는 이 rx 스레드가 유일한 기록자이고, 소멸자가
  // 스레드를 join 한 뒤 닫으므로 잠금이 필요 없다. 타임스탬프는 header.stamp 와
  // 같은 값이라 bag 과 CSV 를 틱 단위로 겹칠 수 있다.
  void publishVehicle(const VehicleVector & vv)
  {
    vv_pub_->publish(vv);
    ++rx_count_;
    if (vehicle_csv_.is_open()) {
      writeVehicleCsvRow(vehicle_csv_, vv);
      if ((rx_count_.load() % 100) == 0) {
        vehicle_csv_.flush();   // 1초마다 — 전원이 끊겨도 직전 1초만 잃는다
      }
    }
  }

  std::string can_interface_;
  std::string vehicle_csv_path_;
  std::ofstream vehicle_csv_;      // dSPACE RX 피드백 (기록자 = rx 스레드 하나)
  bool can_fd_{true}, can_fd_brs_{true};
  bool rx_fmt_logged_{false};   // 첫 수신 프레임의 포맷을 1회만 로깅 (rx 스레드 전용)
  std::atomic<uint64_t> tx_frames_{0};          // write() 에 넘긴 누적 프레임 수
  uint64_t last_tx_frames_{0}, last_wire_tx_{0};  // 5초 대조용 (타이머 스레드 전용)
  // atomic shared_ptr로 TX callback·RX thread가 교체 중인 닫힌 fd를 재사용하지 않게 한다.
  // 각 작업이 잡은 handle이 끝난 뒤에만 옛 fd가 close된다.
  SocketHandle sock_;
  uint16_t tx_counter_{0};
  std::atomic<bool> running_{true};
  std::atomic<uint64_t> reconnect_count_{0};
  std::atomic<uint64_t> tx_count_{0}, rx_count_{0};
  std::thread rx_thread_;
  rclcpp::Publisher<VehicleVector>::SharedPtr vv_pub_;
  rclcpp::Subscription<TargetRef>::SharedPtr ref_sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
};

}  // namespace bridge_dspace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bridge_dspace::CanBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
