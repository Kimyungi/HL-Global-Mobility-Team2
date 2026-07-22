// mgm_node — Decision 계층 wrapper (CLAUDE.md §2, §5, §5.5).
//
// 판단·조립·병합 로직은 전부 core/mgm_step.cpp에 있다 — 이 파일은 wrapper만:
//   구독 msg → CoreSnapshot 변환, 10ms 틱 → mgm_step 호출,
//   CoreOutput → TargetRef 변환·발행, 지터 로깅, (옵션) 스냅샷 덤프.
// 여기에 판단 로직(if 장애물, if 신호등 …)을 추가하는 것은 §5.1·§5.5 위반.
//
// 실시간 구조:
//  - 10ms 루프는 전용 스레드 (clock_nanosleep 절대시각 + SCHED_FIFO 시도).
//    인지 콜백은 스냅샷 갱신만 — 루프를 블로킹하지 않는다 (§5.2 pull 방식).
//  - 주기 지터 로깅은 처음부터 내장 (§5.3) — §7 v1/v3 판정의 근거 데이터.
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <atomic>
#include <fstream>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "fma_interfaces/msg/lane_path.hpp"
#include "fma_interfaces/msg/gps_path.hpp"
#include "fma_interfaces/msg/avoid_status.hpp"
#include "fma_interfaces/msg/parking_status.hpp"
#include "fma_interfaces/msg/traffic_stop.hpp"
#include "fma_interfaces/msg/estop_request.hpp"
#include "fma_interfaces/msg/target_ref.hpp"

#include "core/mgm_step.hpp"
#include "tools/dump_format.hpp"

using fma_interfaces::msg::TargetRef;

namespace adas_mgm
{

constexpr int64_t kPeriodNs = 10'000'000;  // 10ms 고정

// 인지 콜백이 갱신하는 최신 msg 보관함 — 루프가 매 틱 복사(pull)
struct LatestMsgs
{
  fma_interfaces::msg::LanePath lane;
  fma_interfaces::msg::GpsPath gps;
  fma_interfaces::msg::AvoidStatus avoid;
  fma_interfaces::msg::ParkingStatus parking;
  fma_interfaces::msg::TrafficStop traffic;
  fma_interfaces::msg::EstopRequest estop;
};

// msg → CoreSnapshot 변환 (포맷 변환만 — 판단 금지)
void toCorePath(const std::vector<fma_interfaces::msg::RefPoint> & in, CorePath & out)
{
  out.n = static_cast<int32_t>(std::min<size_t>(in.size(), MGM_NUM_POINTS));
  for (int32_t i = 0; i < out.n; ++i) {
    out.pts[i] = CorePoint{in[i].x, in[i].y, in[i].yaw, in[i].curvature};
  }
}

CoreSnapshot toSnapshot(const LatestMsgs & m)
{
  CoreSnapshot s{};
  s.lane_confidence = m.lane.confidence;
  toCorePath(m.lane.points, s.lane_path);
  toCorePath(m.gps.points, s.gps_path);
  s.gps_accel_zone = m.gps.accel_zone;
  s.gps_parking_zone = m.gps.parking_zone;
  s.avoid_obstacle_detected = m.avoid.obstacle_detected;
  s.avoid_avoidable = m.avoid.avoidable;
  s.avoid_ttc = m.avoid.ttc;
  s.avoid_narrow_gap = m.avoid.narrow_gap;
  s.avoid_maneuver_done = m.avoid.maneuver_done;
  toCorePath(m.avoid.points, s.avoid_path);
  s.avoid_v_suggest = m.avoid.v_suggest;
  s.parking_space_found = m.parking.space_found;
  s.parking_path_blocked = m.parking.path_blocked;
  s.parking_done = m.parking.done;
  toCorePath(m.parking.points, s.parking_path);
  s.parking_v_suggest = m.parking.v_suggest;
  s.traffic_stop_required = m.traffic.stop_required;
  s.estop = m.estop.estop;
  return s;
}

// ── 주기 지터 로거 (§5.3, §7) — 최악값 기준. 윈도 단위로 통계·CSV 기록.
class JitterLogger
{
public:
  JitterLogger(rclcpp::Logger logger, const std::string & csv_path, int window)
  : logger_(logger), window_(window)
  {
    period_us_.reserve(window_);
    late_us_.reserve(window_);
    if (!csv_path.empty()) {
      csv_.open(csv_path, std::ios::app);
      csv_ << "# window_end_epoch_us,period_min,period_mean,period_max,period_p99,late_max\n";
    }
  }

  void record(int64_t period_ns, int64_t lateness_ns)
  {
    period_us_.push_back(period_ns / 1000.0);
    late_us_.push_back(lateness_ns / 1000.0);
    if (static_cast<int>(period_us_.size()) < window_) {
      return;
    }
    std::vector<double> sorted = period_us_;
    std::sort(sorted.begin(), sorted.end());
    const double mean =
      std::accumulate(sorted.begin(), sorted.end(), 0.0) / sorted.size();
    const double p99 = sorted[static_cast<size_t>(sorted.size() * 0.99)];
    const double late_max = *std::max_element(late_us_.begin(), late_us_.end());
    worst_late_us_ = std::max(worst_late_us_, late_max);

    RCLCPP_INFO(
      logger_, "period[us] min=%.0f mean=%.1f max=%.0f p99=%.0f | late max=%.0f (worst %.0f)",
      sorted.front(), mean, sorted.back(), p99, late_max, worst_late_us_);
    if (csv_.is_open()) {
      csv_ << std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count()
           << ',' << sorted.front() << ',' << mean << ',' << sorted.back()
           << ',' << p99 << ',' << late_max << '\n';
      csv_.flush();
    }
    period_us_.clear();
    late_us_.clear();
  }

private:
  rclcpp::Logger logger_;
  int window_;
  std::vector<double> period_us_, late_us_;
  double worst_late_us_{0.0};
  std::ofstream csv_;
};

class MgmNode : public rclcpp::Node
{
public:
  MgmNode()
  : Node("mgm_node")
  {
    CoreParams p{};
    p.lane_conf_exit = static_cast<float>(declare_parameter<double>("lane_conf_exit", 0.4));
    p.lane_conf_return = static_cast<float>(declare_parameter<double>("lane_conf_return", 0.6));
    p.n_cycles = static_cast<int32_t>(declare_parameter<int>("n_cycles", 20));
    p.v_base = static_cast<float>(declare_parameter<double>("v_base", 0.5));
    p.v_accel_zone = static_cast<float>(declare_parameter<double>("v_accel_zone", 1.0));
    p.v_narrow = static_cast<float>(declare_parameter<double>("v_narrow", 0.2));
    p.ttc_stop = static_cast<float>(declare_parameter<double>("ttc_stop", 0.8));
    p.blend_cycles = static_cast<int32_t>(declare_parameter<int>("blend_cycles", 10));
    p.a_up = static_cast<float>(declare_parameter<double>("a_up", 0.5));      // [m/s^2]
    p.a_down = static_cast<float>(declare_parameter<double>("a_down", 1.5));  // [m/s^2]
    mgm_init(core_state_, p);

    jitter_ = std::make_unique<JitterLogger>(
      get_logger(),
      declare_parameter<std::string>("jitter_csv_path", ""),
      static_cast<int>(declare_parameter<int>("jitter_window", 1000)));
    cpu_core_ = static_cast<int>(declare_parameter<int>("cpu_core", -1));

    // back-to-back 검증용 스냅샷 덤프 (tools/dump_format.hpp) — 미지정 시 비활성
    const auto dump_path = declare_parameter<std::string>("snapshot_dump_path", "");
    if (!dump_path.empty()) {
      dump_.open(dump_path, std::ios::binary | std::ios::trunc);
      DumpHeader h{kDumpMagic, kDumpVersion,
        static_cast<uint32_t>(sizeof(CoreSnapshot)),
        static_cast<uint32_t>(sizeof(CoreParams)), p};
      dump_.write(reinterpret_cast<const char *>(&h), sizeof(h));
      RCLCPP_INFO(get_logger(), "snapshot dump → %s", dump_path.c_str());
    }

    pub_ = create_publisher<TargetRef>("/adas/target_ref", rclcpp::QoS(1));

    // 인지 콜백 — 스냅샷 갱신만. 여기서 어떤 계산도 하지 않는다.
    auto qos = rclcpp::QoS(1);
    sub_lane_ = create_subscription<fma_interfaces::msg::LanePath>(
      "/perception/lane_path", qos,
      [this](fma_interfaces::msg::LanePath::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); msgs_.lane = *m;});
    sub_gps_ = create_subscription<fma_interfaces::msg::GpsPath>(
      "/perception/gps_path", qos,
      [this](fma_interfaces::msg::GpsPath::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); msgs_.gps = *m;});
    sub_avoid_ = create_subscription<fma_interfaces::msg::AvoidStatus>(
      "/perception/avoid", qos,
      [this](fma_interfaces::msg::AvoidStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); msgs_.avoid = *m;});
    sub_parking_ = create_subscription<fma_interfaces::msg::ParkingStatus>(
      "/perception/parking", qos,
      [this](fma_interfaces::msg::ParkingStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); msgs_.parking = *m;});
    sub_traffic_ = create_subscription<fma_interfaces::msg::TrafficStop>(
      "/perception/traffic_stop", qos,
      [this](fma_interfaces::msg::TrafficStop::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); msgs_.traffic = *m;});
    sub_estop_ = create_subscription<fma_interfaces::msg::EstopRequest>(
      "/perception/estop", qos,
      [this](fma_interfaces::msg::EstopRequest::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); msgs_.estop = *m;});

    msgs_.avoid.ttc = 1e9f;  // 인지 도착 전 TTC=0으로 오인해 정지하는 것 방지

    loop_thread_ = std::thread([this] {loop();});
  }

  ~MgmNode() override
  {
    running_ = false;
    if (loop_thread_.joinable()) {
      loop_thread_.join();
    }
  }

private:
  void loop()
  {
    // SCHED_FIFO + 코어 고정 시도 (§5.2) — 실패해도 동작은 하되 지터로 드러난다
    sched_param sp{};
    sp.sched_priority = 80;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
      RCLCPP_WARN(get_logger(),
        "SCHED_FIFO 설정 실패 (권한 필요: ulimit -r 또는 CAP_SYS_NICE) — 일반 스케줄러로 동작");
    }
    if (cpu_core_ >= 0) {
      cpu_set_t set;
      CPU_ZERO(&set);
      CPU_SET(cpu_core_, &set);
      if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
        RCLCPP_WARN(get_logger(), "CPU 코어 고정 실패 (core=%d)", cpu_core_);
      }
    }

    timespec deadline;
    clock_gettime(CLOCK_MONOTONIC, &deadline);
    int64_t prev_ns = toNs(deadline);

    while (running_ && rclcpp::ok()) {
      addNs(deadline, kPeriodNs);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);

      timespec now_ts;
      clock_gettime(CLOCK_MONOTONIC, &now_ts);
      const int64_t now_ns = toNs(now_ts);
      jitter_->record(now_ns - prev_ns, now_ns - toNs(deadline));
      prev_ns = now_ns;

      tick();
    }
  }

  void tick()
  {
    LatestMsgs m;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      m = msgs_;  // pull — 이후 인지가 갱신해도 이번 틱은 일관된 스냅샷 사용
    }
    const CoreSnapshot s = toSnapshot(m);

    if (dump_.is_open()) {
      dump_.write(reinterpret_cast<const char *>(&s), sizeof(s));
    }

    const CoreOutput out = mgm_step(s, core_state_);  // 판단+실행 전부 코어에서

    TargetRef msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.state = out.state;
    msg.v_ref = out.v_ref;
    msg.ref_points.resize(MGM_NUM_POINTS);
    for (int32_t i = 0; i < MGM_NUM_POINTS; ++i) {
      msg.ref_points[i].x = out.ref_points[i].x;
      msg.ref_points[i].y = out.ref_points[i].y;
      msg.ref_points[i].yaw = out.ref_points[i].yaw;
      msg.ref_points[i].curvature = out.ref_points[i].curvature;
    }
    pub_->publish(msg);
  }

  static int64_t toNs(const timespec & t)
  {
    return static_cast<int64_t>(t.tv_sec) * 1'000'000'000 + t.tv_nsec;
  }
  static void addNs(timespec & t, int64_t ns)
  {
    t.tv_nsec += ns;
    while (t.tv_nsec >= 1'000'000'000) {
      t.tv_nsec -= 1'000'000'000;
      ++t.tv_sec;
    }
  }

  CoreState core_state_;
  std::unique_ptr<JitterLogger> jitter_;
  int cpu_core_{-1};
  std::ofstream dump_;

  std::mutex mtx_;
  LatestMsgs msgs_;

  rclcpp::Publisher<TargetRef>::SharedPtr pub_;
  rclcpp::Subscription<fma_interfaces::msg::LanePath>::SharedPtr sub_lane_;
  rclcpp::Subscription<fma_interfaces::msg::GpsPath>::SharedPtr sub_gps_;
  rclcpp::Subscription<fma_interfaces::msg::AvoidStatus>::SharedPtr sub_avoid_;
  rclcpp::Subscription<fma_interfaces::msg::ParkingStatus>::SharedPtr sub_parking_;
  rclcpp::Subscription<fma_interfaces::msg::TrafficStop>::SharedPtr sub_traffic_;
  rclcpp::Subscription<fma_interfaces::msg::EstopRequest>::SharedPtr sub_estop_;

  std::atomic<bool> running_{true};
  std::thread loop_thread_;
};

}  // namespace adas_mgm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<adas_mgm::MgmNode>());
  rclcpp::shutdown();
  return 0;
}
