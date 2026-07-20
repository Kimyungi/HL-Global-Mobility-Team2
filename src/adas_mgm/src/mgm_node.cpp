// mgm_node — Decision 계층 본체 (CLAUDE.md §2, §4, §5).
//
// 구조 원칙:
//  - 10ms 루프는 전용 스레드 (clock_nanosleep 절대시각 + SCHED_FIFO 시도).
//    인지 콜백은 스냅샷 갱신만 — 루프를 블로킹하지 않는다 (§5.2 pull 방식).
//  - 판단은 StateMachine 한 곳. 이 파일의 조립/병합은 결정의 실행부일 뿐이며
//    요구 조건 분기(if 장애물, if 신호등 …)를 여기 추가하는 것은 §5.1 위반.
//  - 주기 지터 로깅은 처음부터 내장 (§5.3) — §7 v1/v3 판정의 근거 데이터.
#include <pthread.h>
#include <sched.h>
#include <time.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <fstream>
#include <mutex>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "fma_interfaces/msg/target_ref.hpp"
#include "state_machine.hpp"

using fma_interfaces::msg::RefPoint;
using fma_interfaces::msg::TargetRef;

namespace adas_mgm
{

constexpr int kNumPoints = 20;          // dSPACE MPC 지평과 일치 (PROTOCOL.md)
constexpr int64_t kPeriodNs = 10'000'000;  // 10ms 고정

// ── ref points 조립 — 스테이트가 고른 경로를 포맷 변환·전환 연속 처리만 (§5.1, §5.6)
class RefAssembler
{
public:
  explicit RefAssembler(int blend_cycles)
  : blend_cycles_(blend_cycles)
  {
    RefPoint origin{};
    out_.assign(kNumPoints, origin);  // 인지 도착 전: 제자리 (v_ref가 어차피 속도를 지배)
  }

  const std::vector<RefPoint> & assemble(PathSource src, const Snapshot & s)
  {
    const auto * pts = select(src, s);
    if (pts == nullptr || pts->empty()) {
      return out_;  // 선택 소스 미도착 → 직전 출력 유지 (판단 아님 — 데이터 hold)
    }
    std::vector<RefPoint> target = normalize(*pts);

    if (src != last_src_) {           // 스테이트 전환 → ref 불연속 방지 블렌드 시작
      blend_from_ = out_;
      blend_left_ = blend_cycles_;
      last_src_ = src;
    }
    if (blend_left_ > 0) {
      const double a = 1.0 - static_cast<double>(blend_left_) / (blend_cycles_ + 1);
      for (int i = 0; i < kNumPoints; ++i) {
        out_[i].x = lerp(blend_from_[i].x, target[i].x, a);
        out_[i].y = lerp(blend_from_[i].y, target[i].y, a);
        out_[i].yaw = lerp(blend_from_[i].yaw, target[i].yaw, a);
        out_[i].curvature = lerp(blend_from_[i].curvature, target[i].curvature, a);
      }
      --blend_left_;
    } else {
      out_ = std::move(target);
    }
    return out_;
  }

private:
  static float lerp(float a, float b, double t)
  {
    return static_cast<float>(a + (b - a) * t);
  }

  static const std::vector<RefPoint> * select(PathSource src, const Snapshot & s)
  {
    switch (src) {
      case PathSource::Lane: return &s.lane.points;
      case PathSource::Gps: return &s.gps.points;
      case PathSource::Avoid: return &s.avoid.points;
      case PathSource::Parking: return &s.parking.points;
    }
    return nullptr;
  }

  static std::vector<RefPoint> normalize(const std::vector<RefPoint> & in)
  {
    std::vector<RefPoint> out(kNumPoints);
    for (int i = 0; i < kNumPoints; ++i) {
      out[i] = in[std::min<size_t>(i, in.size() - 1)];  // 부족분은 마지막 점 복제
    }
    return out;
  }

  int blend_cycles_;
  int blend_left_{0};
  PathSource last_src_{PathSource::Lane};
  std::vector<RefPoint> out_, blend_from_;
};

// ── 종방향 병합 — rate limit만 (§5.6). immediate_stop은 스테이트의 결정으로 우회.
class VrefMerger
{
public:
  VrefMerger(double a_up, double a_down)
  : dv_up_(a_up * 0.01), dv_down_(a_down * 0.01) {}

  double merge(const Decision & d)
  {
    if (d.immediate_stop) {
      v_ = 0.0;  // 긴급 정지·TTC 바닥은 램프 없이 즉시 (스테이트 머신이 결정)
      return v_;
    }
    v_ = std::clamp(d.v_ref, v_ - dv_down_, v_ + dv_up_);
    return v_;
  }

private:
  double dv_up_, dv_down_, v_{0.0};
};

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
    Params p;
    p.lane_conf_exit = declare_parameter<double>("lane_conf_exit", p.lane_conf_exit);
    p.lane_conf_return = declare_parameter<double>("lane_conf_return", p.lane_conf_return);
    p.n_cycles = declare_parameter<int>("n_cycles", p.n_cycles);
    p.v_base = declare_parameter<double>("v_base", p.v_base);
    p.v_accel_zone = declare_parameter<double>("v_accel_zone", p.v_accel_zone);
    p.v_narrow = declare_parameter<double>("v_narrow", p.v_narrow);
    p.ttc_stop = declare_parameter<double>("ttc_stop", p.ttc_stop);
    sm_ = std::make_unique<StateMachine>(p);

    assembler_ = std::make_unique<RefAssembler>(
      static_cast<int>(declare_parameter<int>("blend_cycles", 10)));
    merger_ = std::make_unique<VrefMerger>(
      declare_parameter<double>("a_up", 0.5),      // [m/s^2]
      declare_parameter<double>("a_down", 1.5));   // [m/s^2] 일반 감속 한계
    jitter_ = std::make_unique<JitterLogger>(
      get_logger(),
      declare_parameter<std::string>("jitter_csv_path", ""),
      static_cast<int>(declare_parameter<int>("jitter_window", 1000)));
    cpu_core_ = static_cast<int>(declare_parameter<int>("cpu_core", -1));

    pub_ = create_publisher<TargetRef>("/adas/target_ref", rclcpp::QoS(1));

    // 인지 콜백 — 스냅샷 갱신만. 여기서 어떤 계산도 하지 않는다.
    auto qos = rclcpp::QoS(1);
    sub_lane_ = create_subscription<fma_interfaces::msg::LanePath>(
      "/perception/lane_path", qos,
      [this](fma_interfaces::msg::LanePath::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); snap_.lane = *m;});
    sub_gps_ = create_subscription<fma_interfaces::msg::GpsPath>(
      "/perception/gps_path", qos,
      [this](fma_interfaces::msg::GpsPath::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); snap_.gps = *m;});
    sub_avoid_ = create_subscription<fma_interfaces::msg::AvoidStatus>(
      "/perception/avoid", qos,
      [this](fma_interfaces::msg::AvoidStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); snap_.avoid = *m;});
    sub_parking_ = create_subscription<fma_interfaces::msg::ParkingStatus>(
      "/perception/parking", qos,
      [this](fma_interfaces::msg::ParkingStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); snap_.parking = *m;});
    sub_traffic_ = create_subscription<fma_interfaces::msg::TrafficStop>(
      "/perception/traffic_stop", qos,
      [this](fma_interfaces::msg::TrafficStop::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); snap_.traffic = *m;});
    sub_estop_ = create_subscription<fma_interfaces::msg::EstopRequest>(
      "/perception/estop", qos,
      [this](fma_interfaces::msg::EstopRequest::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); snap_.estop = *m;});

    snap_.avoid.ttc = 1e9f;  // 인지 도착 전 TTC=0으로 오인해 정지하는 것 방지

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
    Snapshot s;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      s = snap_;  // pull — 이후 인지가 갱신해도 이번 틱은 일관된 스냅샷 사용
    }

    const Decision d = sm_->decide(s);                    // 판단 (유일한 곳)
    const auto & points = assembler_->assemble(d.path_source, s);  // 실행: 조립
    const double v = merger_->merge(d);                   // 실행: 병합(rate limit)

    TargetRef msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.state = static_cast<uint8_t>(d.state);
    msg.v_ref = static_cast<float>(v);
    msg.ref_points.assign(points.begin(), points.end());
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

  std::unique_ptr<StateMachine> sm_;
  std::unique_ptr<RefAssembler> assembler_;
  std::unique_ptr<VrefMerger> merger_;
  std::unique_ptr<JitterLogger> jitter_;
  int cpu_core_{-1};

  std::mutex mtx_;
  Snapshot snap_;

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
