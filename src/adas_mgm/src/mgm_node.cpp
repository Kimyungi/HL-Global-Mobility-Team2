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
#include "std_msgs/msg/bool.hpp"
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
  s.gps_at_end = m.gps.at_end;
  s.gps_cross_track = m.gps.cross_track_m;
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
    p.wrongway_yaw = static_cast<float>(declare_parameter<double>("wrongway_yaw_rad", 2.1));
    p.wrongway_cycles = static_cast<int32_t>(declare_parameter<int>("wrongway_cycles", 50));
    // avoid→waypoint 복귀 후 lane 전이 보류 틱 (§4 복귀 정책 — 2026-08-14)
    p.avoid_return_hold_cycles =
      static_cast<int32_t>(declare_parameter<int>("avoid_return_hold_cycles", 300));
    // waypoint→lane 전이 허용 최대 횡오차 [m] (0 이하 = 게이트 끔)
    p.lane_entry_max_cross =
      static_cast<float>(declare_parameter<double>("lane_entry_max_cross_m", 0.5));
    mgm_init(core_state_, p);

    // estop 입력 신선도 watchdog 한도 — stack_estop 하트비트 50ms의 5주기
    estop_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("estop_stale_timeout_sec", 0.25) * 1e9);

    // lane_path 입력 신선도 watchdog (2026-08-07 실차 테스트에서 발견 — estop과
    // 달리 lane_path엔 신선도 체크가 없어서, stack_lane이 죽어도 MGM이 죽기
    // 직전 마지막 값을 계속 재사용해 계속 주행 명령을 냈음). estop과 동일하게
    // "입력 컨디셔닝"으로 처리 — state==LANE일 때만 적용(다른 스테이트는
    // lane_path를 안 쓰므로), 판단(정지)은 기존 estop 경로를 그대로 재사용.
    lane_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("lane_stale_timeout_sec", 0.5) * 1e9);
    // gps_path도 동일 — waypoint 스테이트가 gps_path 없이도 v_base로 계속
    // 주행하던 문제 방지 (2026-08-07, lane watchdog과 같은 날 발견).
    gps_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("gps_stale_timeout_sec", 0.5) * 1e9);
    // traffic_stop 신선도 watchdog — 수신 이력이 있은 뒤 끊긴 경우만 (§5.7 ③)
    traffic_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("traffic_stale_timeout_sec", 0.5) * 1e9);
    // avoid 신선도 watchdog (§5.7 ⑤, 2026-08-12 회피 통합) — stale이면 진입 재료
    // 무효화, AVOID 스테이트 중이면 estop 보정 (낡은 회피 경로 주행 차단)
    avoid_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("avoid_stale_timeout_sec", 0.5) * 1e9);

    // 출발 인가 게이트 (2026-08-11, 실차 launch 전용) — true면 /operator/go 수신
    // 전까지 estop 보정 유지(v_ref 0 대기). launch 직후 바로 출발해 출발 전
    // 점검이 불가능하던 문제 해결. 판단이 아니라 운용 입력 컨디셔닝 — §5.7의
    // estop 경로 재사용(코어에 새 정지 로직 없음). 인가는 tools/go 스크립트가
    // RTK FIXED 확인 후 발행한다.
    wait_go_ = declare_parameter<bool>("wait_go", false);
    if (wait_go_) {
      sub_go_ = create_subscription<std_msgs::msg::Bool>(
        "/operator/go", rclcpp::QoS(1),
        [this](std_msgs::msg::Bool::ConstSharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          if (m->data && !go_received_) {
            RCLCPP_INFO(get_logger(), "출발 인가 수신 — 주행 시작");
          }
          go_received_ = m->data;});
      RCLCPP_INFO(get_logger(),
        "출발 대기 모드 — 점검 후 `ros2 run adas_mgm go` 로 출발 인가");
    }

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
        std::lock_guard<std::mutex> lk(mtx_);
        msgs_.lane = *m;
        last_lane_rx_ns_ = monotonicNs();});
    sub_gps_ = create_subscription<fma_interfaces::msg::GpsPath>(
      "/perception/gps_path", qos,
      [this](fma_interfaces::msg::GpsPath::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        msgs_.gps = *m;
        last_gps_rx_ns_ = monotonicNs();});
    sub_avoid_ = create_subscription<fma_interfaces::msg::AvoidStatus>(
      "/perception/avoid", qos,
      [this](fma_interfaces::msg::AvoidStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        msgs_.avoid = *m;
        last_avoid_rx_ns_ = monotonicNs();});
    sub_parking_ = create_subscription<fma_interfaces::msg::ParkingStatus>(
      "/perception/parking", qos,
      [this](fma_interfaces::msg::ParkingStatus::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); msgs_.parking = *m;});
    sub_traffic_ = create_subscription<fma_interfaces::msg::TrafficStop>(
      "/perception/traffic_stop", qos,
      [this](fma_interfaces::msg::TrafficStop::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        msgs_.traffic = *m;
        last_traffic_rx_ns_ = monotonicNs();});
    sub_estop_ = create_subscription<fma_interfaces::msg::EstopRequest>(
      "/perception/estop", qos,
      [this](fma_interfaces::msg::EstopRequest::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        msgs_.estop = *m;
        last_estop_rx_ns_ = monotonicNs();});

    msgs_.avoid.ttc = 1e9f;   // 인지 도착 전 TTC=0으로 오인해 정지하는 것 방지
    msgs_.estop.estop = true;  // 첫 EstopRequest 수신 전 fail-safe — 미수신 = 정지

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
    int64_t estop_rx_ns;
    int64_t lane_rx_ns;
    int64_t gps_rx_ns;
    int64_t traffic_rx_ns;
    int64_t avoid_rx_ns;
    bool go;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      m = msgs_;  // pull — 이후 인지가 갱신해도 이번 틱은 일관된 스냅샷 사용
      estop_rx_ns = last_estop_rx_ns_;
      lane_rx_ns = last_lane_rx_ns_;
      gps_rx_ns = last_gps_rx_ns_;
      traffic_rx_ns = last_traffic_rx_ns_;
      avoid_rx_ns = last_avoid_rx_ns_;
      go = go_received_;
    }
    // estop 입력 신선도 watchdog — 판단이 아니라 입력 컨디셔닝 (§3 dSPACE
    // counter watchdog의 PC측 대응물). stack_estop 미수신/사망 시 스냅샷의
    // estop을 true로 보정하고, 정지 판단(v_ref=0) 자체는 코어가 한다.
    const bool estop_stale = estop_rx_ns < 0 || monotonicNs() - estop_rx_ns > estop_stale_ns_;
    // 보정 전 "실제 수신값" 보존 — at_end 래치 해제는 이 값으로만 한다 (CLAUDE.md
    // §4 래치). stale이면 마지막 값을 조작 의사로 신뢰할 수 없으므로 false.
    const bool estop_real = !estop_stale && m.estop.estop;
    if (estop_stale) {
      m.estop.estop = true;
    }
    // 출발 인가 게이트 — 인가 전까지 estop 보정으로 정지 대기 (운용 입력
    // 컨디셔닝, §5.7과 동류). estop_real(래치 해제용)에는 영향 없음.
    if (wait_go_ && !go) {
      m.estop.estop = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "출발 대기 중 — 점검 완료 후 `ros2 run adas_mgm go` 로 출발");
    }
    // lane_path 입력 신선도 watchdog (2026-08-07 실차 테스트에서 발견) — state가
    // 현재 LANE일 때만 적용(다른 스테이트는 lane_path를 안 씀). stack_lane이
    // 죽었는데도 MGM이 마지막 값을 계속 재사용해 주행을 계속하던 문제를 막는다.
    // estop 경로를 그대로 재사용 — 새 판단 로직을 추가하는 게 아니라 기존
    // "estop=true → 전 스테이트 정지" 판단에 태우는 것 (§5.1 준수).
    const bool lane_stale = lane_rx_ns < 0 || monotonicNs() - lane_rx_ns > lane_stale_ns_;
    if (core_state_.state == MGM_STATE_LANE && lane_stale && !estop_stale) {
      m.estop.estop = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "lane_path 신선도 초과(state=lane) — estop 강제 (stack_lane 확인 필요)");
    }
    // gps_path 신선도 watchdog — lane과 대칭. waypoint 스테이트가 gps_path 없이도
    // v_base로 계속 주행하던 문제 방지 (실제로 lane→waypoint 자동 전이 후 이
    // 경로로 재현됨, 2026-08-07).
    // fix 상실도 동일 취급 (2026-08-11 통합 점검에서 발견): stack_gps는 fix가
    // 없거나 stale이면 fix_quality=0인 빈 GpsPath를 계속 발행한다 — 수신 시각만
    // 보면 watchdog을 통과해, 조립 블록이 직전 ref를 hold한 채 v_base로 계속
    // 주행 명령을 내던 구멍. "신선하지만 무효인 입력"을 stale과 같은 경로로
    // 태운다 — 새 판단이 아니라 입력 컨디셔닝(§5.7 ②의 확장).
    const bool gps_stale = gps_rx_ns < 0 || monotonicNs() - gps_rx_ns > gps_stale_ns_;
    const bool gps_no_fix = m.gps.fix_quality == 0 || m.gps.points.empty();
    if (core_state_.state == MGM_STATE_WAYPOINT && (gps_stale || gps_no_fix) && !estop_stale) {
      m.estop.estop = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        gps_stale ? "gps_path 신선도 초과(state=waypoint) — estop 강제 (stack_gps 확인 필요)"
                  : "gps fix 상실(state=waypoint, fix_quality=0/빈 경로) — estop 강제 (RTK 확인 필요)");
    }
    // traffic_stop 신선도 watchdog (§5.7 ③) — lane/gps와 달리 **수신 이력이 있은
    // 뒤 끊긴 경우만** 보정한다(사망 감지). 미수신은 보정하지 않음: traffic은
    // 경로가 아니라 제약 입력이라, 단독 스택 시험(GPS 단독 등)이 stack_traffic
    // 없이도 성립해야 하기 때문. 적색 래치 중 사망(마지막 발행 true)은 msgs_에
    // true가 남아 어차피 정지 유지 — 위험 케이스는 false 상태로 죽은 뒤 적색이
    // 켜지는 경우이며 이 보정이 그걸 막는다. estop이 아닌 traffic 요구로 태워
    // 일반 감속 정지(rate limit)로 선다. (2026-08-08, PR #21 검토에서 도출)
    if (traffic_rx_ns >= 0 && monotonicNs() - traffic_rx_ns > traffic_stale_ns_ &&
      !m.traffic.stop_required)
    {
      m.traffic.stop_required = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "traffic_stop 신선도 초과 — 정지 요구 강제 (stack_traffic 확인 필요)");
    }
    // avoid 신선도 watchdog (§5.7 ⑤, 2026-08-12 회피 통합) — 미수신/staleness 시
    // 진입 재료를 무효화해 죽은 stack_avoid의 마지막 메시지로 AVOID에 진입하는
    // 것을 막는다 (미수신 무효화는 단독 스택 시험과도 양립 — 원래 기본값이 false).
    // 이미 AVOID 스테이트면 lane/gps와 동일하게 estop 보정 — 낡은 회피 경로로
    // 계속 주행하던 구멍 차단. ttc는 미수신 초기값과 같은 1e9로 (즉시정지 바닥 오인 방지).
    const bool avoid_stale = avoid_rx_ns < 0 || monotonicNs() - avoid_rx_ns > avoid_stale_ns_;
    if (avoid_stale) {
      m.avoid.obstacle_detected = false;
      m.avoid.avoidable = false;
      m.avoid.ttc = 1e9f;
      if (core_state_.state == MGM_STATE_AVOID && !estop_stale) {
        m.estop.estop = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "avoid 신선도 초과(state=avoid) — estop 강제 (stack_avoid 확인 필요)");
      }
    }
    CoreSnapshot s = toSnapshot(m);
    s.estop_latch_release = estop_real;  // toSnapshot은 LatestMsgs만 알므로 여기서 주입

    if (dump_.is_open()) {
      dump_.write(reinterpret_cast<const char *>(&s), sizeof(s));
    }

    const CoreOutput out = mgm_step(s, core_state_);  // 판단+실행 전부 코어에서

    TargetRef msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.state = out.state;
    msg.v_ref = out.v_ref;
    msg.ref_points.resize(out.n_points);
    for (int32_t i = 0; i < out.n_points; ++i) {
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
  static int64_t monotonicNs()
  {
    timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return toNs(t);
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
  int64_t last_estop_rx_ns_{-1};  // 마지막 EstopRequest 수신 시각 (미수신 = -1)
  int64_t estop_stale_ns_{250'000'000};
  int64_t last_lane_rx_ns_{-1};   // 마지막 LanePath 수신 시각 (미수신 = -1)
  int64_t lane_stale_ns_{500'000'000};
  int64_t last_gps_rx_ns_{-1};    // 마지막 GpsPath 수신 시각 (미수신 = -1)
  int64_t gps_stale_ns_{500'000'000};
  int64_t last_traffic_rx_ns_{-1};  // 마지막 TrafficStop 수신 시각 (미수신 = -1)
  int64_t traffic_stale_ns_{500'000'000};
  int64_t last_avoid_rx_ns_{-1};  // 마지막 AvoidStatus 수신 시각 (미수신 = -1)
  int64_t avoid_stale_ns_{500'000'000};
  bool wait_go_{false};             // 출발 인가 게이트 활성 (실차 launch 전용)
  bool go_received_{false};         // /operator/go 마지막 수신값

  rclcpp::Publisher<TargetRef>::SharedPtr pub_;
  rclcpp::Subscription<fma_interfaces::msg::LanePath>::SharedPtr sub_lane_;
  rclcpp::Subscription<fma_interfaces::msg::GpsPath>::SharedPtr sub_gps_;
  rclcpp::Subscription<fma_interfaces::msg::AvoidStatus>::SharedPtr sub_avoid_;
  rclcpp::Subscription<fma_interfaces::msg::ParkingStatus>::SharedPtr sub_parking_;
  rclcpp::Subscription<fma_interfaces::msg::TrafficStop>::SharedPtr sub_traffic_;
  rclcpp::Subscription<fma_interfaces::msg::EstopRequest>::SharedPtr sub_estop_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_go_;

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
