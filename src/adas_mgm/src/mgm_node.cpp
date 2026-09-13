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
#include <cmath>
#include <exception>
#include <fstream>
#include <memory>
#include <limits>
#include <mutex>
#include <numeric>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "fma_interfaces/msg/mgm_state.hpp"
#include "fma_interfaces/msg/lane_path.hpp"
#include "fma_interfaces/msg/gps_path.hpp"
#include "fma_interfaces/msg/avoid_status.hpp"
#include "fma_interfaces/msg/parking_status.hpp"
#include "fma_interfaces/msg/parking_command.hpp"
#include "fma_interfaces/msg/traffic_stop.hpp"
#include "fma_interfaces/msg/estop_request.hpp"
#include "fma_interfaces/msg/can_health.hpp"
#include "fma_interfaces/msg/target_ref.hpp"
#include "fma_interfaces/msg/vehicle_vector.hpp"

#include "core/mgm_step.hpp"
#include "src/decision_backend.hpp"
#include "src/reference_clock.hpp"
#include "core/reference_safety.hpp"
#include "src/transition_log.hpp"
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
  fma_interfaces::msg::CanHealth can;   // 브리지 CAN 링크 건전성 (§5.7 ⑥)
  fma_interfaces::msg::VehicleVector vehicle;  // dSPACE 실차속도 피드백
};

fma_interfaces::msg::ZoneContext toRosZone(const ZoneContext & zone)
{
  fma_interfaces::msg::ZoneContext msg;
  msg.zone_valid = zone.zone_valid;
  msg.zone_id = zone.zone_id;
  msg.zone_type = static_cast<uint8_t>(zone.zone_type);
  msg.mission_type = static_cast<uint8_t>(zone.mission_type);
  msg.mission_id = zone.mission_id;
  msg.in_zone = zone.in_zone;
  msg.zone_entered = zone.zone_entered;
  msg.zone_exited = zone.zone_exited;
  msg.mission_entry_suppressed = zone.mission_entry_suppressed;
  msg.raw_in_zone = zone.raw_in_zone; msg.stable_in_zone = zone.stable_in_zone;
  msg.enter_count = zone.enter_count; msg.exit_count = zone.exit_count;
  msg.boundary_distance_m = zone.boundary_distance_m;
  return msg;
}

// msg → CoreSnapshot 변환 (포맷 변환만 — 판단 금지)
void toCorePath(const std::vector<fma_interfaces::msg::RefPoint> & in, CorePath & out,
  bool single_point)
{
  const int32_t limit = single_point ? MGM_CONTROL_POINTS : MGM_NUM_POINTS;
  // Preserve an oversized-input marker; never silently accept a partial path.
  out.n = static_cast<int32_t>(std::min<size_t>(in.size(), limit + 1));
  for (int32_t i = 0; i < std::min(out.n, limit); ++i) {
    out.pts[i] = CorePoint{in[i].x, in[i].y, in[i].yaw, in[i].curvature};
  }
}

CoreSnapshot toSnapshot(const LatestMsgs & m, bool single_point)
{
  CoreSnapshot s{};
  s.lane_confidence = m.lane.confidence;
  toCorePath(m.lane.points, s.lane_path, single_point);
  toCorePath(m.gps.points, s.gps_path, single_point);
  s.gps_accel_zone = m.gps.accel_zone;
  s.gps_parking_zone = m.gps.parking_zone;
  s.gps_at_end = m.gps.at_end;
  const auto & route = m.gps.route;
  s.route.enabled = route.enabled;
  s.route.connecting = route.connecting; s.route.next_connecting = route.next_connecting;
  s.route.completion = static_cast<adas_mgm::RouteCompletion>(route.completion);
  s.route.sequence_id = route.sequence_id; s.route.instance_id = route.instance_id;
  s.route.acknowledged_request = route.acknowledged_request;
  s.route.index = route.index; s.route.count = route.count;
  s.route.required_count = static_cast<int32_t>(std::min<size_t>(route.required_missions.size(), 257));
  for (int i=0; i<std::min(s.route.required_count, 256); ++i) {s.route.required_missions[i] = route.required_missions[i];}
  s.gps_cross_track = m.gps.cross_track_m;
  s.gps_stop_zone = m.gps.stop_zone;      // 0 = 아님, 1~ = 지정 정지 지점 번호
  s.gps_avoid_zone = m.gps.avoid_zone;    // 회피 허용 구간 안인가
  s.gps_gps_only_zone = m.gps.gps_only_zone;  // legacy/generated only
  s.zones.zone_valid = m.gps.zone_valid;
  s.zones.count = static_cast<int32_t>(std::min<size_t>(m.gps.zones.size(), MGM_ZONE_CAPACITY));
  for (int i = 0; i < s.zones.count; ++i) {
    const auto & zone = m.gps.zones[i];
    s.zones.zone_valid = s.zones.zone_valid && zone.zone_valid;
    s.zones.observations[i] = ZoneObservation{
      zone.zone_id, static_cast<ZoneType>(zone.zone_type),
      static_cast<MissionType>(zone.mission_type), zone.mission_id, zone.in_zone, zone.boundary_distance_m};
  }
  // 접선 폴백(HEADING_TANGENT)은 헤딩을 모를 때의 가정이라 신뢰 불가 (§4 역방향 래치)
  s.gps_heading_valid = (m.gps.heading_source != fma_interfaces::msg::GpsPath::HEADING_TANGENT);
  s.avoid_obstacle_detected = m.avoid.obstacle_detected;
  s.avoid_avoidable = m.avoid.avoidable;
  s.avoid_ttc = m.avoid.ttc;
  s.avoid_narrow_gap = m.avoid.narrow_gap;
  s.avoid_maneuver_done = m.avoid.maneuver_done;
  toCorePath(m.avoid.points, s.avoid_path, single_point);
  s.avoid_v_suggest = m.avoid.v_suggest;
  s.parking_space_found = m.parking.space_found;
  s.parking_path_blocked = m.parking.path_blocked;
  s.parking_done = m.parking.done;
  toCorePath(m.parking.points, s.parking_path, single_point);
  s.parking_v_suggest = m.parking.v_suggest;
  s.traffic_stop_required = m.traffic.stop_required;
  s.traffic_red_active = m.traffic.red_active;
  s.traffic_green_active = m.traffic.green_active;
  s.traffic_stopline_detected = m.traffic.stopline_detected;
  s.traffic_stop_distance = m.traffic.stop_distance;
  s.traffic_fail_safe_stop = m.traffic.fail_safe_stop;
  s.vehicle_speed = m.vehicle.v;
  s.estop = m.estop.estop;
  // 후방 여유 (§4 후진 탈출) — staleness 보정은 loop()에서 estop 과 함께 처리한다.
  s.estop_rear_clear = m.estop.rear_clear;
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
    p.lane_conf_exit = static_cast<float>(declare_parameter<double>("lane_conf_exit", 0.35));
    p.lane_conf_return = static_cast<float>(declare_parameter<double>("lane_conf_return", 0.7));
    p.n_cycles = static_cast<int32_t>(declare_parameter<int>("n_cycles", 50));
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
    // AVOID 최대 지속 틱 (0 이하 = 상한 없음)
    p.avoid_max_cycles =
      static_cast<int32_t>(declare_parameter<int>("avoid_max_cycles", 1200));
    // 0 = 상한 없음(구동작). params.yaml 미적용 launch에서 조용히 감속되지 않도록
    // 기본값은 끔으로 둔다 — 켜는 건 params.yaml의 명시적 선택이어야 한다.
    p.v_avoid = static_cast<float>(declare_parameter<double>("v_avoid", 0.0));
    // 지정 지점 정지 [틱] — GpsPath.stop_zone 지점에서 정차하는 시간 (0 = 끔).
    // 지점 자체는 stack_gps 의 stop_points_latlon 이 정한다.
    p.stop_zone_hold_cycles =
      static_cast<int32_t>(declare_parameter<int>("stop_zone_hold_cycles", 0));
    // 회피 허용 구간 밖에서는 AVOID 전이 금지 (stack_gps 의 avoid_zone_latlon 과 짝).
    // 기본 false = 구동작(어디서나 회피) — 켜는 것은 launch/params의 명시적 선택.
    p.avoid_zone_only = declare_parameter<bool>("avoid_zone_only", false) ? 1 : 0;
    // ── 후진 탈출 (§4, 2026-08-24). 기본 끔 — 켜는 것은 params.yaml/launch 의
    // 명시적 선택이어야 한다. 후진은 사람이 뒤를 확인한 상태에서만 시험할 동작이다.
    p.escape_after_cycles =
      static_cast<int32_t>(declare_parameter<int>("escape_after_cycles", 0));
    // 음수여야 의미가 있다 — 0 이상이면 코어가 기능을 끈다(안전 불변식).
    p.v_escape = static_cast<float>(declare_parameter<double>("v_escape", -0.3));
    // 200틱 × 10ms × 0.3m/s = 0.6m
    p.escape_max_cycles =
      static_cast<int32_t>(declare_parameter<int>("escape_max_cycles", 200));
    // 기본 켬 — 후방 센서가 붙기 전에는 rear_clear 가 항상 false 라 기능이 잠긴다.
    p.escape_require_rear_clear =
      declare_parameter<bool>("escape_require_rear_clear", true) ? 1 : 0;
    p.traffic_state_enabled =
      declare_parameter<bool>("traffic_state_enabled", true) ? 1 : 0;
    // ── 정지 거리 추적 (2026-09-02 개정 — 정지선 소실 edge 기준,
    // mgm_types.hpp 주석 참조). 정지선을 한 번도 못 본 run(정지선 인식이
    // 실패하는 조건 포함)에서는 MGM_STATE_TRAFFIC의 !traffic_distance_latched
    // 분기가 v_base로 그냥 통과시킨다(사용자 지정 — 검증 단계에서 정지선
    // 인지가 미더운 채로 "못 봤으면 무조건 정지"는 잦은 오정지를 만든다).
    p.traffic_ramp_distance_m = static_cast<float>(
      declare_parameter<double>("traffic_ramp_distance_m", 1.5));
    // 가드 문턱 겸 완전 정지 문턱 [m] — mgm_types.hpp의 CoreParams::
    // traffic_stop_offset 주석 참조. "seed(1.5m)에서 0.5m 이상 진행해야만
    // 실제 정지가 성립"과 동일한 값(1.0m, 사용자 지정 2026-09-02). ⚠ seed와의
    // 차(0.5m)가 곧 ramp 구간이라 제동 여유가 빡빡하다 — 2m/s로 달리면 더
    // 그렇다(사용자 확인). seed(traffic_ramp_distance_m)를 같이 올릴지는 별도 검토.
    p.traffic_stop_offset = static_cast<float>(
      declare_parameter<double>("traffic_stop_offset_m", 1.0));
    if (!std::isfinite(p.traffic_ramp_distance_m) || !std::isfinite(p.traffic_stop_offset) ||
      p.traffic_stop_offset <= 0 || p.traffic_ramp_distance_m <= p.traffic_stop_offset)
    {
      throw std::invalid_argument("Traffic distances require finite seed > remaining target > 0");
    }
    rcl_interfaces::msg::ParameterDescriptor backend_descriptor;
    backend_descriptor.read_only = true;
    backend_descriptor.description = "startup-only decision backend: core or generated";
    const auto backend_name = declare_parameter<std::string>(
      "backend", "core", backend_descriptor);
    rcl_interfaces::msg::ParameterDescriptor acknowledgement_descriptor;
    acknowledgement_descriptor.read_only = true;
    acknowledgement_descriptor.description =
      "startup-only acknowledgement for the limited generated backend";
    const bool generated_scope_acknowledged = declare_parameter<bool>(
      "generated_backend_acknowledge_limited_scope", false, acknowledgement_descriptor);
    base_managers_ = declare_parameter<bool>(
      "base_state_machine_enabled", backend_name == "core");
    p.base_state_machine_enabled = base_managers_ ? 1 : 0;
    rcl_interfaces::msg::ParameterDescriptor zone_descriptor;
    zone_descriptor.read_only = true;
    p.avoidance_enabled = declare_parameter<bool>("avoidance_enabled", true, zone_descriptor) ? 1 : 0;
    if (base_managers_ && !p.avoidance_enabled) {
      RCLCPP_WARN(get_logger(), "Ordinary avoidance OFF: no AVOID/fallback or avoidance TTC stop; perception remains available");
    }
    p.route_sequence_enabled = declare_parameter<bool>("route_sequence_enabled", false, zone_descriptor) ? 1 : 0;
    if (p.route_sequence_enabled && (!base_managers_ || backend_name != "core")) {
      throw std::runtime_error("route sequence requires the parallel core backend");
    }
    zone_descriptor.description = "GNSS confirmation calibration; restart to change fixed zone context policy";
    p.zone_enter_confirm_samples = declare_parameter<int>("zone_enter_confirm_samples", 0, zone_descriptor);
    p.zone_exit_confirm_samples = declare_parameter<int>("zone_exit_confirm_samples", 0, zone_descriptor);
    p.parking_zone_entry_active = declare_parameter<bool>("parking_zone_entry_active", base_managers_, zone_descriptor) ? 1 : 0;
    if (p.parking_zone_entry_active && (!base_managers_ || backend_name != "core")) {
      throw std::runtime_error("immediate Parking entry requires the parallel core backend");
    }
    p.parking_search_zone_only = declare_parameter<bool>("parking_search_zone_only", base_managers_, zone_descriptor) ? 1 : 0;
    if (p.parking_search_zone_only && (!base_managers_ || backend_name != "core")) {
      throw std::runtime_error("zone-only Parking search requires the parallel core backend");
    }
    p.parking_search_timeout = declare_parameter<double>("parking_search_timeout", -1.0);
    p.max_parking_search_distance = declare_parameter<double>("max_parking_search_distance", -1.0);
    if (base_managers_ && !p.parking_zone_entry_active && !p.parking_search_zone_only && (!std::isfinite(p.parking_search_timeout) ||
      p.parking_search_timeout <= 0 || !std::isfinite(p.max_parking_search_distance) ||
      p.max_parking_search_distance <= 0))
    {
      RCLCPP_WARN(get_logger(), "Parking search limits uncalibrated: requests will be cancelled (CALIBRATION_REQUIRED)");
    }
    const auto zone_csv = declare_parameter<std::string>("zone_observations_csv_path", "");
    if (!zone_csv.empty()) {
      zone_events_.open(zone_csv, std::ios::trunc);
      if (!zone_events_) {throw std::runtime_error("cannot open zone_observations_csv_path");}
      zone_events_ << "generation,time_ns,zone_id,zone_valid,raw_in,stable_in,enter_count,exit_count,x,y,position_valid,track_index,previous_track_index,position_step_m,heading_rad,heading_valid,boundary_endpoint_distance_m,calibration_state\n";
      zone_events_.precision(17);
    }
    const auto mission_csv = declare_parameter<std::string>("mission_events_csv_path", "");
    if (!mission_csv.empty()) {
      mission_events_.open(mission_csv, std::ios::trunc);
      if (!mission_events_) {throw std::runtime_error("cannot open mission_events_csv_path");}
      mission_events_ << "event,request_id,mission_id,mission_type,source_zone_id,time_ns,position_valid,x,y,track_index,speed_valid,actual_speed,zone_id,travel_distance,elapsed_s,cancel_reason\n";
      mission_events_.precision(17);
    }
    backend_ = std::make_unique<DecisionBackend>(
      backend_name, generated_scope_acknowledged, p);
    RCLCPP_INFO(
      get_logger(), "decision backend=%s%s", backend_->name().c_str(),
      backend_->name() == "generated" ?
      " (ADAS_MGR2 v1.88 four-state; rear escape disabled)" : "");

    // 스테이트 전이 이유 CSV — 빈 값이면 콘솔 로그만 (§4 전이 조건 관찰)
    transition_csv_path_ = declare_parameter<std::string>("transition_csv_path", "");
    if (!transition_csv_path_.empty()) {
      transitions_.open(transition_csv_path_, std::ios::trunc);
      if (transitions_) {
        transitions_ << (base_managers_ ?
          "tick,top,navigation,avoidance,signal,safety,mission,mission_type,reference_source,speed_owner,v_ref,ref_valid,ref_fresh,ref_age_s,stop_reasons\n" :
          transitionCsvHeader());
        transitions_.flush();
      } else {
        RCLCPP_WARN(get_logger(), "전이 CSV를 열 수 없음: %s", transition_csv_path_.c_str());
      }
    }

    rcl_interfaces::msg::ParameterDescriptor lidar_estop_descriptor;
    lidar_estop_descriptor.read_only = true;
    lidar_estop_descriptor.description =
      "startup-only LiDAR E-stop input enable; false is an explicit test configuration";
    lidar_estop_enabled_ = declare_parameter<bool>(
      "lidar_estop_enabled", true, lidar_estop_descriptor);
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
    vehicle_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("vehicle_stale_timeout_sec", 0.2) * 1e9);
    // avoid 신선도 watchdog (§5.7 ⑤, 2026-08-12 회피 통합) — stale이면 진입 재료
    // 무효화, AVOID 스테이트 중이면 estop 보정 (낡은 회피 경로 주행 차단)
    avoid_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("avoid_stale_timeout_sec", 0.5) * 1e9);
    // Existing parking timeout also bounds actual reference generation age in
    // the parallel core. Receipt time still independently checks module health.
    parking_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("parking_stale_timeout_sec", 0.5) * 1e9);

    // ── CAN 헬스 watchdog (§5.7 ⑥, 2026-08-26) ─────────────────────────────
    // traffic(③)과 같은 규약: **수신 이력이 있은 뒤에만** 판정한다. 미수신을
    // 보정하면 브리지 없이 도는 단독 스택 시험·재생이 전부 estop 이 된다.
    can_stale_ns_ = static_cast<int64_t>(
      declare_parameter<double>("can_stale_timeout_sec", 0.5) * 1e9);
    // 몇 주기 연속 송신 실패를 고장으로 볼 것인가. 기본 3 = 30ms — §3 dSPACE
    // counter watchdog 의 3주기와 같은 눈금이다. 프레임 하나 흘린 것으로 서지 않는다.
    can_fail_ticks_ = static_cast<uint32_t>(
      declare_parameter<int>("can_fail_ticks", 3));
    // 이 시간 넘게 불건전이 지속되면 **래치**를 건다 — 링크가 살아나도 스스로
    // 재출발하지 않고 /operator/go 재인가를 기다린다.
    //   근거: PROTOCOL.md:78 은 "복구는 자동, 래치 없음"인데 그 규정이 전제한 두절은
    //   프레임 몇 개(수십 ms) 수준이다. 어댑터 이탈 같은 초 단위 고장에서 자동
    //   재출발하면, 그 사이 마지막 v_ref 로 굴러갔을 수 있는 차가 사람이 옆에 선
    //   채로 다시 움직인다. 그래서 **짧은 두절은 그의 규정대로 자동 복귀**시키고
    //   긴 고장만 래치한다 (INTEGRATION_TEST_0826.md §6 ①).
    //   0 이하면 래치 없음 = PROTOCOL.md 규정 그대로.
    can_relatch_ns_ = static_cast<int64_t>(
      declare_parameter<double>("can_relatch_sec", 1.0) * 1e9);

    // 출발 인가 게이트 (2026-08-11, 실차 launch 전용) — true면 /operator/go 수신
    // 전까지 estop 보정 유지(v_ref 0 대기). launch 직후 바로 출발해 출발 전
    // 점검이 불가능하던 문제 해결. 판단이 아니라 운용 입력 컨디셔닝 — §5.7의
    // estop 경로 재사용(코어에 새 정지 로직 없음). 인가는 tools/go 스크립트가
    // RTK FIXED 확인 후 발행한다.
    wait_go_ = declare_parameter<bool>("wait_go", false);
    if (!lidar_estop_enabled_) {
      if (!base_managers_ || backend_name != "core" || !wait_go_ || p.escape_after_cycles != 0) {
        throw std::runtime_error(
          "lidar_estop_enabled=false requires parallel core, wait_go=true and escape_after_cycles=0");
      }
      RCLCPP_WARN(get_logger(),
        "LiDAR E-stop input DISABLED: static/dynamic stop and input watchdog excluded. "
        "Operator/CAN/reference/traffic/avoidance TTC stops remain active.");
    }
    // 구독은 wait_go 와 무관하게 **항상** 만든다 — CAN 고장 래치(§5.7 ⑥)의 해제도
    // 같은 인가를 쓰기 때문이다. wait_go 는 "출발 전에도 인가가 필요한가"만 정한다.
    sub_go_ = create_subscription<std_msgs::msg::Bool>(
      "/operator/go", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (m->data && !go_received_) {
          RCLCPP_INFO(get_logger(), "출발 인가 수신 — 주행 시작");
        }
        if (m->data && can_latched_.load()) {
          can_latched_ = false;
          RCLCPP_WARN(get_logger(), "CAN 고장 래치 해제 — 재인가로 주행 재개");
        }
        go_received_ = m->data;});
    sub_session_ = create_subscription<std_msgs::msg::Bool>(
      "/operator/start_session", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data) {
          std::lock_guard<std::mutex> lk(mtx_);
          session_requested_ = true;
        }
      });
    sub_operator_stop_ = create_subscription<std_msgs::msg::Bool>(
      "/operator/stop", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_); operator_stop_ = m->data;
      });
    sub_mission_cancel_ = create_subscription<std_msgs::msg::Bool>(
      "/operator/cancel_mission", rclcpp::QoS(1),
      [this](std_msgs::msg::Bool::ConstSharedPtr m) {
        if (m->data) {std::lock_guard<std::mutex> lk(mtx_); mission_cancel_requested_ = true;}
      });
    mission_command_pub_ = create_publisher<fma_interfaces::msg::ParkingCommand>(
      "/parking/mission_command", 10);
    manager_pub_ = create_publisher<fma_interfaces::msg::MgmState>("/adas/mgm_state", 1);
    if (wait_go_) {
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
        std::lock_guard<std::mutex> lk(mtx_); msgs_.parking = *m;
        last_parking_rx_ns_ = monotonicNs();});
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
    // CAN 링크 건전성 (§5.7 ⑥, 2026-08-26) — bridge_dspace 가 발행. 정지 명령이
    // 실제로 버스에 나가고 있는지를 MGM 이 알 수 있는 유일한 경로다.
    sub_can_ = create_subscription<fma_interfaces::msg::CanHealth>(
      "/bridge/can_health", qos,
      [this](fma_interfaces::msg::CanHealth::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        msgs_.can = *m;
        last_can_rx_ns_ = monotonicNs();});
    // dSPACE VEH_FEEDBACK 에코 — MGM_STATE_TRAFFIC의 dead-reckoning 감쇠 입력.
    sub_vehicle_ = create_subscription<fma_interfaces::msg::VehicleVector>(
      "/vehicle/vector", rclcpp::SensorDataQoS(),
      [this](fma_interfaces::msg::VehicleVector::ConstSharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        msgs_.vehicle = *m;
        last_vehicle_rx_ns_ = monotonicNs();});

    msgs_.avoid.ttc = 1e9f;   // 인지 도착 전 TTC=0으로 오인해 정지하는 것 방지
    msgs_.estop.estop = true;  // 첫 EstopRequest 수신 전 fail-safe — 미수신 = 정지

    pending_parking_timeout_ = p.parking_search_timeout;
    pending_parking_distance_ = p.max_parking_search_distance;
    calibration_callback_ = add_on_set_parameters_callback(
      [this, zone_only=bool(p.parking_zone_entry_active || p.parking_search_zone_only)](const std::vector<rclcpp::Parameter> & values) {
        rcl_interfaces::msg::SetParametersResult result; result.successful = true;
        bool changed = false;
        std::lock_guard<std::mutex> lock(mtx_);
        double timeout = pending_parking_timeout_, distance = pending_parking_distance_;
        for (const auto & value : values) {
          if (value.get_name() != "parking_search_timeout" && value.get_name() != "max_parking_search_distance") {continue;}
          changed = true;
          if (value.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
            result.successful = false; result.reason = "Parking calibration requires double values"; return result;
          }
          if (value.get_name() == "parking_search_timeout") {timeout = value.as_double();}
          else {distance = value.as_double();}
        }
        if (changed && zone_only) {
          result.successful = false; result.reason = "Zone-only search does not use time/distance limits";
        } else if (changed && (!base_managers_ || dump_.is_open())) {
          result.successful = false;
          result.reason = "Parking tuning requires base core without snapshot dump; a recorded dump has fixed header parameters. Set YAML and restart.";
        } else if (changed) {
          pending_parking_timeout_ = timeout; pending_parking_distance_ = distance;
          calibration_pending_ = true;
        }
        return result;
      });
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
  static fma_interfaces::msg::MissionObservation toRosObservation(const MissionObservation & in)
  {
    fma_interfaces::msg::MissionObservation out;
    out.recorded = in.recorded;
    if (in.time_ns >= 0) {out.stamp = rclcpp::Time(in.time_ns);}
    out.position_valid = in.position_valid;
    out.x = in.x; out.y = in.y; out.track_index = in.track_index;
    out.speed_valid = in.speed_valid; out.actual_speed = in.actual_speed;
    out.zone_id = in.zone_id; out.travel_distance = in.travel_distance;
    return out;
  }
  void logMissionEvents(const CoreSnapshot & s, const CoreOutput & out)
  {
    if (out.mission_events == 0) {return;}
    const auto & r = out.mission_request;
    if (out.mission_cancel) {
      RCLCPP_WARN(get_logger(), "Mission request %lu cancelled: reason=%u elapsed=%.3f travel=%.3f",
        static_cast<unsigned long>(r.request_id), static_cast<unsigned>(r.cancel_reason),
        r.elapsed_s, r.travel_distance);
    }
    if (!mission_events_) {return;}
    const MissionObservation end{true, s.event_time_ns, s.gps_position_valid,
      s.gps_x, s.gps_y, s.gps_track_index, s.vehicle_speed_valid, s.vehicle_speed,
      out.zones.selected.zone_id, r.travel_distance};
    const MissionObservation * observations[] = {
      &r.zone_entry, &r.search_start, &r.space, &r.ready, &r.handoff, &end, &end};
    const char * names[] = {"zone_entry", "search_start", "space_found", "ready", "handoff", "cancel", "done"};
    for (int i = 0; i < 7; ++i) {
      if (!(out.mission_events & (1u << i))) {continue;}
      const auto & o = *observations[i];
      mission_events_ << names[i] << ',' << r.request_id << ',' << +r.mission_id << ','
        << static_cast<unsigned>(r.mission_type) << ',' << +r.source_zone_id << ',' << o.time_ns << ','
        << o.position_valid << ',' << o.x << ',' << o.y << ',' << o.track_index << ','
        << o.speed_valid << ',' << o.actual_speed << ',' << +o.zone_id << ',' << o.travel_distance << ','
        << r.elapsed_s << ',' << static_cast<unsigned>(r.cancel_reason) << '\n';
    }
    mission_events_.flush();
  }
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
    int64_t parking_rx_ns;
    int64_t can_rx_ns;
    int64_t vehicle_rx_ns;
    bool go;
    bool new_session;
    bool mission_cancel;
    bool operator_stop;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      m = msgs_;  // pull — 이후 인지가 갱신해도 이번 틱은 일관된 스냅샷 사용
      estop_rx_ns = last_estop_rx_ns_;
      lane_rx_ns = last_lane_rx_ns_;
      gps_rx_ns = last_gps_rx_ns_;
      traffic_rx_ns = last_traffic_rx_ns_;
      avoid_rx_ns = last_avoid_rx_ns_;
      parking_rx_ns = last_parking_rx_ns_;
      can_rx_ns = last_can_rx_ns_;
      vehicle_rx_ns = last_vehicle_rx_ns_;
      go = go_received_;
      operator_stop = operator_stop_;
      if (calibration_pending_) {
        backend_->setParkingCalibration(pending_parking_timeout_, pending_parking_distance_);
        calibration_pending_ = false;
      }
      new_session = session_requested_;
      session_requested_ = false;
      mission_cancel = mission_cancel_requested_;
      mission_cancel_requested_ = false;

    }
    // estop 입력 신선도 watchdog — 판단이 아니라 입력 컨디셔닝 (§3 dSPACE
    // counter watchdog의 PC측 대응물). stack_estop 미수신/사망 시 스냅샷의
    // estop을 true로 보정하고, 정지 판단(v_ref=0) 자체는 코어가 한다.
    // A disabled test input is removed BEFORE operator/CAN conditioning, so
    // no later blanket estop=false can erase another stop source. Even an
    // unexpected publisher cannot restore LiDAR/rear authority in this mode.
    if (!lidar_estop_enabled_) {m.estop = fma_interfaces::msg::EstopRequest{};}
    const bool estop_stale = lidar_estop_enabled_ &&
      (estop_rx_ns < 0 || monotonicNs() - estop_rx_ns > estop_stale_ns_);
    // 보정 전 "실제 수신값" 보존 — at_end 래치 해제는 이 값으로만 한다 (CLAUDE.md
    // §4 래치). stale이면 마지막 값을 조작 의사로 신뢰할 수 없으므로 false.
    const bool estop_real = !estop_stale && m.estop.estop;
    bool external_stop = operator_stop || (wait_go_ && !go);
    if (estop_stale) {
      m.estop.estop = true;
      // 후방 여유는 반대 방향으로 보정한다 — "모르면 false"(§4 후진 탈출).
      // stack_estop 이 죽었는데 마지막 "뒤가 비었다"를 믿고 후진하면 안 된다.
      m.estop.rear_clear = false;
    }
    // 출발 인가 게이트 — 인가 전까지 estop 보정으로 정지 대기 (운용 입력
    // 컨디셔닝, §5.7과 동류). estop_real(래치 해제용)에는 영향 없음.
    if (external_stop) {
      m.estop.estop = true;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
        "출발 대기 중 — 점검 완료 후 `ros2 run adas_mgm go` 로 출발");
    }
    // lane_path 입력 신선도 watchdog (2026-08-07 실차 테스트에서 발견) — state가
    // 현재 LANE이거나 LANE에서 진입한 TRAFFIC일 때 적용한다. stack_lane이
    // 죽었는데도 MGM이 마지막 값을 계속 재사용해 주행을 계속하던 문제를 막는다.
    // estop 경로를 그대로 재사용 — 새 판단 로직을 추가하는 게 아니라 기존
    // "estop=true → 전 스테이트 정지" 판단에 태우는 것 (§5.1 준수).
    const uint8_t active_state = backend_->activeState();
    const uint8_t traffic_entry_state = backend_->trafficEntryState();
    const bool traffic_uses_gps = active_state == MGM_STATE_TRAFFIC &&
      traffic_entry_state == MGM_STATE_WAYPOINT;
    const bool lane_stale = lane_rx_ns < 0 || monotonicNs() - lane_rx_ns > lane_stale_ns_;
    if (!base_managers_ && (active_state == MGM_STATE_LANE ||
      (active_state == MGM_STATE_TRAFFIC && !traffic_uses_gps)) &&
      lane_stale && !estop_stale)
    {
      m.estop.estop = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "lane_path 신선도 초과(state=lane/traffic-lane) — estop 강제 "
        "(stack_lane 확인 필요)");
    }
    // gps_path 신선도 watchdog — lane과 대칭. WAYPOINT 또는 그 상태에서 진입한
    // TRAFFIC이 gps_path 없이도
    // v_base로 계속 주행하던 문제 방지 (실제로 lane→waypoint 자동 전이 후 이
    // 경로로 재현됨, 2026-08-07).
    // fix 상실도 동일 취급 (2026-08-11 통합 점검에서 발견): stack_gps는 fix가
    // 없거나 stale이면 fix_quality=0인 빈 GpsPath를 계속 발행한다 — 수신 시각만
    // 보면 watchdog을 통과해, 조립 블록이 직전 ref를 hold한 채 v_base로 계속
    // 주행 명령을 내던 구멍. "신선하지만 무효인 입력"을 stale과 같은 경로로
    // 태운다 — 새 판단이 아니라 입력 컨디셔닝(§5.7 ②의 확장).
    const bool gps_stale = gps_rx_ns < 0 || monotonicNs() - gps_rx_ns > gps_stale_ns_;
    const bool gps_no_fix = m.gps.fix_quality == 0 || m.gps.points.empty();
    if (!base_managers_ && (active_state == MGM_STATE_WAYPOINT || traffic_uses_gps) &&
      (gps_stale || gps_no_fix) && !estop_stale)
    {
      m.estop.estop = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        gps_stale ? "gps_path 신선도 초과(state=waypoint/traffic-gps) — "
          "estop 강제 (stack_gps 확인 필요)"
                  : "gps fix 상실(state=waypoint/traffic-gps, "
          "fix_quality=0/빈 경로) — estop 강제 (RTK 확인 필요)");
    }
    // traffic_stop 신선도 watchdog (§5.7 ③) — lane/gps와 달리 **수신 이력이 있은
    // 뒤 끊긴 경우만** 보정한다(사망 감지). 미수신은 보정하지 않음: traffic은
    // 경로가 아니라 제약 입력이라, 단독 스택 시험(GPS 단독 등)이 stack_traffic
    // 없이도 성립해야 하기 때문. 적색 래치 중 사망(마지막 발행 true)은 msgs_에
    // true가 남아 어차피 정지 유지 — 위험 케이스는 false 상태로 죽은 뒤 적색이
    // 켜지는 경우이며 이 보정이 그걸 막는다. estop이 아닌 traffic 요구로 태워
    // 일반 감속 정지(rate limit)로 선다. (2026-08-08, PR #21 검토에서 도출)
    if (traffic_rx_ns >= 0 && monotonicNs() - traffic_rx_ns > traffic_stale_ns_) {
      m.traffic.stop_required = true;
      m.traffic.fail_safe_stop = true;
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
      if (!base_managers_ && backend_->activeState() == MGM_STATE_AVOID && !estop_stale) {
        m.estop.estop = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "avoid 신선도 초과(state=avoid) — estop 강제 (stack_avoid 확인 필요)");
      }
    }
    // parking 신선도 watchdog (§5.7 ⑦, 2026-08-31) — avoid(⑤)와 같은 구조다.
    // 미수신/staleness 시 **진입 재료**(space_found)를 무효화해 죽은 stack_parking 의
    // 마지막 메시지로 PARKING 에 진입하는 것을 막고, 이미 PARKING 이면 estop 을
    // 보정한다 — 낡은 주차 경로로 계속 움직이는 것을 막기 위해서다. 주차는 후진이
    // 섞이므로(parking_v_suggest 음수) 낡은 값으로 굴러가는 것이 특히 위험하다.
    //
    // done 도 함께 무효화한다: 죽기 직전 메시지의 done 으로 PARKING 을 빠져나가면
    // 차가 어디에 서 있는지 모르는 채 주행 스테이트로 올라간다. estop 이 걸린 채
    // PARKING 에 머무는 쪽이 안전하다.
    //
    // 미수신(-1)도 stale 로 본다. stack_parking 없이 도는 단독 스택 시험과 양립한다 —
    // space_found 기본값이 false 라 PARKING 에 못 들어가고, PARKING 이 아니면
    // estop 보정도 걸리지 않기 때문이다.
    const bool parking_stale =
      parking_rx_ns < 0 || monotonicNs() - parking_rx_ns > parking_stale_ns_;
    if (parking_stale) {
      m.parking.space_found = false;
      m.parking.done = false;
      if (!base_managers_ && backend_->activeState() == MGM_STATE_PARKING && !estop_stale) {
        m.estop.estop = true;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "parking 신선도 초과(state=parking) — estop 강제 (stack_parking 확인 필요)");
      }
    }
    // CAN 헬스 watchdog (§5.7 ⑥, 2026-08-26). 다른 watchdog 이 "인지가 살아 있는가"를
    // 보는 데 비해 이것은 **"우리 명령이 실제로 버스에 나가고 있는가"**를 본다.
    // dSPACE counter watchdog 이 미구현인 동안(HANDOVER.md §3.6) PC 가 못 보내면
    // dSPACE 는 마지막 v_ref 를 유지한다 — 그 구간에 차를 세울 방법은 PC 에 없다.
    // 그래서 이 보정의 목적은 "지금 세우는 것"이 아니라 **링크가 살아난 뒤 나가는
    // 첫 프레임이 정지값이 되게 하는 것**이다.
    // traffic(③)과 같은 규약으로 **수신 이력이 있은 뒤에만** 판정한다 — 브리지 없이
    // 도는 단독 스택 시험·재생을 깨지 않기 위해서다.
    if (can_rx_ns >= 0) {
      const bool can_stale = monotonicNs() - can_rx_ns > can_stale_ns_;
      const bool can_bad = can_stale || !m.can.link_up ||
        m.can.consecutive_tx_fail >= can_fail_ticks_;
      if (!can_bad) {
        can_bad_since_ns_ = -1;
      } else if (can_bad_since_ns_ < 0) {
        can_bad_since_ns_ = monotonicNs();
      }
      // 긴 고장이면 래치 — 링크가 살아나도 스스로 재출발하지 않는다 (§6 ① 절충).
      if (can_relatch_ns_ > 0 && can_bad_since_ns_ >= 0 && !can_latched_.load() &&
        monotonicNs() - can_bad_since_ns_ >= can_relatch_ns_)
      {
        can_latched_ = true;
        RCLCPP_ERROR(
          get_logger(),
          "CAN 고장 %.1fs 지속 — 래치. 확인 후 `ros2 run adas_mgm go` 로 재인가할 것",
          static_cast<double>(can_relatch_ns_) * 1e-9);
      }
      external_stop = external_stop || can_bad || can_latched_.load();
      if ((can_bad || can_latched_.load()) && !estop_stale) {
        m.estop.estop = true;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "CAN %s — estop 강제 (link_up=%d tx_fail=%u errno=%d%s)",
          can_stale ? "헬스 신선도 초과(브리지 확인 필요)" : "송신 불가",
          static_cast<int>(m.can.link_up), m.can.consecutive_tx_fail, m.can.last_errno,
          can_latched_.load() ? " · 래치" : "");
      }
    }
    CoreSnapshot s = toSnapshot(m, base_managers_);
    if (base_managers_) {
      const auto stamp_ns = [](const auto & stamp) {
        return static_cast<int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
      };
      const int64_t stamps[] = {stamp_ns(m.lane.reference_stamp), stamp_ns(m.gps.reference_stamp),
        stamp_ns(m.avoid.reference_stamp), stamp_ns(m.parking.reference_stamp)};
      const int64_t timeouts[] = {lane_stale_ns_, gps_stale_ns_, avoid_stale_ns_, parking_stale_ns_};
      const int64_t ros_now = now().nanoseconds(), steady_now = monotonicNs();
      for (int source = 0; source < MGM_SRC_ESCAPE; ++source) {
        s.references[source] = reference_clocks_[source].observe(
          stamps[source], ros_now, steady_now, timeouts[source]);
      }
    }
    s.autonomous_enabled = !wait_go_ || go;
    s.new_session = new_session;
    s.monotonic_ns = monotonicNs();
    s.event_time_ns = now().nanoseconds();
    s.mission_cancel_requested = mission_cancel;
    s.parking_request_id = m.parking.request_id;
    s.parking_search_active = m.parking.search_active;
    s.parking_search_space_found = m.parking.search_space_found;
    s.parking_preparation_ready = m.parking.preparation_ready;
    const auto & ready_stamp = m.parking.preparation_stamp;
    s.parking_preparation_reference = preparation_clock_.observe(
      static_cast<int64_t>(ready_stamp.sec) * 1'000'000'000 + ready_stamp.nanosec,
      s.event_time_ns, s.monotonic_ns, parking_stale_ns_);
    const auto & gps_sample = s.references[MGM_SRC_GPS];
    s.zones.generation = gps_sample.generation;
    const auto & rear_stamp = m.estop.rear_reference_stamp;
    const auto rear_sample = rear_clock_.observe(
      static_cast<int64_t>(rear_stamp.sec) * 1'000'000'000 + rear_stamp.nanosec,
      s.event_time_ns, s.monotonic_ns, estop_stale_ns_);
    s.rear_sensor_valid = !estop_stale && m.estop.rear_sensor_valid && rear_sample.generation != 0 &&
      std::isfinite(rear_sample.age_s) && rear_sample.age_s >= 0 &&
      rear_sample.timeout_s > 0 && rear_sample.age_s <= rear_sample.timeout_s;
    s.rear_corridor_state = s.rear_sensor_valid && m.estop.rear_corridor_state <= 2 ?
      static_cast<RearCorridorState>(m.estop.rear_corridor_state) : RearCorridorState::UNKNOWN;
    s.gps_position_valid = !gps_stale && m.gps.fix_quality != 0 && m.gps.position_valid &&
      gps_sample.generation != 0 && std::isfinite(gps_sample.age_s) &&
      gps_sample.age_s >= 0 && gps_sample.age_s <= gps_sample.timeout_s &&
      std::isfinite(m.gps.position_x) && std::isfinite(m.gps.position_y);
    s.gps_x = m.gps.position_x;
    s.gps_y = m.gps.position_y;
    s.gps_track_index = s.gps_position_valid ? m.gps.track_index : -1;
    s.external_stop = external_stop;
    // Sensor/message availability stays separate from drivable path validity.
    s.camera_line_valid = !lane_stale;
    s.gps_valid = !gps_stale && m.gps.fix_quality != 0;
    s.lidar_valid = !avoid_stale && m.avoid.scan_valid;
    s.auto_estop = estop_real && m.estop.scan_valid;
    s.parking_valid = !parking_stale;
    s.parking_mission_active = m.parking.mission_active;
    s.parking_mission_mode = m.parking.mission_mode;
    s.parking_updated = parking_rx_ns != last_parking_rx_used_;
    last_parking_rx_used_ = parking_rx_ns;
    if (!s.camera_line_valid) {s.lane_path.n = 0;}
    if (!s.gps_valid) {s.gps_path.n = 0;}
    if (!s.lidar_valid) {s.avoid_path.n = 0;}
    if (!s.parking_valid) {s.parking_path.n = 0;}
    const bool vehicle_stale = vehicle_rx_ns < 0 ||
      monotonicNs() - vehicle_rx_ns > vehicle_stale_ns_;
    s.vehicle_speed_valid = !vehicle_stale && std::isfinite(s.vehicle_speed);
    if (!base_managers_ && backend_->activeState() == MGM_STATE_TRAFFIC && !s.vehicle_speed_valid) {
      s.traffic_fail_safe_stop = true;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "vehicle/vector 신선도 초과(state=traffic) — 정지 강제");
    }
    s.estop_latch_release = estop_real;  // toSnapshot은 LatestMsgs만 알므로 여기서 주입
    // 소스별 "이번 틱에 새 메시지 도착" — §5.8 이동 보정의 판정 근거.
    // 값 동일성으로 판정하면 인지가 상수를 낼 때 무한 감쇠한다 (mgm_types.hpp 주석).
    s.lane_updated = lane_rx_ns != last_lane_rx_used_;
    s.gps_updated = gps_rx_ns != last_gps_rx_used_;
    s.avoid_updated = avoid_rx_ns != last_avoid_rx_used_;
    last_lane_rx_used_ = lane_rx_ns;
    last_gps_rx_used_ = gps_rx_ns;
    last_avoid_rx_used_ = avoid_rx_ns;

    if (dump_.is_open()) {
      dump_.write(reinterpret_cast<const char *>(&s), sizeof(s));
    }

    // 전이 이유 로깅용 스냅샷 — 카운터는 **step 직전** 값이어야 한다.
    // 전이가 일어나면 코어가 카운터를 리셋하므로(2026-08-14 규약) 이후 값은 0이다.
    const uint8_t state_before = backend_->activeState();
    const int32_t low_before = backend_->laneLowCnt();
    const int32_t high_before = backend_->laneHighCnt();
    const int32_t avoid_ticks_before = backend_->avoidTicks();
    const int32_t return_hold_before = backend_->returnHoldLeft();

    const bool was_faulted = backend_->faulted();
    const CoreOutput out = backend_->step(s);  // 판단+실행은 선택된 backend 한 곳에서만
    if (!was_faulted && backend_->faulted()) {
      RCLCPP_ERROR(
        get_logger(), "decision backend fault latched; publishing fail-stop: %s",
        backend_->faultReason().c_str());
    }

    // 스테이트 전이 이유 — 바뀐 그 틱의 결정 변수를 그대로 남긴다.
    // 판단이 아니라 관찰이다: 전이는 이미 backend가 했고 여기서 되먹임하지 않는다.
    // MBD 시험에서 "레퍼런스와 같은 조건으로 바뀌었나"가 이 줄로 판별된다.
    if (!base_managers_ && out.state != state_before) {
      const TransitionRecord tr = explainTransition(
        state_before, out.state, s, backend_->params(),
        low_before, high_before, avoid_ticks_before, return_hold_before,
        out.v_ref, tick_);
      RCLCPP_INFO(get_logger(), "전이 %s → %s @%.2fs | %s%s | %s",
        stateName(tr.from), stateName(tr.to), tr.tick * 0.01,
        tr.rule.c_str(), tr.spec_match ? "" : "  ★ 스펙 불일치",
        tr.detail.c_str());
      if (transitions_) {
        transitions_ << tr.csv;
        transitions_.flush();       // 현장에서 중단돼도 남아야 한다
      }
    }
    ++tick_;

    // 지정 지점 정차 로그 — 현장에서 "왜 섰나"가 즉시 보이게 (판단 아님, 코어
    // 상태 관찰). 정차 중에는 남은 시간을 1초마다 흘린다.
    if (backend_->stopZoneHolding() != stop_holding_prev_) {
      stop_holding_prev_ = backend_->stopZoneHolding();
      if (stop_holding_prev_) {
        RCLCPP_INFO(get_logger(), "지정 정지 지점 %u 진입 — 정지 후 %.1fs 정차",
          static_cast<unsigned>(s.gps_stop_zone),
          backend_->params().stop_zone_hold_cycles * 0.01);
      } else {
        RCLCPP_INFO(get_logger(), "정차 완료 — 재출발");
      }
    }
    if (backend_->stopZoneHolding()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "정차 중 — 남은 %.1fs (v_ref %.2f)",
        backend_->stopHoldLeft() * 0.01, static_cast<double>(out.v_ref));
    }
    if (out.state == MGM_STATE_TRAFFIC) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "TRAFFIC — red=%d green=%d line_latched=%d remaining=%.2fm v_actual=%.2f v_ref=%.2f",
        static_cast<int>(s.traffic_red_active), static_cast<int>(s.traffic_green_active),
        static_cast<int>(backend_->trafficDistanceLatched()),
        static_cast<double>(backend_->trafficStoplineDistance()),
        static_cast<double>(s.vehicle_speed), static_cast<double>(out.v_ref));
    }

    if (base_managers_) {
      fma_interfaces::msg::MgmState status;
      status.header.stamp = now();
      status.route.enabled = out.route.enabled;
      status.route.connecting = out.route.connecting;
      status.route.next_connecting = out.route.next_connecting;
      status.route.requested_connecting = out.route.requested_connecting;
      status.route.phase = static_cast<uint8_t>(out.route.phase);
      status.route.completion = static_cast<uint8_t>(out.route.completion);
      status.route.sequence_id = out.route.sequence_id; status.route.instance_id = out.route.instance_id;
      status.route.request_id = out.route.request_id;
      status.route.index = out.route.index; status.route.count = out.route.count;
      status.route.requested_index = out.route.requested_index;
      status.route.seen_nonterminal = out.route.seen_nonterminal;
      status.route.end_reached = out.route.end_reached;
      status.top = static_cast<uint8_t>(out.top);
      status.navigation = static_cast<uint8_t>(out.nav);
      status.avoidance = static_cast<uint8_t>(out.avoid);
      status.signal = static_cast<uint8_t>(out.signal);
      status.safety = static_cast<uint8_t>(out.safety);
      status.mission = static_cast<uint8_t>(out.mission);
      status.mission_type = static_cast<uint8_t>(out.mission_type);
      status.reference_source = out.path_source;
      status.speed_owner = static_cast<uint8_t>(out.speed_owner);
      status.reference_available = out.reference_available;
      status.selected_reference_valid = out.selected_reference.valid;
      status.selected_reference_fresh = out.selected_reference.fresh;
      status.selected_reference_age_s = out.selected_reference.age_s;
      status.selected_reference_generation = out.selected_reference.generation;
      status.line_ref_valid = out.references[MGM_SRC_LANE].valid;
      status.gps_ref_valid = out.references[MGM_SRC_GPS].valid;
      status.avoid_ref_valid = out.references[MGM_SRC_AVOID].valid;
      status.mission_ref_valid = out.references[MGM_SRC_PARKING].valid;
      status.recovery_ref_valid = out.references[MGM_SRC_ESCAPE].valid;
      status.avoid_episode_reference_seen = out.avoid_episode_reference_seen;
      status.active_safe_stop_reasons = out.safe_stop_reasons;
      status.immediate_stop = out.immediate_stop;
      status.zone = toRosZone(out.zones.selected);
      status.in_gps_only_zone = out.zones.in_gps_only_zone;
      status.active_mission_id = out.active_mission_id;
      for (const auto & zone : out.zones.contexts) {
        if (zone.zone_id != 0 || zone.in_zone || zone.zone_exited) {status.zones.push_back(toRosZone(zone));}
      }
      const auto & request = out.mission_request;
      status.mission_request_active = request.active;
      status.mission_request_id = request.request_id;
      status.mission_source_zone_id = request.source_zone_id;
      status.request_mission_id = request.mission_id;
      status.request_mission_type = static_cast<uint8_t>(request.mission_type);
      status.request_start_monotonic_ns = request.start_time_ns;
      status.search_elapsed_s = request.elapsed_s;
      status.search_travel_distance = request.travel_distance;
      status.parking_search_acknowledged = request.search_acknowledged;
      status.parking_space_found = request.space_found;
      status.parking_ready = request.preparation_ready;
      status.mission_completed = out.active_mission_completed;
      status.mission_failed = out.active_mission_failed;
      status.parking_search_zone_only = backend_->params().parking_search_zone_only != 0;
      status.parking_zone_entry_active = backend_->params().parking_zone_entry_active != 0;
      status.mission_cancel_reason = static_cast<uint8_t>(request.cancel_reason);
      status.mission_events = out.mission_events;
      status.zone_entry_observation = toRosObservation(request.zone_entry);
      status.search_start_observation = toRosObservation(request.search_start);
      status.space_found_observation = toRosObservation(request.space);
      status.ready_observation = toRosObservation(request.ready);
      status.handoff_observation = toRosObservation(request.handoff);
      status.parking_calibration_state = static_cast<uint8_t>(out.parking_calibration);
      status.zone_calibration_state = static_cast<uint8_t>(out.zones.calibration);
      status.zone_generation = out.zones.last_generation;
      status.zone_enter_confirm_samples = backend_->params().zone_enter_confirm_samples;
      status.zone_exit_confirm_samples = backend_->params().zone_exit_confirm_samples;
      status.gps_position_valid = s.gps_position_valid;
      status.gps_x = s.gps_x; status.gps_y = s.gps_y; status.gps_track_index = s.gps_track_index;
      status.gps_previous_track_index = m.gps.previous_track_index;
      status.gps_position_step_m = m.gps.position_step_m;
      status.gps_vehicle_heading_rad = m.gps.vehicle_heading_rad;
      status.gps_vehicle_heading_valid = s.gps_position_valid && m.gps.vehicle_heading_valid;
      const auto & recovery = out.recovery;
      status.recovery_configured = recovery.configured;
      status.rear_sensor_valid = recovery.rear_sensor_valid;
      status.rear_corridor_state = static_cast<uint8_t>(recovery.rear_corridor_state);
      status.recovery_eligible = recovery.eligible;
      status.recovery_block_reason = static_cast<uint8_t>(recovery.block_reason);
      status.recovery_attempt_count = recovery.attempt_count;
      status.cumulative_reverse_command_time = recovery.command_time_s;
      status.cumulative_reverse_measured_distance = recovery.measured_distance_m;
      status.reverse_measured_distance_complete = recovery.measured_distance_complete;
      status.last_recovery_reason = static_cast<uint8_t>(recovery.last_reason);
      status.traffic_distance_known = out.traffic_distance_known;
      status.traffic_remaining_m = out.traffic_remaining_m;
      status.traffic_stop_in_success_region = out.traffic_stop_in_success_region;
      if (zone_events_ && s.zones.generation != last_zone_log_generation_) {
        last_zone_log_generation_ = s.zones.generation;
        for (const auto & zone : status.zones) {
          zone_events_ << s.zones.generation << ',' << s.event_time_ns << ',' << +zone.zone_id << ','
            << zone.zone_valid << ',' << zone.raw_in_zone << ',' << zone.stable_in_zone << ','
            << zone.enter_count << ',' << zone.exit_count << ',' << s.gps_x << ',' << s.gps_y << ','
            << s.gps_position_valid << ',' << s.gps_track_index << ',' << m.gps.previous_track_index << ','
            << m.gps.position_step_m << ',' << m.gps.vehicle_heading_rad << ','
            << status.gps_vehicle_heading_valid << ',' << zone.boundary_distance_m << ','
            << +status.zone_calibration_state << '\n';
        }
        zone_events_.flush();
      }
      manager_pub_->publish(status);
      logMissionEvents(s, out);
      const bool changed = !has_manager_status_ || status.top != previous_manager_status_.top ||
        status.zone.zone_id != previous_manager_status_.zone.zone_id ||
        status.zone.zone_valid != previous_manager_status_.zone.zone_valid ||
        status.navigation != previous_manager_status_.navigation ||
        status.avoidance != previous_manager_status_.avoidance ||
        status.signal != previous_manager_status_.signal ||
        status.safety != previous_manager_status_.safety ||
        status.mission != previous_manager_status_.mission ||
        status.mission_type != previous_manager_status_.mission_type ||
        status.reference_source != previous_manager_status_.reference_source ||
        status.speed_owner != previous_manager_status_.speed_owner ||
        status.active_safe_stop_reasons != previous_manager_status_.active_safe_stop_reasons ||
        status.selected_reference_valid != previous_manager_status_.selected_reference_valid;
      if (changed) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
          "MGM top=%u nav=%u avoid=%u signal=%u safety=%u mission=%u type=%u ref=%u speed=%u valid=%d age=%.3f reasons=0x%x",
          status.top, status.navigation, status.avoidance, status.signal, status.safety,
          status.mission, status.mission_type, status.reference_source, status.speed_owner,
          status.selected_reference_valid, status.selected_reference_age_s, status.active_safe_stop_reasons);
        if (transitions_) {
          transitions_ << (tick_ - 1) << ',' << +status.top << ',' << +status.navigation << ','
                       << +status.avoidance << ',' << +status.signal << ',' << +status.safety << ','
                       << +status.mission << ',' << +status.mission_type << ','
                       << +status.reference_source << ',' << +status.speed_owner << ',' << out.v_ref << ','
                       << status.selected_reference_valid << ',' << status.selected_reference_fresh << ','
                       << status.selected_reference_age_s << ',' << status.active_safe_stop_reasons << '\n';
          transitions_.flush();
        }
        previous_manager_status_ = status;
        has_manager_status_ = true;
      }
      using ParkingCommand = fma_interfaces::msg::ParkingCommand;
      if (out.mission_cancel) {
        pending_mission_command_.request_id = request.request_id;
        pending_mission_command_.mission_mode = static_cast<uint8_t>(request.mission_type);
        pending_mission_command_.action = ParkingCommand::CANCEL;
      } else if (request.active) {
        pending_mission_command_.request_id = request.request_id;
        pending_mission_command_.mission_mode = static_cast<uint8_t>(request.mission_type);
        pending_mission_command_.action = !request.preparation_ready ?
          ParkingCommand::PREPARE : ParkingCommand::ACTIVATE;
      } else if (pending_mission_command_.action != ParkingCommand::CANCEL) {
        pending_mission_command_.action = 0;
      }
      const bool command_matching = s.parking_valid &&
        s.parking_request_id == pending_mission_command_.request_id &&
        s.parking_mission_mode == pending_mission_command_.mission_mode;
      const bool acknowledged = command_matching && (
        (pending_mission_command_.action == ParkingCommand::PREPARE && s.parking_search_active) ||
        (pending_mission_command_.action == ParkingCommand::ACTIVATE && s.parking_mission_active) ||
        (pending_mission_command_.action == ParkingCommand::CANCEL &&
         !s.parking_search_active && !s.parking_mission_active));
      if (!acknowledged && pending_mission_command_.action != 0) {
        mission_command_pub_->publish(pending_mission_command_);
      }
      if (acknowledged && pending_mission_command_.action == ParkingCommand::CANCEL) {
        pending_mission_command_.action = 0;
      }
      if (!out.selected_reference.valid) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "MGM source %u reference invalid/stale: SAFE_STOP",
          static_cast<unsigned>(out.path_source));
      }
    }
    TargetRef msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.state = out.state;
    msg.v_ref = out.v_ref;
    // Pose delta follows the selected localization source. Parking reference
    // points are LiDAR-SLAM-relative, so pairing them with GNSS deltas would
    // mix frames even though both fields share the TargetRef wire layout.
    if (out.state == MGM_STATE_PARKING) {
      msg.dx = m.parking.dx;
      msg.dy = m.parking.dy;
      msg.dyaw = m.parking.dyaw;
      msg.update = m.parking.update;
    } else {
      msg.dx = m.gps.dx;
      msg.dy = m.gps.dy;
      msg.dyaw = m.gps.dyaw;
      msg.update = m.gps.update;
    }
    msg.ref_points.resize(out.n_points);
    for (int32_t i = 0; i < out.n_points; ++i) {
      msg.ref_points[i].x = out.ref_points[i].x;
      msg.ref_points[i].y = out.ref_points[i].y;
      msg.ref_points[i].yaw = out.ref_points[i].yaw;
      msg.ref_points[i].curvature = out.ref_points[i].curvature;
    }
    // Last command boundary guard, also independent of the speed ramp.
    if (base_managers_ && (!out.selected_reference.valid || !std::isfinite(msg.v_ref) ||
      !reference_geometry_valid(out.ref_points, out.n_points))) {msg.v_ref = 0.0f;}
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

  std::unique_ptr<DecisionBackend> backend_;
  std::ofstream transitions_;          // 전이 이유 CSV (판단 아님 — 관찰 기록)
  std::string transition_csv_path_;
  int64_t tick_{0};
  std::unique_ptr<JitterLogger> jitter_;
  int cpu_core_{-1};
  std::ofstream dump_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr calibration_callback_;
  double pending_parking_timeout_{-1.0}, pending_parking_distance_{-1.0};
  bool calibration_pending_{false};
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
  int64_t last_parking_rx_ns_{-1};  // 마지막 ParkingStatus 수신 시각 (미수신 = -1)
  // 직전 틱에 사용한 수신 시각 — 비교해서 "이번 틱에 새 메시지" 판정 (§5.8)
  int64_t last_lane_rx_used_{-1};
  int64_t last_gps_rx_used_{-1};
  int64_t last_avoid_rx_used_{-1};
  int64_t avoid_stale_ns_{500'000'000};
  int64_t parking_stale_ns_{500'000'000};
  int64_t last_can_rx_ns_{-1};    // 마지막 CanHealth 수신 시각 (미수신 = -1)
  int64_t can_stale_ns_{500'000'000};
  uint32_t can_fail_ticks_{3};    // 연속 송신 실패 몇 주기부터 고장으로 볼 것인가
  int64_t can_relatch_ns_{1'000'000'000};  // 이만큼 지속되면 래치 (0 이하 = 래치 없음)
  int64_t last_vehicle_rx_ns_{-1};
  int64_t vehicle_stale_ns_{200'000'000};
  int64_t can_bad_since_ns_{-1};  // CAN 불건전 시작 시각 (건전 = -1, 루프 스레드 전용)
  std::atomic<bool> can_latched_{false};   // CAN 고장 래치 — /operator/go 재인가로만 해제
  bool stop_holding_prev_{false};   // 지정 지점 정차 로그용 (코어 상태의 직전 값)
  bool wait_go_{false};             // 출발 인가 게이트 활성 (실차 launch 전용)
  bool lidar_estop_enabled_{true};   // startup-only, separate test launcher
  bool base_managers_{false};
  ReferenceClock reference_clocks_[MGM_SRC_ESCAPE];
  ReferenceClock preparation_clock_;
  bool session_requested_{false};
  bool operator_stop_{false};
  bool mission_cancel_requested_{false};
  fma_interfaces::msg::ParkingCommand pending_mission_command_;
  std::ofstream mission_events_;
  std::ofstream zone_events_;
  uint64_t last_zone_log_generation_{0};
  ReferenceClock rear_clock_;
  fma_interfaces::msg::MgmState previous_manager_status_;
  bool has_manager_status_{false};
  int64_t last_parking_rx_used_{-1};
  bool go_received_{false};         // /operator/go 마지막 수신값

  rclcpp::Publisher<TargetRef>::SharedPtr pub_;
  rclcpp::Publisher<fma_interfaces::msg::ParkingCommand>::SharedPtr mission_command_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mission_cancel_;
  rclcpp::Publisher<fma_interfaces::msg::MgmState>::SharedPtr manager_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_session_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_operator_stop_;
  rclcpp::Subscription<fma_interfaces::msg::LanePath>::SharedPtr sub_lane_;
  rclcpp::Subscription<fma_interfaces::msg::GpsPath>::SharedPtr sub_gps_;
  rclcpp::Subscription<fma_interfaces::msg::AvoidStatus>::SharedPtr sub_avoid_;
  rclcpp::Subscription<fma_interfaces::msg::ParkingStatus>::SharedPtr sub_parking_;
  rclcpp::Subscription<fma_interfaces::msg::TrafficStop>::SharedPtr sub_traffic_;
  rclcpp::Subscription<fma_interfaces::msg::EstopRequest>::SharedPtr sub_estop_;
  rclcpp::Subscription<fma_interfaces::msg::CanHealth>::SharedPtr sub_can_;
  rclcpp::Subscription<fma_interfaces::msg::VehicleVector>::SharedPtr sub_vehicle_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_go_;

  std::atomic<bool> running_{true};
  std::thread loop_thread_;
};

}  // namespace adas_mgm

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<adas_mgm::MgmNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("mgm_node"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
