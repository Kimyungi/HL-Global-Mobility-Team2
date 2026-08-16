// multi_lidar_fusion — 단계 8: 진단 (Diagnostics)
//
// 이 파일의 역할:
//   "지금 몇 대가 살아 있고, 얼마나 어긋나 있으며, 몇 점이 버려졌는가"를 한곳에 모은다.
//   콜백마다 INFO 를 찍어 터미널을 도배하지 않고(요구 §19), report_period_s 마다
//   한 줄 요약 + /diagnostics(DiagnosticArray) 를 낸다.
//
//   센서가 다 죽으면 ERROR 로 올린다(요구 §20 Case6) — 회피 로직이 "장애물 없음"과
//   "아무것도 못 봄"을 구분할 수 있어야 하기 때문이다.
//
// 입력  : 각 단계의 통계 구조체 (ConvertStats/SyncResult/FilterStats/ScanStats)
// 출력  : diagnostic_msgs::msg::DiagnosticArray, 사람이 읽는 요약 문자열
// 파라미터: diagnostics.report_period_s, diagnostics.min_active_sensors
// 관계  : 노드가 콜백/주기마다 record*() 로 먹이고, 주기적으로 publish 한다.

#ifndef MULTI_LIDAR_FUSION__DIAGNOSTICS_HPP_
#define MULTI_LIDAR_FUSION__DIAGNOSTICS_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "multi_lidar_fusion/cloud_filter.hpp"
#include "multi_lidar_fusion/cloud_synchronizer.hpp"
#include "multi_lidar_fusion/lidar_converter.hpp"
#include "multi_lidar_fusion/types.hpp"
#include "multi_lidar_fusion/virtual_laserscan.hpp"

namespace multi_lidar_fusion
{

/// 한 융합 주기의 결과 요약 (노드가 채워서 넘긴다).
struct CycleReport
{
  rclcpp::Time t_ref{0, 0, RCL_ROS_TIME};
  std::size_t contributing{0};
  std::size_t merged_points{0};
  std::size_t published_points{0};
  double max_dt_spread_s{0.0};   ///< 기여 센서들 stamp 의 최대-최소 차
  bool published{false};
  FilterStats filter;
  ScanStats scan;
};

class Diagnostics
{
public:
  struct Config
  {
    double report_period_s{2.0};
    /// 활성 센서가 이 수 미만이면 WARN, 0이면 ERROR.
    std::size_t min_active_sensors{2};
    std::string hardware_id{"multi_lidar_fusion"};
  };

  explicit Diagnostics(const Config & config);

  void registerSensor(std::uint8_t index, const std::string & id);

  /// 센서 콜백에서 1회 호출 (수신 사실과 정규화 결과 기록).
  void recordMessage(
    std::uint8_t index, const rclcpp::Time & stamp, const ConvertStats & conv, bool convert_ok);

  /// 동기화 결과 기록 (센서별 배제 사유 카운트).
  void recordSync(const SyncResult & sync);

  /// TF 실패 기록.
  void recordTfFailure(std::uint8_t index);

  /// 한 주기 마무리 기록.
  void recordCycle(const CycleReport & report);

  /// report_period_s 가 지났는가. true 를 돌려준 시점에 내부 창이 리셋된다.
  bool due(const rclcpp::Time & now);

  /// 사람이 읽는 한 줄 요약 (마지막 due() 창 기준).
  std::string summary() const;

  /// /diagnostics 로 낼 메시지.
  diagnostic_msgs::msg::DiagnosticArray toMessage(const rclcpp::Time & now) const;

  /// 전체 심각도 (diagnostic_msgs::msg::DiagnosticStatus::OK/WARN/ERROR).
  std::uint8_t level() const {return level_;}

private:
  struct SensorCounters
  {
    std::uint8_t index{0};
    std::string id;
    // 창(window) 카운터 — due() 마다 리셋
    std::uint64_t msgs{0};
    std::uint64_t points_in{0};
    std::uint64_t points_out{0};
    std::uint64_t dropped_invalid{0};
    std::uint64_t dropped_range{0};
    std::uint64_t dropped_fov{0};
    std::uint64_t convert_failed{0};
    std::uint64_t used{0};
    std::uint64_t reused{0};
    std::uint64_t too_old{0};
    std::uint64_t out_of_sync{0};
    std::uint64_t sync_warn{0};
    std::uint64_t tf_failed{0};
    // 누적 (리셋 안 함)
    std::uint64_t total_msgs{0};
    std::uint64_t total_tf_failed{0};
    // 최신 상태
    rclcpp::Time last_stamp{0, 0, RCL_ROS_TIME};
    double last_dt_to_ref_s{0.0};
    double fps{0.0};
    FrameStatus last_status{FrameStatus::kNeverReceived};
    bool ever_received{false};
  };

  SensorCounters * find(std::uint8_t index);

  Config config_;
  std::vector<SensorCounters> sensors_;

  rclcpp::Time window_start_{0, 0, RCL_ROS_TIME};
  bool window_started_{false};
  double window_len_s_{0.0};

  // 창 카운터
  std::uint64_t cycles_{0};
  std::uint64_t published_{0};
  std::uint64_t empty_cycles_{0};       ///< 기여 센서 0 (요구 §20 Case6)
  std::uint64_t sync_warn_cycles_{0};
  std::uint64_t merged_points_{0};
  std::uint64_t filtered_points_{0};
  std::uint64_t dropped_points_{0};
  double max_spread_s_{0.0};
  double last_coverage_{0.0};
  std::size_t last_active_{0};
  std::size_t min_active_seen_{kMaxSensors + 1};

  // 누적 카운터
  std::uint64_t total_cycles_{0};
  std::uint64_t total_empty_cycles_{0};
  std::uint64_t total_sync_failures_{0};
  std::uint64_t total_tf_failures_{0};

  std::uint8_t level_{0};
  std::string summary_;

  void resetWindow(const rclcpp::Time & now);
  void computeLevelAndSummary();
};

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__DIAGNOSTICS_HPP_
