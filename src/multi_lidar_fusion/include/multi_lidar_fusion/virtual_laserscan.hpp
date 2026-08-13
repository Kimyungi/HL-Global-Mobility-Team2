// multi_lidar_fusion — 단계 7: 가상 LaserScan 생성
//
// 이 파일의 역할:
//   병합·필터된 base_link 점군을 라이다 1대가 낸 것처럼 보이는 LaserScan 으로 굽는다.
//   이 토픽 하나가 stack_avoid 가 보게 될 전부다 — 회피 로직은 라이다가 4대라는
//   사실을 알 필요가 없다(요구 §30).
//
//     r     = sqrt(x^2 + y^2)
//     theta = atan2(y, x)
//     index = (theta - angle_min) / angle_increment
//     ranges[index] = min(ranges[index], r)      ← 요구 §14, §16
//
//   같은 방향에 여러 센서의 점이 있으면 **가장 가까운 것**만 남는다. 회피에서 위험한
//   것은 가장 가까운 물체이기 때문이다.
//
//   관측되지 않은 bin 은 +inf (LaserScan 관례, 요구 §17). "멀리 있다"와 "아무도
//   보지 않았다"가 같은 값이 되는 위험은 coverage mask 로 분리한다 — 여기서는
//   bin 별 관측 여부(observed_)를 계산해 두고, 노드가 필요하면 별도 토픽으로 낸다.
//
// 입력  : CloudFrame (frame_id = base_link, 필터 완료본)
// 출력  : sensor_msgs::msg::LaserScan (frame_id = base_link)
// 파라미터: scan.angle_min/angle_max/angle_increment/range_min/range_max/scan_time
// 관계  : 파이프라인의 마지막. CloudFilter 출력 → 이 클래스 → /lidar/merged_scan

#ifndef MULTI_LIDAR_FUSION__VIRTUAL_LASERSCAN_HPP_
#define MULTI_LIDAR_FUSION__VIRTUAL_LASERSCAN_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "multi_lidar_fusion/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace multi_lidar_fusion
{

struct ScanParams
{
  double angle_min{-M_PI};
  double angle_max{M_PI};
  double angle_increment{0.00872665};   ///< 0.5 deg
  double range_min{0.1};
  double range_max{20.0};
  double scan_time{0.05};               ///< 융합 주기 (1/fusion_rate_hz)
  double time_increment{0.0};           ///< 합성 스캔이므로 0

  /// 스캔에 넣을 점의 높이 창 [m, base_link 기준]. 2D 라이다 4대가 서로 다른
  /// 높이에 달리므로 기본은 넉넉하게 열어둔다. 좁히려면 여기서.
  double z_min{-10.0};
  double z_max{10.0};

  /// true = 미관측 bin 을 +inf 로. false 면 no_return_value 를 쓴다
  /// (일부 소비자가 inf 를 못 다루는 경우의 탈출구).
  bool use_inf_for_no_return{true};
  double no_return_value{0.0};

  /// intensity 배열도 채울지 (해당 bin 대표점의 intensity).
  bool publish_intensities{false};
};

struct ScanStats
{
  std::size_t bins{0};
  std::size_t points_in{0};
  std::size_t points_used{0};       ///< bin 에 반영된 점
  std::size_t dropped_angle{0};     ///< angle 범위 밖
  std::size_t dropped_range{0};     ///< range_min/max 밖
  std::size_t dropped_z{0};         ///< 높이 창 밖
  std::size_t observed_bins{0};     ///< 값이 들어간 bin 수
  double coverage{0.0};             ///< observed_bins / bins
};

class VirtualLaserScan
{
public:
  explicit VirtualLaserScan(const ScanParams & params);

  /// cloud 로부터 LaserScan 을 만든다. out 은 재사용 버퍼여야 한다.
  ScanStats build(const CloudFrame & cloud, sensor_msgs::msg::LaserScan & out) const;

  /// 직전 build() 에서 bin 별로 관측이 있었는지 (coverage mask 확장 지점).
  const std::vector<std::uint8_t> & observed() const {return observed_;}

  std::size_t binCount() const {return bin_count_;}

  const ScanParams & params() const {return params_;}
  void setParams(const ScanParams & p);

private:
  void recomputeBins();

  ScanParams params_;
  std::size_t bin_count_{0};
  bool full_circle_{false};
  mutable std::vector<std::uint8_t> observed_;
};

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__VIRTUAL_LASERSCAN_HPP_
