#include "multi_lidar_fusion/virtual_laserscan.hpp"

#include <algorithm>
#include <cmath>

namespace multi_lidar_fusion
{

VirtualLaserScan::VirtualLaserScan(const ScanParams & params)
: params_(params)
{
  recomputeBins();
}

void VirtualLaserScan::setParams(const ScanParams & p)
{
  params_ = p;
  recomputeBins();
}

void VirtualLaserScan::recomputeBins()
{
  const double span = params_.angle_max - params_.angle_min;
  if (params_.angle_increment <= 0.0 || span <= 0.0) {
    bin_count_ = 0;
    full_circle_ = false;
    observed_.clear();
    return;
  }
  bin_count_ = static_cast<std::size_t>(std::ceil(span / params_.angle_increment));
  // 360도 스캔이면 마지막 bin 다음은 첫 bin 이다. atan2 가 +pi 를 돌려줄 때
  // index 가 bin_count_ 로 넘치는데, 그 한 줄기를 버리지 않고 0번으로 감는다.
  full_circle_ = span >= (2.0 * M_PI - params_.angle_increment * 0.5);
  observed_.assign(bin_count_, 0U);
}

ScanStats VirtualLaserScan::build(const CloudFrame & cloud, sensor_msgs::msg::LaserScan & out) const
{
  ScanStats stats;
  stats.bins = bin_count_;
  stats.points_in = cloud.points.size();

  out.header.stamp = cloud.stamp;
  out.header.frame_id = cloud.frame_id;
  out.angle_min = static_cast<float>(params_.angle_min);
  out.angle_max = static_cast<float>(params_.angle_max);
  out.angle_increment = static_cast<float>(params_.angle_increment);
  out.time_increment = static_cast<float>(params_.time_increment);
  out.scan_time = static_cast<float>(params_.scan_time);
  out.range_min = static_cast<float>(params_.range_min);
  out.range_max = static_cast<float>(params_.range_max);

  if (bin_count_ == 0) {
    out.ranges.clear();
    out.intensities.clear();
    return stats;
  }

  const float empty_value = params_.use_inf_for_no_return ?
    kNoReturn : static_cast<float>(params_.no_return_value);

  out.ranges.assign(bin_count_, empty_value);
  if (params_.publish_intensities) {
    out.intensities.assign(bin_count_, 0.0F);
  } else {
    out.intensities.clear();
  }
  if (observed_.size() != bin_count_) {
    observed_.assign(bin_count_, 0U);
  } else {
    std::fill(observed_.begin(), observed_.end(), static_cast<std::uint8_t>(0));
  }

  const double inv_inc = 1.0 / params_.angle_increment;

  for (const auto & p : cloud.points) {
    if (p.z < params_.z_min || p.z > params_.z_max) {
      ++stats.dropped_z;
      continue;
    }
    const double x = p.x;
    const double y = p.y;
    const double r = std::sqrt(x * x + y * y);
    if (r < params_.range_min || r > params_.range_max) {
      ++stats.dropped_range;
      continue;
    }
    const double theta = std::atan2(y, x);
    double idx_f = (theta - params_.angle_min) * inv_inc;
    std::ptrdiff_t idx = static_cast<std::ptrdiff_t>(std::floor(idx_f));
    if (full_circle_) {
      // -pi..pi 스캔에서 theta 가 경계에 걸린 경우를 감아서 살린다.
      const std::ptrdiff_t n = static_cast<std::ptrdiff_t>(bin_count_);
      idx = ((idx % n) + n) % n;
    } else if (idx < 0 || idx >= static_cast<std::ptrdiff_t>(bin_count_)) {
      ++stats.dropped_angle;
      continue;
    }

    const std::size_t i = static_cast<std::size_t>(idx);
    const float rf = static_cast<float>(r);
    // 요구 §16 — 같은 각도에 여러 센서가 있으면 가장 가까운 것이 이긴다.
    if (observed_[i] == 0U || rf < out.ranges[i]) {
      out.ranges[i] = rf;
      if (params_.publish_intensities) {
        out.intensities[i] = p.intensity;
      }
    }
    if (observed_[i] == 0U) {
      observed_[i] = 1U;
      ++stats.observed_bins;
    }
    ++stats.points_used;
  }

  stats.coverage = bin_count_ > 0 ?
    static_cast<double>(stats.observed_bins) / static_cast<double>(bin_count_) : 0.0;
  return stats;
}

}  // namespace multi_lidar_fusion
