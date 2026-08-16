#include "multi_lidar_fusion/cloud_filter.hpp"

#include <algorithm>
#include <cmath>

namespace multi_lidar_fusion
{

namespace
{

/// 21비트 부호 있는 격자 좌표 3개를 int64 하나로 정확히 포장한다.
/// 해시 충돌이 아니라 완전한 키 — leaf 0.03m 기준 ±31km 까지 표현된다.
constexpr std::int64_t kVoxelBias = 1 << 20;      // 1,048,576
constexpr std::int64_t kVoxelLimit = (1 << 21) - 1;

inline std::int64_t voxelIndex(double v, double inv_leaf)
{
  const double f = std::floor(v * inv_leaf);
  const std::int64_t i = static_cast<std::int64_t>(f) + kVoxelBias;
  return std::min<std::int64_t>(std::max<std::int64_t>(i, 0), kVoxelLimit);
}

inline std::int64_t voxelKey(std::int64_t i, std::int64_t j, std::int64_t k)
{
  return (i << 42) | (j << 21) | k;
}

}  // namespace

CloudFilter::CloudFilter(const FilterParams & params)
: params_(params)
{
}

FilterStats CloudFilter::apply(CloudFrame & frame) const
{
  FilterStats stats;
  stats.input = frame.points.size();
  if (frame.points.empty()) {
    return stats;
  }

  const double min_r2 = params_.min_range * params_.min_range;
  const double max_r2 = params_.max_range * params_.max_range;

  const double half_l = params_.vehicle_length * 0.5 + params_.self_margin;
  const double half_w = params_.vehicle_width * 0.5 + params_.self_margin;
  const double sx = params_.vehicle_center_x;
  const double sy = params_.vehicle_center_y;

  // ── ①②③ 거리 / ROI / self : 한 번의 순회로 in-place 압축 ──────────────
  std::size_t w = 0;
  for (std::size_t r = 0; r < frame.points.size(); ++r) {
    const FusionPoint & p = frame.points[r];
    const double x = p.x;
    const double y = p.y;
    const double z = p.z;

    if (params_.range_enabled) {
      const double d2 = x * x + y * y;
      if (d2 < min_r2 || d2 > max_r2) {
        ++stats.dropped_range;
        continue;
      }
    }
    if (params_.roi_enabled) {
      if (x < params_.min_x || x > params_.max_x ||
        y < params_.min_y || y > params_.max_y ||
        z < params_.min_z || z > params_.max_z)
      {
        ++stats.dropped_roi;
        continue;
      }
    }
    if (params_.self_filter_enabled) {
      if (std::fabs(x - sx) <= half_l && std::fabs(y - sy) <= half_w) {
        ++stats.dropped_self;
        continue;
      }
    }
    if (w != r) {
      frame.points[w] = p;
    }
    ++w;
  }
  frame.points.resize(w);

  // ── ④ voxel : 칸마다 원점에 가장 가까운 점 하나만 남긴다 ───────────────
  if (params_.voxel_enabled && params_.voxel_leaf_size > 0.0 && !frame.points.empty()) {
    const double inv_leaf = 1.0 / params_.voxel_leaf_size;
    voxel_map_.clear();
    voxel_map_.reserve(frame.points.size());

    std::size_t vw = 0;
    for (std::size_t r = 0; r < frame.points.size(); ++r) {
      const FusionPoint p = frame.points[r];
      const std::int64_t key = voxelKey(
        voxelIndex(p.x, inv_leaf),
        voxelIndex(p.y, inv_leaf),
        params_.voxel_2d ? kVoxelBias : voxelIndex(p.z, inv_leaf));

      auto it = voxel_map_.find(key);
      if (it == voxel_map_.end()) {
        voxel_map_.emplace(key, static_cast<std::uint32_t>(vw));
        frame.points[vw] = p;
        ++vw;
        continue;
      }
      ++stats.dropped_voxel;
      FusionPoint & kept = frame.points[it->second];
      const double d_new = static_cast<double>(p.x) * p.x + static_cast<double>(p.y) * p.y;
      const double d_old = static_cast<double>(kept.x) * kept.x +
        static_cast<double>(kept.y) * kept.y;
      if (d_new < d_old) {
        kept = p;     // 더 가까운 점이 그 칸의 대표 (회피에서는 가까운 쪽이 안전)
      }
    }
    frame.points.resize(vw);
  }

  stats.output = frame.points.size();
  return stats;
}

}  // namespace multi_lidar_fusion
