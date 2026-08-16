#include "multi_lidar_fusion/lidar_converter.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace multi_lidar_fusion
{

namespace
{

/// PointCloud2 필드 조회. 없으면 nullptr.
const sensor_msgs::msg::PointField * findField(
  const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & f : cloud.fields) {
    if (f.name == name) {
      return &f;
    }
  }
  return nullptr;
}

bool isFloat32(const sensor_msgs::msg::PointField * f)
{
  return f != nullptr && f->datatype == sensor_msgs::msg::PointField::FLOAT32;
}

}  // namespace

LidarConverter::LidarConverter(const SensorConfig & config)
: config_(config)
{
}

bool LidarConverter::convert(
  const sensor_msgs::msg::LaserScan & scan, CloudFrame & out, ConvertStats & stats) const
{
  stats = ConvertStats{};
  out.points.clear();
  out.stamp = rclcpp::Time(scan.header.stamp, RCL_ROS_TIME);
  out.frame_id = config_.frame_id_override.empty() ?
    scan.header.frame_id : config_.frame_id_override;
  out.sensor_id = config_.index;
  out.duration = 0.0;

  const std::size_t n = scan.ranges.size();
  stats.input_points = n;
  if (n == 0) {
    return false;
  }
  if (!std::isfinite(scan.angle_increment) || scan.angle_increment == 0.0F) {
    return false;   // 드라이버가 헤더를 채우지 못한 프레임 — 각도를 복원할 수 없다.
  }

  // 센서별 신뢰거리와 드라이버가 보고한 유효범위의 교집합을 쓴다.
  // (드라이버 값이 실측보다 낙관적인 경우가 흔해 파라미터가 더 좁으면 그쪽을 따른다.)
  const float min_r = std::max(
    static_cast<float>(config_.min_range),
    std::isfinite(scan.range_min) ? scan.range_min : 0.0F);
  const float max_r = std::min(
    static_cast<float>(config_.max_range),
    std::isfinite(scan.range_max) ? scan.range_max : static_cast<float>(config_.max_range));

  const bool has_intensity = scan.intensities.size() == n;
  const float t_inc = std::isfinite(scan.time_increment) ? scan.time_increment : 0.0F;
  const bool mask = config_.hasAngleMask();

  // 마지막 ray 까지 걸린 시간. 신선도 판정의 기준점이 stamp + duration 이 된다.
  // time_increment 를 안 주는 드라이버는 scan_time 으로 축퇴한다.
  const double span = static_cast<double>(t_inc) * static_cast<double>(n > 0 ? n - 1 : 0);
  out.duration = span > 0.0 ? span :
    (std::isfinite(scan.scan_time) && scan.scan_time > 0.0F ?
    static_cast<double>(scan.scan_time) : 0.0);

  out.points.reserve(std::max(capacity_hint_, n));

  const float roff = static_cast<float>(config_.range_offset_m);

  for (std::size_t i = 0; i < n; ++i) {
    const float raw = scan.ranges[i];
    if (!std::isfinite(raw)) {         // NaN / ±Inf = 미반사 → 제거 (요구 §20 Case 4)
      ++stats.dropped_invalid;
      continue;
    }
    // 센서별 거리 보정. 신뢰구간(min/max)은 **보정 후** 값으로 판정한다.
    const float r = raw - roff;
    if (r < min_r || r > max_r) {
      ++stats.dropped_range;
      continue;
    }
    const float theta = scan.angle_min + scan.angle_increment * static_cast<float>(i);
    // "이 라이다가 보는 범위·방향" — 센서 좌표계에서 자른다. YAML 로만 바뀌는 값.
    if (mask && !config_.angleAccepted(theta)) {
      ++stats.dropped_fov;
      continue;
    }
    FusionPoint p;
    p.x = r * std::cos(theta);
    p.y = r * std::sin(theta);
    p.z = 0.0F;                        // 2D LiDAR — 스캔 평면 높이는 TF(z)가 준다.
    p.intensity = has_intensity ? scan.intensities[i] : 0.0F;
    p.dt = t_inc * static_cast<float>(i);
    p.sensor_id = config_.index;
    out.points.push_back(p);
  }
  stats.output_points = out.points.size();
  return true;
}

bool LidarConverter::convert(
  const sensor_msgs::msg::PointCloud2 & cloud, CloudFrame & out, ConvertStats & stats) const
{
  stats = ConvertStats{};
  out.points.clear();
  out.stamp = rclcpp::Time(cloud.header.stamp, RCL_ROS_TIME);
  out.frame_id = config_.frame_id_override.empty() ?
    cloud.header.frame_id : config_.frame_id_override;
  out.sensor_id = config_.index;
  out.duration = 0.0;

  const auto * fx = findField(cloud, "x");
  const auto * fy = findField(cloud, "y");
  const auto * fz = findField(cloud, "z");
  if (!isFloat32(fx) || !isFloat32(fy) || !isFloat32(fz)) {
    return false;   // x/y/z(float32) 없는 cloud 는 이 파이프라인에서 다루지 않는다.
  }
  const std::size_t n = static_cast<std::size_t>(cloud.width) *
    static_cast<std::size_t>(cloud.height);
  stats.input_points = n;
  if (n == 0) {
    return true;    // 빈 cloud 는 오류가 아니다 (요구 §20 Case 5).
  }

  // 있으면 쓰고 없으면 무시 — 센서 A/B 의 필드 구성이 달라도 되는 이유.
  const bool has_intensity = isFloat32(findField(cloud, "intensity"));
  const bool has_time = !config_.time_field.empty() &&
    isFloat32(findField(cloud, config_.time_field));

  sensor_msgs::PointCloud2ConstIterator<float> it_x(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> it_y(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> it_z(cloud, "z");

  const double min_r2 = config_.min_range * config_.min_range;
  const double max_r2 = config_.max_range * config_.max_range;
  const bool mask = config_.hasAngleMask();

  // PointCloud2 의 duration 은 점별 dt 의 최대값으로, 아래 루프에서 채운다.
  out.points.reserve(std::max(capacity_hint_, n));

  // intensity / time 은 존재할 때만 별도 iterator 를 돌린다.
  auto it_i = has_intensity ?
    sensor_msgs::PointCloud2ConstIterator<float>(cloud, "intensity") : it_x;
  auto it_t = has_time ?
    sensor_msgs::PointCloud2ConstIterator<float>(cloud, config_.time_field) : it_x;

  for (std::size_t i = 0; i < n; ++i, ++it_x, ++it_y, ++it_z) {
    const float x = *it_x;
    const float y = *it_y;
    const float z = *it_z;
    float intensity = 0.0F;
    float dt = 0.0F;
    if (has_intensity) {
      intensity = *it_i;
      ++it_i;
    }
    if (has_time) {
      dt = *it_t;
      ++it_t;
    }
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
      ++stats.dropped_invalid;
      continue;
    }
    // 2D 라이다 기준 — 수평거리로 판정한다 (z 는 장착 높이라 거리 판정에 넣지 않는다).
    double rr = std::sqrt(static_cast<double>(x) * x + static_cast<double>(y) * y);
    float xc = x, yc = y;
    if (config_.range_offset_m != 0.0 && rr > 1e-6) {
      // 보정은 **반경 방향**이다. x/y 를 같은 비율로 줄여 방향을 보존한다.
      const double scale = (rr - config_.range_offset_m) / rr;
      xc = static_cast<float>(x * scale);
      yc = static_cast<float>(y * scale);
      rr -= config_.range_offset_m;
    }
    const double r2 = rr * rr;
    if (rr < 0.0 || r2 < min_r2 || r2 > max_r2) {
      ++stats.dropped_range;
      continue;
    }
    if (mask && !config_.angleAccepted(std::atan2(static_cast<double>(y), static_cast<double>(x)))) {
      ++stats.dropped_fov;
      continue;
    }
    FusionPoint p;
    p.x = xc;
    p.y = yc;
    p.z = z;
    p.intensity = intensity;
    p.dt = std::isfinite(dt) ? dt : 0.0F;
    p.sensor_id = config_.index;
    out.duration = std::max(out.duration, static_cast<double>(p.dt));
    out.points.push_back(p);
  }
  stats.output_points = out.points.size();
  return true;
}

}  // namespace multi_lidar_fusion
