#include "multi_lidar_fusion/cloud_merger.hpp"

#include <cstring>
#include <string>
#include <vector>

namespace multi_lidar_fusion
{

MergeStats CloudMerger::merge(
  const std::vector<const CloudFrame *> & frames,
  const rclcpp::Time & stamp,
  const std::string & frame_id,
  CloudFrame & out) const
{
  MergeStats stats;

  std::size_t total = 0;
  for (const auto * f : frames) {
    if (f != nullptr) {
      total += f->points.size();
    }
  }
  stats.input_points = total;

  out.points.clear();          // capacity 유지 — 매 주기 재할당 방지
  out.points.reserve(total);
  out.stamp = stamp;
  out.frame_id = frame_id;
  out.sensor_id = 0;
  out.seq = 0;

  for (const auto * f : frames) {
    if (f == nullptr || f->points.empty()) {
      continue;
    }
    ++stats.sources;
    out.points.insert(out.points.end(), f->points.begin(), f->points.end());
  }
  stats.output_points = out.points.size();
  return stats;
}

void toPointCloud2(
  const CloudFrame & in,
  bool with_intensity,
  bool with_sensor_id,
  sensor_msgs::msg::PointCloud2 & msg)
{
  using sensor_msgs::msg::PointField;

  msg.header.stamp = in.stamp;
  msg.header.frame_id = in.frame_id;

  // 필드 레이아웃은 옵션 조합에 따라서만 바뀌므로, 바뀔 때만 다시 만든다.
  const std::size_t want_fields = 3U + (with_intensity ? 1U : 0U) + (with_sensor_id ? 1U : 0U);
  const bool layout_ok = msg.fields.size() == want_fields &&
    (!with_intensity || msg.fields[3].name == "intensity") &&
    (!with_sensor_id || msg.fields.back().name == "sensor_id");
  if (!layout_ok) {
    msg.fields.clear();
    msg.fields.reserve(want_fields);
    std::uint32_t offset = 0;
    auto add = [&msg, &offset](const char * name, std::uint8_t type, std::uint32_t size) {
        PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = type;
        f.count = 1;
        msg.fields.push_back(f);
        offset += size;
      };
    add("x", PointField::FLOAT32, 4);
    add("y", PointField::FLOAT32, 4);
    add("z", PointField::FLOAT32, 4);
    if (with_intensity) {
      add("intensity", PointField::FLOAT32, 4);
    }
    if (with_sensor_id) {
      add("sensor_id", PointField::UINT8, 1);
    }
    // 4바이트 정렬로 패딩 — RViz·PCL 쪽 리더가 편하다.
    msg.point_step = (offset + 3U) & ~3U;
  }

  const std::uint32_t step = msg.point_step;
  const std::size_t n = in.points.size();

  msg.height = 1;
  msg.width = static_cast<std::uint32_t>(n);
  msg.is_bigendian = false;
  msg.row_step = step * msg.width;
  msg.is_dense = true;          // 정규화 단계에서 NaN/Inf 를 이미 제거했다.
  msg.data.resize(static_cast<std::size_t>(step) * n);

  // 오프셋은 위 레이아웃 그대로 (x,y,z[,intensity][,sensor_id]).
  const std::uint32_t off_i = 12;
  const std::uint32_t off_s = with_intensity ? 16 : 12;

  std::uint8_t * base = msg.data.data();
  for (std::size_t i = 0; i < n; ++i) {
    std::uint8_t * p = base + static_cast<std::size_t>(step) * i;
    const FusionPoint & src = in.points[i];
    std::memcpy(p + 0, &src.x, 4);
    std::memcpy(p + 4, &src.y, 4);
    std::memcpy(p + 8, &src.z, 4);
    if (with_intensity) {
      std::memcpy(p + off_i, &src.intensity, 4);
    }
    if (with_sensor_id) {
      p[off_s] = src.sensor_id;
    }
  }
}

}  // namespace multi_lidar_fusion
