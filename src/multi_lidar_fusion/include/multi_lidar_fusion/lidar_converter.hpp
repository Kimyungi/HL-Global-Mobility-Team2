// multi_lidar_fusion — 단계 1: 메시지 정규화 (Sensor acquisition → Normalization)
//
// 이 파일의 역할:
//   드라이버가 무엇을 뱉든(LaserScan / PointCloud2, 필드 구성이 달라도) 내부 표현
//   CloudFrame 하나로 통일한다. 여기서 센서 개별 특성(최소·최대 신뢰거리, NaN/Inf,
//   점별 시각)을 흡수해, 이후 단계가 "어느 모델의 라이다인지" 전혀 모르게 만든다.
//   → 요구 §33(모델 미정), §34-J(모델 교체 시 코어 무수정)의 실현 지점.
//
// 입력  : sensor_msgs/LaserScan 또는 sensor_msgs/PointCloud2 (센서 원본 토픽)
// 출력  : CloudFrame (센서 frame 유지 — 좌표변환은 다음 단계 CloudTransformer 담당)
// frame : 입력 메시지의 header.frame_id (SensorConfig::frame_id_override 로 덮어쓰기 가능)
// 파라미터: SensorConfig{min_range, max_range, time_field, input_type, ...}
// 관계  : 노드의 구독 콜백이 호출 → 결과를 CloudSynchronizer::push() 로 넘긴다.

#ifndef MULTI_LIDAR_FUSION__LIDAR_CONVERTER_HPP_
#define MULTI_LIDAR_FUSION__LIDAR_CONVERTER_HPP_

#include <cstddef>

#include "multi_lidar_fusion/types.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace multi_lidar_fusion
{

/// 정규화 1회의 결과 요약 (진단용).
struct ConvertStats
{
  std::size_t input_points{0};    ///< 메시지에 들어 있던 점/빔 수
  std::size_t dropped_invalid{0}; ///< NaN/Inf 로 버린 수
  std::size_t dropped_range{0};   ///< min/max range 밖이라 버린 수
  std::size_t dropped_fov{0};     ///< 센서 FOV/blind sector 밖이라 버린 수
  std::size_t output_points{0};   ///< CloudFrame 에 남은 수
};

class LidarConverter
{
public:
  explicit LidarConverter(const SensorConfig & config);

  /// LaserScan → CloudFrame.
  ///   x = r cos(theta), y = r sin(theta), z = 0
  ///   theta = angle_min + i * angle_increment
  ///   dt    = i * time_increment  (0 이면 프레임 단위 보상으로 축퇴)
  /// 반환값 false = 사용할 수 없는 메시지(빔 0개 등). out 은 항상 초기화된다.
  bool convert(const sensor_msgs::msg::LaserScan & scan, CloudFrame & out, ConvertStats & stats)
  const;

  /// PointCloud2 → CloudFrame.
  /// x/y/z 는 필수, intensity 는 있으면 쓰고 없으면 0, time_field 가 지정되고
  /// 실제로 존재하면 점별 dt 로 쓴다. 필드 순서/개수가 센서마다 달라도 무관하다.
  bool convert(const sensor_msgs::msg::PointCloud2 & cloud, CloudFrame & out, ConvertStats & stats)
  const;

  const SensorConfig & config() const {return config_;}

  /// 다음 convert 호출에서 reserve 할 점 수 힌트 (직전 프레임 크기).
  void setCapacityHint(std::size_t n) {capacity_hint_ = n;}
  std::size_t capacityHint() const {return capacity_hint_;}

private:
  SensorConfig config_;
  std::size_t capacity_hint_{1024};
};

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__LIDAR_CONVERTER_HPP_
