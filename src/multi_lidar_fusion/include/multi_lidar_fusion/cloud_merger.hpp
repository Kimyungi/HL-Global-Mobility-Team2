// multi_lidar_fusion — 단계 5: 병합 + PointCloud2 직렬화
//
// 이 파일의 역할:
//   (1) base_link 로 맞춰진 프레임 여러 개를 점 배열 하나로 잇는다.
//       P_merged = P_A1 ∪ P_A2 ∪ P_B1 ∪ P_B2
//       센서 A/B 의 원본 필드 구성이 달라도 여기서는 이미 FusionPoint 로 통일돼 있어
//       필드 협상이 필요 없다(요구 §10). 최소 보장 필드는 x/y/z.
//   (2) 파이프라인에서 **유일하게** ROS 메시지로 되돌리는 지점.
//       PointCloud2 → PCL → PointCloud2 같은 왕복이 없다(요구 §28).
//
// 입력  : const CloudFrame* 목록 (모두 frame_id = base_link)
// 출력  : CloudFrame 누적본 / sensor_msgs::msg::PointCloud2
// frame : base_link
// 파라미터: publish_intensity, publish_sensor_id (debug)
// 관계  : CloudSynchronizer 가 고르고 CloudTransformer·MotionCompensator 가 손본
//         프레임들을 받아, CloudFilter 로 넘길 누적본을 만든다.

#ifndef MULTI_LIDAR_FUSION__CLOUD_MERGER_HPP_
#define MULTI_LIDAR_FUSION__CLOUD_MERGER_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "multi_lidar_fusion/types.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace multi_lidar_fusion
{

struct MergeStats
{
  std::size_t sources{0};        ///< 실제로 기여한 센서 수
  std::size_t input_points{0};
  std::size_t output_points{0};
};

class CloudMerger
{
public:
  /// frames 를 out 에 이어 붙인다. out 은 노드가 들고 있는 재사용 버퍼여야 한다
  /// (clear() 는 capacity 를 유지하므로 정상 상태에서 재할당이 없다 — 요구 §28).
  /// nullptr 원소는 그냥 건너뛴다(그 주기에 빠진 센서).
  MergeStats merge(
    const std::vector<const CloudFrame *> & frames,
    const rclcpp::Time & stamp,
    const std::string & frame_id,
    CloudFrame & out) const;
};

/// CloudFrame → PointCloud2. 파이프라인의 마지막 한 번만 호출된다.
///  - x/y/z (FLOAT32) 는 항상
///  - intensity (FLOAT32) 는 with_intensity 일 때
///  - sensor_id (UINT8)  는 with_sensor_id 일 때 (요구 §18 — debug 용)
/// msg 는 재사용 가능한 버퍼여야 한다 (data 벡터 capacity 유지).
void toPointCloud2(
  const CloudFrame & in,
  bool with_intensity,
  bool with_sensor_id,
  sensor_msgs::msg::PointCloud2 & msg);

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__CLOUD_MERGER_HPP_
