// multi_lidar_fusion — 단계 2: 좌표변환 (Coordinate transformation)
//
// 이 파일의 역할:
//   센서 좌표계의 CloudFrame 을 차량 기준 base_link 로 옮긴다.
//       P_base = T_base_lidar * P_lidar
//   변환행렬을 파라미터에서 직접 조립하지 않고 **tf2 tree 에서 조회**한다(요구 §6).
//   extrinsic 이 바뀌면 YAML→static_transform_publisher 만 바뀌고 이 코드는 그대로다.
//
// 입력  : CloudFrame (frame_id = lidar_xx_link)
// 출력  : 같은 CloudFrame in-place (frame_id = base_link)
// frame : target_frame_ (기본 base_link)
// 파라미터: target_frame, tf_timeout_s, allow_static_fallback
// 관계  : 노드가 동기화 단계에서 고른 프레임마다 호출. 실패한 센서는 그 주기에서만
//         빠지고 나머지 센서로 융합은 계속된다 (요구 §20 Case 2 / §29 금지4).

#ifndef MULTI_LIDAR_FUSION__CLOUD_TRANSFORMER_HPP_
#define MULTI_LIDAR_FUSION__CLOUD_TRANSFORMER_HPP_

#include <memory>
#include <string>

#include "multi_lidar_fusion/types.hpp"
#include "tf2_ros/buffer.h"

namespace multi_lidar_fusion
{

class CloudTransformer
{
public:
  CloudTransformer(
    std::shared_ptr<tf2_ros::Buffer> buffer,
    std::string target_frame,
    double tf_timeout_s,
    bool allow_static_fallback);

  /// frame 을 target_frame 으로 in-place 변환.
  /// 성공 시 true, 실패 시 false 이며 error 에 사유가 담긴다(프레임 내용은 불변).
  bool transformInPlace(CloudFrame & frame, std::string & error) const;

  const std::string & targetFrame() const {return target_frame_;}

private:
  std::shared_ptr<tf2_ros::Buffer> buffer_;
  std::string target_frame_;
  double tf_timeout_s_;
  /// stamp 시각 조회가 실패하면 TimePointZero(=최신/static)로 재시도할지.
  /// static extrinsic 만 쓰는 우리 구성에서는 이게 사실상 정상 경로다.
  bool allow_static_fallback_;
};

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__CLOUD_TRANSFORMER_HPP_
