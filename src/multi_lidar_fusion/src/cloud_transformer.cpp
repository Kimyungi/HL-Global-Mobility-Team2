#include "multi_lidar_fusion/cloud_transformer.hpp"

#include <string>
#include <utility>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"

namespace multi_lidar_fusion
{

CloudTransformer::CloudTransformer(
  std::shared_ptr<tf2_ros::Buffer> buffer,
  std::string target_frame,
  double tf_timeout_s,
  bool allow_static_fallback)
: buffer_(std::move(buffer)),
  target_frame_(std::move(target_frame)),
  tf_timeout_s_(tf_timeout_s),
  allow_static_fallback_(allow_static_fallback)
{
}

bool CloudTransformer::transformInPlace(CloudFrame & frame, std::string & error) const
{
  error.clear();
  if (frame.frame_id.empty()) {
    error = "empty frame_id";
    return false;
  }
  if (frame.frame_id == target_frame_) {
    return true;    // 이미 base_link — 변환 불필요.
  }

  geometry_msgs::msg::TransformStamped tf_msg;
  bool ok = false;
  try {
    tf_msg = buffer_->lookupTransform(
      target_frame_, frame.frame_id, frame.stamp,
      tf2::durationFromSec(tf_timeout_s_));
    ok = true;
  } catch (const tf2::TransformException & ex) {
    error = ex.what();
  }

  if (!ok && allow_static_fallback_) {
    // static extrinsic 은 시각과 무관하다. 드라이버 stamp 가 TF 버퍼 밖이라는 이유로
    // 멀쩡한 센서를 버리지 않기 위한 경로.
    try {
      tf_msg = buffer_->lookupTransform(
        target_frame_, frame.frame_id, tf2::TimePointZero);
      ok = true;
      error.clear();
    } catch (const tf2::TransformException & ex) {
      error = ex.what();
    }
  }
  if (!ok) {
    return false;
  }

  // 메시지 복사를 피하려고 회전행렬 + 평행이동만 뽑아 점마다 직접 적용한다.
  // (PointCloud2 로 직렬화 → tf2::doTransform → 다시 역직렬화 하는 왕복을 없앤다.)
  const auto & q_msg = tf_msg.transform.rotation;
  const tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
  const tf2::Matrix3x3 rot(q);
  const double tx = tf_msg.transform.translation.x;
  const double ty = tf_msg.transform.translation.y;
  const double tz = tf_msg.transform.translation.z;

  const double r00 = rot[0].x(), r01 = rot[0].y(), r02 = rot[0].z();
  const double r10 = rot[1].x(), r11 = rot[1].y(), r12 = rot[1].z();
  const double r20 = rot[2].x(), r21 = rot[2].y(), r22 = rot[2].z();

  for (auto & p : frame.points) {
    const double x = p.x, y = p.y, z = p.z;
    p.x = static_cast<float>(r00 * x + r01 * y + r02 * z + tx);
    p.y = static_cast<float>(r10 * x + r11 * y + r12 * z + ty);
    p.z = static_cast<float>(r20 * x + r21 * y + r22 * z + tz);
  }
  frame.frame_id = target_frame_;
  return true;
}

}  // namespace multi_lidar_fusion
