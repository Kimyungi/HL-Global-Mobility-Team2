#include "multi_lidar_fusion/motion_compensator.hpp"

#include <cmath>

namespace multi_lidar_fusion
{

namespace
{

/// 구간 d 동안의 body 이동량 (t_p 좌표계 기준) + 회전각.
/// 등속·등각속 가정의 닫힌 해. w -> 0 에서 0/0 이 되므로 분기한다.
struct BodyDelta
{
  double dx{0.0};
  double dy{0.0};
  double theta{0.0};
};

BodyDelta integrateTwist(double vx, double vy, double w, double d)
{
  BodyDelta out;
  out.theta = w * d;
  if (std::fabs(w) < 1e-6) {
    out.dx = vx * d;
    out.dy = vy * d;
    return out;
  }
  const double s = std::sin(out.theta);
  const double c = std::cos(out.theta);
  out.dx = (vx * s + vy * (c - 1.0)) / w;
  out.dy = (vx * (1.0 - c) + vy * s) / w;
  return out;
}

}  // namespace

MotionCompensator::MotionCompensator(const MotionParams & params)
: params_(params)
{
}

MotionStats MotionCompensator::compensate(CloudFrame & frame, const rclcpp::Time & t_ref) const
{
  MotionStats stats;
  stats.frame_dt_s = (t_ref - frame.stamp).seconds();

  if (!params_.enabled) {
    stats.skip_reason = "disabled";
    return stats;
  }
  if (!twist_.valid) {
    stats.skip_reason = "no_twist";
    return stats;
  }
  const double twist_age = (t_ref - twist_.stamp).seconds();
  if (std::fabs(twist_age) > params_.max_twist_age_s) {
    stats.skip_reason = "twist_too_old";
    return stats;
  }
  // yaw_rate 만 있어도 1m 떨어진 점은 |w|*1m 만큼 움직인다 — 속도 문턱에 함께 넣는다.
  const double speed = std::hypot(twist_.vx, twist_.vy) + std::fabs(twist_.yaw_rate);
  if (speed < params_.min_speed) {
    stats.skip_reason = "below_min_speed";
    return stats;
  }
  if (frame.points.empty()) {
    stats.skip_reason = "empty";
    return stats;
  }

  const double vx = twist_.vx;
  const double vy = twist_.vy;
  const double w = twist_.yaw_rate;
  const double d0 = stats.frame_dt_s;

  if (!params_.use_point_dt) {
    // 빠른 경로 — 프레임 전체에 같은 변환 한 번. 점별 시각을 못 주는 드라이버용.
    const BodyDelta bd = integrateTwist(vx, vy, w, d0);
    const double ct = std::cos(-bd.theta);
    const double st = std::sin(-bd.theta);
    for (auto & p : frame.points) {
      const double px = static_cast<double>(p.x) - bd.dx;
      const double py = static_cast<double>(p.y) - bd.dy;
      p.x = static_cast<float>(ct * px - st * py);
      p.y = static_cast<float>(st * px + ct * py);
    }
    stats.applied = true;
    stats.shift_m = std::hypot(bd.dx, bd.dy);
    stats.skip_reason = "";
    return stats;
  }

  // 점별 경로 — 한 스캔 안의 왜곡(rolling shutter)까지 편다.
  for (auto & p : frame.points) {
    const double d = d0 - static_cast<double>(p.dt);
    const BodyDelta bd = integrateTwist(vx, vy, w, d);
    const double ct = std::cos(-bd.theta);
    const double st = std::sin(-bd.theta);
    const double px = static_cast<double>(p.x) - bd.dx;
    const double py = static_cast<double>(p.y) - bd.dy;
    p.x = static_cast<float>(ct * px - st * py);
    p.y = static_cast<float>(st * px + ct * py);
  }
  const BodyDelta bd0 = integrateTwist(vx, vy, w, d0);
  stats.applied = true;
  stats.shift_m = std::hypot(bd0.dx, bd0.dy);
  stats.skip_reason = "";
  // 보상 후 모든 점의 기준시각은 t_ref 다.
  frame.stamp = t_ref;
  return stats;
}

}  // namespace multi_lidar_fusion
