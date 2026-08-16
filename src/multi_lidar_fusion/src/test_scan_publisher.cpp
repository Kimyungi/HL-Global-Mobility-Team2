// multi_lidar_fusion — 합성 LiDAR 시뮬레이터 (실 센서 없이 파이프라인 검증)
//
// 이 파일의 역할:
//   실제 라이다 4대가 없어도 Phase 1~8 을 전부 돌려볼 수 있게 한다(요구 §27).
//   odom 좌표계에 직사각형 방 + 장애물 상자들을 놓고, 각 센서 위치에서 광선을
//   쏴서 LaserScan 을 만든다. 센서마다 주기·FOV·해상도·노이즈가 다르게 설정되므로
//   §7·§8(주기 불일치)과 §25(같은 벽이 겹치는가) 검증이 그대로 된다.
//
//   vehicle.vx / vehicle.yaw_rate 를 주면 차량이 움직이고, odom->base_link TF 와
//   /odom 을 함께 낸다 → motion compensation(§9) 검증용.
//
// 입력 topic : 없음
// 출력 topic : sensors.<id>.topic (LaserScan) x N, /odom (옵션)
// 출력 TF    : odom -> base_link (옵션). base_link -> lidar_xx_link 는 융합 노드가 낸다.
// frame      : 월드 = odom, 차량 = base_link, 센서 = lidar_xx_link

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

constexpr double kDeg2Rad = M_PI / 180.0;

struct Segment
{
  double x1, y1, x2, y2;
};

/// 원점 o 에서 방향 d(단위벡터) 로 쏜 광선과 선분의 교점 거리. 없으면 -1.
double raySegment(double ox, double oy, double dx, double dy, const Segment & s)
{
  const double ex = s.x2 - s.x1;
  const double ey = s.y2 - s.y1;
  const double denom = dx * ey - dy * ex;
  if (std::fabs(denom) < 1e-12) {
    return -1.0;      // 평행
  }
  const double px = s.x1 - ox;
  const double py = s.y1 - oy;
  const double t = (px * ey - py * ex) / denom;   // 광선 파라미터 (= 거리)
  const double u = (px * dy - py * dx) / denom;   // 선분 파라미터 [0,1]
  if (t < 0.0 || u < 0.0 || u > 1.0) {
    return -1.0;
  }
  return t;
}

void addBox(std::vector<Segment> & out, double cx, double cy, double sx, double sy)
{
  const double x0 = cx - sx * 0.5, x1 = cx + sx * 0.5;
  const double y0 = cy - sy * 0.5, y1 = cy + sy * 0.5;
  out.push_back({x0, y0, x1, y0});
  out.push_back({x1, y0, x1, y1});
  out.push_back({x1, y1, x0, y1});
  out.push_back({x0, y1, x0, y0});
}

}  // namespace

class TestScanPublisher : public rclcpp::Node
{
public:
  TestScanPublisher()
  : rclcpp::Node("test_scan_publisher"), rng_(12345)
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // ── 월드: 방 + 장애물 ──
    const double rx0 = declare_parameter<double>("room.min_x", -6.0);
    const double rx1 = declare_parameter<double>("room.max_x", 6.0);
    const double ry0 = declare_parameter<double>("room.min_y", -4.0);
    const double ry1 = declare_parameter<double>("room.max_y", 4.0);
    segments_.push_back({rx0, ry0, rx1, ry0});
    segments_.push_back({rx1, ry0, rx1, ry1});
    segments_.push_back({rx1, ry1, rx0, ry1});
    segments_.push_back({rx0, ry1, rx0, ry0});

    // [cx, cy, size_x, size_y] 반복
    const auto obs = declare_parameter<std::vector<double>>(
      "obstacles", std::vector<double>{2.0, 0.5, 0.4, 0.4, -2.0, -1.0, 0.6, 0.3});
    for (std::size_t i = 0; i + 3 < obs.size(); i += 4) {
      addBox(segments_, obs[i], obs[i + 1], obs[i + 2], obs[i + 3]);
    }

    // ── 차량 운동 ──
    vx_ = declare_parameter<double>("vehicle.vx", 0.0);
    vy_ = declare_parameter<double>("vehicle.vy", 0.0);
    yaw_rate_ = declare_parameter<double>("vehicle.yaw_rate", 0.0);
    pose_x_ = declare_parameter<double>("vehicle.start_x", 0.0);
    pose_y_ = declare_parameter<double>("vehicle.start_y", 0.0);
    pose_yaw_ = declare_parameter<double>("vehicle.start_yaw", 0.0);
    publish_odom_ = declare_parameter<bool>("vehicle.publish_odom", true);
    publish_tf_ = declare_parameter<bool>("vehicle.publish_tf", true);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }
    if (publish_odom_) {
      odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
        declare_parameter<std::string>("vehicle.odom_topic", "/odom"),
        rclcpp::QoS(rclcpp::KeepLast(10)));
    }

    // ── 센서들 ──
    const auto ids = declare_parameter<std::vector<std::string>>(
      "sensor_ids", std::vector<std::string>{"a1", "a2", "b1", "b2"});
    for (const auto & id : ids) {
      Sim s;
      s.id = id;
      const std::string ns = "sensors." + id;
      const std::string ens = "extrinsics." + id;
      s.topic = declare_parameter<std::string>(ns + ".topic", "/lidar/" + id + "/scan");
      s.frame_id = declare_parameter<std::string>(ns + ".frame_id", "lidar_" + id + "_link");
      s.rate_hz = declare_parameter<double>(ns + ".rate_hz", 10.0);
      s.angle_min = declare_parameter<double>(ns + ".angle_min_deg", -180.0) * kDeg2Rad;
      s.angle_max = declare_parameter<double>(ns + ".angle_max_deg", 180.0) * kDeg2Rad;
      s.angle_inc = declare_parameter<double>(ns + ".angle_increment_deg", 1.0) * kDeg2Rad;
      s.range_min = declare_parameter<double>(ns + ".range_min", 0.05);
      s.range_max = declare_parameter<double>(ns + ".range_max", 12.0);
      s.noise_std = declare_parameter<double>(ns + ".noise_std", 0.01);
      s.x = declare_parameter<double>(ens + ".x", 0.0);
      s.y = declare_parameter<double>(ens + ".y", 0.0);
      s.yaw = declare_parameter<double>(ens + ".yaw", 0.0);

      s.pub = create_publisher<sensor_msgs::msg::LaserScan>(
        s.topic, rclcpp::SensorDataQoS());
      sims_.push_back(std::move(s));
    }

    for (std::size_t k = 0; k < sims_.size(); ++k) {
      const double hz = std::max(sims_[k].rate_hz, 0.1);
      sims_[k].timer = create_wall_timer(
        std::chrono::duration<double>(1.0 / hz),
        [this, k]() {this->publishScan(k);});
      RCLCPP_INFO(
        get_logger(), "sim %s -> %s @ %.1fHz, frame=%s, pose=(%.2f, %.2f, %.1fdeg)",
        sims_[k].id.c_str(), sims_[k].topic.c_str(), hz, sims_[k].frame_id.c_str(),
        sims_[k].x, sims_[k].y, sims_[k].yaw / kDeg2Rad);
    }

    // 차량 상태는 센서와 무관하게 100Hz 로 적분한다.
    pose_timer_ = create_wall_timer(
      std::chrono::milliseconds(10), [this]() {this->stepPose();});
    last_pose_time_ = now();
  }

private:
  struct Sim
  {
    std::string id;
    std::string topic;
    std::string frame_id;
    double rate_hz{10.0};
    double angle_min{-M_PI}, angle_max{M_PI}, angle_inc{kDeg2Rad};
    double range_min{0.05}, range_max{12.0};
    double noise_std{0.01};
    double x{0.0}, y{0.0}, yaw{0.0};
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub;
    rclcpp::TimerBase::SharedPtr timer;
  };

  void stepPose()
  {
    const rclcpp::Time t = now();
    double dt = (t - last_pose_time_).seconds();
    last_pose_time_ = t;
    if (dt <= 0.0 || dt > 0.5) {
      dt = 0.01;
    }
    // body twist 적분 (융합 노드의 보상 모델과 같은 가정).
    const double c = std::cos(pose_yaw_), s = std::sin(pose_yaw_);
    pose_x_ += (vx_ * c - vy_ * s) * dt;
    pose_y_ += (vx_ * s + vy_ * c) * dt;
    pose_yaw_ += yaw_rate_ * dt;

    if (tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = t;
      tf.header.frame_id = world_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = pose_x_;
      tf.transform.translation.y = pose_y_;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, pose_yaw_);
      tf.transform.rotation.x = q.x();
      tf.transform.rotation.y = q.y();
      tf.transform.rotation.z = q.z();
      tf.transform.rotation.w = q.w();
      tf_broadcaster_->sendTransform(tf);
    }
    if (odom_pub_) {
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = t;
      odom.header.frame_id = world_frame_;
      odom.child_frame_id = base_frame_;
      odom.pose.pose.position.x = pose_x_;
      odom.pose.pose.position.y = pose_y_;
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, pose_yaw_);
      odom.pose.pose.orientation.x = q.x();
      odom.pose.pose.orientation.y = q.y();
      odom.pose.pose.orientation.z = q.z();
      odom.pose.pose.orientation.w = q.w();
      odom.twist.twist.linear.x = vx_;      // body frame — 융합 노드가 그대로 쓴다
      odom.twist.twist.linear.y = vy_;
      odom.twist.twist.angular.z = yaw_rate_;
      odom_pub_->publish(odom);
    }
  }

  void publishScan(std::size_t k)
  {
    const Sim & s = sims_[k];

    // 센서의 월드 위치 = odom <- base_link <- lidar_xx_link
    const double cb = std::cos(pose_yaw_), sb = std::sin(pose_yaw_);
    const double sx = pose_x_ + cb * s.x - sb * s.y;
    const double sy = pose_y_ + sb * s.x + cb * s.y;
    const double syaw = pose_yaw_ + s.yaw;

    const auto n = static_cast<std::size_t>(
      std::floor((s.angle_max - s.angle_min) / s.angle_inc)) + 1U;

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = now();
    msg.header.frame_id = s.frame_id;
    msg.angle_min = static_cast<float>(s.angle_min);
    msg.angle_max = static_cast<float>(s.angle_max);
    msg.angle_increment = static_cast<float>(s.angle_inc);
    msg.range_min = static_cast<float>(s.range_min);
    msg.range_max = static_cast<float>(s.range_max);
    msg.scan_time = static_cast<float>(1.0 / std::max(s.rate_hz, 0.1));
    // 한 바퀴 도는 동안 빔이 순차 획득된다 — dt 가 실제로 생겨야 §9 검증이 의미 있다.
    msg.time_increment = static_cast<float>(msg.scan_time / static_cast<double>(n));
    msg.ranges.resize(n);

    std::normal_distribution<double> noise(0.0, std::max(s.noise_std, 0.0));

    for (std::size_t i = 0; i < n; ++i) {
      const double th = syaw + s.angle_min + s.angle_inc * static_cast<double>(i);
      const double dx = std::cos(th), dy = std::sin(th);
      double best = -1.0;
      for (const auto & seg : segments_) {
        const double t = raySegment(sx, sy, dx, dy, seg);
        if (t > 0.0 && (best < 0.0 || t < best)) {
          best = t;
        }
      }
      if (best < 0.0 || best > s.range_max) {
        msg.ranges[i] = std::numeric_limits<float>::infinity();   // 미반사
        continue;
      }
      const double r = s.noise_std > 0.0 ? best + noise(rng_) : best;
      msg.ranges[i] = r < s.range_min ?
        std::numeric_limits<float>::infinity() : static_cast<float>(r);
    }
    s.pub->publish(msg);
  }

  std::string world_frame_, base_frame_;
  std::vector<Segment> segments_;
  std::vector<Sim> sims_;

  double vx_{0.0}, vy_{0.0}, yaw_rate_{0.0};
  double pose_x_{0.0}, pose_y_{0.0}, pose_yaw_{0.0};
  bool publish_odom_{true}, publish_tf_{true};
  rclcpp::Time last_pose_time_{0, 0, RCL_ROS_TIME};

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr pose_timer_;
  std::mt19937 rng_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestScanPublisher>());
  rclcpp::shutdown();
  return 0;
}
