// dummy_ref_publisher — 부트스트래핑 ① 루프백 검증용.
// 직선 ref points + 고정 v_ref를 10ms로 퍼블리시. adas_mgm이 올라오면 대체된다.
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "fma_interfaces/msg/target_ref.hpp"
#include "fma_interfaces/msg/ref_point.hpp"
#include "packet.hpp"

using fma_interfaces::msg::RefPoint;
using fma_interfaces::msg::TargetRef;

class DummyRefPublisher : public rclcpp::Node
{
public:
  DummyRefPublisher()
  : Node("dummy_ref_publisher")
  {
    v_ref_ = declare_parameter<double>("v_ref", 0.3);           // [m/s]
    curvature_ = declare_parameter<double>("curvature", 0.0);   // [1/m] 0 = 직선
    period_ms_ = declare_parameter<int>("period_ms", 10);

    pub_ = create_publisher<TargetRef>("/adas/target_ref", rclcpp::QoS(1));
    timer_ = create_wall_timer(
      std::chrono::milliseconds(period_ms_), [this] {tick();});
    RCLCPP_INFO(get_logger(), "dummy ref: v_ref=%.2f m/s, curvature=%.3f", v_ref_, curvature_);
  }

private:
  void tick()
  {
    TargetRef msg;
    msg.header.stamp = now();
    msg.header.frame_id = "base_link";
    msg.state = TargetRef::STATE_LANE;
    msg.v_ref = static_cast<float>(v_ref_);

    // MPC 샘플링과 동일하게 v_ref × Ts(10ms) 간격으로 N=20점 생성
    const double ds = std::max(v_ref_, 0.1) * 0.01;
    for (int i = 0; i < bridge_dspace::kNumPoints; ++i) {
      RefPoint p;
      const double s = ds * (i + 1);
      if (std::abs(curvature_) < 1e-6) {
        p.x = static_cast<float>(s);
        p.y = 0.0f;
        p.yaw = 0.0f;
      } else {
        const double th = s * curvature_;
        p.x = static_cast<float>(std::sin(th) / curvature_);
        p.y = static_cast<float>((1.0 - std::cos(th)) / curvature_);
        p.yaw = static_cast<float>(th);
      }
      p.curvature = static_cast<float>(curvature_);
      msg.ref_points.push_back(p);
    }
    pub_->publish(msg);
  }

  double v_ref_{}, curvature_{};
  int period_ms_{};
  rclcpp::Publisher<TargetRef>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyRefPublisher>());
  rclcpp::shutdown();
  return 0;
}
