// multi_lidar_fusion — 단계 4: 자차 운동 보상 (Motion compensation)
//
// 이 파일의 역할:
//   센서 stamp 가 t_ref 와 다른 만큼, 그리고 한 스캔 안에서 빔마다 획득시각이 다른
//   만큼(FusionPoint::dt), 차량이 이동한다. 그 이동량을 되돌려 **모든 점을 t_ref
//   시점의 base_link 로** 표현한다. 정지 중에는 무해하고, 이동 중에는 같은 벽이
//   두 겹으로 보이거나 길게 늘어나는 현상을 줄인다(요구 §9, §25 이동 검증).
//
//   초기 버전은 enable_motion_compensation=false 로 꺼둔다. 켜는 것만으로 동작하도록
//   인터페이스는 처음부터 완성해 둔다 — 나중에 추가하려고 파이프라인을 뜯지 않기 위해서.
//
// 모델: 구간 [t_p, t_ref] 동안 body twist (vx, vy, yaw_rate) 가 일정하다고 본다.
//       10ms~100ms 규모 구간에서 저속 차량이면 충분한 근사다.
//
//       theta = w * d,   d = t_ref - t_p
//       |w|>eps :  dx = ( vx*sin(theta) + vy*(cos(theta)-1) ) / w
//                  dy = ( vx*(1-cos(theta)) + vy*sin(theta) ) / w
//       |w|~0   :  dx = vx*d,  dy = vy*d
//       P_ref = R(-theta) * (P_p - [dx, dy])
//
// 입력  : CloudFrame (frame_id = base_link), VehicleTwist (base_link body frame)
// 출력  : 같은 CloudFrame in-place (t_ref 시점 base_link)
// frame : base_link 전용 — 변환 전 프레임에 부르면 안 된다.
// 파라미터: enable_motion_compensation, motion_max_age_s, motion_min_speed,
//           motion_use_point_dt
// 관계  : 노드가 /odom 또는 /vehicle/twist 콜백에서 setTwist(), 융합 주기에
//         CloudTransformer 다음·CloudMerger 앞에서 compensate() 를 부른다.

#ifndef MULTI_LIDAR_FUSION__MOTION_COMPENSATOR_HPP_
#define MULTI_LIDAR_FUSION__MOTION_COMPENSATOR_HPP_

#include "multi_lidar_fusion/types.hpp"

namespace multi_lidar_fusion
{

struct MotionParams
{
  bool enabled{false};
  /// twist 가 이보다 오래됐으면 보상하지 않는다 (엉뚱한 속도로 점을 밀지 않기 위해).
  double max_twist_age_s{0.2};
  /// 이 속도(합성 속도 또는 |yaw_rate|*1m) 미만이면 보상 생략 — 정지 중 잡음 방지.
  double min_speed{0.05};
  /// true = 점별 dt 까지 반영(스캔 내 왜곡 보정), false = 프레임 단위 한 번만.
  bool use_point_dt{true};
};

/// 보상 1회의 결과 (진단용).
struct MotionStats
{
  bool applied{false};
  double frame_dt_s{0.0};    ///< t_ref - frame.stamp
  double shift_m{0.0};       ///< 프레임 중심이 밀린 거리 (크면 의심 신호)
  const char * skip_reason{"disabled"};
};

class MotionCompensator
{
public:
  explicit MotionCompensator(const MotionParams & params);

  /// 최신 차량 운동 상태 갱신. /odom, /vehicle/twist, 혹은 dSPACE vehicle_vector
  /// 어느 쪽에서 왔든 이 구조체 하나로 환원해서 넣는다.
  void setTwist(const VehicleTwist & twist) {twist_ = twist;}
  const VehicleTwist & twist() const {return twist_;}

  const MotionParams & params() const {return params_;}
  void setParams(const MotionParams & p) {params_ = p;}

  /// frame 의 모든 점을 t_ref 시점 base_link 로 옮긴다.
  /// 보상하지 않은 경우에도 false 만 돌려줄 뿐 프레임은 그대로 쓸 수 있다.
  MotionStats compensate(CloudFrame & frame, const rclcpp::Time & t_ref) const;

private:
  MotionParams params_;
  VehicleTwist twist_;
};

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__MOTION_COMPENSATOR_HPP_
