#pragma once
#include <fma_interfaces/msg/avoid_plan.hpp>
#include <fma_interfaces/msg/gps_path.hpp>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace adas_mgm {
inline int64_t avoid_time_ns(const builtin_interfaces::msg::Time & t)
{return int64_t(t.sec)*1000000000LL+t.nanosec;}

// Checked on every MGM tick, including when the planner process stops publishing.
inline fma_interfaces::msg::AvoidStatus wall_plan_input(
  const fma_interfaces::msg::AvoidPlan & p,const fma_interfaces::msg::GpsPath & gps,
  int64_t now_ns,int64_t received_age_ns,int64_t activation_ns)
{
  fma_interfaces::msg::AvoidStatus out;out.ttc=1e9f;
  const auto stamp=avoid_time_ns(p.header.stamp),expiry=avoid_time_ns(p.valid_until);
  const auto observed=avoid_time_ns(p.observation_stamp),pose=avoid_time_ns(p.pose_stamp);
  const bool route=p.route.enabled==gps.route.enabled&&(!p.route.enabled||
    (p.route_id==gps.route.route_id&&p.route.instance_id==gps.route.instance_id&&
     p.route.acknowledged_request==gps.route.acknowledged_request&&p.route.connecting==gps.route.connecting));
  // Obstacle evidence and path authority are independent, as in AvoidStatus.
  // A freshly observed obstacle during HOLD must still count as a seen maneuver.
  if(p.control_enabled&&p.episode_active&&p.perception_active&&route&&
    p.episode_id&&p.observation_generation&&stamp>=activation_ns&&stamp>0&&
    now_ns>=stamp&&now_ns-stamp<=200000000LL&&received_age_ns>=0&&received_age_ns<=200000000LL&&
    observed>0&&observed<=stamp&&now_ns-observed<=300000000LL){
    out.obstacle_detected=p.obstacle_detected;
  }
  if(!p.control_enabled||!p.plan_valid||p.deadline_hit||!p.episode_active||!p.perception_active||
    p.phase!=p.WALL_FOLLOW||!p.episode_id||!p.plan_id||!p.observation_generation||!route||
    p.header.frame_id!="map"||p.reference.header.frame_id!="base_link"||
    stamp<=0||stamp<activation_ns||now_ns<stamp||now_ns>=expiry||expiry-stamp>300000000LL||
    received_age_ns<0||received_age_ns>=expiry-stamp||observed<=0||pose<=0||
    observed>stamp||pose>stamp||p.reference.points.size()!=1||
    !p.reference.scan_valid||!p.reference.avoidable||
    avoid_time_ns(p.reference.reference_stamp)!=std::min(observed,pose)||
    !std::isfinite(p.speed_limit_mps)||p.speed_limit_mps<0||p.speed_limit_mps>3.) {return out;}
  const auto & q=p.reference.points[0];
  if(!std::isfinite(q.x)||!std::isfinite(q.y)||!std::isfinite(q.yaw)||!std::isfinite(q.curvature)||
    (q.x==0&&q.y==0)){return out;}
  out=p.reference;out.obstacle_detected=p.obstacle_detected;
  out.maneuver_done=p.complete&&!p.obstacle_detected;
  out.v_suggest=p.speed_limit_mps;out.narrow_gap=false;
  return out;
}
}
