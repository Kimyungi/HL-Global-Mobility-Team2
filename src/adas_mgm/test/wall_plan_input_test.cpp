#include "src/avoid_plan_input.hpp"
#include "manager_test_fixture.hpp"
#include <limits>
using namespace manager_test;
int main(){
  fma_interfaces::msg::AvoidPlan p;
  fma_interfaces::msg::GpsPath gps;
  p.control_enabled=p.plan_valid=p.episode_active=p.perception_active=true;
  p.phase=p.WALL_FOLLOW;p.episode_id=p.plan_id=p.observation_generation=1;
  p.header.frame_id="map";p.header.stamp.sec=10;p.valid_until.sec=10;p.valid_until.nanosec=100000000;
  p.pose_stamp=p.observation_stamp=p.header.stamp;
  p.reference.header.frame_id="base_link";p.reference.reference_stamp=p.header.stamp;
  p.reference.scan_valid=p.reference.avoidable=true;
  fma_interfaces::msg::RefPoint q;q.x=1;q.y=.4;q.yaw=.1;q.curvature=.05;
  p.reference.points={q};p.speed_limit_mps=.37;
  auto read=[&](const auto & v,int64_t now=10010000000LL,int64_t age=10000000LL,int64_t activated=10000000000LL){
    return adas_mgm::wall_plan_input(v,gps,now,age,activated);};
  check(read(p).points.size()==1&&std::fabs(read(p).v_suggest-.37)<1e-6,"valid plan carries actual adaptive speed");
  check(read(p,10100000000LL).points.empty(),"expiry stops exactly at lease boundary");
  check(read(p,10010000000LL,110000000LL).points.empty(),"monotonic expiry survives frozen ROS clock");
  check(read(p,9999000000LL).points.empty(),"future plan rejected");
  check(read(p,10010000000LL,10000000LL,10005000000LL).points.empty(),"previous activation plan rejected");
  auto bad=p;bad.control_enabled=false;check(read(bad).points.empty(),"shadow plan has no authority");
  bad=p;bad.plan_valid=false;bad.complete=true;check(!read(bad).maneuver_done,"HOLD cannot signal completion");
  bad=p;bad.plan_valid=false;bad.phase=bad.HOLD;bad.obstacle_detected=true;
  check(read(bad).obstacle_detected&&read(bad).points.empty(),"fresh HOLD preserves obstacle evidence without motion authority");
  bad=p;bad.deadline_hit=true;check(read(bad).points.empty(),"processing overrun stops");
  bad=p;bad.speed_limit_mps=std::numeric_limits<double>::quiet_NaN();check(read(bad).points.empty(),"NaN speed rejected");
  bad=p;bad.route.enabled=true;check(read(bad).points.empty(),"route mismatch rejected");
  bad=p;bad.reference.reference_stamp.nanosec=1;check(read(bad).points.empty(),"invented input timestamp rejected");
  bad=p;bad.complete=true;bad.obstacle_detected=true;check(!read(bad).maneuver_done,"blocked GPS cannot finish");
  Run r;r.st.params.avoid_zone_only=1;r.st.params.avoid_unblended=1;r.st.params.blend_cycles=10;
  r.st.params.safe_stop_all_sensors_only=1;r.s.sensor_alive_mask=127;
  r.s.gps_avoid_zone=true;r.s.avoid_path.pts[0]={1,.4f,.1f,.05f};r.s.avoid_v_suggest=.37f;r.tick();
  check(r.out.avoid==AvoidState::AVOID_ACTIVE,"zone still claims avoidance without obstacle");
  check(std::fabs(r.out.ref_points[0].x-1)<1e-6&&std::fabs(r.out.ref_points[0].y-.4)<1e-6,
    "entry preserves exact wall target despite configured legacy blend");
  check(std::fabs(r.out.v_ref-.37)<1e-6,"MGM preserves adaptive speed");
  r.s.avoid_path.n=0;r.tick();check(r.out.reference_motion_blocked&&r.out.v_ref==0&&r.out.avoid==AvoidState::AVOID_ACTIVE,
    "invalid plan holds while zone ownership persists");
  std::printf("wall_plan_input_test: %d checks, %d failures\n",checks,failures);return failures?1:0;
}
