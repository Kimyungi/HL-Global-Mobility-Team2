#include "fixtures.hpp"
#include <random>
using namespace fixture;

int main()
{
  Config cfg;const auto route=straight(7.);
  Grid grid(cfg);grid.configure(route);Planner planner(cfg);planner.set_course(route);
  auto plan=planner.plan({1,0,0,.6,0},grid,1,1);
  check(!plan.valid&&!plan.obstacle_detected,"missing zone membership never authorizes overtaking");
  planner.set_zone(true,false);
  plan=planner.plan({.5,0,0,.6,0},grid,1,2);
  check(plan.phase==Phase::READY&&!plan.perception_active&&plan.expanded==0&&plan.path.empty(),
    "outside zone yields GPS ownership without requiring LiDAR");
  planner.set_zone(true,true);
  plan=planner.plan({1,0,0,.6,0},grid,1,3);
  check(!plan.valid&&plan.phase==Phase::HOLD&&!plan.obstacle_detected&&!plan.maneuver_active,
    "unknown space is HOLD, never an overtaking trigger");
  fill(grid,2,4);box(grid,4,2.1,.7,.65,2);
  plan=planner.plan({1,0,0,.6,0},grid,2,4);
  check(plan.valid&&plan.phase==Phase::WALL_FOLLOW&&!plan.maneuver_active&&plan.expanded>0,
    "off-route vehicles are walls in the same corridor calculation");
  Planner wrong_way(cfg);wrong_way.set_course(route);wrong_way.set_zone(true,true);
  const auto wrong=wrong_way.plan({1,0,3.14,.6,0},grid,2,10);
  check(!wrong.valid&&!wrong.maneuver_active,"GPS continuation cannot authorize opposite-direction travel");
  box(grid,4,0,.7,.65,2);
  PlanTrace trace;trace.limit=3;
  plan=planner.plan({1,0,0,.6,0},grid,2,5,&trace);
  check(plan.valid&&plan.obstacle_detected&&!plan.maneuver_active&&!plan.gps_follow,
    "occupied GPS corridor still uses the wall midpoint planner");
  check(trace.edges.size()==3&&trace.counts[0]>3,"bounded diagnostics retain full counts");
  planner.set_zone(false,false);planner.set_zone(true,true);
  check(grid.occupied({4,0}),"zone validity dropout preserves occupied memory");

  // Convergence after an avoidance-like lateral/heading disturbance, without obstacles.
  Grid clear(cfg);clear.configure(route);Planner recovery(cfg);recovery.set_course(route);
  State vehicle{2,-1.,.25,.6,.2};double last_error=1.;unsigned large_crossings=0;
  // 24s observes convergence while leaving forward geometry in the 20m fixture.
  for(unsigned i=0;i<240;++i){
    double t=3.+i*.1;fill(clear,t,100+i);recovery.set_zone(true,true);
    auto p=recovery.plan(vehicle,clear,t,100+i);
    check(p.valid&&p.phase==Phase::WALL_FOLLOW&&p.expanded>0&&!p.maneuver_active,"empty-road recovery follows wall centers");
    if(!p.valid){break;}
    vehicle=sample_path_time(p.path,.1);
    if(vehicle.y*last_error<0&&std::fabs(vehicle.y)>.1){++large_crossings;}
    last_error=vehicle.y;
  }
  check(std::fabs(vehicle.y)<.05&&std::fabs(vehicle.yaw)<.05&&std::fabs(vehicle.steer)<.03,
    "GPS recovery settles position, heading and steering");
  check(large_crossings<=1,"empty-road recovery does not sustain lateral oscillation");

  std::mt19937 rng(20260915);std::uniform_real_distribution<double> offset(-.8,.8);
  unsigned complete=0;
  for(unsigned layout=0;layout<8;++layout){
    const unsigned count=layout%2?3:2;
    std::vector<Point> boxes;
    for(unsigned j=0;j<count;++j){boxes.push_back({4.+2.7*j,offset(rng)});}
    auto r=route;r.exit=14.;r.validate();Grid g(cfg);g.configure(r);Planner p(cfg);p.set_course(r);
    State ego{1,0,0,.6,0};bool done=false;unsigned passing=0;double worst=0;
    for(unsigned i=0;i<1800;++i){
      const double t=30.+i*.1;fill(g,t,20000+i);
      for(auto b:boxes){box(g,b.x,b.y,.7,.65,t);}
      p.set_zone(true,ego.x<=r.exit);
      const auto next=p.plan(ego,g,t,20000+i);worst=std::max(worst,next.compute_ms);
      if(!next.valid){std::printf("layout %u HOLD x=%.3f y=%.3f %s\n",layout,ego.x,ego.y,next.reason.c_str());break;}
      if(next.phase==Phase::WALL_FOLLOW){++passing;}
      for(const auto & q:next.path){for(auto b:boxes){
        check(!touches_box(q,cfg,b.x,b.y),"entire two/three-car path passes independent collision oracle");
      }}
      if(next.complete){done=true;break;}
      ego=sample_path_time(next.path,.1);
    }
    complete+=done;
    check(done&&passing>0,"feasible two/three-car layout completes and rejoins GPS");
    std::printf("zone layout %u cars=%u complete=%d max_ms=%.3f\n",layout,count,done,worst);
  }
  std::printf("zone scenarios: %u/8 complete, %d checks, %d failures\n",complete,checks,failures);
  return failures?1:0;
}
