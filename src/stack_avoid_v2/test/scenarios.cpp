#include "fixtures.hpp"
#include <random>
using namespace fixture;
int main()
{
  Config c;
  const auto course=straight(7.);
  for(int scene=0;scene<4;++scene){
    Grid grid(c);grid.configure(course);fill(grid,1,1);
    if(scene>=1){box(grid,4,0);}
    if(scene>=2){box(grid,7,scene==3?.8:0);}
    Planner planner(c);planner.set_course(course);planner.set_zone(true,true);
    auto plan=planner.plan({1,0,0,.6,0},grid,1,1);
    std::printf("scene %d valid=%d nodes=%zu points=%zu ms=%.3f reason=%s\n",
      scene,plan.valid,plan.expanded,plan.path.size(),plan.compute_ms,plan.reason.c_str());
    check(plan.valid,"feasible geometry has a certified forward plan");
    for(const auto & p:plan.path){
      check(grid.clear(p),"all planned body footprints clear");
      check(std::fabs(p.steer)<=c.max_steer+1e-9,"steering bound respected");
      if(scene>=1){check(!touches_box(p,c,4,0),"independent first car collision oracle");}
      if(scene>=2){check(!touches_box(p,c,7,scene==3?.8:0),"independent second car collision oracle");}
    }
  }
  // Curved centerline and explicit boundaries, with two stationary cars.
  Course s;s.id="synthetic-s";s.entry=1;s.exit=15;
  for(int i=0;i<=88;++i){double x=i*.25;s.center.push_back({x,1.1*std::sin(x*.35)});}
  s.boundary.push_back({-2,-3.5});
  for(auto p:s.center){s.boundary.push_back({p.x,p.y-3.5});}
  s.boundary.push_back({24,s.center.back().y-3.5});
  s.boundary.push_back({24,s.center.back().y+3.5});
  for(auto i=s.center.rbegin();i!=s.center.rend();++i){s.boundary.push_back({i->x,i->y+3.5});}
  s.boundary.push_back({-2,3.5});
  s.validate();Grid grid(c);grid.configure(s);fill(grid,1,1);box(grid,5,1);box(grid,8,.3);
  Planner planner(c);planner.set_course(s);planner.set_zone(true,true);
  auto p=planner.plan({1,1.1*std::sin(.35),std::atan(1.1*.35*std::cos(.35)),.6,0},grid,1,1);
  std::printf("S course: valid=%d nodes=%zu ms=%.3f reason=%s\n",p.valid,p.expanded,p.compute_ms,p.reason.c_str());
  check(p.valid,"S course can plan around stationary vehicles");
  State vehicle{1,1.1*std::sin(.35),std::atan(1.1*.35*std::cos(.35)),.6,0};
  Planner rolling(c);rolling.set_course(s);rolling.set_zone(true,true);bool finished=false;size_t frames=0;double max_ms=0;
  for(unsigned frame=1;frame<=1800;++frame){
    const double t=2.+frame*.1;fill(grid,t,1000+frame);
    box(grid,5,1,.7,.65,t);box(grid,8,.3,.7,.65,t);
    rolling.set_zone(true,vehicle.x<=s.at(s.exit).x);
    auto next=rolling.plan(vehicle,grid,t,1000+frame);
    max_ms=std::max(max_ms,next.compute_ms);++frames;
    if(!next.valid){
      std::printf("rolling blocked at frame=%u x=%.3f y=%.3f station=%.3f reason=%s\n",
        frame,vehicle.x,vehicle.y,next.station,next.reason.c_str());break;
    }
    if(next.complete){finished=true;break;}
    for(const auto & q:next.path){
      check(!touches_box(q,c,5,1)&&!touches_box(q,c,8,.3),"independent rolling path collision oracle");
    }
    // Execute the timed speed profile for 100ms; ideal geometry replay,
    // not a simulation of the one-point dSPACE controller.
    if(next.path.size()>=3){
      vehicle=sample_path_time(next.path,.1);
    }
  }
  std::printf("rolling S: complete=%d frames=%zu maximum_ms=%.3f\n",finished,frames,max_ms);
  check(finished,"rolling S geometry passes both cars and the exit");
  std::mt19937 rng(20260915);std::uniform_real_distribution<double> offset(-.65,.65);
  unsigned random_valid=0;
  for(unsigned seed=0;seed<12;++seed){
    Grid scene(c);scene.configure(course);fill(scene,1,1);
    const double y1=offset(rng),y2=offset(rng),x1=4.+offset(rng)*.3,x2=7.+offset(rng)*.3;
    box(scene,x1,y1);box(scene,x2,y2);
    Planner random(c);random.set_course(course);random.set_zone(true,true);
    const auto candidate=random.plan({1,0,0,.6,0},scene,1,1);
    if(candidate.valid){++random_valid;}
    for(const auto & q:candidate.path){
      check(!touches_box(q,c,x1,y1)&&!touches_box(q,c,x2,y2),"seeded layouts use independent box oracle");
    }
  }
  check(random_valid>=8,"most feasible seeded layouts find forward candidates");
  std::printf("seeded two-car plans: %u/12 valid (partial plans count; not full-course trials)\n",random_valid);
  std::printf("scenarios: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
