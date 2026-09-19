#include "fixtures.hpp"
#include "s_curve_layouts.hpp"
#include <algorithm>
#include <limits>
using namespace fixture;

// Independent analytic polygon containment of all four vehicle corners.
bool in_road(const State & s,const Config & cfg,const Course & route)
{
  for(double x:{-cfg.rear,cfg.front}){for(double y:{-cfg.width/2,cfg.width/2}){
    Point p{s.x+x*std::cos(s.yaw)-y*std::sin(s.yaw),s.y+x*std::sin(s.yaw)+y*std::cos(s.yaw)};
    int winding=0;
    for(size_t i=0;i<route.boundary.size();++i){
      const auto a=route.boundary[i],b=route.boundary[(i+1)%route.boundary.size()];
      const double cross=(b.x-a.x)*(p.y-a.y)-(p.x-a.x)*(b.y-a.y);
      if(a.y<=p.y&&b.y>p.y&&cross>0){++winding;}
      if(a.y>p.y&&b.y<=p.y&&cross<0){--winding;}
    }
    if(winding==0){return false;}
  }}return true;
}

int main()
{
  Config cfg;unsigned completed=0,total=0;
  for(const auto & layout:synthetic::s_curve_layouts()){
    const auto route=synthetic::s_curve();std::vector<Point> obstacles;
    for(auto sd:layout.station_offset){
      const auto p=synthetic::place_on_normal(route,sd);obstacles.push_back(p);
      const auto projection=route.project(p,sd.x-.4,sd.x+.4);
      check(projection.cross*sd.y>0,"left/right placement uses local GPS normal");
    }
    for(unsigned trial=0;trial<3;++trial){
      ++total;Grid grid(cfg);grid.configure(route);Planner planner(cfg);planner.set_course(route);
      State car{1,1.1*std::sin(.35),std::atan(1.1*.35*std::cos(.35)),.6,0};
      bool done=false,collision=false,outside=false;unsigned frames=0,pass_frames=0,reused=0;
      double maximum_ms=0,min_clearance=std::numeric_limits<double>::infinity();
      std::string reason;
      for(unsigned frame=0;frame<1800;++frame){
        const double t=1.+frame*.1;fill(grid,t,1000+frame);
        for(auto b:obstacles){box(grid,b.x,b.y,.7,.65,t);}
        planner.set_zone(true,car.x<=route.at(route.exit).x);
        const auto p=planner.plan(car,grid,t,1000+frame);++frames;reason=p.reason;
        maximum_ms=std::max(maximum_ms,p.compute_ms);
        if(!p.valid){break;}
        pass_frames+=p.maneuver_active;reused+=p.reason=="previous overtake path revalidated";
        min_clearance=std::min(min_clearance,p.min_clearance);
        for(const auto & q:p.path){
          outside=outside||!in_road(q,cfg,route);
          for(auto b:obstacles){collision=collision||touches_box(q,cfg,b.x,b.y);}
        }
        // Also inspect the executed 100ms prefix at 1cm spacing, independent of
        // the planner's 5cm samples and 3-disc collision representation.
        for(double d=0;d<=.100001;d+=.01){
          const auto q=sample_path_time(p.path,d);outside=outside||!in_road(q,cfg,route);
          for(auto b:obstacles){collision=collision||touches_box(q,cfg,b.x,b.y);}
        }
        if(p.complete){done=true;break;}
        car=sample_path_time(p.path,.1);
      }
      completed+=done;
      const auto at=route.project({car.x,car.y},0,route.length());
      std::printf("%s trial=%u complete=%d frames=%u pass=%u reused=%u collision=%d road_exit=%d cross=%.4f yaw_error=%.4f clearance=%.4f max_ms=%.3f reason=%s\n",
        layout.id,trial+1,done,frames,pass_frames,reused,collision,outside,at.cross,
        wrap(car.yaw-at.yaw),min_clearance,maximum_ms,reason.c_str());
      check(done,"S left/right layout reaches GPS-aligned exit");
      check(!collision&&!outside,"S path and executed prefix avoid cars and road boundaries");
      check(pass_frames==0,
        "all wall following avoids legacy overtake state");
      if(done){check(std::fabs(at.cross)<=cfg.cross_tolerance&&
        std::fabs(wrap(car.yaw-at.yaw))<=cfg.yaw_tolerance,"exit confirms GPS position and heading");}
    }
  }
  std::printf("S lateral: %u/%u completed, %d checks, %d failures\n",completed,total,checks,failures);
  return failures?1:0;
}
