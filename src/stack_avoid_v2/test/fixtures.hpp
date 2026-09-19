#pragma once
#include "stack_avoid_v2/core.hpp"
#include <cmath>
#include <cstdio>
#include <stdexcept>
namespace fixture
{
using namespace avoid_v2;
inline int failures=0, checks=0;
inline void check(bool ok,const char * why)
{++checks;if(!ok){++failures;std::fprintf(stderr,"FAIL: %s\n",why);}}
inline Course straight(double width=6.)
{
  Course c;c.id="synthetic-straight";c.entry=1.;c.exit=12.;
  for(int i=0;i<=80;++i){c.center.push_back({i*.25,0});}
  c.boundary={{-2,-width/2},{22,-width/2},{22,width/2},{-2,width/2}};
  c.validate();return c;
}
// A geometry-only fixture: full independently known free space, NOT a LiDAR simulation.
inline void fill(Grid & g,double stamp,uint64_t gen)
{
  std::vector<Ray> rays;
  for(int i=-160;i<=160;++i){rays.push_back({{-2,i*.05},{22,i*.05},stamp,false});}
  g.observe(rays,gen);g.prepare(stamp);
}
inline void box(Grid & g,double x,double y,double length=.7,double width=.65,double stamp=1.)
{
  std::vector<Ray> rays;
  for(double dx=-length/2;dx<=length/2+.001;dx+=.04){
    for(double dy=-width/2;dy<=width/2+.001;dy+=.04){
      rays.push_back({{x+dx-.001,y+dy},{x+dx,y+dy},stamp,true});
    }
  }
  g.observe(rays,uint64_t(stamp*10000)+1);g.prepare(stamp);
}
// Independent rectangle separating-axis oracle against analytic vehicle boxes.
// Does not query the occupancy grid or the planner's three-disc approximation.
inline bool touches_box(const State & s,const Config & cfg,double x,double y,
  double length=.7,double width=.65)
{
  const double c=std::cos(s.yaw),sn=std::sin(s.yaw);
  const Point vehicle_center{s.x+(cfg.front-cfg.rear)*.5*c,s.y+(cfg.front-cfg.rear)*.5*sn};
  const Point delta{vehicle_center.x-x,vehicle_center.y-y};
  for(Point axis:std::vector<Point>{{1,0},{0,1},{c,sn},{-sn,c}}){
    const double separation=std::fabs(delta.x*axis.x+delta.y*axis.y);
    const double ego=(cfg.front+cfg.rear)*.5*std::fabs(c*axis.x+sn*axis.y)+
      cfg.width*.5*std::fabs(-sn*axis.x+c*axis.y);
    const double object=length*.5*std::fabs(axis.x)+width*.5*std::fabs(axis.y);
    if(separation>ego+object){return false;}
  }
  return true;
}
}
