#pragma once
#include "fixtures.hpp"
#include <array>
#include <algorithm>

namespace synthetic
{
inline std::array<avoid_v2::Point,4> body_corners(const avoid_v2::State & s,const avoid_v2::Config & cfg)
{
  std::array<avoid_v2::Point,4> local{{{-cfg.rear,-cfg.width/2},{cfg.front,-cfg.width/2},
    {cfg.front,cfg.width/2},{-cfg.rear,cfg.width/2}}};
  for(auto & p:local){const auto q=p;p={s.x+q.x*std::cos(s.yaw)-q.y*std::sin(s.yaw),
    s.y+q.x*std::sin(s.yaw)+q.y*std::cos(s.yaw)};}return local;
}
inline double orientation(avoid_v2::Point a,avoid_v2::Point b,avoid_v2::Point p)
{return (b.x-a.x)*(p.y-a.y)-(p.x-a.x)*(b.y-a.y);}
inline bool body_inside_curbs(const avoid_v2::State & s,const avoid_v2::Config & cfg,
  const avoid_v2::Course & route)
{
  const auto corners=body_corners(s,cfg);
  // Winding-number containment plus edge intersection: corners alone are not
  // sufficient when a vehicle edge spans a concave part of an S boundary.
  for(auto p:corners){int winding=0;
    for(size_t i=0;i<route.boundary.size();++i){
      const auto a=route.boundary[i],b=route.boundary[(i+1)%route.boundary.size()];
      const double side=orientation(a,b,p);
      if(a.y<=p.y&&b.y>p.y&&side>0){++winding;}
      if(a.y>p.y&&b.y<=p.y&&side<0){--winding;}
    }
    if(winding==0){return false;}
  }
  for(size_t i=0;i<route.boundary.size();++i){
    const auto a=route.boundary[i],b=route.boundary[(i+1)%route.boundary.size()];
    for(size_t j=0;j<4;++j){const auto p=corners[j],q=corners[(j+1)%4];
      if(std::max(a.x,b.x)<std::min(p.x,q.x)||std::max(p.x,q.x)<std::min(a.x,b.x)||
         std::max(a.y,b.y)<std::min(p.y,q.y)||std::max(p.y,q.y)<std::min(a.y,b.y)){continue;}
      if(orientation(a,b,p)*orientation(a,b,q)<=0&&orientation(p,q,a)*orientation(p,q,b)<=0){return false;}
    }
  }
  return true;
}
struct VehicleCaseAudit
{
  bool vehicle_contact{},curb_contact{};
  unsigned samples{};
  void inspect(const avoid_v2::State & s,const avoid_v2::Config & cfg,
    const avoid_v2::Course & route,const std::vector<avoid_v2::Point> & boxes)
  {
    ++samples;curb_contact=curb_contact||!body_inside_curbs(s,cfg,route);
    for(auto b:boxes){vehicle_contact=vehicle_contact||
      fixture::touches_box(s,cfg,b.x,b.y,cfg.front+cfg.rear,cfg.width);}
  }
};
}
