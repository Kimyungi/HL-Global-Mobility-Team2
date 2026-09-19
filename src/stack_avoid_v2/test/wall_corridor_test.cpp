#include "same_vehicle_cases.hpp"
#include "vehicle_case_oracle.hpp"
using namespace fixture;

std::vector<Ray> scan(Point origin,const std::vector<std::pair<Point,Point>> & walls,double time)
{
  std::vector<Ray> rays;
  for(int i=0;i<1440;++i){
    const double angle=i*2*std::acos(-1.)/1440;Point d{std::cos(angle),std::sin(angle)};
    double nearest=10.;bool hit=false;
    for(auto line:walls){auto a=line.first,b=line.second;Point e{b.x-a.x,b.y-a.y},v{a.x-origin.x,a.y-origin.y};
      const double divisor=d.x*e.y-d.y*e.x;if(std::fabs(divisor)<1e-9)continue;
      const double t=(v.x*e.y-v.y*e.x)/divisor,u=(v.x*d.y-v.y*d.x)/divisor;
      if(t>1e-4&&t<nearest&&u>=0&&u<=1){nearest=t;hit=true;}
    }
    rays.push_back({origin,{origin.x+nearest*d.x,origin.y+nearest*d.y},time,hit});
  }
  return rays;
}
int main()
{
  const auto cfg=synthetic::vehicle_case_config();auto course=straight(7.);
  // The supplied map is wider than the sensed road: only LiDAR walls constrain
  // this 3m lane, so a hard-coded map edge cannot create the expected midline.
  std::vector<std::pair<Point,Point>> walls{{{-2,-1.5},{22,-1.5}},{{-2,1.5},{22,1.5}}};
  Grid g(cfg);g.configure(course);Planner planner(cfg);planner.set_course(course);planner.set_zone(true,true);
  auto rays=scan({1.76,0},walls,1.);auto rear=scan({.91,0},walls,1.);rays.insert(rays.end(),rear.begin(),rear.end());g.observe(rays,1);
  PlanTrace trace;auto plan=planner.plan({1,0,0,1,0},g,1,1,&trace);
  check(plan.valid&&plan.phase==Phase::WALL_FOLLOW&&!plan.maneuver_active&&!plan.gps_follow,"LiDAR curbs alone produce wall following");
  check(!trace.midpoints.empty(),"wall pairs and midpoints exported");
  for(size_t i=0;i<trace.midpoints.size();++i){
    const auto a=trace.wall_left[i],b=trace.wall_right[i],m=trace.midpoints[i];
    check(std::hypot(m.x-(a.x+b.x)/2,m.y-(a.y+b.y)/2)<1e-8,"midpoint equals opposing wall average");
    check(std::fabs(m.y)<.11,"sensed symmetric walls center the path despite wider map");
  }
  for(auto s:plan.path){check(synthetic::body_inside_curbs(s,cfg,straight(3.)),"planned body remains between LiDAR curbs");}
  // A pair of vehicle boxes is sent as wall segments without labels/IDs.
  for(Point box:std::vector<Point>{{5,.6},{8.5,-.6}}){
    const double hx=(cfg.front+cfg.rear)/2,hy=cfg.width/2;
    std::array<Point,4> corners{{{box.x-hx,box.y-hy},{box.x+hx,box.y-hy},{box.x+hx,box.y+hy},{box.x-hx,box.y+hy}}};
    for(size_t i=0;i<4;++i)walls.push_back({corners[i],corners[(i+1)%4]});
  }
  Grid occluded(cfg);occluded.configure(course);Planner p(cfg);p.set_course(course);p.set_zone(true,true);
  rays=scan({1.76,0},walls,2.);rear=scan({.91,0},walls,2.);rays.insert(rays.end(),rear.begin(),rear.end());occluded.observe(rays,2);
  auto sensed=p.plan({1,0,0,1,0},occluded,2,2,&trace);
  check(!occluded.observed_free({5.5,.6}),"vehicle shadow is unknown, never filled through a wall");
  check(!sensed.maneuver_active,"vehicle walls never start an overtaking state");
  check(sensed.valid,"occluded wall scan produces a certified observed prefix");
  for(auto s:sensed.path){
    check(!touches_box(s,cfg,5,.6,.85,.62)&&!touches_box(s,cfg,8.5,-.6,.85,.62),"LiDAR path avoids same-size vehicles");
    check(synthetic::body_inside_curbs(s,cfg,straight(3.)),"LiDAR path respects sensed curbs");
  }
  // Cell SAT must still reject a body touching either curb, including yaw.
  Grid geometry(cfg);auto narrow=straight(3.);geometry.configure(narrow);fill(geometry,3,3);
  for(State s:std::vector<State>{{4,1.3,0,1,0},{4,-1.3,0,1,0},{4,1.,.7,1,0}}){
    check(!geometry.clear(s),"exact fallback rejects curb overlap");
  }
  box(geometry,5,0,.85,.62,3);
  check(geometry.clear({4.8,.93,0,1,0}),"rectangle fallback accepts a narrow parallel wall gap with full margin");
  check(!geometry.clear({4.8,.55,0,1,0}),"fallback never admits vehicle contact");
  check(!geometry.clear({4.8,.93,0,1,0},.08),"additional swept margin is retained in rectangle fallback");
  const auto expired=p.plan({1,0,0,1,0},occluded,2.31,3);
  check(!expired.valid&&expired.path.empty(),"expired wall free-space cannot produce a path");
  std::printf("wall corridor: checks=%d failures=%d lidar_curb_valid=%d occluded_valid=%d reason=%s\n",
    checks,failures,plan.valid,sensed.valid,sensed.reason.c_str());
  return failures?1:0;
}
