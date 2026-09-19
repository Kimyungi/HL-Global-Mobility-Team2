#pragma once
#include "s_curve_layouts.hpp"
#include <string>

namespace synthetic
{
struct VehicleCase
{
  std::string id,title,description;
  std::vector<avoid_v2::Point> placements;  // GPS station, signed lateral offset
  bool curved{true},moving{true},expire{},expected_hold{};
  double entry{1.},visible_until{22.},corridor_width{3.};
};
inline avoid_v2::Config vehicle_case_config()
{
  avoid_v2::Config cfg;cfg.cruise_speed=1.;return cfg;
}
inline avoid_v2::Course vehicle_case_course(const VehicleCase & scene)
{
  auto c=s_curve();c.id=scene.id;c.entry=scene.entry;c.exit=scene.curved?15.:12.;
  if(!scene.curved){for(auto & p:c.center){p.y=0;}}
  // Offset every GPS segment by half the lane width along its normal. Miter
  // intersections preserve perpendicular width through bends, unlike Y +/- w/2.
  const auto unit=[](avoid_v2::Point a,avoid_v2::Point b){
    const double d=std::hypot(b.x-a.x,b.y-a.y);return avoid_v2::Point{(b.x-a.x)/d,(b.y-a.y)/d};
  };
  auto spine=c.center;const auto start=unit(spine[0],spine[1]),end=unit(spine[spine.size()-2],spine.back());
  spine.insert(spine.begin(),{spine.front().x-2*start.x,spine.front().y-2*start.y});
  spine.push_back({spine.back().x+2*end.x,spine.back().y+2*end.y});
  std::vector<avoid_v2::Point> left,right;const double half=scene.corridor_width/2;
  for(size_t i=0;i<spine.size();++i){
    const auto before=i?unit(spine[i-1],spine[i]):start;
    const auto after=i+1<spine.size()?unit(spine[i],spine[i+1]):end;
    const double divisor=1+before.x*after.x+before.y*after.y;
    const avoid_v2::Point offset{-half*(before.y+after.y)/divisor,half*(before.x+after.x)/divisor};
    left.push_back({spine[i].x+offset.x,spine[i].y+offset.y});
    right.push_back({spine[i].x-offset.x,spine[i].y-offset.y});
  }
  c.boundary=right;c.boundary.insert(c.boundary.end(),left.rbegin(),left.rend());
  c.validate();return c;
}
inline std::vector<VehicleCase> same_vehicle_cases()
{
  std::vector<VehicleCase> cases;
  for(unsigned count:{2,3}){
    const std::string suffix=std::to_string(count),cars=suffix+"대";
    const auto add=[&](std::string id,std::string title,std::vector<avoid_v2::Point> points,bool curved=true){
      cases.push_back({id+"_"+suffix,title+" "+cars,
        "1차로 3.0m · 최대 1m/s · 유동 속도 · 차량 0.85×0.62m · 좌우 연석 통과 금지",points,curved});
    };
    for(int side:{1,-1}){
      std::vector<avoid_v2::Point> alternating,same;
      for(unsigned i=0;i<count;++i){
        const double station=i==0?5.:i==1?8.5:11.;
        alternating.push_back({station,side*(i%2?-.6:.6)});same.push_back({station,side*.6});
      }
      add(side>0?"s_lr":"s_rl",side>0?"S자 · 좌→우 교대":"S자 · 우→좌 교대",alternating);
      add(side>0?"s_left":"s_right",side>0?"S자 · 왼쪽 연속":"S자 · 오른쪽 연속",same);
      add(side>0?"straight_lr":"straight_rl",side>0?"직선 · 좌→우 교대":"직선 · 우→좌 교대",alternating,false);
    }
    std::vector<avoid_v2::Point> central{{7,1.0},{7,-1.0}},centerline,off_path,entry,unknown,expired,close;
    if(count==3){central.push_back({10.5,1.0});}
    add("s_center_gap","S자 · 좌우 차량 / 중앙 통로",central);
    for(unsigned i=0;i<count;++i){
      centerline.push_back({5.+3*i,0});off_path.push_back({5.+3*i,i%2?-1.0:1.0});
      entry.push_back({6.+2.5*i,i%2?.6:-.6});unknown.push_back({6.+3*i,i%2?.6:-.6});
      expired.push_back({5.+3*i,i%2?.6:-.6});close.push_back({2.2+2.7*i,0});
    }
    add("straight_center","직선 · 중심선 차량",centerline,false);
    add("off_path","S자 · GPS 진행 공간 밖",off_path);
    add("zone_gate","zone 진입 후 인식",entry,false);cases.back().entry=3.;
    add("unknown","앞 공간 미관측",unknown,false);cases.back().visible_until=3.2;cases.back().expected_hold=true;
    add("expiry","관측 만료 · 점유 기억",expired,false);
    cases.back().expire=true;cases.back().expected_hold=true;
    add("close_braking","진입 직후 근접 차량",close,false);cases.back().expected_hold=true;
  }
  return cases;
}
inline void observe_vehicle_case(avoid_v2::Grid & grid,const VehicleCase & scene,
  const avoid_v2::Course & route,const avoid_v2::Config & cfg,double t,unsigned frame)
{
  std::vector<avoid_v2::Ray> rays;const bool local=scene.expire&&frame>0;
  const double right=local?2.:scene.visible_until;
  for(double y=local?-1.:-8.;y<=(local?1.:8.)+.001;y+=.05){
    rays.push_back({{local?0.:-2.,y},{right,y},t,false});
  }
  grid.observe(rays,1000+frame*2);
  if(!local){
    rays.clear();const double length=cfg.front+cfg.rear;
    for(auto sd:scene.placements){const auto b=place_on_normal(route,sd);
      if(b.x-length/2>right){continue;}  // Vehicles beyond the observed region remain unknown.
      // Include every edge even when the vehicle dimension is not a step multiple.
      const int nx=int(std::ceil(length/.04)),ny=int(std::ceil(cfg.width/.04));
      for(int x=0;x<=nx;++x){for(int y=0;y<=ny;++y){
        const avoid_v2::Point p{b.x-length/2+length*x/nx,b.y-cfg.width/2+cfg.width*y/ny};
        rays.push_back({{p.x-.001,p.y},p,t,true});
      }}
    }
    grid.observe(rays,1001+frame*2);
  }
}
}
