// Export actual C++ planner results for a standalone, read-only HTML replay.
#include "stack_avoid_v2/core.hpp"
#include "../test/same_vehicle_cases.hpp"
#include "../test/vehicle_case_oracle.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

using namespace avoid_v2;
namespace
{
struct Box {double x,y,length,width;};
using Scene=synthetic::VehicleCase;
std::string quote(const std::string & s)
{
  std::ostringstream out;out<<'"';
  for(char c:s){if(c=='"'||c=='\\'){out<<'\\'<<c;}else if(c=='\n'){out<<"\\n";}else{out<<c;}}
  out<<'"';return out.str();
}
void number(std::ostream & out,double d){if(std::isfinite(d)){out<<d;}else{out<<"null";}}
void xy(std::ostream & out,Point p){out<<'[';number(out,p.x);out<<',';number(out,p.y);out<<']';}
void state(std::ostream & out,State p)
{
  // Retained trajectories can start partway through a millimetre-scale segment.
  // Keep enough digits to independently reconstruct acceleration from v²/2ds.
  const auto precision=out.precision();out<<std::setprecision(10);
  out<<'['<<p.x<<','<<p.y<<','<<p.yaw<<','<<p.speed<<','<<p.steer<<']';
  out.precision(precision);
}
void points(std::ostream & out,const std::vector<Point> & ps)
{out<<'[';bool first=true;for(auto p:ps){if(!first){out<<',';}first=false;xy(out,p);}out<<']';}
void snapshot(std::ostream & out,const GridSnapshot & s)
{
  out<<"{\"origin\":";xy(out,s.origin);out<<",\"res\":"<<s.resolution<<",\"width\":"<<s.width
    <<",\"height\":"<<s.height<<",\"rle\":[";
  bool first=true;
  for(size_t i=0;i<s.cells.size();){size_t j=i+1;while(j<s.cells.size()&&s.cells[i]==s.cells[j]){++j;}
    if(!first){out<<',';}first=false;out<<j-i<<','<<unsigned(s.cells[i]);i=j;}
  out<<"]}";
}
void generate(std::ostream & out,const Scene & scene)
{
  const Config cfg=synthetic::vehicle_case_config();const auto route=synthetic::vehicle_case_course(scene);Grid grid(cfg);grid.configure(route);Planner planner(cfg);planner.set_course(route);
  const double yaw=scene.curved?std::atan(1.1*.35*std::cos(.35)):0;
  State car{1,scene.curved?1.1*std::sin(.35):0,yaw,cfg.cruise_speed,0};
  std::vector<Box> boxes;std::vector<Point> centers;
  for(auto sd:scene.placements){const auto c=synthetic::place_on_normal(route,sd);centers.push_back(c);boxes.push_back({c.x,c.y,cfg.front+cfg.rear,cfg.width});}
  synthetic::VehicleCaseAudit audit;
  std::vector<GridSnapshot> maps;std::ostringstream frames;frames<<std::setprecision(6);
  unsigned count=0,holds=0;bool completed=false;double worst=0;
  const unsigned limit=scene.expire?7:1800;
  for(unsigned i=0;i<limit;++i){
    const double t=1.+i*.1;PlanTrace trace;
    const bool in_zone=car.x>=route.at(route.entry).x-.001&&car.x<=route.at(route.exit).x;
    planner.set_zone(true,in_zone);
    if(planner.perception_required()){synthetic::observe_vehicle_case(grid,scene,route,cfg,t,i);}
    const auto plan=planner.plan(car,grid,t,1000+i,&trace);const auto view=grid.debug_snapshot(t);
    audit.inspect(car,cfg,route,centers);
    for(const auto & p:plan.path){audit.inspect(p,cfg,route,centers);}
    if(plan.valid){for(double t=0;t<=.1+1e-8;t+=.01){audit.inspect(sample_path_time(plan.path,t),cfg,route,centers);}}
    if(maps.empty()||view.cells!=maps.back().cells){maps.push_back(view);}
    if(i){frames<<',';}++count;worst=std::max(worst,plan.compute_ms);
    frames<<"{\"time\":"<<i*.1<<",\"ego\":";state(frames,car);
    frames<<",\"valid\":"<<(plan.valid?"true":"false")<<",\"complete\":"<<(plan.complete?"true":"false")
      <<",\"inZone\":"<<(in_zone?"true":"false")<<",\"perception\":"<<(plan.perception_active?"true":"false")
      <<",\"detected\":"<<(plan.obstacle_detected?"true":"false")<<",\"maneuver\":"<<(plan.maneuver_active?"true":"false")
      <<",\"gpsFollow\":"<<(plan.gps_follow?"true":"false")
      <<",\"vehicleContact\":"<<(audit.vehicle_contact?"true":"false")
      <<",\"curbContact\":"<<(audit.curb_contact?"true":"false")
      <<",\"speedLimit\":"<<plan.speed_limit
      <<",\"phase\":"<<unsigned(plan.phase)<<",\"deadline\":"<<(plan.deadline_hit?"true":"false")
      <<",\"reason\":"<<quote(plan.reason)<<",\"ms\":"<<plan.compute_ms<<",\"station\":"<<plan.station
      <<",\"stop\":"<<plan.stop_distance<<",\"clearance\":";number(frames,plan.min_clearance);
    frames<<",\"nodes\":"<<plan.expanded<<",\"map\":"<<maps.size()-1<<",\"reference\":";
    if(plan.valid){state(frames,plan.reference);}else{frames<<"null";}
    frames<<",\"path\":[";bool first=true;for(auto p:plan.path){if(!first){frames<<',';}first=false;state(frames,p);}
    frames<<"],\"counts\":[";first=true;for(auto n:trace.counts){if(!first){frames<<',';}first=false;frames<<n;}
    frames<<"],\"trace\":[";first=true;for(const auto & edge:trace.edges){
      if(!first){frames<<',';}first=false;frames<<'['<<edge.start.x<<','<<edge.start.y<<','
        <<edge.middle.x<<','<<edge.middle.y<<','<<edge.end.x<<','<<edge.end.y<<','<<unsigned(edge.result)<<']';
    }
    frames<<"],\"wallLeft\":";points(frames,trace.wall_left);
    frames<<",\"wallRight\":";points(frames,trace.wall_right);
    frames<<",\"midpoints\":";points(frames,trace.midpoints);
    frames<<",\"corridor\":";points(frames,trace.corridor);frames<<'}';
    if(audit.vehicle_contact||audit.curb_contact){break;}
    if(plan.complete){completed=true;break;}
    if(!plan.valid&&plan.phase==Phase::HOLD){++holds;break;}
    if(!plan.valid&&plan.phase==Phase::READY&&scene.moving){car.x+=car.speed*.1;}
    if(scene.moving&&plan.valid&&plan.path.size()>=3){
      car=sample_path_time(plan.path,.1);
    }
  }
  out<<"{\"id\":"<<quote(scene.id)<<",\"title\":"<<quote(scene.title)<<",\"description\":"<<quote(scene.description)
    <<",\"curved\":"<<(scene.curved?"true":"false")<<",\"moving\":"<<(scene.moving?"true":"false")
    <<",\"expired\":"<<(scene.expire?"true":"false")<<",\"visibleUntil\":"<<scene.visible_until
    <<",\"center\":";points(out,route.center);out<<",\"boundary\":";points(out,route.boundary);
  out<<",\"entry\":"<<route.entry<<",\"exit\":"<<route.exit<<",\"exitPoint\":";xy(out,route.at(route.exit));
  out<<",\"laneWidth\":"<<scene.corridor_width;
  out<<",\"placements\":";points(out,scene.placements);
  out<<",\"curbs\":[";bool first_curb=true;
  for(size_t i=0;i<route.boundary.size();++i){const auto a=route.boundary[i],b=route.boundary[(i+1)%route.boundary.size()];
    if(i+1==route.boundary.size()/2||i+1==route.boundary.size()){continue;} // End caps, not roadside curbs.
    if(!first_curb){out<<',';}first_curb=false;out<<'['<<a.x<<','<<a.y<<','<<b.x<<','<<b.y<<']';
  }out<<']';
  out<<",\"boxes\":[";bool first=true;for(auto b:boxes){if(!first){out<<',';}first=false;out<<'['<<b.x<<','<<b.y<<','<<b.length<<','<<b.width<<']';}
  out<<"],\"maps\":[";first=true;for(const auto & map:maps){if(!first){out<<',';}first=false;snapshot(out,map);}
  out<<"],\"frames\":["<<frames.str()<<"],\"audit\":{\"vehicleContact\":"<<(audit.vehicle_contact?"true":"false")
    <<",\"curbContact\":"<<(audit.curb_contact?"true":"false")<<",\"samples\":"<<audit.samples
    <<"},\"expectedHold\":"<<(scene.expected_hold?"true":"false")<<",\"outcome\":"<<quote(audit.vehicle_contact||audit.curb_contact?"unsafe":completed?"complete":holds?"hold":"limit")<<'}';
  std::cout<<scene.id<<": frames="<<count<<" complete="<<completed<<" hold="<<holds<<" max_ms="<<worst<<'\n';
}
}
int main(int argc,char ** argv)
{
  if(argc!=2){std::cerr<<"usage: avoid_v2_export output.json\n";return 2;}
  try{
    const auto scenes=synthetic::same_vehicle_cases();
    std::ofstream out(argv[1]);if(!out){throw std::runtime_error("cannot write export");}
    const Config cfg=synthetic::vehicle_case_config();
    out<<std::setprecision(6)<<"{\"schema\":4,\"algorithm\":\"wall_midpoint_corridor\",\"stepSeconds\":0.1,\"model\":{\"width\":"<<cfg.width
      <<",\"front\":"<<cfg.front<<",\"rear\":"<<cfg.rear<<",\"margin\":"<<cfg.margin
      <<",\"wheelbase\":"<<cfg.wheelbase<<",\"maxSteer\":"<<cfg.max_steer<<",\"preview\":"<<cfg.preview
      <<",\"horizon\":"<<cfg.horizon<<",\"budget\":"<<cfg.budget_ms
      <<",\"adaptiveSpeed\":true,\"crawlSpeed\":"<<cfg.crawl_speed
      <<",\"acceleration\":"<<cfg.acceleration<<",\"deceleration\":"<<cfg.planning_decel
      <<",\"steerRate\":"<<cfg.steer_rate
      <<",\"cruiseSpeed\":"<<cfg.cruise_speed
      <<",\"curbPlacement\":\"road_edges\",\"traceLimit\":"<<PlanTrace{}.limit<<"},\"scenarios\":[";
    for(size_t i=0;i<scenes.size();++i){if(i){out<<',';}generate(out,scenes[i]);}
    out<<"]}\n";if(!out){throw std::runtime_error("export write failed");}
  }catch(const std::exception & e){std::cerr<<e.what()<<'\n';return 1;}
  return 0;
}
