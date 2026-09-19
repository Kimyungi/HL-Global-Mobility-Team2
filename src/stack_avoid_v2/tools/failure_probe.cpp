// Bounded diagnostic sweep. Does not alter the planner or its safety margins.
#include "../test/same_vehicle_cases.hpp"
#include "../test/vehicle_case_oracle.hpp"
#include <iostream>
#include <iomanip>
using namespace avoid_v2;
struct Probe {synthetic::VehicleCase scene;double initial_speed;std::string family;double spacing;double cruise_speed{1.};};
int main(int argc,char ** argv)
{
  const bool trace_slow=argc==2&&std::string(argv[1])=="--trace-slow-center";
  if(argc>1&&!trace_slow){std::cerr<<"usage: failure_probe [--trace-slow-center]\n";return 2;}
  std::vector<Probe> probes;
  for(const auto & s:synthetic::same_vehicle_cases()){
    if(s.id.rfind("s_center_gap_",0)==0||s.id.rfind("straight_center_",0)==0){
      for(double speed:{.2,.4,.6,.8,1.}){probes.push_back({s,speed,"entry_speed",0});probes.push_back({s,speed,"cruise_speed",0,speed});}
    }
  }
  for(bool curved:{false,true})for(unsigned n:{2,3})for(int side:{-1,1}){
    for(double spacing:{1.,1.5,2.,2.5,3.5,4.5}){
      synthetic::VehicleCase s;s.curved=curved;s.id=std::string(curved?"s":"straight")+"_alternate_"+std::to_string(n)+"_"+(side>0?"lr":"rl");
      for(unsigned i=0;i<n;++i){s.placements.push_back({5.+spacing*i,side*(i%2?-.6:.6)});}
      probes.push_back({s,1.,"longitudinal_spacing",spacing});
    }
  }
  unsigned unsafe_transitions=0;
  std::cout<<"family,scene,initial_speed,cruise_speed,spacing,cars,result,time,x,y,speed_min,vehicle_contact,curb_contact,max_ms,next_braking_failures,reason\n"<<std::setprecision(10);
  for(const auto & probe:probes){
    if(trace_slow&&(probe.family!="cruise_speed"||probe.scene.id!="s_center_gap_3"||probe.cruise_speed!=.2)){continue;}
    auto cfg=synthetic::vehicle_case_config();cfg.cruise_speed=probe.cruise_speed;cfg.validate();const auto & s=probe.scene;
    const auto route=synthetic::vehicle_case_course(s);Grid grid(cfg);grid.configure(route);
    Planner planner(cfg);planner.set_course(route);std::vector<Point> boxes;
    for(auto sd:s.placements){boxes.push_back(synthetic::place_on_normal(route,sd));}
    bool fits=true;
    for(auto b:boxes){fits&=synthetic::body_inside_curbs({b.x-(cfg.front-cfg.rear)/2,b.y,0,0,0},cfg,route);}
    State car{1,s.curved?1.1*std::sin(.35):0,s.curved?std::atan(1.1*.35*std::cos(.35)):0,probe.initial_speed,0};
    synthetic::VehicleCaseAudit audit;std::string result="timeout",reason;
    unsigned frame=0,next_braking_failures=0;double minimum=car.speed,maximum_ms=0;
    if(!fits){result="invalid_fixture";}else for(;frame<1800;++frame){
      const double now=1.+frame*.1;
      planner.set_zone(true,car.x>=route.at(route.entry).x-.001&&car.x<=route.at(route.exit).x);
      if(planner.perception_required()){synthetic::observe_vehicle_case(grid,s,route,cfg,now,frame);}
      const auto plan=planner.plan(car,grid,now,1000+frame);reason=plan.reason;
      maximum_ms=std::max(maximum_ms,plan.compute_ms);minimum=std::min(minimum,car.speed);
      audit.inspect(car,cfg,route,boxes);
      for(auto p:plan.path){audit.inspect(p,cfg,route,boxes);}
      if(plan.valid){for(double t=0;t<=.100001;t+=.01){audit.inspect(sample_path_time(plan.path,t),cfg,route,boxes);}}
      if(audit.vehicle_contact||audit.curb_contact){result="contact";break;}
      if(plan.complete){result="complete";break;}
      if(!plan.valid){result="hold";break;}
      const auto next=sample_path_time(plan.path,.1);
      const bool next_brakeable=planner.braking_clear(next,grid);
      next_braking_failures+=!next_brakeable;unsafe_transitions+=!next_brakeable;
      if(trace_slow&&!next_brakeable){
        std::cerr<<std::setprecision(10)<<"next frame braking already blocked in unchanged current grid: frame="<<frame
          <<" time="<<frame*.1<<" plan_valid="<<plan.valid<<" reused="<<plan.reused
          <<" current_braking_clear="<<planner.braking_clear(car,grid)<<" next_body_clear="<<grid.clear(next)
          <<" next_x="<<next.x<<" next_y="<<next.y<<" next_speed="<<next.speed<<" next_steer="<<next.steer<<'\n';
      }
      car=next;
    }
    std::cout<<probe.family<<','<<s.id<<','<<probe.initial_speed<<','<<probe.cruise_speed<<','<<probe.spacing<<','<<boxes.size()<<','<<result<<','<<frame*.1<<','<<car.x<<','<<car.y<<','<<minimum<<','<<audit.vehicle_contact<<','<<audit.curb_contact<<','<<maximum_ms<<','<<next_braking_failures<<','<<reason<<'\n'<<std::flush;
  }
  return unsafe_transitions?1:0;
}
