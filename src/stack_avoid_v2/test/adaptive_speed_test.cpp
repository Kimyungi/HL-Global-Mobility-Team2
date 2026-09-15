#include "same_vehicle_cases.hpp"
#include "vehicle_case_oracle.hpp"
using namespace fixture;
int main()
{
  auto cfg=synthetic::vehicle_case_config();
  const std::vector<State> braking{{0,0,0,1,0},{.375,0,0,.5,0}};
  const auto timed=sample_path_time(braking,.25);
  check(std::fabs(timed.x-.21875)<1e-9&&std::fabs(timed.speed-.75)<1e-9,"timed replay integrates acceleration, not constant distance");
  auto bad=cfg;bad.planning_decel=2.;bool rejected=false;
  try{bad.validate();}catch(const std::invalid_argument &){rejected=true;}
  check(rejected,"planning deceleration cannot exceed calibrated braking");
  {
    auto course=straight(3.);Grid grid(cfg);grid.configure(course);Planner planner(cfg);
    planner.set_course(course);planner.set_zone(true,true);fill(grid,1.,1);
    const State initial{1,0,0,1,0};const auto first=planner.plan(initial,grid,1.,1);
    check(first.valid,"initial speed profile exists");
    if(first.valid){
      const auto current=sample_path_time(first.path,.1);fill(grid,1.1,2);
      const auto continued=planner.plan(current,grid,1.1,2);
      check(continued.valid&&continued.reused,"fresh scan preserves an executable speed profile");
      box(grid,current.x+.6,current.y,.85,.62,1.2);
      const auto blocked=planner.plan(current,grid,1.2,3);
      check(!blocked.valid&&blocked.path.empty(),"new occupied wall invalidates retained trajectory");
    }
    Planner stopped(cfg);stopped.set_course(course);stopped.set_zone(true,true);
    Grid free(cfg);free.configure(course);fill(free,2.,10);
    const auto launch=stopped.plan({1,0,0,0,0},free,2.,10);
    check(launch.valid&&launch.path.front().speed==0,"profile can start from measured standstill");
    if(launch.valid){check(sample_path_time(launch.path,.1).speed<=cfg.acceleration*.1+1e-5,"standstill launch respects acceleration");}
  }
  // Constant wall separation is measured between faces, not vehicle centers.
  for(double gap:{.64,.8,1.0,1.1,1.2,1.3}){
    auto course=straight(3.);course.exit=10.;course.validate();Grid grid(cfg);grid.configure(course);Planner planner(cfg);planner.set_course(course);
    State car{1,0,0,1,0};bool complete=false,hold=false;double minimum=1;unsigned frames=0;
    for(unsigned i=0;i<1000;++i){double now=1.+i*.1;fill(grid,now,1000+i*2);
      std::vector<Ray> rays;
      for(double x=4;x<=7.00001;x+=.04){for(double y:{-gap/2,gap/2})rays.push_back({{x-.001,y},{x,y},now,true});}
      grid.observe(rays,1001+i*2);planner.set_zone(true,car.x<=course.exit);
      const auto plan=planner.plan(car,grid,now,1000+i);++frames;minimum=std::min(minimum,car.speed);
      if(!plan.valid){hold=true;break;}
      check(std::fabs(plan.path.front().speed-car.speed)<1e-9,"planned speed starts at measured speed");
      check(plan.speed_limit>=cfg.crawl_speed&&plan.speed_limit<=cfg.cruise_speed,"bounded adaptive speed target");
      for(size_t j=1;j<plan.path.size();++j){const auto a=plan.path[j-1],b=plan.path[j];
        const double ds=std::hypot(b.x-a.x,b.y-a.y),dt=2*ds/(a.speed+b.speed),accel=(b.speed*b.speed-a.speed*a.speed)/(2*ds);
        check(accel>=-cfg.planning_decel-.0001&&accel<=cfg.acceleration+.0001,"longitudinal acceleration limits preserved");
        check(std::fabs(b.steer-a.steer)<=cfg.steer_rate*dt+1e-6,"steering slew cannot be increased to force a fit");
        check(grid.clear(b),"speed reduction never disables footprint checks");
      }
      if(plan.complete){complete=true;break;}
      car=sample_path_time(plan.path,.1);
    }
    if(gap<cfg.width+2*cfg.margin){check(hold&&!complete,"a gap wider than the body can still violate required margin");}
    if(gap>=1.2){check(complete&&minimum<.3,"sub-1.3m parallel passage completes at crawl speed");}
    std::printf("gap=%.2f complete=%d hold=%d min_speed=%.3f time=%.1f\n",gap,complete,hold,minimum,(frames-1)*.1);
  }
  std::printf("adaptive speed: checks=%d failures=%d\n",checks,failures);
  return failures?1:0;
}
