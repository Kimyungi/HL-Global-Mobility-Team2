#include "same_vehicle_cases.hpp"
#include "vehicle_case_oracle.hpp"
using namespace fixture;

int main()
{
  const auto cfg=synthetic::vehicle_case_config();const double dt=.1;
  check(cfg.cruise_speed==1.&&cfg.adaptive_speed,"adaptive speed has 1m/s cruise ceiling");
  Planner model(cfg);check(std::fabs(model.stopping_distance(1.)-1.05)<1e-9,"1m/s braking includes observation and processing delays");
  unsigned completed=0,expected_stops=0,failed=0,total=0;
  for(const auto & scene:synthetic::same_vehicle_cases()){
    const auto route=synthetic::vehicle_case_course(scene);std::vector<Point> boxes;
    check(scene.placements.size()==2||scene.placements.size()==3,"every driving scenario has two or three same-size vehicles");
    for(auto sd:scene.placements){boxes.push_back(synthetic::place_on_normal(route,sd));}
    check(scene.corridor_width==3.,"single lane width is 3m");
    for(auto box:boxes){
      // Boxes are centered; State uses the rear axle.
      check(synthetic::body_inside_curbs({box.x-(cfg.front-cfg.rear)/2,box.y,0,0,0},cfg,route),
        "every fixed vehicle must fit wholly inside the narrower curbs");
    }
    for(unsigned trial=0;trial<3;++trial){
      ++total;Grid grid(cfg);grid.configure(route);Planner planner(cfg);planner.set_course(route);
      State car{1,scene.curved?1.1*std::sin(.35):0,
        scene.curved?std::atan(1.1*.35*std::cos(.35)):0,cfg.cruise_speed,0};
      bool done=false,hold=false;unsigned frames=0;double maximum_ms=0,minimum_speed=car.speed,maximum_speed=car.speed;std::string reason;
      synthetic::VehicleCaseAudit audit;
      for(unsigned i=0;i<1800;++i){
        const double t=1.+i*dt;
        planner.set_zone(true,car.x>=route.at(route.entry).x-.001&&car.x<=route.at(route.exit).x);
        if(planner.perception_required()){synthetic::observe_vehicle_case(grid,scene,route,cfg,t,i);}
        PlanTrace trace;const auto plan=planner.plan(car,grid,t,1000+i,&trace);
        ++frames;reason=plan.reason;maximum_ms=std::max(maximum_ms,plan.compute_ms);
        minimum_speed=std::min(minimum_speed,car.speed);maximum_speed=std::max(maximum_speed,car.speed);
        audit.inspect(car,cfg,route,boxes);
        if(!plan.valid&&plan.phase==Phase::HOLD){hold=true;break;}
        for(const auto & p:plan.path){audit.inspect(p,cfg,route,boxes);}
        if(plan.complete){done=true;break;}
        if(plan.valid){
          for(double t=0;t<=dt+1e-8;t+=.01){audit.inspect(sample_path_time(plan.path,t),cfg,route,boxes);}
          car=sample_path_time(plan.path,dt);
        }else if(plan.phase==Phase::READY){car.x+=car.speed*dt;}
      }
      completed+=done;expected_stops+=scene.expected_hold&&hold;
      const bool correct=(scene.expected_hold?hold:done)&&!audit.vehicle_contact&&!audit.curb_contact;
      failed+=!correct;
      std::printf("%s trial=%u cars=%zu cruise=1.00 speed_min=%.3f speed_max=%.3f size=%.2fx%.2f lane_width=3.00 complete=%d hold=%d expected_hold=%d time=%.1f vehicle_contact=%d curb_contact=%d samples=%u max_ms=%.3f x=%.4f y=%.4f reason=%s\n",
        scene.id.c_str(),trial+1,boxes.size(),minimum_speed,maximum_speed,cfg.front+cfg.rear,cfg.width,done,hold,scene.expected_hold,
        (frames-1)*dt,audit.vehicle_contact,audit.curb_contact,audit.samples,maximum_ms,car.x,car.y,reason.c_str());
      check(correct,"1m/s two/three-car case must complete or perform its expected diagnostic HOLD without contacts");
      std::fflush(stdout);
    }
  }
  std::printf("same vehicle matrix: trials=%u completed=%u expected_stops=%u failed=%u checks=%d failures=%d\n",
    total,completed,expected_stops,failed,checks,failures);
  return failures?1:0;
}
