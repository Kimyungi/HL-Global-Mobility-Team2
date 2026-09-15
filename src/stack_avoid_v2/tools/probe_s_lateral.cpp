// Diagnostic: compare blocked central S corridor with independent rectangle checks.
#include "../test/fixtures.hpp"
#include "../test/s_curve_layouts.hpp"
using namespace fixture;
int main(){
 Config cfg;auto route=synthetic::s_curve();auto layout=synthetic::s_curve_layouts().back();
 Grid grid(cfg);grid.configure(route);Planner planner(cfg);planner.set_course(route);
 State car{1,1.1*std::sin(.35),std::atan(1.1*.35*std::cos(.35)),.6,0};
 for(unsigned i=0;i<100;++i){double t=1.+i*.1;fill(grid,t,1000+i);
  for(auto sd:layout.station_offset){auto b=synthetic::place_on_normal(route,sd);box(grid,b.x,b.y,.7,.65,t);}
  planner.set_zone(true,true);auto plan=planner.plan(car,grid,t,1000+i);
  if(plan.valid){car=sample_path(plan.path,.06);continue;}
  printf("HOLD frame=%u t=%.1f x=%.4f y=%.4f yaw=%.4f steer=%.4f reason=%s trigger=%d\n",i,t-1,car.x,car.y,car.yaw,car.steer,plan.reason.c_str(),plan.obstacle_detected);
  Grid free(cfg);free.configure(route);fill(free,t,1000+i);Planner gps(cfg);gps.set_course(route);gps.set_zone(true,true);
  auto desired=gps.plan(car,free,t,1000+i);double arc=0;bool actual_collision=false;unsigned rejected=0;bool brake_collision=false;
  for(size_t j=0;j<desired.path.size();++j){auto q=desired.path[j];
   if(j){arc+=std::hypot(q.x-desired.path[j-1].x,q.y-desired.path[j-1].y);}
   for(auto sd:layout.station_offset){auto b=synthetic::place_on_normal(route,sd);actual_collision|=touches_box(q,cfg,b.x,b.y);}
   const double k=std::tan(q.steer)/cfg.wheelbase;
   for(double d=0;d<=planner.stopping_distance(q.speed);d+=.01){
    State stop=q;
    stop.x+=std::fabs(k)<1e-8?d*std::cos(q.yaw):(std::sin(q.yaw+k*d)-std::sin(q.yaw))/k;
    stop.y+=std::fabs(k)<1e-8?d*std::sin(q.yaw):(std::cos(q.yaw)-std::cos(q.yaw+k*d))/k;
    stop.yaw=wrap(q.yaw+k*d);
    for(auto sd:layout.station_offset){auto b=synthetic::place_on_normal(route,sd);brake_collision|=touches_box(stop,cfg,b.x,b.y);}
   }
   if(!planner.braking_clear(q,grid,.06)&&rejected++<5){printf("GPS sample arc=%.2f x=%.3f y=%.3f body_clearance=%.4f body_clear=%d brake_clear=%d brake_with_extra=%d\n",arc,q.x,q.y,grid.clearance(q),grid.clear(q),planner.braking_clear(q,grid),planner.braking_clear(q,grid,.06));}
  }
  printf("GPS desired valid=%d analytic_vehicle_collision=%d braking_rejected_samples=%u analytic_brake_collision=%d\n",desired.valid,actual_collision,rejected,brake_collision);
  break;
 }
}
