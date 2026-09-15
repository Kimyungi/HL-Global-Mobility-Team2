#include "same_vehicle_cases.hpp"
#include "fixtures.hpp"
using namespace fixture;
int main()
{
  unsigned valid_frames=0,reused_frames=0;
  for(auto scene:synthetic::same_vehicle_cases()){
    if(scene.id!="s_center_gap_2"&&scene.id!="s_center_gap_3"&&scene.id!="straight_center_2"){continue;}
    for(double cruise:{.2,1.}){
      auto cfg=synthetic::vehicle_case_config();cfg.cruise_speed=cruise;
      const auto route=synthetic::vehicle_case_course(scene);Grid grid(cfg);grid.configure(route);
      Planner planner(cfg);planner.set_course(route);
      State car{1,scene.curved?1.1*std::sin(.35):0,scene.curved?std::atan(1.1*.35*std::cos(.35)):0,cruise,0};
      unsigned frame=0;
      for(;frame<1800;++frame){
        const double now=1.+frame*.1;
        synthetic::observe_vehicle_case(grid,scene,route,cfg,now,frame);
        planner.set_zone(true,car.x<=route.at(route.exit).x);
        const auto plan=planner.plan(car,grid,now,1000+frame);
        if(!plan.valid){break;}
        ++valid_frames;reused_frames+=plan.reused;
        // Consumer times deliberately differ from the planner's distance samples.
        // Freeze the observation grid: failures here cannot be blamed on a new scan.
        for(unsigned tick=0;tick<=30;++tick){
          const auto executed=sample_path_time(plan.path,tick*.005);
          check(grid.clear(executed),"valid path keeps body clear at consumer times");
          check(planner.braking_clear(executed,grid),"valid path remains brakeable at consumer times in the same grid");
        }
        if(plan.complete){break;}
        car=sample_path_time(plan.path,.1);
      }
      std::printf("execution safety: %s cruise=%.1f frames=%u\n",scene.id.c_str(),cruise,frame);
    }
  }
  check(valid_frames>100&&reused_frames>100,"execution checks cover moving and retained trajectories");
  std::printf("execution safety: valid_frames=%u reused_frames=%u checks=%d failures=%d\n",valid_frames,reused_frames,checks,failures);
  return failures?1:0;
}
