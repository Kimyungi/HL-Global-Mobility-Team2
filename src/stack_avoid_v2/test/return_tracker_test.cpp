#include "fixtures.hpp"
#include "stack_avoid_v2/return_tracker.hpp"
using namespace fixture;
int main(){
  Config cfg;auto c=straight();Grid grid(cfg);grid.configure(c);fill(grid,1,1);
  ReturnTracker tracker;Plan p;p.valid=true;p.station=5;State car{5,0,0,.6,0};
  auto update=[&](){tracker.update(p,car,c,grid,cfg,1.);};
  for(int i=0;i<5;++i){update();}
  check(!tracker.done(),"clear GPS before any obstacle cannot complete zone");
  p.obstacle_detected=true;update();p.obstacle_detected=false;
  update();update();check(!tracker.done(),"two independent clear returns do not complete");
  update();check(tracker.done(),"three aligned observed onward paths complete");
  p.valid=false;update();check(!tracker.done(),"failed plan revokes completion");p.valid=true;
  car.y=.11;for(int i=0;i<4;++i){update();}check(!tracker.done(),"cross-track error prevents return");car.y=0;
  car.yaw=.36;for(int i=0;i<4;++i){update();}check(!tracker.done(),"heading error prevents return");car.yaw=0;
  box(grid,6,0);for(int i=0;i<4;++i){update();}check(!tracker.done(),"occupied onward GPS path prevents return");
  grid.configure(c);for(int i=0;i<4;++i){update();}check(!tracker.done(),"unknown onward GPS path prevents return");
  fill(grid,2,2);tracker.reset();for(int i=0;i<4;++i){update();}check(!tracker.done(),"new episode cannot reuse obstacle evidence");
  std::printf("return_tracker_test: %d checks, %d failures\n",checks,failures);return failures?1:0;
}
