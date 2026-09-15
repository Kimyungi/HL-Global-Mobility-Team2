#include "fixtures.hpp"
#include <limits>
using namespace fixture;
int main()
{
  Config c;c.validate();
  for(double bad:{0.,-1.,std::numeric_limits<double>::quiet_NaN()}) {
    auto invalid=c;invalid.brake_decel=bad;bool threw=false;
    try{invalid.validate();}catch(const std::invalid_argument &){threw=true;}
    check(threw,"reject invalid braking calibration");
  }
  PoseHistory history;State s;
  check(history.push({1.,{0,0,3.12,.6,.1}}),"first pose");
  check(history.push({1.1,{.06,0,-3.12,.6,.1}}),"second pose");
  check(history.at(1.05,s)&&std::fabs(s.x-.03)<1e-9&&std::fabs(s.yaw)>3.,"interpolation wraps heading");
  check(!history.at(1.2,s),"no future pose extrapolation");
  check(!history.push({1.05,{}}),"out-of-order pose rejected");
  check(history.push({1.5,{}})&&!history.at(1.3,s),"pose gap rejected");

  auto course=straight();Grid grid(c);grid.configure(course);grid.prepare(1.);
  check(!grid.clear({2,0,0,.6,0}),"unknown footprint rejected");
  fill(grid,1,1);
  check(grid.clear({2,0,0,.6,0}),"known free body accepted");
  check(!grid.clear({2,2.8,0,.6,0}),"body cannot cross course boundary");
  grid.observe({{{1,0},{5,0},1.2,false}},10);grid.prepare(1.2);
  check(std::fabs(grid.valid_until()-1.3)<1e-9,"new ray cannot renew old free-cell lease");
  grid.prepare(1.31);check(!grid.clear({2,0,0,.6,0}),"old free space expires");
  fill(grid,2,2);box(grid,3,0,.6,.6,2);
  check(grid.occupied({3,0}),"box occupies cells");
  grid.observe({{{1,0},{5,0},2.1,false}},3);grid.prepare(2.1);
  check(grid.occupied({3,0}),"single clear ray cannot erase remembered obstacle");
  grid.observe({{{1,0},{5,0},2.1,false}},3);
  check(grid.occupied({3,0}),"republication is not another clear generation");
  grid.observe({{{1,0},{5,0},2.1,false}},300);
  check(grid.occupied({3,0}),"new generation with same measurement time cannot clear twice");
  grid.observe({{{1,0},{5,0},2.2,false}},4);
  check(!grid.occupied({3,0}),"two new clear scans release cell");
  grid.observe({{{1,0},{3,0},2.3,true},{{1,0},{5,0},2.3,false}},5);
  check(grid.occupied({3,0}),"hit wins over clear ray in same scan");
  grid.observe({{{1,0},{5,0},1.9,false}},6);
  check(grid.occupied({3,0}),"old clear observation cannot erase newer obstacle");
  grid.prepare(3.);check(grid.occupied({3,0}),"occluded occupancy never times out to free");
  const auto snapshot=grid.debug_snapshot(3.);
  check(snapshot.cells.size()==size_t(snapshot.width*snapshot.height),"debug snapshot covers grid");
  const auto cell=[&](Point p){return snapshot.cells.at(
    size_t(int((p.y-snapshot.origin.y)/snapshot.resolution)*snapshot.width+
    int((p.x-snapshot.origin.x)/snapshot.resolution)));};
  check(cell({3,0})==2&&cell({8,0})==0,"snapshot preserves occupied memory and expired unknown");

  Planner planner(c);planner.set_course(course);planner.set_zone(true,true);
  check(std::fabs(planner.stopping_distance(.6)-.59)<1e-9,"stopping distance includes latency and margin");
  check(std::fabs(planner.stopping_distance(2.)-2.9)<1e-9,"speed squared braking contribution");
  Grid free(c);free.configure(course);fill(free,4,40);
  PlanTrace trace;trace.limit=3;
  auto plan=planner.plan({1,0,0,.6,0},free,4,40,&trace);
  std::printf("straight: valid=%d nodes=%zu ms=%.3f reason=%s\n",plan.valid,plan.expanded,plan.compute_ms,plan.reason.c_str());
  check(plan.valid&&plan.path.size()>10&&plan.reference.x>1.9,"straight produces forward preview");
  check(!plan.complete,"obstacle-free observation is not course completion");
  check(trace.edges.size()==3&&plan.expanded>0&&plan.phase==Phase::WALL_FOLLOW&&!plan.maneuver_active,
    "clear road uses wall midpoints without overtaking");
  check(!planner.plan({1,0,0,.6,0},free,4,40,&trace).valid,"duplicate scan rejected");
  check(trace.edges.empty()&&trace.counts[0]==0,"rejected input does not retain previous trace");
  Grid blocked(c);blocked.configure(course);fill(blocked,5,50);box(blocked,1.8,0,1.,5.,5);
  check(!planner.plan({1,0,0,.6,0},blocked,5,50).valid,"blocked braking footprint causes HOLD");

  auto bad_course=course;bad_course.boundary={{0,0},{3,3},{0,3},{3,0}};
  bool rejected=false;try{bad_course.validate();}catch(const std::invalid_argument &){rejected=true;}
  check(rejected,"self crossing boundary rejected");
  rejected=false;try{Grid invalid(c);invalid.configure(Course{});}catch(const std::invalid_argument &){rejected=true;}
  check(rejected,"empty course rejected without indexing boundary");
  check(!planner.braking_clear({1,0,0,1e30,0},free),"unbounded speed cannot overflow sample counts");
  auto limited=c;limited.budget_ms=.000001;Planner timed(limited);timed.set_course(course);timed.set_zone(true,true);
  const auto timeout=timed.plan({1,0,0,.6,0},free,4,41);
  check(!timeout.valid&&timeout.deadline_hit,"deadline has no unchecked fallback");

  Planner complete(c);complete.set_course(course);complete.set_zone(true,true);
  for(unsigned i=0;i<3;++i){
    const double t=6.+i*.1;fill(free,t,60+i);
    complete.set_zone(true,false);
    const auto done=complete.plan({12.5,0,0,.6,0},free,t,60+i);
    check(done.valid,"onward route remains valid at exit");
    check(done.complete==(i==2),"exit requires three independent aligned observations");
  }
  Planner yaw_bad(c);yaw_bad.set_course(course);yaw_bad.set_zone(true,true);fill(free,7,70);
  auto wrong=yaw_bad.plan({12.5,0,.7,.6,0},free,7,70);
  check(!wrong.complete,"cross track alone cannot complete return");
  std::printf("core: %d checks, %d failures\n",checks,failures);
  return failures?1:0;
}
