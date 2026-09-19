#pragma once
#include "stack_avoid_v2/core.hpp"
#include <algorithm>
#include <cmath>

namespace avoid_v2 {
// Completion evidence for the existing MGM zone state machine. Geographic
// membership/CSV endpoint alone is never a completion signal.
class ReturnTracker {
public:
  void reset(){seen_=done_=false;count_=0;}
  void invalidate(){done_=false;count_=0;}
  bool done() const{return done_;}
  void update(const Plan & p,const State & pose,const Course & c,const Grid & grid,
    const Config & cfg,double stopping_distance){
    seen_=seen_||p.obstacle_detected;
    if(!p.valid||p.obstacle_detected||!seen_){invalidate();return;}
    const auto at=c.project({pose.x,pose.y},std::max(0.,p.station-.5),p.station+.5);
    bool clear=std::fabs(at.cross)<=cfg.cross_tolerance&&std::fabs(wrap(pose.yaw-at.yaw))<=cfg.yaw_tolerance;
    const double distance=std::max(cfg.preview,stopping_distance);
    clear=clear&&at.station+distance<c.length();
    for(double d=0;clear&&d<=distance;d+=cfg.sample_step){
      const auto q=c.at(at.station+d);const auto pr=c.project(q,at.station+d-.01,at.station+d+.01);
      clear=grid.clear({q.x,q.y,pr.yaw,0,0},cfg.sample_step);
    }
    count_=clear?count_+1:0;done_=count_>=cfg.completion_samples;
  }
private:
  bool seen_{},done_{};unsigned count_{};
};
}
