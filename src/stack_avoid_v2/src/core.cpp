#include "stack_avoid_v2/core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace avoid_v2
{
namespace
{
constexpr double pi = 3.14159265358979323846;
double distance(Point a, Point b) {return std::hypot(a.x-b.x, a.y-b.y);}
double cross(Point a, Point b, Point c)
{return (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);}
bool intersects(Point a, Point b, Point c, Point d)
{
  const double x = cross(a,b,c), y = cross(a,b,d), z = cross(c,d,a), w = cross(c,d,b);
  if (std::max(a.x,b.x) < std::min(c.x,d.x) || std::max(c.x,d.x) < std::min(a.x,b.x) ||
    std::max(a.y,b.y) < std::min(c.y,d.y) || std::max(c.y,d.y) < std::min(a.y,b.y)) {return false;}
  return x*y <= 0 && z*w <= 0;
}
bool inside(Point p, const std::vector<Point> & polygon)
{
  bool result = false;
  for (size_t i=0,j=polygon.size()-1;i<polygon.size();j=i++) {
    const auto a=polygon[i], b=polygon[j];
    if ((a.y>p.y)!=(b.y>p.y) && p.x<(b.x-a.x)*(p.y-a.y)/(b.y-a.y)+a.x) {result=!result;}
  }
  return result;
}
bool valid_point(Point p) {return std::isfinite(p.x) && std::isfinite(p.y);}
State interpolate(State a, State b, double t)
{
  return {a.x+t*(b.x-a.x), a.y+t*(b.y-a.y), wrap(a.yaw+t*wrap(b.yaw-a.yaw)),
    a.speed+t*(b.speed-a.speed), a.steer+t*(b.steer-a.steer)};
}
}
double wrap(double a) {return std::atan2(std::sin(a), std::cos(a));}
bool finite(const State & s)
{
  return std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.yaw) &&
    std::isfinite(s.speed) && std::isfinite(s.steer);
}
State sample_path(const std::vector<State> & path,double length)
{
  if(path.empty()||!std::isfinite(length)||length<0){throw std::invalid_argument("invalid path sample");}
  for(size_t i=1;i<path.size();++i){
    const double ds=distance({path[i-1].x,path[i-1].y},{path[i].x,path[i].y});
    if(ds>1e-9&&length<=ds){auto out=interpolate(path[i-1],path[i],length/ds);
      out.speed=std::sqrt(std::max(0.,path[i-1].speed*path[i-1].speed+(path[i].speed*path[i].speed-path[i-1].speed*path[i-1].speed)*length/ds));return out;}
    length-=ds;
  }
  return path.back();
}
State sample_path_time(const std::vector<State> & path,double seconds)
{
  if(path.empty()||!std::isfinite(seconds)||seconds<0){throw std::invalid_argument("invalid timed path sample");}
  for(size_t i=1;i<path.size();++i){const auto a=path[i-1],b=path[i];
    const double ds=distance({a.x,a.y},{b.x,b.y}),sum=a.speed+b.speed;
    if(ds<1e-9){continue;}
    if(sum<=0){return a;}
    const double dt=2*ds/sum;
    if(seconds<=dt){const double acceleration=(b.speed-a.speed)/dt;
      auto out=interpolate(a,b,std::clamp((a.speed*seconds+.5*acceleration*seconds*seconds)/ds,0.,1.));
      out.speed=a.speed+acceleration*seconds;return out;}
    seconds-=dt;
  }
  return path.back();
}
bool PoseHistory::push(TimedState s)
{
  if (!std::isfinite(s.time) || s.time <= 0 || !finite(s.state) ||
    (!samples_.empty() && s.time <= samples_.back().time)) {return false;}
  samples_.push_back(s);
  while (samples_.size()>2 && samples_.front().time < s.time-3.) {samples_.pop_front();}
  return true;
}
bool PoseHistory::at(double t, State & s, double gap) const
{
  if (samples_.empty() || !std::isfinite(t) || t < samples_.front().time ||
    t > samples_.back().time) {return false;}
  auto b = std::lower_bound(samples_.begin(),samples_.end(),t,
    [](const TimedState & a,double time) {return a.time<time;});
  if (b == samples_.end()) {return false;}
  if (b->time == t) {s=b->state; return true;}
  if (b == samples_.begin()) {return false;}
  const auto a=std::prev(b);
  if (b->time-a->time>gap) {return false;}
  s=interpolate(a->state,b->state,(t-a->time)/(b->time-a->time));
  return true;
}
void Config::validate() const
{
  const double positive[] = {resolution,free_ttl,width,front,rear,wheelbase,max_steer,
    steer_rate,steer_tau,cruise_speed,brake_decel,lateral_accel,preview,horizon,step,
    sample_step,cross_tolerance,yaw_tolerance,budget_ms,crawl_speed,acceleration,planning_decel};
  for (double v:positive) {if (!std::isfinite(v)||v<=0) {throw std::invalid_argument("invalid positive planner parameter");}}
  for (double v:{margin,brake_delay,observation_gap,processing_delay,longitudinal_margin}) {
    if (!std::isfinite(v)||v<0) {throw std::invalid_argument("invalid nonnegative planner parameter");}
  }
  if (crawl_speed>cruise_speed||planning_decel>brake_decel||max_steer>=pi/2 || sample_step>step || sample_step>.1 || resolution>.2 ||
    resolution<.02 || !completion_samples || max_nodes<10 || max_cells<100 ||
    max_nodes>100000 || max_cells>2000000 || horizon<preview || horizon>30 ||
    budget_ms>20 || cruise_speed>3.) {throw std::invalid_argument("planner bounds exceeded");}
}
void Course::validate()
{
  if (id.empty() || center.size()<2 || boundary.size()<3 || center.size()>10000 ||
    boundary.size()>2000) {throw std::invalid_argument("course identity/geometry missing or oversized");}
  for (auto p:center) {if (!valid_point(p)) {throw std::invalid_argument("invalid centerline");}}
  for (auto p:boundary) {if (!valid_point(p)) {throw std::invalid_argument("invalid boundary");}}
  stations={0};
  for (size_t i=1;i<center.size();++i) {
    const auto d=distance(center[i-1],center[i]);
    if (d<1e-5 || d>5.) {throw std::invalid_argument("duplicate or sparse centerline");}
    stations.push_back(stations.back()+d);
  }
  double area=0;
  for (size_t i=0;i<boundary.size();++i) {
    size_t j=(i+1)%boundary.size();
    if (distance(boundary[i],boundary[j])<1e-6) {throw std::invalid_argument("duplicate boundary vertex");}
    area+=boundary[i].x*boundary[j].y-boundary[j].x*boundary[i].y;
    for (size_t k=i+1;k<boundary.size();++k) {
      size_t l=(k+1)%boundary.size();
      if (j==k || l==i) {continue;}
      if (intersects(boundary[i],boundary[j],boundary[k],boundary[l])) {
        throw std::invalid_argument("self-intersecting course boundary");
      }
    }
  }
  if (std::fabs(area)<.1 || !std::isfinite(entry)||!std::isfinite(exit)||
    entry<0 || exit<=entry || exit>=length()) {throw std::invalid_argument("invalid entry/exit; onward route required");}
  for (auto p:center) {if (!inside(p,boundary)) {throw std::invalid_argument("centerline outside boundary");}}
}
Projection Course::project(Point p,double low,double high) const
{
  Projection best{0,std::numeric_limits<double>::infinity(),0};
  double best_d=std::numeric_limits<double>::infinity();
  for (size_t i=1;i<center.size();++i) {
    if (stations[i]<low || stations[i-1]>high) {continue;}
    const auto a=center[i-1], b=center[i]; const double len=stations[i]-stations[i-1];
    const double lo=std::clamp((low-stations[i-1])/len,0.,1.);
    const double hi=std::clamp((high-stations[i-1])/len,0.,1.);
    const double t=std::clamp(((p.x-a.x)*(b.x-a.x)+(p.y-a.y)*(b.y-a.y))/(len*len),lo,hi);
    const Point q{a.x+t*(b.x-a.x),a.y+t*(b.y-a.y)};
    const double d=distance(p,q);
    if (d<best_d) {best_d=d; best={stations[i-1]+t*len,
      std::copysign(d,cross(a,b,p)),std::atan2(b.y-a.y,b.x-a.x)};}
  }
  return best;
}
Point Course::at(double s) const
{
  s=std::clamp(s,0.,length());
  const auto it=std::upper_bound(stations.begin(),stations.end(),s);
  const size_t i=std::min(center.size()-1,std::max(size_t(1),size_t(it-stations.begin())));
  const double t=(s-stations[i-1])/(stations[i]-stations[i-1]);
  return {center[i-1].x+t*(center[i].x-center[i-1].x),center[i-1].y+t*(center[i].y-center[i-1].y)};
}
Grid::Grid(Config c):cfg_(c) {cfg_.validate();}
int Grid::index(Point p) const
{
  if (!valid_point(p)) {return -1;}
  const double x=(p.x-origin_.x)/cfg_.resolution, y=(p.y-origin_.y)/cfg_.resolution;
  if (x<0||y<0||x>=nx_||y>=ny_) {return -1;}
  return int(y)*nx_+int(x);
}
void Grid::configure(const Course & course)
{
  auto checked=course;checked.validate();
  double minx=course.boundary.front().x,maxx=minx,miny=course.boundary.front().y,maxy=miny;
  for (auto p:course.boundary) {minx=std::min(minx,p.x);maxx=std::max(maxx,p.x);miny=std::min(miny,p.y);maxy=std::max(maxy,p.y);}
  const double r=cfg_.resolution;
  origin_={minx-r,miny-r};
  const double nx=std::ceil((maxx-minx)/r)+2,ny=std::ceil((maxy-miny)/r)+2;
  if (nx*ny>double(cfg_.max_cells)) {throw std::invalid_argument("course exceeds grid capacity");}
  nx_=int(nx);ny_=int(ny);cells_.assign(nx_*ny_,{});distance_.resize(cells_.size());
  for (int y=0;y<ny_;++y) {for (int x=0;x<nx_;++x) {
    std::array<Point,4> corners{{{origin_.x+x*r,origin_.y+y*r},
      {origin_.x+(x+1)*r,origin_.y+y*r},{origin_.x+(x+1)*r,origin_.y+(y+1)*r},
      {origin_.x+x*r,origin_.y+(y+1)*r}}};
    bool good=true;
    for (auto p:corners) {good=good&&inside(p,course.boundary);}
    // Corners alone do not certify a cell spanning a concave boundary.
    if (good) {for (size_t i=0;i<course.boundary.size();++i) {
      for (size_t j=0;j<4;++j) {
        if (intersects(course.boundary[i],course.boundary[(i+1)%course.boundary.size()],
          corners[j],corners[(j+1)%4])) {good=false;}
      }
    }}
    cells_[y*nx_+x].inside=good;
  }}
  prepared_=false;
}
void Grid::observe(const std::vector<Ray> & rays,uint64_t generation)
{
  if (!generation) {return;}
  for (const auto & ray:rays) {
    if (!valid_point(ray.origin)||!valid_point(ray.end)||!std::isfinite(ray.stamp)||ray.stamp<=0) {continue;}
    const double len=distance(ray.origin,ray.end);
    if (len>30 || len<1e-6) {continue;}
    const int end=index(ray.end),steps=int(std::ceil(len/(cfg_.resolution*.4)));
    for (int i=0;i<=steps;++i) {
      const double t=double(i)/steps;
      const int k=index({ray.origin.x+t*(ray.end.x-ray.origin.x),ray.origin.y+t*(ray.end.y-ray.origin.y)});
      if (k<0 || (ray.hit&&k==end)) {continue;}
      auto & c=cells_[k];
      if (!c.inside || ray.stamp<=c.stamp) {continue;}
      if (c.hit && ray.stamp>c.free_stamp && c.clear_generation!=generation) {
        c.clear_generation=generation;++c.clear_count;
        if (c.clear_count>=2) {c.hit=false;}
      }
      c.free_stamp=std::max(c.free_stamp,ray.stamp);
    }
  }
  // Endpoint wins against free rays in the same scan, independent of ray order.
  for (const auto & ray:rays) {
    if (!ray.hit || !valid_point(ray.origin)||!valid_point(ray.end)||
      !std::isfinite(ray.stamp)||ray.stamp<=0 || distance(ray.origin,ray.end)>30) {continue;}
    const int k=index(ray.end); if (k<0) {continue;}
    auto & c=cells_[k];
    if (ray.stamp>=std::max(c.stamp,c.free_stamp)) {
      c.hit=true;c.stamp=ray.stamp;c.clear_count=0;c.clear_generation=0;
    }
  }
  prepared_=false;
}
void Grid::prepare(double now)
{
  // Exact Chebyshev distance transform is a conservative Euclidean lower bound.
  // Subtract cell uncertainty below; unlike chamfer length it cannot overestimate.
  const int inf=nx_+ny_+1;
  valid_until_=now+cfg_.free_ttl;
  for (size_t i=0;i<cells_.size();++i) {
    const auto & c=cells_[i];
    distance_[i]=c.inside&&!c.hit&&c.free_stamp>0&&now>=c.free_stamp&&
      now-c.free_stamp<=cfg_.free_ttl ? inf:0;
    // Conservative lease over ALL usable free cells, not merely newest sensor input.
    // A future optimization may restrict this to the verified body/braking envelope.
    if(distance_[i]==inf){valid_until_=std::min(valid_until_,c.free_stamp+cfg_.free_ttl);}
  }
  for (int y=0;y<ny_;++y) {for (int x=0;x<nx_;++x) {
    int & d=distance_[y*nx_+x];
    if (x) {d=std::min(d,distance_[y*nx_+x-1]+1);}
    if (y) {for (int dx=-1;dx<=1;++dx) {if (x+dx>=0&&x+dx<nx_) {d=std::min(d,distance_[(y-1)*nx_+x+dx]+1);}}}
  }}
  for (int y=ny_-1;y>=0;--y) {for (int x=nx_-1;x>=0;--x) {
    int & d=distance_[y*nx_+x];
    if (x+1<nx_) {d=std::min(d,distance_[y*nx_+x+1]+1);}
    if (y+1<ny_) {for (int dx=-1;dx<=1;++dx) {if (x+dx>=0&&x+dx<nx_) {d=std::min(d,distance_[(y+1)*nx_+x+dx]+1);}}}
  }}
  prepared_=true;
}
double Grid::point_clearance(Point p) const
{
  const int k=index(p);
  if (k<0||!prepared_) {return 0;}
  return std::max(0.,(distance_[k]-1.5)*cfg_.resolution);
}
double Grid::disc_clearance(const State & s) const
{
  if (!finite(s)) {return -1;}
  // Three overlapping discs enclose the entire rectangle, including its corners.
  const double half=(cfg_.front+cfg_.rear)/6.;
  const double radius=std::hypot(half,cfg_.width/2)+cfg_.margin;
  double result=std::numeric_limits<double>::infinity();
  for (int i=0;i<3;++i) {
    const double x=-cfg_.rear+half+2*half*i;
    result=std::min(result,point_clearance({s.x+x*std::cos(s.yaw),s.y+x*std::sin(s.yaw)})-radius);
  }
  return result;
}
bool Grid::clear(const State & s,double extra) const
{
  if(!prepared_||!finite(s)||!std::isfinite(extra)||extra<0){return false;}
  if(disc_clearance(s)>extra){return true;}
  // Exact oriented rectangle versus forbidden cell squares. The disc distance
  // field is only a fast acceptance test, not a reason to discard a narrow gap.
  const double c=std::cos(s.yaw),sn=std::sin(s.yaw),pad=cfg_.margin+extra;
  const double hx=(cfg_.front+cfg_.rear)/2+pad,hy=cfg_.width/2+pad;
  const Point center{s.x+(cfg_.front-cfg_.rear)/2*c,s.y+(cfg_.front-cfg_.rear)/2*sn};
  const double ex=std::fabs(c)*hx+std::fabs(sn)*hy,ey=std::fabs(sn)*hx+std::fabs(c)*hy;
  if(center.x-ex<origin_.x||center.y-ey<origin_.y||
    center.x+ex>=origin_.x+nx_*cfg_.resolution||center.y+ey>=origin_.y+ny_*cfg_.resolution){return false;}
  const int x0=int(std::floor((center.x-ex-origin_.x)/cfg_.resolution));
  const int x1=int(std::floor((center.x+ex-origin_.x)/cfg_.resolution));
  const int y0=int(std::floor((center.y-ey-origin_.y)/cfg_.resolution));
  const int y1=int(std::floor((center.y+ey-origin_.y)/cfg_.resolution));
  if(x0<0||y0<0||x1>=nx_||y1>=ny_){return false;}
  const double h=cfg_.resolution/2,cell_projection=h*(std::fabs(c)+std::fabs(sn));
  for(int y=y0;y<=y1;++y){for(int x=x0;x<=x1;++x){
    if(distance_[y*nx_+x]>0){continue;}
    const double dx=origin_.x+(x+.5)*cfg_.resolution-center.x;
    const double dy=origin_.y+(y+.5)*cfg_.resolution-center.y;
    if(std::fabs(dx*c+dy*sn)<=hx+cell_projection&&
       std::fabs(-dx*sn+dy*c)<=hy+cell_projection){return false;}
  }}
  return true;
}
double Grid::clearance(const State & s) const
{
  const double lower=disc_clearance(s);
  if(lower>0){return lower;}
  if(!clear(s)){return -1;}
  double low=0,high=.6;
  for(int i=0;i<6;++i){double mid=(low+high)/2;if(clear(s,mid)){low=mid;}else{high=mid;}}
  return low;
}
bool Grid::observed_free(Point p) const
{const int k=index(p);return prepared_&&k>=0&&distance_[k]>0;}
bool Grid::occupied(Point p) const
{const int k=index(p);return k>=0&&cells_[k].hit;}
bool Grid::blocks_gps(const Course & course,double low,double high) const
{
  // Only positive occupancy can request an overtake. Unknown space never can.
  const double half=std::hypot((cfg_.front+cfg_.rear)/6,cfg_.width/2)+cfg_.margin+
    cfg_.resolution*1.5;
  for(int y=0;y<ny_;++y){for(int x=0;x<nx_;++x){
    const auto & cell=cells_[y*nx_+x];if(!cell.inside||!cell.hit){continue;}
    const Point p{origin_.x+(x+.5)*cfg_.resolution,origin_.y+(y+.5)*cfg_.resolution};
    const auto pr=course.project(p,std::max(0.,low-1.),std::min(course.length(),high+1.));
    if(pr.station>=low&&pr.station<=high&&std::fabs(pr.cross)<=half){return true;}
  }}
  return false;
}
GridSnapshot Grid::debug_snapshot(double now) const
{
  GridSnapshot out{origin_,cfg_.resolution,nx_,ny_,{}};out.cells.reserve(cells_.size());
  for(const auto & c:cells_){
    out.cells.push_back(!c.inside?3:c.hit?2:
      c.free_stamp>0&&now>=c.free_stamp&&now-c.free_stamp<=cfg_.free_ttl?1:0);
  }
  return out;
}
Planner::Planner(Config c):cfg_(c) {cfg_.validate();}
void Planner::reset()
{station_=0;generation_=0;aligned_count_=0;initialized_=completed_=false;
  zone_valid_=in_zone_=entered_=false;previous_corridor_.clear();previous_path_.clear();previous_geometry_=PlanTrace{};}
void Planner::set_course(Course c) {c.validate();course_=std::move(c);reset();}
void Planner::set_zone(bool valid,bool in_zone)
{
  zone_valid_=valid;in_zone_=valid&&in_zone;
  if(in_zone_){
    if(completed_){completed_=false;entered_=false;aligned_count_=0;}
    entered_=true;
  }
}
double Planner::stopping_distance(double v) const
{
  if (!std::isfinite(v)||v<0) {return std::numeric_limits<double>::infinity();}
  return v*(cfg_.observation_gap+cfg_.processing_delay+cfg_.brake_delay)+
    v*v/(2*cfg_.brake_decel)+cfg_.longitudinal_margin;
}
State Planner::advance(State s,double target,double ds,double speed) const
{
  const double dt=ds/std::max(.05,speed);
  const double change=std::clamp((target-s.steer)*(1-std::exp(-dt/cfg_.steer_tau)),
    -cfg_.steer_rate*dt,cfg_.steer_rate*dt);
  const double k=std::tan(s.steer+change*.5)/cfg_.wheelbase;
  s.x+=ds*std::cos(s.yaw+k*ds*.5);s.y+=ds*std::sin(s.yaw+k*ds*.5);
  s.yaw=wrap(s.yaw+k*ds);s.steer+=change;s.speed=speed;return s;
}
bool Planner::segment(State start,double target,double length,double speed,
  const Grid & grid,std::vector<State> & samples,double extra) const
{
  if (!finite(start)||!std::isfinite(target)||std::fabs(target)>cfg_.max_steer||
    !std::isfinite(length)||length<0||length>30||!std::isfinite(speed)||speed<0||speed>3.) {return false;}
  const int n=std::max(1,int(std::ceil(length/cfg_.sample_step)));
  const double ds=length/n;
  const double radius=std::hypot(std::max(cfg_.front,cfg_.rear)+cfg_.margin,cfg_.width/2+cfg_.margin);
  // Steering evolves monotonically toward target; use its actual bounded range.
  // Assuming maximum lock even during straight braking falsely closes narrow gaps.
  const double steering_bound=std::max(std::fabs(start.steer),std::fabs(target));
  const double swept_pad=extra+ds*.5*(1+radius*std::tan(steering_bound)/cfg_.wheelbase);
  if (!grid.clear(start,swept_pad)) {return false;}
  for (int i=0;i<n;++i) {
    start=advance(start,target,ds,speed);
    if (speed*speed*std::fabs(std::tan(start.steer)/cfg_.wheelbase)>cfg_.lateral_accel ||
      !grid.clear(start,swept_pad)) {return false;}
    samples.push_back(start);
  }
  return true;
}
bool Planner::braking_clear(const State & s,const Grid & grid,double extra) const
{
  if (!finite(s)||s.speed<0||std::fabs(s.steer)>cfg_.max_steer||!grid.clear(s)) {return false;}
  // Hold measured steering through the stop; never assume instantaneous straightening.
  std::vector<State> tail;
  return segment(s,s.steer,stopping_distance(s.speed),s.speed,grid,tail,extra);
}
bool Planner::braking_transition_clear(const State & a,const State & b,const Grid & grid,unsigned depth) const
{
  if(!finite(a)||!finite(b)||a.speed<0||b.speed<0||a.speed>3||b.speed>3||
    std::max(std::fabs(a.steer),std::fabs(b.steer))>cfg_.max_steer||depth>6){return false;}
  const double ds=distance({a.x,a.y},{b.x,b.y});
  const auto mid=ds>1e-9?sample_path({a,b},ds/2):interpolate(a,b,.5);
  if(!braking_clear(mid,grid)){return false;}
  const double maximum_speed=std::max(a.speed,b.speed);
  const double maximum_steer=std::max(std::fabs(a.steer),std::fabs(b.steer));
  if(maximum_speed*maximum_speed*std::tan(maximum_steer)/cfg_.wheelbase>cfg_.lateral_accel+1e-9){return false;}
  // All intermediate poses are covered around the midpoint's frozen-steer stop.
  // |d(position)/d(kappa)| <= D²/2, |d(yaw)/d(kappa)| <= D.
  // Include variation in stopping length and the full padded body's rotation.
  const double da=stopping_distance(a.speed),db=stopping_distance(b.speed),dm=stopping_distance(mid.speed);
  const double stop=std::max(da,db),delta_stop=std::max(std::fabs(da-dm),std::fabs(db-dm));
  const double delta_yaw=std::fabs(wrap(b.yaw-a.yaw))/2;
  const double delta_k=std::fabs(b.steer-a.steer)/2/(cfg_.wheelbase*std::pow(std::cos(maximum_steer),2));
  const double k=std::tan(maximum_steer)/cfg_.wheelbase;
  const double radius=std::hypot(std::max(cfg_.front,cfg_.rear)+cfg_.margin,cfg_.width/2+cfg_.margin);
  const double pad=ds/2+stop*delta_yaw+stop*stop*delta_k/2+delta_stop+
    radius*(delta_yaw+stop*delta_k+k*delta_stop);
  if(braking_clear(mid,grid,pad+1e-7)){return true;}
  if(depth==6){return false;}
  return braking_transition_clear(a,mid,grid,depth+1)&&braking_transition_clear(mid,b,grid,depth+1);
}
bool Planner::wall_corridor(const State & current,const Grid & grid,double goal,
  std::vector<Point> & guide,PlanTrace * trace,double lead,double smoothing,double side_bias,
  std::chrono::steady_clock::time_point deadline) const
{
  struct Gap {double low,high,mid,cost;int parent;};
  struct Slice {Point center;double yaw,station;std::vector<Gap> gaps;};
  std::vector<Slice> slices;
  const double ds=.2,dd=.05,half=cfg_.width/2+cfg_.margin;
  const double first=std::floor(station_/ds)*ds;
  const double current_cross=course_.project({current.x,current.y},std::max(0.,station_-.5),station_+.5).cross;
  for(double station=first;station<=goal+1e-6;station+=ds){
    if(std::chrono::steady_clock::now()>=deadline){return false;}
    Slice slice; slice.center=course_.at(station);slice.station=station;
    slice.yaw=course_.project(slice.center,std::max(0.,station-.01),station+.01).yaw;
    const auto at=[&](double d){return Point{slice.center.x-std::sin(slice.yaw)*d,slice.center.y+std::cos(slice.yaw)*d};};
    double opening=0;bool open=false;
    for(int j=0;j<=160;++j){
      const double d=-4+j*dd;const bool free=j<160&&grid.observed_free(at(d));
      if(free&&!open){opening=d;open=true;}
      if(!free&&open){
        // Refine free/blocked boundaries instead of averaging probe centres.
        // The old last-free sample shortened one side by up to dd and biased
        // narrow midlines toward an occupied cell even for a parallel passage.
        double lower=opening,upper=d-dd;
        if(opening>-4+1e-8){double blocked=opening-dd;
          for(int n=0;n<10;++n){const double middle=(blocked+lower)/2;
            if(grid.observed_free(at(middle))){lower=middle;}else{blocked=middle;}}
        }
        double blocked=d;
        for(int n=0;n<10;++n){const double middle=(upper+blocked)/2;
          if(grid.observed_free(at(middle))){upper=middle;}else{blocked=middle;}}
        const double low=lower+half,high=upper-half;open=false;
        if(high<low){continue;}
        const double mid=(low+high)/2;
        double prior=mid;
        if(!previous_corridor_.empty()){
          double nearest=1e9;
          for(auto p:previous_corridor_){double e=distance(p,slice.center);if(e<nearest){nearest=e;prior=-(p.x-slice.center.x)*std::sin(slice.yaw)+(p.y-slice.center.y)*std::cos(slice.yaw);}}
        }
        Gap gap{low,high,mid,std::numeric_limits<double>::infinity(),-1};
        const double preference=.08*mid*mid+.2*(mid-prior)*(mid-prior)+.05/(high-low+.05)+side_bias*mid;
        if(slices.empty()){
          if(current_cross>=low-.1&&current_cross<=high+.1){gap.cost=preference+2*std::pow(mid-current_cross,2);}
        }else{
          const auto & before=slices.back().gaps;
          for(size_t k=0;k<before.size();++k){const auto & p=before[k];
            if(std::max(low,p.low)>std::min(high,p.high)+.02){continue;}
            const double cost=p.cost+preference+3*std::pow(mid-p.mid,2);
            if(cost<gap.cost){gap.cost=cost;gap.parent=int(k);}
          }
        }
        if(std::isfinite(gap.cost)){slice.gaps.push_back(gap);}
      }
    }
    if(slice.gaps.empty()){break;}
    slices.push_back(std::move(slice));
  }
  if(slices.size()<8){return false;}
  int chosen=0;const auto & last=slices.back().gaps;
  for(size_t k=1;k<last.size();++k){if(last[k].cost<last[chosen].cost){chosen=int(k);}}
  std::vector<Gap> selected(slices.size());
  for(int i=int(slices.size())-1;i>=0;--i){selected[i]=slices[i].gaps[chosen];chosen=selected[i].parent;}
  std::vector<Gap> bounds=selected;
  const size_t front_steps=size_t(std::ceil(lead/ds));
  for(size_t i=0;i<bounds.size();++i){
    for(size_t j=i;j<std::min(bounds.size(),i+front_steps+1);++j){
      bounds[i].low=std::max(bounds[i].low,selected[j].low);
      bounds[i].high=std::min(bounds[i].high,selected[j].high);
    }
    if(bounds[i].low>bounds[i].high){return false;}
  }
  std::vector<double> offset;for(auto g:bounds){offset.push_back((g.low+g.high)/2);}
  // Constrained smoothing connects the wall midpoints without cutting through a
  // different gap. Looking ahead spreads lateral motion before a wall begins.
  for(int pass=0;pass<100;++pass){
    if(pass%10==0&&std::chrono::steady_clock::now()>=deadline){return false;}
    for(size_t i=0;i<offset.size();++i){
      const double weight=selected[i].high-selected[i].low<1.?6.:1.;
      double num=weight*(bounds[i].low+bounds[i].high)/2,den=weight;
      if(i){num+=smoothing*offset[i-1];den+=smoothing;}
      if(i+1<offset.size()){num+=smoothing*offset[i+1];den+=smoothing;}
      if(i==0){num+=8*current_cross;den+=8;}
      offset[i]=std::clamp(num/den,bounds[i].low,bounds[i].high);
    }
  }
  for(size_t i=0;i<slices.size();++i){const auto & s=slices[i];const auto & g=selected[i];
    const auto at=[&](double d){return Point{s.center.x-std::sin(s.yaw)*d,s.center.y+std::cos(s.yaw)*d};};
    guide.push_back(at(offset[i]));
    if(trace){trace->wall_left.push_back(at(g.high+half));trace->wall_right.push_back(at(g.low-half));
      trace->midpoints.push_back(at(g.mid));trace->corridor.push_back(guide.back());}
  }
  return true;
}
Plan Planner::plan(const State & current,Grid & grid,double now,uint64_t generation,PlanTrace * trace)
{
  if(trace){trace->edges.clear();trace->counts.assign(5,0);trace->edges.reserve(trace->limit);
    trace->wall_left.clear();trace->wall_right.clear();trace->midpoints.clear();trace->corridor.clear();}
  auto record=[&](State start,const std::vector<State> & samples,uint8_t result){
    if(!trace){return;}
    ++trace->counts[result];
    if(trace->edges.size()>=trace->limit){return;}
    const auto middle=samples.empty()?start:samples[samples.size()/2];
    const auto end=samples.empty()?start:samples.back();
    trace->edges.push_back({{start.x,start.y},{middle.x,middle.y},{end.x,end.y},result});
  };
  const auto begin=std::chrono::steady_clock::now();
  auto elapsed=[&]() {return std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();};
  Plan out;out.generation=generation;out.id=++plan_id_;
  auto finish=[&](const char * reason) {
    out.maneuver_active=false;
    out.reason=reason;out.compute_ms=elapsed();
    if (out.valid&&out.compute_ms>cfg_.budget_ms) {
      out.valid=out.complete=false;out.path.clear();out.phase=Phase::HOLD;
      out.reason="planner processing deadline exceeded";out.deadline_hit=true;
      completed_=false;aligned_count_=0;
    }
    out.perception_active=perception_required();
    return out;
  };
  if(!zone_valid_){return finish("obstacle zone membership unavailable");}
  if(!perception_required()){
    out.phase=completed_?Phase::COMPLETE:Phase::READY;out.complete=completed_;out.gps_follow=true;
    return finish("outside obstacle zone; GPS owns reference");
  }
  if (course_.stations.empty()||!finite(current)||!std::isfinite(now)||now<=0||
    !generation||generation<=generation_||current.speed<0||current.speed>3.||std::fabs(current.steer)>cfg_.max_steer) {
    aligned_count_=0;return finish("invalid or repeated input");
  }
  generation_=generation;grid.prepare(now);out.valid_until=grid.valid_until();
  const auto projection=course_.project({current.x,current.y},initialized_?std::max(0.,station_-.5):0.,
    initialized_?std::min(course_.length(),station_+std::fabs(current.speed)*cfg_.observation_gap*1.5+.5):course_.length());
  if (!std::isfinite(projection.cross)||std::fabs(projection.cross)>3.) {aligned_count_=0;return finish("route projection unavailable");}
  station_=projection.station;initialized_=true;out.station=station_;
  out.obstacle_detected=grid.blocks_gps(course_,std::max(0.,station_-cfg_.rear),
    std::min(course_.length(),station_+cfg_.horizon+cfg_.front));
  if (!braking_clear(current,grid)) {aligned_count_=0;return finish("current braking envelope blocked or unknown");}
  const double speed=std::max(current.speed,cfg_.cruise_speed);
  const double stopping=stopping_distance(current.speed);out.stop_distance=stopping;
  const double goal=std::min(course_.length()-cfg_.front-cfg_.margin,station_+cfg_.horizon);
  // Course must contain enough onward geometry for the complete body and stopping.
  if (goal<=station_+cfg_.preview || stopping>cfg_.horizon) {aligned_count_=0;return finish("insufficient planning horizon");}
  if(std::fabs(wrap(current.yaw-projection.yaw))>1.2){return finish("vehicle heading opposes GPS direction");}
  std::vector<Point> corridor,best_corridor;PlanTrace best_geometry;
  const auto deadline=begin+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double,std::milli>(std::max(0.,cfg_.budget_ms-2.)));
  double best_length=0,best_score=-1e9;bool connected=false;
  // Continue the already issued speed/steering trajectory when its remaining
  // prefix is still observed and safe. Replanning must not restart a braking
  // delay or change the midline under the vehicle every 100ms.
  std::vector<State> retained;
  double retained_length=0,nearest=1e9;size_t cut=0;double cut_fraction=0;
  for(size_t i=1;i<previous_path_.size();++i){const auto a=previous_path_[i-1],b=previous_path_[i];
    if(distance({current.x,current.y},{a.x,a.y})>.5){if(i>12)break;continue;}
    const double dx=b.x-a.x,dy=b.y-a.y,ds2=dx*dx+dy*dy;
    const double f=ds2>1e-9?std::clamp(((current.x-a.x)*dx+(current.y-a.y)*dy)/ds2,0.,1.):0;
    const auto at=sample_path({a,b},f*std::sqrt(ds2));const double error=distance({current.x,current.y},{at.x,at.y});
    if(error<nearest&&std::fabs(at.speed-current.speed)<.03&&std::fabs(wrap(at.yaw-current.yaw))<.02&&std::fabs(at.steer-current.steer)<.02){nearest=error;cut=i;cut_fraction=f;}
  }
  if(nearest<.015&&cut){
    retained.push_back(current);
    for(size_t i=cut;i<previous_path_.size();++i){
      if(i==cut&&cut_fraction>.99999){continue;}
      const auto a=retained.back(),b=previous_path_[i];const double ds=distance({a.x,a.y},{b.x,b.y});
      if(elapsed()>=cfg_.budget_ms-3.){retained.clear();break;}
      if(ds<1e-8){continue;}
      const double duration=2*ds/std::max(.001,a.speed+b.speed);
      const double accel=(b.speed*b.speed-a.speed*a.speed)/(2*ds);
      if(accel < -cfg_.planning_decel-1e-5 || accel > cfg_.acceleration+1e-5 ||
        std::fabs(b.steer-a.steer)>cfg_.steer_rate*duration+1e-6){break;}
      const bool safe=braking_transition_clear(a,b,grid);
      ++out.expanded;
      if(!safe||!grid.clear(b,.02)||!braking_clear(b,grid)){record(a,{b},2);break;}
      record(a,{b},0);
      retained.push_back(b);retained_length+=ds;
    }
  }
  if(retained.size()>1&&retained_length>=cfg_.preview+stopping+1.){
    out.path=retained;best_length=retained_length;out.reused=true;best_corridor=previous_corridor_;best_geometry=previous_geometry_;
    if(trace){trace->wall_left=previous_geometry_.wall_left;trace->wall_right=previous_geometry_.wall_right;
      trace->midpoints=previous_geometry_.midpoints;trace->corridor=previous_geometry_.corridor;}
  }
  // Each policy uses the same measured wall gaps, with a bounded choice of
  // longitudinal anticipation and smoothing. No obstacle IDs or pass sides.

  struct Policy {double lead,smoothing,bias;};
  for(auto policy:std::array<Policy,8>{{{1.,60.,0.},{1.,60.,1.},{1.,60.,-1.},{0.,12.,0.},{.6,24.,0.},{1.4,100.,0.},{1.,20.,0.},{2.,100.,0.}}}){
    if(out.reused){break;}
    if(elapsed()>=cfg_.budget_ms-2.){out.deadline_hit=true;break;}
    corridor.clear();PlanTrace geometry;
    if(!wall_corridor(current,grid,std::min(course_.length()-.2,goal+2.),corridor,
      &geometry,policy.lead,policy.smoothing,policy.bias,deadline)){continue;}
    connected=true;
    std::vector<double> limits(corridor.size(),cfg_.cruise_speed),arc(corridor.size(),0),steering(corridor.size(),0);
    for(size_t i=1;i<corridor.size();++i){arc[i]=arc[i-1]+distance(corridor[i-1],corridor[i]);}
    if(cfg_.adaptive_speed){
      for(size_t i=0;i<corridor.size();++i){
        const double width=distance(geometry.wall_left[i],geometry.wall_right[i]);
        const double reserve=std::max(0.,width-cfg_.width-2*cfg_.margin);
        limits[i]=cfg_.crawl_speed+(cfg_.cruise_speed-cfg_.crawl_speed)*std::clamp((reserve-.6)/.9,0.,1.);
        if(i&&i+1<corridor.size()){
          const auto a=corridor[i-1],b=corridor[i],c=corridor[i+1];
          const double product=distance(a,b)*distance(b,c)*distance(a,c);
          const double curvature=product>1e-9?2*cross(a,b,c)/product:0;
          steering[i]=std::atan(cfg_.wheelbase*curvature);
          limits[i]=std::min(limits[i],std::sqrt(cfg_.lateral_accel/std::max(.001,std::fabs(curvature))));
        }
      }
      if(steering.size()>2){steering.front()=steering[1];steering.back()=steering[steering.size()-2];}
      for(size_t i=1;i<limits.size();++i){
        const double gradient=std::fabs(steering[i]-steering[i-1])/std::max(.01,arc[i]-arc[i-1]);
        limits[i]=std::clamp(std::min(limits[i],.8*cfg_.steer_rate/std::max(.001,gradient)),cfg_.crawl_speed,cfg_.cruise_speed);
      }
      // Move restrictions forward to account for the front bumper and command
      // latency, then propagate required deceleration backwards along the route.
      auto shifted=limits;
      for(size_t i=0;i<limits.size();++i){for(size_t j=i>2?i-2:0;j<limits.size()&&arc[j]-arc[i]<=cfg_.front+stopping_distance(speed);++j){shifted[i]=std::min(shifted[i],limits[j]);}}
      limits=std::move(shifted);
      for(size_t i=limits.size()-1;i>0;--i){limits[i-1]=std::min(limits[i-1],std::sqrt(limits[i]*limits[i]+2*cfg_.planning_decel*(arc[i]-arc[i-1])));}
    }
    auto speed_at=[&](State state){size_t nearest=0;double best=1e9;
      for(size_t i=0;i<corridor.size();++i){double error=distance({state.x,state.y},corridor[i]);if(error<best){best=error;nearest=i;}}
      return limits[nearest];
    };
    // Track the connected midline through the steering-lag model. GPS chooses the
    // forward station/exit; it does not pull the path through an occupied center.
    auto command=[&](State s,double lookahead){
      // Absolute speed scaling: lowering the cruise ceiling must not enlarge
      // the lookahead for a vehicle travelling at the same physical speed.
      if(cfg_.adaptive_speed){lookahead*=.5+.5*std::clamp(s.speed,0.,1.);}
      size_t near=0;double best=1e9,fraction=0;
      for(size_t i=0;i+1<corridor.size();++i){const auto a=corridor[i],b=corridor[i+1];
        const double dx=b.x-a.x,dy=b.y-a.y,ds2=dx*dx+dy*dy;
        const double f=ds2>1e-9?std::clamp(((s.x-a.x)*dx+(s.y-a.y)*dy)/ds2,0.,1.):0;
        const double d=distance({s.x,s.y},{a.x+f*dx,a.y+f*dy});
        if(d<best){best=d;near=i;fraction=f;}
      }
      Point p=corridor.back();
      for(size_t i=near;i+1<corridor.size();++i){const auto a=corridor[i],b=corridor[i+1];
        const double ds=distance(a,b),begin=i==near?fraction:0,available=ds*(1-begin);
        if(lookahead<=available&&ds>1e-9){const double f=begin+lookahead/ds;p={a.x+f*(b.x-a.x),a.y+f*(b.y-a.y)};break;}
        lookahead-=available;
      }
      const double dx=p.x-s.x,dy=p.y-s.y;
      const double lateral=-dx*std::sin(s.yaw)+dy*std::cos(s.yaw);
      const double curvature=2*lateral/std::max(.04,dx*dx+dy*dy);
      const double maximum=std::min(std::tan(cfg_.max_steer)/cfg_.wheelbase,cfg_.lateral_accel/std::max(.0025,s.speed*s.speed));
      return std::atan(cfg_.wheelbase*std::clamp(curvature,-maximum,maximum));
    };
    for(double lookahead:{.8,1.1,.6,1.4}){
      for(bool crawl_policy:{false,true}){
        std::vector<State> candidate{current};double length=0,previous_station=station_,time=0;
        while(length<cfg_.horizon){
          if(elapsed()>=cfg_.budget_ms-2.){out.deadline_hit=true;break;}
          std::vector<State> samples;const State start=candidate.back();
          ++out.expanded;
          double ds=cfg_.sample_step,next_speed=speed;
          if(cfg_.adaptive_speed){
            const double target_speed=crawl_policy?cfg_.crawl_speed:speed_at(start);
            if(time<cfg_.processing_delay&&start.speed>.01){
              ds=std::min(ds,start.speed*(cfg_.processing_delay-time));next_speed=start.speed;
            }else if(target_speed<start.speed){next_speed=std::max(target_speed,std::sqrt(std::max(0.,start.speed*start.speed-2*cfg_.planning_decel*ds)));}
            else{next_speed=std::min(target_speed,std::sqrt(start.speed*start.speed+2*cfg_.acceleration*ds));}
          }
          if(ds<1e-6){time=cfg_.processing_delay;continue;}
          const double integration_speed=(start.speed+next_speed)/2;
          if(!segment(start,command(start,lookahead),ds,integration_speed,grid,samples)){record(start,samples,1);break;}
          samples.back().speed=next_speed;
          if(std::max(start.speed*start.speed,next_speed*next_speed)*std::fabs(std::tan(samples.back().steer)/cfg_.wheelbase)>cfg_.lateral_accel+1e-8){record(start,samples,1);break;}
          if(!braking_clear(samples.back(),grid)||!braking_transition_clear(start,samples.back(),grid)){record(start,samples,2);break;}
          time+=ds/std::max(.001,integration_speed);
          const auto pr=course_.project({samples.back().x,samples.back().y},std::max(0.,station_-.5),goal+2.);
          if(pr.station<previous_station-1e-4||std::fabs(wrap(samples.back().yaw-pr.yaw))>1.2){record(start,samples,3);break;}
          previous_station=pr.station;candidate.push_back(samples.back());length+=ds;record(start,samples,0);
        }
        const double score=length-.02*std::fabs(lookahead-.8)-(crawl_policy?.01:0.);
        if(score>best_score){best_score=score;best_length=length;out.path=std::move(candidate);best_corridor=corridor;best_geometry=geometry;
          if(trace){trace->wall_left=geometry.wall_left;trace->wall_right=geometry.wall_right;
            trace->midpoints=geometry.midpoints;trace->corridor=geometry.corridor;}
        }
        if(length>=cfg_.horizon-.01){break;}
      }
      if(best_length>=cfg_.horizon-.01){break;}
    }
    if(best_length>=cfg_.horizon-.01){break;}
  }
  if(best_length<cfg_.preview+.1&&retained.size()>1&&retained_length>=cfg_.preview+.1){
    out.path=std::move(retained);best_length=retained_length;out.reused=true;best_corridor=previous_corridor_;best_geometry=previous_geometry_;
    if(trace){trace->wall_left=previous_geometry_.wall_left;trace->wall_right=previous_geometry_.wall_right;
      trace->midpoints=previous_geometry_.midpoints;trace->corridor=previous_geometry_.corridor;}
  }
  if(best_length<cfg_.preview+.1){out.path.clear();aligned_count_=0;
    if(elapsed()>=cfg_.budget_ms-2.){out.deadline_hit=true;return finish("wall corridor processing deadline exceeded");}
    return finish(connected?"wall midline has no certified forward rollout":"no connected observed wall corridor");
  }
  previous_corridor_=best_corridor;
  out.gps_follow=false; // Legacy overtaking flag is never asserted.
  // A stop at any prefix must remain in observed space, including intermediate turns.
  for (size_t i=0;i<out.path.size();i+=std::max(size_t(1),size_t(.2/cfg_.sample_step))) {
    if (!braking_clear(out.path[i],grid)) {out.path.clear();aligned_count_=0;return finish("intermediate braking envelope blocked");}
  }
  if (!braking_clear(out.path.back(),grid)) {out.path.clear();return finish("terminal braking envelope blocked");}
  out.min_clearance=std::numeric_limits<double>::infinity();double length=0;bool preview=false;
  for (size_t i=0;i<out.path.size();++i) {
    out.min_clearance=std::min(out.min_clearance,grid.clearance(out.path[i]));
    if (i) {length+=distance({out.path[i-1].x,out.path[i-1].y},{out.path[i].x,out.path[i].y});}
    if (!preview&&length>=cfg_.preview) {out.reference=out.path[i];preview=true;}
  }
  if (!preview) {out.path.clear();return finish("preview outside certified path");}
  // Three fresh scans must agree on physical exit + alignment + safe onward path.
  const auto exit_point=course_.at(course_.exit);
  const double exit_yaw=course_.project(exit_point,course_.exit-.01,course_.exit+.01).yaw;
  bool rear_past=true;
  for (double y:{-cfg_.width/2,cfg_.width/2}) {
    const Point rear{current.x-cfg_.rear*std::cos(current.yaw)-y*std::sin(current.yaw),
      current.y-cfg_.rear*std::sin(current.yaw)+y*std::cos(current.yaw)};
    rear_past=rear_past&&((rear.x-exit_point.x)*std::cos(exit_yaw)+(rear.y-exit_point.y)*std::sin(exit_yaw)>cfg_.margin);
  }
  const bool aligned=std::fabs(projection.cross)<=cfg_.cross_tolerance&&
    std::fabs(wrap(current.yaw-projection.yaw))<=cfg_.yaw_tolerance;
  aligned_count_=rear_past&&station_>course_.exit&&aligned ? aligned_count_+1:0;
  completed_=completed_||(!in_zone_&&aligned_count_>=cfg_.completion_samples);
  out.valid=true;out.complete=completed_;out.speed_limit=sample_path_time(out.path,.1).speed;
  out.phase=completed_?Phase::COMPLETE:Phase::WALL_FOLLOW;
  previous_path_=out.path;previous_geometry_=std::move(best_geometry);
  if(trace){previous_geometry_.wall_left=trace->wall_left;previous_geometry_.wall_right=trace->wall_right;
    previous_geometry_.midpoints=trace->midpoints;previous_geometry_.corridor=trace->corridor;}
  return finish(out.reused?"wall corridor speed profile revalidated":"wall midpoint corridor certified");
}
}  // namespace avoid_v2
