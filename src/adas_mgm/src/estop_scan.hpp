#ifndef ADAS_MGM_ESTOP_SCAN_HPP
#define ADAS_MGM_ESTOP_SCAN_HPP
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>
namespace adas_mgm {
// Same calibrated sensor frame/FOV/range offsets as lidar_fusion_v2.
// Returns the closest exterior clearance, not distance to the sensor origin.
inline float body_clearance(const std::vector<float> & ranges, double angle_min,
  double increment, double range_min, double range_max, const std::vector<double> & mount,
  double front, double rear, double half_width) {
  if (mount.size() != 8) {return std::numeric_limits<float>::quiet_NaN();}
  constexpr double rad = 3.14159265358979323846 / 180.;
  float best = std::numeric_limits<float>::infinity();
  bool observed = false;
  for (size_t i = 0; i < ranges.size(); ++i) {
    const double angle = angle_min + i * increment;
    const double degrees = std::remainder(angle / rad, 360.);
    const double normalized = degrees < mount[3] ? degrees + 360. : degrees;
    if (normalized < mount[3] || normalized > mount[4]) {continue;}
    const float raw = ranges[i];
    if (raw == std::numeric_limits<float>::infinity()) {observed = true; continue;}
    if (!std::isfinite(raw) || raw < std::max(range_min, mount[6]) || raw > std::min(range_max, mount[7])) {continue;}
    observed = true;
    const double r = raw + mount[5];
    const double x = mount[0] + r * std::cos(angle + mount[2]*rad);
    const double y = mount[1] + r * std::sin(angle + mount[2]*rad);
    if (x >= -rear && x <= front && std::fabs(y) <= half_width) {continue;} // vehicle self returns
    const double dx = std::max({-rear-x, x-front, 0.});
    const double dy = std::max(std::fabs(y)-half_width, 0.);
    best = std::min(best, static_cast<float>(std::hypot(dx, dy)));
  }
  return observed ? best : std::numeric_limits<float>::quiet_NaN();
}
struct FrontCorridorObservation {
  uint32_t obstacle_points{0};
  bool valid{false};
  bool clear{false};
};
// Vehicle-frame rectangle: [front, front+5m] x [-half_width,+half_width].
// Count points in ONE contiguous cluster, never sum unrelated objects.
// An adjacent-point gap over 5cm splits clusters. All ray/point
// transforms use the same measured mount/FOV/range offset as body_clearance.
inline FrontCorridorObservation front_corridor(
  const std::vector<float> & ranges, double angle_min, double increment,
  double range_min, double range_max, const std::vector<double> & mount,
  double front, double half_width) {
  FrontCorridorObservation out;
  if (mount.size()!=8 || ranges.empty() || !std::isfinite(angle_min) ||
    !std::isfinite(increment) || increment==0 || !std::isfinite(range_min) ||
    !std::isfinite(range_max) || range_max<=range_min || half_width<=0 ||
    !std::all_of(mount.begin(),mount.end(),[](double v){return std::isfinite(v);})) {return out;}
  constexpr double rad=3.14159265358979323846/180.;
  struct Point {double x,y,angle;};
  std::vector<Point> points;
  bool clear=true;
  int observed=0, corridor_rays=0;
  // Prove that the scan FOV actually covers both near and far corridor edges.
  const auto angle_covered = [&](double x,double y) {
    const double raw=std::atan2(y-mount[1],x-mount[0])-mount[2]*rad;
    for (int turn=-2;turn<=2;++turn) {
      const double a=raw+turn*360.*rad;
      const double index=(a-angle_min)/increment;
      const double degrees=std::remainder(a/rad,360.);
      const double normalized=degrees<mount[3]?degrees+360.:degrees;
      if (index>=-.5 && index<=static_cast<double>(ranges.size())-.5 &&
        normalized>=mount[3] && normalized<=mount[4]) {return true;}
    }
    return false;
  };
  for (double x : {front+1e-4,front+5.}) {
    for (double y : {-half_width,0.,half_width}) {clear &= angle_covered(x,y);}
  }
  for (size_t i=0;i<ranges.size();++i) {
    const double a=angle_min+i*increment;
    const double degrees=std::remainder(a/rad,360.);
    const double normalized=degrees<mount[3]?degrees+360.:degrees;
    if (normalized<mount[3] || normalized>mount[4]) {continue;}
    const double dx=std::cos(a+mount[2]*rad),dy=std::sin(a+mount[2]*rad);
    double enter=0,leave=std::numeric_limits<double>::infinity();
    const auto slab=[&](double origin,double direction,double lo,double hi) {
      if (std::fabs(direction)<1e-12) {return origin>=lo && origin<=hi;}
      double t1=(lo-origin)/direction,t2=(hi-origin)/direction;
      if (t1>t2) {std::swap(t1,t2);}
      enter=std::max(enter,t1);leave=std::min(leave,t2);
      return leave>=enter;
    };
    if (!slab(mount[0],dx,front,front+5.) || !slab(mount[1],dy,-half_width,half_width) || leave<=0) {continue;}
    ++corridor_rays;
    const float raw=ranges[i];
    const double maximum=std::min(range_max,mount[7]);
    const bool no_return=raw==std::numeric_limits<float>::infinity();
    const bool finite=std::isfinite(raw) && raw>=std::max(range_min,mount[6]) && raw<=maximum;
    if ((!no_return && !finite) || maximum+mount[5]<leave) {clear=false;}
    if (!no_return && !finite) {continue;}
    ++observed;
    if (no_return) {continue;}
    const double r=raw+mount[5];
    if (r<=leave) {clear=false;} // includes occlusion before the corridor
    const double x=mount[0]+r*dx,y=mount[1]+r*dy;
    if (x>=front && x<=front+5. && std::fabs(y)<=half_width) {
      points.push_back({x,y,std::atan2(y-mount[1],x-mount[0])});
    }
  }
  std::sort(points.begin(),points.end(),[](const Point&a,const Point&b){return a.angle<b.angle;});
  uint32_t cluster_points=0;
  for (size_t i=0;i<points.size();++i) {
    const auto & p=points[i];
    const bool connected=i && std::hypot(p.x-points[i-1].x,p.y-points[i-1].y)<=.05+1e-7 &&
      p.angle-points[i-1].angle<=1.5*std::fabs(increment);
    cluster_points=connected ? cluster_points+1 : 1;
    out.obstacle_points=std::max(out.obstacle_points,cluster_points);
  }
  out.valid=observed>0 && corridor_rays>0;
  out.clear=out.valid && clear;
  return out;
}

}
#endif
