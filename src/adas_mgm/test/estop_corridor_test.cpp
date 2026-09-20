#include "src/estop_scan.hpp"
#include <iostream>
using namespace adas_mgm;
int main() {
  int errors=0;
  auto check=[&](bool ok,const char*name){if(!ok){std::cerr<<name<<'\n';++errors;}};
  const double pi=std::acos(-1.),step=pi/720.;
  const std::vector<double> mount{.66,0,0,-180,180,0,.05,12};
  auto scan=[&](double x,double lo,double hi){
    std::vector<float> ranges(1441,INFINITY);
    for (int i=0;i<1441;++i) {
      double a=-pi+i*step;
      if(std::cos(a)<=0)continue;
      double r=(x-.66)/std::cos(a),y=r*std::sin(a);
      if(y>=lo && y<=hi) ranges[i]=r;
    }
    return ranges;
  };
  auto observe=[&](const std::vector<float>&v){return front_corridor(v,-pi,step,.05,12,mount,.76,.31);};
  auto wall=observe(scan(2.,-.12,.12));
  check(wall.valid && !wall.clear && wall.obstacle_points>=5,"five or more points in rectangle");
  auto small=observe(scan(2.,-.08,.08));
  check(small.valid && !small.clear && small.obstacle_points>=5,"narrow obstacle now qualifies by point count");
  auto sides=observe(scan(2.,.32,.60));
  check(sides.valid && sides.clear && sides.obstacle_points==0,"outside vehicle width excluded");
  check(observe(scan(5.77,-.3,.3)).clear,"past bumper plus 5m excluded");
  check(!observe(scan(5.75,-.15,.15)).clear,"inside bumper plus 5m included");
  auto split=scan(2.,-.16,-.06),right=scan(2.,.06,.16);
  for(size_t i=0;i<split.size();++i)split[i]=std::min(split[i],right[i]);
  check(observe(split).obstacle_points==observe(right).obstacle_points,
    "separate objects never sum point counts");
  std::vector<float> invalid(1441,NAN),empty(1441,INFINITY);
  check(!observe(invalid).valid && !observe(invalid).clear,"invalid scan not clear");
  check(observe(empty).valid && observe(empty).clear,"full valid no-return scan clear");
  empty[720]=NAN;check(!observe(empty).clear,"invalid forward beam cannot prove empty");
  check(!front_corridor({INFINITY},0,step,.05,12,mount,.76,.31).clear,"partial scan coverage not clear");
  // Reversed scan order is equivalent.
  auto reverse=scan(2.,-.12,.12);std::reverse(reverse.begin(),reverse.end());
  check(front_corridor(reverse,pi,-step,.05,12,mount,.76,.31).obstacle_points>=5,"negative angular increment");
  const std::vector<double> calibrated{.76,0,-87,-3,177,.069,.15,12};
  std::vector<float> calibrated_scan(1441,INFINITY);
  for (int i=0;i<1441;++i) {
    const double a=-pi+i*step-87*pi/180;
    if (std::cos(a)<=0) {continue;}
    const double r=1.0/std::cos(a),y=r*std::sin(a);
    if (y>=-.12 && y<=.12) {calibrated_scan[i]=r-.069;}
  }
  auto real_mount=front_corridor(calibrated_scan,-pi,step,.15,12,calibrated,.76,.31);
  check(real_mount.valid && !real_mount.clear && real_mount.obstacle_points>=5,
    "production a1 yaw and range offset detect point cluster");
  std::fill(calibrated_scan.begin(),calibrated_scan.end(),INFINITY);
  check(front_corridor(calibrated_scan,-pi,step,.15,12,calibrated,.76,.31).clear,
    "production a1 half-plane FOV covers front rectangle");
  // Five close returns qualify even though the entire cluster is <18cm wide.
  std::vector<float> four(1441,INFINITY),five(1441,INFINITY);
  for (int i=718;i<722;++i) {four[i]=1.f;five[i]=1.f;}
  five[722]=1.f;
  check(observe(four).obstacle_points==4,"four points below threshold");
  check(observe(five).obstacle_points==5,"five points meet threshold");
  five[720]=NAN;
  check(observe(five).obstacle_points==2,"invalid intervening beam splits cluster");
  // Equal radius rays: chord distance is exactly gap, independently of FOV.
  auto cluster=[&](double gap) {
    const double da=2*std::asin(gap/2.);
    return front_corridor(std::vector<float>(5,1.f),-2*da,da,.05,12,mount,.76,.31);
  };
  check(cluster(.05).obstacle_points==5,"5cm inclusive point gap");
  check(cluster(.05001).obstacle_points==1,"over 5cm splits objects");
  // Dense-looking returns farther than 5m cannot qualify.
  check(observe(scan(5.77,-.3,.3)).obstacle_points==0,"point count clipped at 5m");
  check(observe(scan(5.75,-.15,.15)).obstacle_points>=5,"new 3-to-5m region qualifies");
  auto two_small=four;
  for(int i=726;i<730;++i) {two_small[i]=1.f;}
  check(observe(two_small).obstacle_points==4,"two four-point objects do not sum to eight");
  return errors?1:0;
}
