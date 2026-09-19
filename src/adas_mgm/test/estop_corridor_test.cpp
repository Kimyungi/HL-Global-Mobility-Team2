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
  check(wall.valid && !wall.clear && wall.obstacle_width_m>=.18,"wide obstacle in rectangle");
  auto small=observe(scan(2.,-.08,.08));
  check(small.valid && !small.clear && small.obstacle_width_m<.18,"narrow obstacle cannot enter but blocks exit");
  auto sides=observe(scan(2.,.32,.60));
  check(sides.valid && sides.clear && sides.obstacle_width_m==0,"outside vehicle width excluded");
  check(observe(scan(3.9,-.3,.3)).clear,"past bumper plus 3m excluded");
  check(!observe(scan(3.7,-.15,.15)).clear,"inside bumper plus 3m included");
  auto split=scan(2.,-.16,-.06),right=scan(2.,.06,.16);
  for(size_t i=0;i<split.size();++i)split[i]=std::min(split[i],right[i]);
  check(observe(split).obstacle_width_m<.18,"separate narrow objects never sum widths");
  std::vector<float> invalid(1441,NAN),empty(1441,INFINITY);
  check(!observe(invalid).valid && !observe(invalid).clear,"invalid scan not clear");
  check(observe(empty).valid && observe(empty).clear,"full valid no-return scan clear");
  empty[720]=NAN;check(!observe(empty).clear,"invalid forward beam cannot prove empty");
  check(!front_corridor({INFINITY},0,step,.05,12,mount,.76,.31).clear,"partial scan coverage not clear");
  // Reversed scan order is equivalent.
  auto reverse=scan(2.,-.12,.12);std::reverse(reverse.begin(),reverse.end());
  check(front_corridor(reverse,pi,-step,.05,12,mount,.76,.31).obstacle_width_m>=.18,"negative angular increment");
  const std::vector<double> calibrated{.76,0,-87,-3,177,.069,.15,12};
  std::vector<float> calibrated_scan(1441,INFINITY);
  for (int i=0;i<1441;++i) {
    const double a=-pi+i*step-87*pi/180;
    if (std::cos(a)<=0) {continue;}
    const double r=1.0/std::cos(a),y=r*std::sin(a);
    if (y>=-.12 && y<=.12) {calibrated_scan[i]=r-.069;}
  }
  auto real_mount=front_corridor(calibrated_scan,-pi,step,.15,12,calibrated,.76,.31);
  check(real_mount.valid && !real_mount.clear && real_mount.obstacle_width_m>=.18,
    "production a1 yaw and range offset detect width");
  std::fill(calibrated_scan.begin(),calibrated_scan.end(),INFINITY);
  check(front_corridor(calibrated_scan,-pi,step,.15,12,calibrated,.76,.31).clear,
    "production a1 half-plane FOV covers front rectangle");
  return errors?1:0;
}
