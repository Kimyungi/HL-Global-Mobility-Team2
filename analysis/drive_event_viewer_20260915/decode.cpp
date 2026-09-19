#include <fstream>
#include <iostream>
#include <iomanip>
#include "tools/dump_format.hpp"
using namespace adas_mgm;
int main(int argc, char**argv){
 if(argc!=3)return 1;
 std::ifstream in(argv[1],std::ios::binary); uint32_t h[4]{};
 in.read(reinterpret_cast<char*>(h),sizeof(h));
 if(h[0]!=kDumpMagic||h[1]!=kDumpVersion||h[2]!=sizeof(CoreSnapshot)||h[3]!=sizeof(CoreParams)){std::cerr<<"ABI mismatch\n";return 2;}
 in.seekg(h[3],std::ios::cur); std::ofstream out(argv[2]);
 out<<"tick,event_time_ns,monotonic_ns,vehicle_speed,vehicle_speed_valid,external_stop,autonomous_enabled,lane_confidence,gps_valid,lidar_valid,avoid_updated,avoid_maneuver_done,avoid_path_n,avoid_generation,avoid_age_s,avoid_timeout_s,obstacle,avoidable,ttc,auto_estop,estop,parking_valid,parking_wall_complete,parking_ready,parking_search,parking_request_id,traffic_fail_safe,avoid_x,avoid_y,avoid_yaw,avoid_k,gps_x,gps_y,gps_heading_valid,rear_clear,red,green,traffic_stop_required,stopline,stop_distance,gps_position_valid\n";
 CoreSnapshot s{}; long long i=0;
 while(in.read(reinterpret_cast<char*>(&s),sizeof(s))){
  auto r=s.references[MGM_SRC_AVOID];
  out<<i++<<','<<s.event_time_ns<<','<<s.monotonic_ns<<','<<std::setprecision(10)<<s.vehicle_speed<<','<<s.vehicle_speed_valid<<','<<s.external_stop<<','<<s.autonomous_enabled<<','<<s.lane_confidence<<','<<s.gps_valid<<','<<s.lidar_valid<<','<<s.avoid_updated<<','<<s.avoid_maneuver_done<<','<<s.avoid_path.n<<','<<r.generation<<','<<r.age_s<<','<<r.timeout_s<<','<<s.avoid_obstacle_detected<<','<<s.avoid_avoidable<<','<<s.avoid_ttc<<','<<s.auto_estop<<','<<s.estop<<','<<s.parking_valid<<','<<s.parking_wall_acquisition_complete<<','<<s.parking_preparation_ready<<','<<s.parking_search_active<<','<<s.parking_request_id<<','<<s.traffic_fail_safe_stop<<','<<s.avoid_path.pts[0].x<<','<<s.avoid_path.pts[0].y<<','<<s.avoid_path.pts[0].yaw<<','<<s.avoid_path.pts[0].curvature<<','<<s.gps_x<<','<<s.gps_y<<','<<s.gps_heading_valid<<','<<s.estop_rear_clear<<','<<s.traffic_red_active<<','<<s.traffic_green_active<<','<<s.traffic_stop_required<<','<<s.traffic_stopline_detected<<','<<s.traffic_stop_distance<<','<<s.gps_position_valid<<'\n';
 }
 std::cout<<"Decoded "<<i<<" records; ABI validated\n";
}
