#include <fstream>
#include <iostream>
#include <iomanip>
#include "core/mgm_step.hpp"
#include "tools/dump_format.hpp"
using namespace adas_mgm;
int main(int argc,char**argv){
if(argc!=3)return 1;std::ifstream in(argv[1],std::ios::binary);uint32_t h[4]{};in.read((char*)h,16);
if(h[0]!=kDumpMagic||h[1]!=kDumpVersion||h[2]!=sizeof(CoreSnapshot)||h[3]!=sizeof(CoreParams)){std::cerr<<"ABI mismatch";return 2;}
CoreParams p{};in.read((char*)&p,sizeof(p));CoreState st;mgm_init(st,p);
std::cerr<<"ABI "<<h[1]<<" params="<<h[3]<<" snapshot="<<h[2]<<" lane_conf_exit="<<p.lane_conf_exit<<" lane_conf_return="<<p.lane_conf_return<<" avoid_zone_only="<<p.avoid_zone_only<<" v_base="<<p.v_base<<" v_avoid="<<p.v_avoid<<"\n";
std::ofstream out(argv[2]);out<<std::setprecision(16);
out<<"tick,t,gps_x,gps_y,gps_position_valid,cross,gps_yaw_error,gps_heading_valid,gps_valid,lidar_valid,lane_conf,go,v,v_valid,obstacle,avoidable,done,avoid_n,avoid_age,avoid_timeout,gps_n,estop,auto_estop,avoid_zone,gps_only_zone,top,nav,avoid,safety,source,v_ref,ref_valid,ref_fresh,ref_age,blocked,route_index,route_connecting,gps_point_x,gps_point_y,gps_point_yaw,gps_point_curvature,avoid_point_x,avoid_point_y,avoid_point_yaw,avoid_point_curvature,target_point_x,target_point_y,target_point_yaw,target_point_curvature,target_n\n";
CoreSnapshot s{};long long i=0;while(in.read((char*)&s,sizeof(s))){auto o=mgm_step(s,st);out<<i++<<','<<s.event_time_ns/1e9<<','<<s.gps_x<<','<<s.gps_y<<','<<s.gps_position_valid<<','<<s.gps_cross_track<<','<<s.gps_station_yaw_error<<','<<s.gps_heading_valid<<','<<s.gps_valid<<','<<s.lidar_valid<<','<<s.lane_confidence<<','<<s.autonomous_enabled<<','<<s.vehicle_speed<<','<<s.vehicle_speed_valid<<','<<s.avoid_obstacle_detected<<','<<s.avoid_avoidable<<','<<s.avoid_maneuver_done<<','<<s.avoid_path.n<<','<<s.references[MGM_SRC_AVOID].age_s<<','<<s.references[MGM_SRC_AVOID].timeout_s<<','<<s.gps_path.n<<','<<s.estop<<','<<s.auto_estop<<','<<s.gps_avoid_zone<<','<<s.gps_gps_only_zone<<','<<int(o.top)<<','<<int(o.nav)<<','<<int(o.avoid)<<','<<int(o.safety)<<','<<int(o.path_source)<<','<<o.v_ref<<','<<o.selected_reference.valid<<','<<o.selected_reference.fresh<<','<<o.selected_reference.age_s<<','<<o.reference_motion_blocked<<','<<o.route.index<<','<<o.route.connecting<<','<<s.gps_path.pts[0].x<<','<<s.gps_path.pts[0].y<<','<<s.gps_path.pts[0].yaw<<','<<s.gps_path.pts[0].curvature<<','<<s.avoid_path.pts[0].x<<','<<s.avoid_path.pts[0].y<<','<<s.avoid_path.pts[0].yaw<<','<<s.avoid_path.pts[0].curvature<<','<<o.ref_points[0].x<<','<<o.ref_points[0].y<<','<<o.ref_points[0].yaw<<','<<o.ref_points[0].curvature<<','<<o.n_points<<'\n';}
std::cerr<<i<<" rows\n";return 0;}
