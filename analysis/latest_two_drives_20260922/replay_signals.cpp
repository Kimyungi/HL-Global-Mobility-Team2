// Compile against the recording version; refuse incompatible binary layouts.
#include <fstream>
#include <iostream>
#include <iomanip>
#include "tools/dump_reader.hpp"
#include "core/mgm_step.hpp"
using namespace adas_mgm;
int main(int argc,char**argv) {
 if(argc!=2)return 1;
 std::ifstream f(argv[1],std::ios::binary);DumpHeader h{};
 if(!read_dump_header(f,h))return 2;
 CoreState st;mgm_init(st,h.params);CoreSnapshot s{};size_t tick=0;
 std::cout<<"tick,time_ns,v_ref,state,gps_x,gps_y,gps_position_valid,top,navigation,avoidance,signal,safety,mission,mission_type,reference_source,speed_owner,last_mission_phase,last_mission_route_id,last_mission_fallback\n"<<std::setprecision(9);
 while(f.read(reinterpret_cast<char*>(&s),sizeof(s))) {
  const auto o=mgm_step(s,st);
  std::cout<<tick++<<','<<s.event_time_ns<<','<<o.v_ref<<','<<+o.state<<','<<s.gps_x<<','<<s.gps_y<<','<<s.gps_position_valid<<','<<int(o.top)<<','<<int(o.nav)<<','<<int(o.avoid)<<','<<int(o.signal)<<','<<int(o.safety)<<','<<int(o.mission)<<','<<int(o.mission_type)<<','<<+o.path_source<<','<<int(o.speed_owner)<<','<<int(o.last_mission.phase)<<','<<+o.last_mission.route_id<<','<<o.last_mission.fallback<<'\n';
 }
}
