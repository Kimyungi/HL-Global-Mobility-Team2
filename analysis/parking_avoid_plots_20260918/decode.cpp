#include <fstream>
#include <iostream>
#include <iomanip>
#include "core/mgm_step.hpp"
#include "tools/dump_format.hpp"
using namespace adas_mgm;
int main(int argc,char**argv){
 if(argc!=2)return 1;
 std::ifstream f(argv[1],std::ios::binary);DumpHeader h{};f.read((char*)&h,sizeof(h));
 if(h.magic!=kDumpMagic||h.version!=kDumpVersion||h.snapshot_size!=sizeof(CoreSnapshot)||h.params_size!=sizeof(CoreParams)){std::cerr<<"Incompatible dump\n";return 2;}
 CoreState st{};mgm_init(st,h.params);CoreSnapshot s{};int tick=0;
 std::cout<<"tick,time,v_ref,top,navigation,avoidance,safety,mission,source,x,y,position_valid\n"<<std::fixed<<std::setprecision(9);
 while(f.read((char*)&s,sizeof(s))){auto o=mgm_step(s,st);
 std::cout<<tick++<<","<<s.event_time_ns/1e9<<","<<o.v_ref<<","<<int(o.top)<<","<<int(o.nav)<<","<<int(o.avoid)<<","<<int(o.safety)<<","<<int(o.mission)<<","<<int(o.path_source)<<","<<s.gps_x<<","<<s.gps_y<<","<<s.gps_position_valid<<"\n";}
 if(f.gcount()){std::cerr<<"Partial final record: "<<f.gcount()<<" bytes\n";return 3;}
}
