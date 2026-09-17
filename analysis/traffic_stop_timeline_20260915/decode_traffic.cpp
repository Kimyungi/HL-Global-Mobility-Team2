#include <fstream>
#include <iostream>
#include <iomanip>
#include "abi_v28/mgm_types.hpp"
using namespace adas_mgm;
int main(int argc,char** argv){
 if(argc!=3)return 1;
 std::ifstream in(argv[1],std::ios::binary);uint32_t h[4]{};
 in.read(reinterpret_cast<char*>(h),16);
 if(h[0]!=0x314d474d||h[1]!=28||h[2]!=sizeof(CoreSnapshot)||h[3]!=152){std::cerr<<"ABI mismatch "<<sizeof(CoreSnapshot)<<"\n";return 2;}
 in.seekg(h[3],std::ios::cur);
 std::ofstream out(argv[2]);out<<"tick,event_time_ns,stopline_detected,stop_distance,red,green,vehicle_speed,vehicle_speed_valid\n";
 CoreSnapshot s{};int tick=0;
 while(in.read(reinterpret_cast<char*>(&s),sizeof(s))){out<<tick++<<','<<s.event_time_ns<<','<<s.traffic_stopline_detected<<','<<std::setprecision(10)<<s.traffic_stop_distance<<','<<s.traffic_red_active<<','<<s.traffic_green_active<<','<<s.vehicle_speed<<','<<s.vehicle_speed_valid<<'\n';}
 std::cout<<"v28 ABI validated: "<<tick<<" samples\n";
}
