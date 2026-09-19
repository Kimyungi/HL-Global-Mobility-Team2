#include "core/zone_step.hpp"
#include <iostream>
using namespace adas_mgm;
int main(){ZoneState st{};ZoneSnapshot s{};s.zone_valid=true;s.count=1;s.observations[0]={1,ZoneType::MISSION_ZONE, MissionType::T_PARKING,0,false,0};int hit,total=0,entered=0;while(std::cin>>hit){++s.generation;s.observations[0].in_zone=hit;zone_step(s,true,st,5,5);total+=hit;entered+=st.contexts[1].zone_entered;}std::cout<<total<<" "<<entered<<"\n";}
