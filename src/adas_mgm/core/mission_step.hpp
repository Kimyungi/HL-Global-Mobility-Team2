#ifndef ADAS_MGM__CORE__MISSION_STEP_HPP_
#define ADAS_MGM__CORE__MISSION_STEP_HPP_
#include "mgm_types.hpp"
namespace adas_mgm
{
CalibrationState parking_calibration(const CoreParams & params);
bool mission_search_zone_known(const CoreState & state);
// Immediate-entry PARKING retains GPS control until this request is ready.
bool mission_searches_along_gps(const CoreState & state);
bool mission_reference_authority(const CoreState & state);
void cancel_mission(CoreState & st, MissionCancelReason reason);
// Returns true when authority/request ended; caller reselects current navigation.
bool mission_step(const CoreSnapshot & s, CoreState & st);
}
#endif
