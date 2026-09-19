#ifndef ADAS_MGM_LEGACY_STATE_IDS_HPP
#define ADAS_MGM_LEGACY_STATE_IDS_HPP
#include "manager_types.hpp"
// Historical-profile replay only. These are NOT states of State v09.16.
// Preserve recorded numeric IDs without exposing them in current state enums.
namespace adas_mgm { namespace legacy {
constexpr SafetyState AUTO_ESTOP = static_cast<SafetyState>(1);
constexpr SafetyState REVERSE_RECOVERY = static_cast<SafetyState>(2);
constexpr MissionState MISSION_PREPARE = static_cast<MissionState>(2);
}}
#endif
