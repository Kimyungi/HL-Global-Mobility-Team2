#ifndef ADAS_MGM__CORE__ZONE_STEP_HPP_
#define ADAS_MGM__CORE__ZONE_STEP_HPP_
#include "manager_types.hpp"
namespace adas_mgm
{
// Complete membership snapshot -> per-ID edges. Invalid observations preserve
// spatial context without generating entry/exit events or reference points.
void zone_step(const ZoneSnapshot & input, bool gps_usable, ZoneState & state,
  int32_t enter_samples, int32_t exit_samples);
}
#endif
