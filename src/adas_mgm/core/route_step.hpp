#ifndef ADAS_MGM__CORE__ROUTE_STEP_HPP_
#define ADAS_MGM__CORE__ROUTE_STEP_HPP_
#include "mgm_types.hpp"
namespace adas_mgm {
void route_reset(const RouteControl & previous, const CoreSnapshot &, CoreState &);
void route_observe(const CoreSnapshot &, CoreState &);
void route_step(const CoreSnapshot &, CoreState &);
bool route_stop(const CoreSnapshot &, const CoreState &);
}
#endif
