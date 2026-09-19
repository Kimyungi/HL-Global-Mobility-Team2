#ifndef ADAS_MGM__CORE__MANAGER_STEP_HPP_
#define ADAS_MGM__CORE__MANAGER_STEP_HPP_
#include "mgm_types.hpp"
namespace adas_mgm
{
uint8_t legacy_state_projection(const CoreState & st);
bool update_escape(const CoreSnapshot & s, CoreState & st, bool eligible);
CoreOutput existing_source_request(
  const CoreSnapshot & s, const CoreState & st, uint8_t state);
void manager_transition(const CoreSnapshot & s, CoreState & st);
// Pure decision layer; the existing assemble/merge blocks execute its output.
CoreOutput manager_decision(const CoreSnapshot & s, const CoreState & st);
}
#endif
