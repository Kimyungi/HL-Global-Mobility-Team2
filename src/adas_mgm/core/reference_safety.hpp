#ifndef ADAS_MGM__CORE__REFERENCE_SAFETY_HPP_
#define ADAS_MGM__CORE__REFERENCE_SAFETY_HPP_
#include "mgm_types.hpp"
namespace adas_mgm
{
bool reference_geometry_valid(const CorePoint * points, int32_t count);
ReferenceStatus provider_reference(const CoreSnapshot & s, uint8_t source);
// Called after assemble/merge; independent of arbitration and rate limiting.
void final_reference_gate(CoreOutput & out, CoreState & st);
}
#endif
