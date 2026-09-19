#ifndef ADAS_MGM__SRC__REFERENCE_CLOCK_HPP_
#define ADAS_MGM__SRC__REFERENCE_CLOCK_HPP_
#include "core/manager_types.hpp"
#include <algorithm>
#include <cstdint>
namespace adas_mgm
{
// Convert actual ROS input stamps to a monotonic age once per generation.
// Repeated publications (even with a refreshed message header) never reset it.
class ReferenceClock
{
public:
  ReferenceSample observe(int64_t stamp_ns, int64_t ros_now_ns, int64_t steady_now_ns,
    int64_t timeout_ns)
  {
    const bool known = stamp_ns > 0 && stamp_ns <= ros_now_ns;
    if (known && stamp_ns > last_stamp_ns_) {
      last_stamp_ns_ = stamp_ns;
      age_at_observation_ns_ = ros_now_ns - stamp_ns;
      observation_ns_ = steady_now_ns;
    }
    ReferenceSample sample{};
    sample.timeout_s = static_cast<float>(timeout_ns * 1e-9);
    sample.age_s = last_stamp_ns_ == 0 ? -1.0f : static_cast<float>(
      (age_at_observation_ns_ + std::max<int64_t>(0, steady_now_ns - observation_ns_)) * 1e-9);
    if (known && stamp_ns == last_stamp_ns_) {sample.generation = static_cast<uint64_t>(stamp_ns);}
    return sample;
  }
private:
  int64_t last_stamp_ns_{0};
  int64_t observation_ns_{0};
  int64_t age_at_observation_ns_{0};
};
}
#endif
