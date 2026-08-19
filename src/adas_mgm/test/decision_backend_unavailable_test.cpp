#include <cstdio>
#include <stdexcept>
#include <string>

#include "src/decision_backend.hpp"

using adas_mgm::CoreParams;
using adas_mgm::DecisionBackend;

namespace
{

CoreParams makeParams()
{
  CoreParams params{};
  params.lane_conf_exit = 0.35f;
  params.lane_conf_return = 0.70f;
  params.n_cycles = 50;
  params.v_base = 0.6f;
  params.v_accel_zone = 1.0f;
  params.v_narrow = 0.2f;
  params.ttc_stop = 0.8f;
  params.blend_cycles = 10;
  params.a_up = 0.5f;
  params.a_down = 1.5f;
  params.wrongway_yaw = 2.1f;
  params.wrongway_cycles = 50;
  params.avoid_return_hold_cycles = 300;
  params.lane_entry_max_cross = 0.5f;
  params.avoid_max_cycles = 1200;
  return params;
}

}  // namespace

int main()
{
  const CoreParams params = makeParams();
  DecisionBackend core("core", false, params);
  if (core.name() != "core") {
    std::fprintf(stderr, "default core backend was not selected\n");
    return 1;
  }

  try {
    DecisionBackend unavailable("generated", true, params);
    std::fprintf(stderr, "generated backend unexpectedly started without compile-time opt-in\n");
    return 1;
  } catch (const std::runtime_error & error) {
    if (std::string(error.what()).find("ADAS_MGM_ENABLE_GENERATED_BACKEND=ON") ==
      std::string::npos)
    {
      std::fprintf(stderr, "unexpected unavailable-backend error: %s\n", error.what());
      return 1;
    }
  }

  std::printf("unavailable generated backend test: pass\n");
  return 0;
}
