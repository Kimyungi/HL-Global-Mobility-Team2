#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>

#include "src/generated_adapter.hpp"

extern "C"
{
#include "ADAS_MGR2.h"
}

using adas_mgm::CoreParams;
using adas_mgm::CoreSnapshot;
using adas_mgm::GeneratedMgmAdapter;

static_assert(!std::is_copy_constructible<GeneratedMgmAdapter>::value);
static_assert(!std::is_copy_assignable<GeneratedMgmAdapter>::value);
static_assert(!std::is_move_constructible<GeneratedMgmAdapter>::value);
static_assert(!std::is_move_assignable<GeneratedMgmAdapter>::value);

namespace
{

int failures = 0;

void check(bool condition, const char * message)
{
  if (!condition) {
    ++failures;
    std::fprintf(stderr, "FAIL: %s\n", message);
  }
}

template<typename Exception, typename Function>
void expectException(Function && function, const char * message_fragment, const char * label)
{
  try {
    function();
    check(false, label);
  } catch (const Exception & error) {
    check(std::string(error.what()).find(message_fragment) != std::string::npos, label);
  } catch (...) {
    check(false, label);
  }
}

CoreParams validParams()
{
  CoreParams params{};
  params.lane_conf_exit = 0.3f;
  params.lane_conf_return = 0.7f;
  params.n_cycles = 20;
  params.v_base = 0.5f;
  params.v_accel_zone = 1.0f;
  params.v_narrow = 0.2f;
  params.ttc_stop = 0.8f;
  params.blend_cycles = 10;
  params.a_up = 0.5f;
  params.a_down = 1.5f;
  params.wrongway_yaw = 2.1f;
  params.wrongway_cycles = 20;
  params.avoid_return_hold_cycles = 30;
  params.lane_entry_max_cross = 0.5f;
  params.avoid_max_cycles = 120;
  params.v_avoid = 0.4f;
  params.stop_zone_hold_cycles = 30;
  params.avoid_zone_only = 1;
  return params;
}

CoreSnapshot validSnapshot()
{
  CoreSnapshot snapshot{};
  snapshot.lane_confidence = 0.9f;
  snapshot.lane_updated = true;
  snapshot.lane_path.n = 1;
  snapshot.lane_path.pts[0].x = 1.0f;
  snapshot.estop = true;
  return snapshot;
}

template<typename Mutator>
void expectInvalidParams(Mutator && mutator, const char * field)
{
  CoreParams params = validParams();
  mutator(params);
  expectException<std::invalid_argument>(
    [&params]() {GeneratedMgmAdapter adapter(params);}, field, field);
}

}  // namespace

int main()
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const float inf = std::numeric_limits<float>::infinity();

  expectInvalidParams([nan](CoreParams & p) {p.lane_conf_exit = nan;}, "lane_conf_exit");
  expectInvalidParams([inf](CoreParams & p) {p.lane_conf_return = inf;}, "lane_conf_return");
  expectInvalidParams([](CoreParams & p) {p.lane_conf_exit = -0.1f;}, "lane_conf_exit");
  expectInvalidParams([](CoreParams & p) {p.lane_conf_return = 1.1f;}, "lane_conf_return");
  expectInvalidParams([](CoreParams & p) {p.lane_conf_exit = 0.8f;}, "lane_conf_exit");
  expectInvalidParams([](CoreParams & p) {p.lane_conf_exit = 2.0f;}, "lane_conf_exit");
  expectInvalidParams([](CoreParams & p) {p.lane_conf_return = 2.0f;}, "lane_conf_return");
  expectInvalidParams([](CoreParams & p) {p.n_cycles = 0;}, "n_cycles");
  expectInvalidParams([nan](CoreParams & p) {p.v_base = nan;}, "v_base");
  expectInvalidParams([](CoreParams & p) {p.v_base = -0.1f;}, "v_base");
  expectInvalidParams([inf](CoreParams & p) {p.v_accel_zone = inf;}, "v_accel_zone");
  expectInvalidParams([](CoreParams & p) {p.v_accel_zone = 0.4f;}, "v_accel_zone");
  expectInvalidParams([nan](CoreParams & p) {p.v_narrow = nan;}, "v_narrow");
  expectInvalidParams([](CoreParams & p) {p.v_narrow = -0.1f;}, "v_narrow");
  expectInvalidParams([inf](CoreParams & p) {p.ttc_stop = inf;}, "ttc_stop");
  expectInvalidParams([](CoreParams & p) {p.ttc_stop = -0.1f;}, "ttc_stop");
  expectInvalidParams([](CoreParams & p) {p.blend_cycles = -1;}, "blend_cycles");
  expectInvalidParams([nan](CoreParams & p) {p.a_up = nan;}, "a_up");
  expectInvalidParams([](CoreParams & p) {p.a_up = 0.0f;}, "a_up");
  expectInvalidParams([inf](CoreParams & p) {p.a_down = inf;}, "a_down");
  expectInvalidParams([](CoreParams & p) {p.a_down = 0.0f;}, "a_down");
  expectInvalidParams([nan](CoreParams & p) {p.wrongway_yaw = nan;}, "wrongway_yaw");
  expectInvalidParams([](CoreParams & p) {p.wrongway_yaw = -0.1f;}, "wrongway_yaw");
  expectInvalidParams([](CoreParams & p) {p.wrongway_yaw = 3.2f;}, "wrongway_yaw");
  expectInvalidParams([](CoreParams & p) {p.wrongway_cycles = 0;}, "wrongway_cycles");
  expectInvalidParams(
    [](CoreParams & p) {p.avoid_return_hold_cycles = -1;}, "avoid_return_hold_cycles");
  expectInvalidParams(
    [nan](CoreParams & p) {p.lane_entry_max_cross = nan;}, "lane_entry_max_cross");
  expectInvalidParams([inf](CoreParams & p) {p.v_avoid = inf;}, "v_avoid");
  expectInvalidParams([](CoreParams & p) {p.avoid_zone_only = 2;}, "avoid_zone_only");
  expectInvalidParams(
    [](CoreParams & p) {p.escape_after_cycles = 1;},
    "escape_after_cycles");
  expectInvalidParams(
    [](CoreParams & p) {p.traffic_state_enabled = 1;},
    "traffic_state_enabled");

  const CoreParams params = validParams();
  const CoreSnapshot snapshot = validSnapshot();

  {
    GeneratedMgmAdapter first(params);
    expectException<std::logic_error>(
      [&params]() {GeneratedMgmAdapter second(params);}, "only one process-wide instance",
      "second simultaneous adapter must be rejected");
    first.step(snapshot);
  }

  // The existing real-vehicle gps_only launch intentionally uses (2, 2) to
  // force LANE -> WAYPOINT and prevent a return to LANE.
  {
    CoreParams gps_only = params;
    gps_only.lane_conf_exit = 2.0f;
    gps_only.lane_conf_return = 2.0f;
    GeneratedMgmAdapter adapter(gps_only);
    adapter.step(snapshot);
  }

  // Releasing the first adapter must permit a new process-wide owner.
  {
    GeneratedMgmAdapter recreated(params);
    recreated.step(snapshot);

    CoreParams invalid_reset = params;
    invalid_reset.a_up = 0.0f;
    expectException<std::invalid_argument>(
      [&recreated, &invalid_reset]() {recreated.reset(invalid_reset);}, "a_up",
      "invalid reset must be rejected before mutating generated state");
    check(MGM_a_up == params.a_up, "invalid reset must preserve generated parameters");
    recreated.step(snapshot);

    std::string worker_error;
    std::thread wrong_thread([&]() {
        try {
          recreated.reset(params);
        } catch (const std::exception & error) {
          worker_error = error.what();
        }
      });
    wrong_thread.join();
    check(
      worker_error.find("non-owning thread") != std::string::npos,
      "reset from a non-owning thread must be rejected");

    constexpr const char * injected_error = "injected model failure";
    rtmSetErrorStatus(ADAS_MGR2_M, injected_error);
    expectException<std::runtime_error>(
      [&recreated, &snapshot]() {recreated.step(snapshot);}, injected_error,
      "model errorStatus must be preserved in the step exception");

    // Reset on the owner thread clears a prior model error and updates all
    // 18 generated tunable parameters.
    CoreParams updated = params;
    updated.lane_conf_exit = 0.25f;
    updated.lane_conf_return = 0.75f;
    updated.n_cycles = 12;
    updated.v_base = 0.4f;
    updated.v_accel_zone = 0.8f;
    updated.v_narrow = 0.15f;
    updated.ttc_stop = 0.6f;
    updated.blend_cycles = 6;
    updated.a_up = 0.4f;
    updated.a_down = 1.2f;
    updated.wrongway_yaw = 1.9f;
    updated.wrongway_cycles = 12;
    updated.avoid_return_hold_cycles = 24;
    updated.lane_entry_max_cross = 0.35f;
    updated.avoid_max_cycles = 80;
    updated.v_avoid = 0.35f;
    updated.stop_zone_hold_cycles = 20;
    updated.avoid_zone_only = 0;
    recreated.reset(updated);
    check(rtmGetErrorStatus(ADAS_MGR2_M) == nullptr, "reset must clear model errorStatus");
    check(
      MGM_lane_conf_exit == updated.lane_conf_exit &&
      MGM_lane_conf_return == updated.lane_conf_return &&
      MGM_n_cycles == updated.n_cycles &&
      MGM_v_base == updated.v_base &&
      MGM_v_accel_zone == updated.v_accel_zone &&
      MGM_v_narrow == updated.v_narrow &&
      MGM_ttc_stop == updated.ttc_stop &&
      MGM_blend_cycles == updated.blend_cycles &&
      MGM_a_up == updated.a_up &&
      MGM_a_down == updated.a_down &&
      MGM_wrongway_yaw == updated.wrongway_yaw &&
      MGM_wrongway_cycles == updated.wrongway_cycles &&
      MGM_avoid_return_hold_cycles == updated.avoid_return_hold_cycles &&
      MGM_lane_entry_max_cross == updated.lane_entry_max_cross &&
      MGM_avoid_max_cycles == updated.avoid_max_cycles &&
      MGM_v_avoid == updated.v_avoid &&
      MGM_stop_zone_hold_cycles == updated.stop_zone_hold_cycles &&
      MGM_avoid_zone_only == updated.avoid_zone_only,
      "reset must update all 18 generated parameters");
    recreated.step(snapshot);
  }

  // The first public operation, rather than construction, owns the adapter.
  // This permits construction on a ROS executor thread and stepping on the
  // dedicated 10 ms loop thread without permitting later cross-thread calls.
  {
    GeneratedMgmAdapter runtime_handoff(params);
    std::string worker_error;
    std::thread loop_thread([&]() {
        try {
          runtime_handoff.step(snapshot);
          runtime_handoff.reset(params);
        } catch (const std::exception & error) {
          worker_error = error.what();
        }
      });
    loop_thread.join();
    check(worker_error.empty(), "first step and reset on the loop thread must succeed");
    expectException<std::logic_error>(
      [&runtime_handoff, &snapshot]() {runtime_handoff.step(snapshot);},
      "non-owning thread", "step after ownership handoff must reject the constructor thread");
  }

  std::printf("generated adapter safety: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
