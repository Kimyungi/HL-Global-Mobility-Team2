// GPS-triggered parking must return to whichever normal driving mode owned
// the vehicle before PARKING: camera LANE or GPS WAYPOINT.
#include <cstdio>

#include "core/mgm_step.hpp"

using namespace adas_mgm;

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

CoreParams params()
{
  CoreParams value{};
  value.lane_conf_exit = 0.4f;
  value.lane_conf_return = 0.7f;
  value.n_cycles = 2;
  value.v_base = 0.5f;
  value.v_accel_zone = 1.0f;
  value.v_narrow = 0.2f;
  value.ttc_stop = 1.0f;
  value.a_up = 100.0f;
  value.a_down = 100.0f;
  value.lane_entry_max_cross = 0.5f;
  return value;
}

CoreSnapshot input()
{
  CoreSnapshot value{};
  value.lane_confidence = 1.0f;
  value.gps_cross_track = 0.0f;
  value.avoid_ttc = 100.0f;
  value.lane_path.n = 1;
  value.gps_path.n = 1;
  value.parking_path.n = 1;
  value.lane_path.pts[0] = {1.5f, 0.0f, 0.0f, 0.0f};
  value.gps_path.pts[0] = {1.5f, 0.0f, 0.0f, 0.0f};
  value.parking_path.pts[0] = {1.0f, 0.0f, 0.0f, 0.0f};
  value.parking_v_suggest = 0.2f;
  return value;
}

void enterAndCompleteParking(CoreState & state, CoreSnapshot & snapshot)
{
  snapshot.gps_parking_zone = true;
  snapshot.parking_space_found = true;
  CoreOutput output = mgm_step(snapshot, state);
  check(output.state == MGM_STATE_PARKING, "GPS zone must enter PARKING");

  snapshot.parking_done = true;
  snapshot.parking_space_found = false;
  mgm_step(snapshot, state);
}

void testLaneReturnsToLane()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot snapshot = input();
  enterAndCompleteParking(state, snapshot);
  check(state.state == MGM_STATE_LANE, "LANE-origin parking must return to LANE");
}

void testWaypointReturnsToWaypoint()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot snapshot = input();
  snapshot.lane_confidence = 0.0f;
  mgm_step(snapshot, state);
  mgm_step(snapshot, state);
  check(state.state == MGM_STATE_WAYPOINT, "test setup must enter WAYPOINT");

  enterAndCompleteParking(state, snapshot);
  check(
    state.state == MGM_STATE_WAYPOINT,
    "WAYPOINT-origin parking must return to WAYPOINT");
}

void testParkingSearchUsesWaypoint()
{
  CoreState state{};
  mgm_init(state, params());
  CoreSnapshot snapshot = input();
  snapshot.gps_parking_zone = true;
  snapshot.parking_space_found = false;

  CoreOutput output = mgm_step(snapshot, state);
  check(
    output.state == MGM_STATE_WAYPOINT,
    "parking-zone search must immediately use GPS WAYPOINT");
  check(
    output.path_source == MGM_SRC_GPS,
    "parking-zone search must select the GPS path");

  for (int i = 0; i < 5; ++i) {
    output = mgm_step(snapshot, state);
  }
  check(
    output.state == MGM_STATE_WAYPOINT,
    "parking-zone search must not return to LANE before space detection");

  snapshot.parking_space_found = true;
  output = mgm_step(snapshot, state);
  check(
    output.state == MGM_STATE_PARKING,
    "detected parking space must hand waypoint approach to PARKING");
}

}  // namespace

int main()
{
  testLaneReturnsToLane();
  testWaypointReturnsToWaypoint();
  testParkingSearchUsesWaypoint();
  if (failures == 0) {
    std::puts("parking_return_state_test: PASS");
  }
  return failures == 0 ? 0 : 1;
}
