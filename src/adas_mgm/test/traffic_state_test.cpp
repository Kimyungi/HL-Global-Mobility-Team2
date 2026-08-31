#include <cmath>
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
  CoreParams p{};
  p.lane_conf_exit = 0.4f;
  p.lane_conf_return = 0.6f;
  p.n_cycles = 20;
  p.v_base = 0.5f;
  p.v_accel_zone = 1.0f;
  p.v_narrow = 0.2f;
  p.ttc_stop = 0.8f;
  p.a_up = 1.0f;
  p.a_down = 1.5f;
  p.wrongway_cycles = 20;
  p.traffic_state_enabled = 1;
  p.traffic_stop_offset = 0.0f;
  p.traffic_min_decel = 0.05f;
  return p;
}

CoreSnapshot input()
{
  CoreSnapshot s{};
  s.lane_confidence = 0.9f;
  s.lane_path.n = 1;
  s.lane_path.pts[0] = CorePoint{1.5f, 0.0f, 0.0f, 0.0f};
  s.avoid_ttc = 100.0f;
  s.vehicle_speed = 0.5f;
  s.vehicle_speed_valid = true;
  return s;
}

}  // namespace

int main()
{
  CoreState state{};
  mgm_init(state, params());

  CoreSnapshot s = input();
  for (int tick = 0; tick < 60; ++tick) {
    mgm_step(s, state);
  }
  s.traffic_red_active = true;
  CoreOutput out = mgm_step(s, state);
  check(out.state == MGM_STATE_TRAFFIC, "확정 적색이면 즉시 TRAFFIC에 진입해야 한다");
  check(out.path_source == MGM_SRC_LANE, "TRAFFIC은 차선 경로를 유지해야 한다");
  check(!state.traffic_distance_latched, "정지선 관측 전에는 거리를 래치하면 안 된다");

  s.traffic_stopline_detected = true;
  s.traffic_stop_distance = 2.0f;
  out = mgm_step(s, state);
  check(state.traffic_distance_latched, "최초 유효 정지선 거리를 래치해야 한다");
  check(std::fabs(state.traffic_stopline_distance - 2.0f) < 1.0e-5f,
    "래치 틱에는 실차속도를 중복 적분하면 안 된다");

  const float captured_command = out.v_ref;
  // 뒤늦은 영상 거리는 흔들리거나 곧 사라질 수 있으므로 최초 래치를 덮지 않는다.
  s.traffic_stopline_detected = true;
  s.traffic_stop_distance = 1.2f;
  out = mgm_step(s, state);
  check(std::fabs(state.traffic_stopline_distance - 1.995f) < 1.0e-4f,
    "후속 영상값 대신 실차속도×10ms로 남은 거리를 갱신해야 한다");
  check(out.v_ref < captured_command,
    "남은 거리가 줄면 제동 프로파일의 목표속도도 감소해야 한다");
  s.traffic_stopline_detected = false;
  s.traffic_stop_distance = -1.0f;

  for (int tick = 0; tick < 500; ++tick) {
    out = mgm_step(s, state);
  }
  check(state.traffic_stopline_distance == 0.0f,
    "적분 거리는 0 아래로 내려가면 안 된다");
  check(std::fabs(out.v_ref) < 1.0e-5f,
    "목표 정지 위치에 도달하면 v_ref가 0이어야 한다");
  check(out.state == MGM_STATE_TRAFFIC,
    "정지선이 사라지거나 적색 관측이 끊겨도 TRAFFIC을 유지해야 한다");

  s.traffic_red_active = false;
  s.traffic_green_active = true;
  out = mgm_step(s, state);
  check(out.state == MGM_STATE_LANE, "확정 초록이면 즉시 LANE으로 복귀해야 한다");
  check(!state.traffic_distance_latched, "TRAFFIC 탈출 시 거리 래치를 폐기해야 한다");

  mgm_init(state, params());
  s = input();
  s.traffic_red_active = true;
  s.vehicle_speed_valid = false;
  out = mgm_step(s, state);
  check(out.state == MGM_STATE_TRAFFIC && out.immediate_stop && out.v_ref == 0.0f,
    "TRAFFIC에서 실차속도 피드백이 없으면 fail-safe 정지해야 한다");

  mgm_init(state, params());
  state.state = MGM_STATE_WAYPOINT;
  s = input();
  s.traffic_red_active = true;
  out = mgm_step(s, state);
  check(out.state == MGM_STATE_TRAFFIC,
    "WAYPOINT에서도 확정 적색이면 TRAFFIC으로 진입해야 한다");
  s.traffic_red_active = false;
  s.traffic_green_active = true;
  out = mgm_step(s, state);
  check(out.state == MGM_STATE_LANE,
    "진입 전 상태와 관계없이 확정 초록은 LANE으로 복귀해야 한다");

  std::printf("traffic state test: failures=%d\n", failures);
  return failures == 0 ? 0 : 1;
}
