#ifndef ADAS_MGM__TYPES_HPP_
#define ADAS_MGM__TYPES_HPP_

#include "fma_interfaces/msg/lane_path.hpp"
#include "fma_interfaces/msg/gps_path.hpp"
#include "fma_interfaces/msg/avoid_status.hpp"
#include "fma_interfaces/msg/parking_status.hpp"
#include "fma_interfaces/msg/traffic_stop.hpp"
#include "fma_interfaces/msg/estop_request.hpp"

namespace adas_mgm
{

// 매 10ms 틱이 읽는 "최신 인지 스냅샷" — 인지 콜백은 이 구조체를 갱신만 하고
// 루프는 복사해서 쓴다 (pull 방식, CLAUDE.md §5.2)
struct Snapshot
{
  fma_interfaces::msg::LanePath lane;
  fma_interfaces::msg::GpsPath gps;
  fma_interfaces::msg::AvoidStatus avoid;
  fma_interfaces::msg::ParkingStatus parking;
  fma_interfaces::msg::TrafficStop traffic;
  fma_interfaces::msg::EstopRequest estop;
};

enum class State : uint8_t {Lane = 0, Waypoint = 1, Avoid = 2, Parking = 3};
enum class PathSource : uint8_t {Lane, Gps, Avoid, Parking};

// 스테이트 머신의 출력 — 이후 단계(조립/병합)는 이 결정을 실행만 한다.
// "무엇을 할지"가 여기 바깥에서 결정되면 CLAUDE.md §5.1 위반.
struct Decision
{
  State state;
  PathSource path_source;
  double v_ref;          // 스테이트 내부 우선권 표 적용 결과
  bool immediate_stop;   // true → 병합의 rate limit 우회 (긴급 정지·TTC 바닥)
};

struct Params
{
  // lane ↔ waypoint 히스테리시스 (이탈/복귀 임계 분리, N주기 연속)
  double lane_conf_exit{0.4};
  double lane_conf_return{0.6};
  int n_cycles{20};
  // 스테이트별 속도
  double v_base{0.5};        // [m/s] lane·waypoint 기본
  double v_accel_zone{1.0};  // [m/s] 가속구간
  double v_narrow{0.2};      // [m/s] avoid 여유 폭 좁을 때 상한
  // avoid 안전 바닥
  double ttc_stop{0.8};      // [s] TTC < 임계 → 즉시 정지
};

}  // namespace adas_mgm

#endif  // ADAS_MGM__TYPES_HPP_
