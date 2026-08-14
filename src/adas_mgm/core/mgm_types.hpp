// core/mgm_types.hpp — MGM 로직 코어의 입·출력·상태 정의 (CLAUDE.md §5.5)
//
// 이 폴더(core/)는 ROS 헤더 include 절대 금지 — Simulink 모델과 스펙 1:1.
// 모든 구조체는 float/bool/int + 고정 길이 배열만 사용한다 (Simulink 버스에 1:1 대응).
// 이 파일이 곧 김재민 MBD 트랙의 입·출력 버스 정의다 — 필드 변경 시 docs/MBD_KIT.md 갱신.
#ifndef ADAS_MGM__CORE__MGM_TYPES_HPP_
#define ADAS_MGM__CORE__MGM_TYPES_HPP_

#include <cstdint>

namespace adas_mgm
{

constexpr int32_t MGM_NUM_POINTS = 20;   // ref points 최대치 (CAN ID 예약 폭, PROTOCOL.md)
                                         // 실제 점 수는 현재 모든 소스 1 (n은 확장 대비 가변)
constexpr float MGM_PERIOD_S = 0.01f;    // 10ms 고정 주기

// CLAUDE.md §4 스테이트 4개 — TargetRef.msg의 STATE_* 상수와 값 일치
enum : uint8_t
{
  MGM_STATE_LANE = 0,
  MGM_STATE_WAYPOINT = 1,
  MGM_STATE_AVOID = 2,
  MGM_STATE_PARKING = 3,
};

// 스테이트가 고른 횡방향 경로 소스
enum : uint8_t
{
  MGM_SRC_LANE = 0,
  MGM_SRC_GPS = 1,
  MGM_SRC_AVOID = 2,
  MGM_SRC_PARKING = 3,
};

// fma_interfaces/RefPoint과 동일 의미 — float 4개 (CAN 전송 시 양자화는 bridge_dspace 담당)
struct CorePoint
{
  float x;          // [m] vehicle frame, 전방 +
  float y;          // [m] 좌측 +
  float yaw;        // [rad]
  float curvature;  // [1/m]
};

// 인지 스택이 주는 경로 — n = 유효 점 수 (0 = 미도착/없음), 앞에서부터 n개 유효
struct CorePath
{
  int32_t n;
  CorePoint pts[MGM_NUM_POINTS];
};

// 매 10ms 틱의 입력 — "최신 인지 스냅샷" (fma_interfaces 6개 토픽의 코어 필요분)
struct CoreSnapshot
{
  // stack_lane
  float lane_confidence;        // 0.0~1.0 — lane↔waypoint 히스테리시스 입력
  CorePath lane_path;
  // stack_gps
  CorePath gps_path;
  bool gps_accel_zone;
  bool gps_parking_zone;
  bool gps_at_end;
  // stack_avoid
  bool avoid_obstacle_detected;
  bool avoid_avoidable;
  float avoid_ttc;              // [s] 미도착 시 wrapper가 큰 값(1e9)으로 초기화
  bool avoid_narrow_gap;
  bool avoid_maneuver_done;
  CorePath avoid_path;
  float avoid_v_suggest;        // [m/s]
  // stack_parking
  bool parking_space_found;
  bool parking_path_blocked;
  bool parking_done;
  CorePath parking_path;
  float parking_v_suggest;      // [m/s] 후진 = 음수
  // stack_traffic
  bool traffic_stop_required;
  // stack_estop
  bool estop;                // 정지 판단 입력 — wrapper의 §5.7 staleness 보정 포함
  bool estop_latch_release;  // at_end 래치 해제 전용 — **실제 EstopRequest 수신값만**
                             // (watchdog 보정 estop으로 래치가 풀려 재출발하던 구멍
                             //  차단, 2026-08-11 — CLAUDE.md §4 래치)
};

// 튜닝 파라미터 — params.yaml과 1:1, Simulink에서는 tunable parameter
struct CoreParams
{
  float lane_conf_exit;    // lane→waypoint 이탈 임계
  float lane_conf_return;  // waypoint→lane 복귀 임계 (히스테리시스 분리)
  int32_t n_cycles;        // N주기 연속 조건
  float v_base;            // [m/s]
  float v_accel_zone;      // [m/s]
  float v_narrow;          // [m/s] avoid 여유 폭 좁을 때 상한
  float ttc_stop;          // [s] TTC 안전 바닥
  int32_t blend_cycles;    // 스테이트 전환 ref 블렌드 구간 (틱)
  float a_up;              // [m/s^2] 가속 rate limit
  float a_down;            // [m/s^2] 일반 감속 rate limit (immediate_stop은 우회)
  float wrongway_yaw;      // [rad] 역방향 판정 |ref[0].yaw| 임계 (waypoint, §4)
  int32_t wrongway_cycles; // 역방향 N주기 연속 조건
  // avoid→waypoint 복귀 후 이 틱수 동안 waypoint→lane 전이를 보류한다.
  // 근거: 회피 직후엔 차가 트랙을 벗어나 있어 GPS 재합류 시간이 필요한데,
  // 카운터 리셋만으로는 n_cycles(수백 ms)밖에 못 번다 (2026-08-14 run_0814_184624
  // 실측: 복귀 4회 중 2회가 0.01s 만에 lane으로 튐 — §4 복귀 정책 무력화).
  // 새 필드는 반드시 구조체 끝에 추가할 것 — core_replay가 옛 덤프의
  // params_size로 앞부분만 읽고 나머지는 기본값으로 채우기 때문.
  int32_t avoid_return_hold_cycles;
};

// mgm_step이 읽고 갱신하는 유일한 내부 상태 — Simulink의 상태 보존 방식과 대칭
struct CoreState
{
  CoreParams params;
  // 스테이트 머신
  uint8_t state;                          // MGM_STATE_*
  int32_t lane_low_cnt;
  int32_t lane_high_cnt;
  int32_t wrongway_cnt;                   // 역방향 지속 카운터 (waypoint, §4)
  int32_t return_hold_left;               // >0이면 waypoint→lane 전이 보류 (avoid 복귀 직후)
  bool at_end_latched;                    // 종점 도달 래치 — estop 인가 시 해제 (§4)
  // ref 조립 (전환 연속 처리)
  uint8_t last_src;                       // MGM_SRC_*
  int32_t blend_left;
  int32_t n_out;                          // 유효 점 수 (선택 소스의 n, 소스 미도착 시 hold)
  CorePoint ref_out[MGM_NUM_POINTS];      // 내부는 20 고정(블렌드용) — 출력 유효분은 n_out개
  CorePoint blend_from[MGM_NUM_POINTS];
  // 인지 갱신 지연 구간 이동 보정 (2026-08-08, 조향 미반영 진단) — 인지 소스가
  // 이전 틱과 완전히 같은 값을 낼 때(아직 새 추론 미도착) ref_out을 그대로
  // 복사하지 않고 x를 st.v만큼 깎아 내보내기 위한 "직전 원본 스냅샷" 기억.
  // ref_out 자체(blend 적용된 출력)와는 별도로 둔다 — target_differs 판정은
  // 항상 "인지가 준 원본"끼리 비교해야 하기 때문.
  bool has_raw_target;
  int32_t raw_n;
  CorePoint last_raw_target[MGM_NUM_POINTS];
  // 종방향 병합 (rate limit)
  float v;
};

// 매 틱의 출력 — wrapper가 TargetRef로 변환·발행
struct CoreOutput
{
  uint8_t state;           // MGM_STATE_* (flags의 state 부분)
  uint8_t path_source;     // MGM_SRC_* (디버그·back-to-back 비교용)
  bool immediate_stop;     // 디버그·back-to-back 비교용
  float v_ref;             // [m/s] 병합 최종 목표 속도. 정지 = 0
  int32_t n_points;        // 유효 점 수 (1~20) — CAN에는 이만큼만 실린다
  CorePoint ref_points[MGM_NUM_POINTS];
};

}  // namespace adas_mgm

#endif  // ADAS_MGM__CORE__MGM_TYPES_HPP_
