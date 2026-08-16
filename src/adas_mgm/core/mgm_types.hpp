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
// §5.8 이동 보정이 ref x를 깎을 수 있는 하한 [m]. 전진밖에 못 하는 차에게
// 차 뒤(x<0) 목표는 도달 불가능하므로 감쇠로 여기를 넘지 않는다 (2026-08-14).
// 값이 작은 이유: avoid 1점 계약을 20점으로 보간하면 첫 점이 목표의 1/20
// (1.5m 목표 → 0.075m)이라 정상값이 원래 작다. 하한을 크게 잡으면 그 정상값을
// 왜곡한다 — 여기서는 "뒤로 넘어가지 않는다"만 보장한다.
constexpr float MGM_MIN_REF_X = 0.01f;

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
  float gps_cross_track;                  // [m] 트랙 최근접점까지 거리 (재합류 판정)
  // 헤딩을 믿어도 되는가 (GpsPath.heading_source != HEADING_TANGENT). 2026-08-16 신설.
  // 접선 폴백은 "최근접 트랙 접선 = 차량 헤딩"을 가정하므로 **ref[0].yaw 가 항상
  // 0 부근으로 나온다** — 즉 차가 실제로 트랙을 등지고 있어도 "정렬됨"으로 보인다.
  // run_0816_184505: 회피 중 130° 돌아버린 차가 역방향 가드에 걸려 정지 → 정지하니
  // COG(속도 문턱 0.25m/s)가 무효 → 접선 폴백이 "오차 0°"를 내놓음 → 가드 해제 →
  // 엉뚱한 방향으로 재출발 → 속도 붙자 COG 복귀 → 다시 130° → 정지. 이 왕복이
  // 반복되며 횡오차가 2.7→5.1m 로 벌어졌다. (COG 자체는 정상이었다 — 값을 wrap 하면
  // +85~+111° 로 일관됐고, 차가 진짜로 트랙 방향과 130° 틀어져 있었다.)
  bool gps_heading_valid;
  // 이번 틱에 해당 소스의 **새 메시지**가 도착했는지 (wrapper가 수신 시각으로 판정).
  // §5.8 이동 보정의 "새 추론 미도착" 판정 근거. 값 동일성으로 판정하면 인지가
  // 의도적으로 상수를 낼 때(회피 통과 유지점 (1.5,0)) 영원히 낡은 것으로 오판해
  // x를 무한 감쇠시킨다 — 2026-08-14 run_0814_200516에서 7.25초 동안 1.5→-2.2m로
  // 밀려 차에게 후진을 명령한 꼴이 됐다.
  bool lane_updated;
  bool gps_updated;
  bool avoid_updated;
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
  // waypoint→lane 전이를 허용하는 최대 횡오차 [m]. 트랙에 **실제로 재합류한 뒤에만**
  // 카메라로 넘어가게 하는 게이트다 (2026-08-14). 시간 게이트
  // (avoid_return_hold_cycles)만으로는 3초 뒤 이탈한 채로도 lane이 돼버린다 —
  // run_0814_195116에서 복귀 시점 횡오차가 0.90m·2.72m였다.
  // 0 이하면 이 게이트를 끈다(구동작).
  float lane_entry_max_cross;
  // AVOID 최대 지속 틱. 초과하면 maneuver_done과 무관하게 waypoint로 복귀한다.
  // AVOID 중 경로는 "전방 직진 유지"라 틀어진 헤딩을 그대로 유지한다 —
  // 상한이 없으면 무한히 트랙에서 멀어진다. 2026-08-15 run_0815_143039:
  // 감지 경계에 걸친 벽이 20초에 28회 깜빡여 클리어런스 타이머가 매번
  // 리셋되고, AVOID가 15초+ 지속되며 횡오차 5.3m까지 발산했다.
  // 안전은 유지된다 — 복귀 후에도 장애물이 실제로 있으면 즉시 재진입하고,
  // TTC 안전 바닥(ttc_stop)은 스테이트와 무관하게 계속 작동한다.
  // 0 이하면 상한 없음(구동작).
  int32_t avoid_max_cycles;
};

// mgm_step이 읽고 갱신하는 유일한 내부 상태 — Simulink의 상태 보존 방식과 대칭
struct CoreState
{
  CoreParams params;
  // 스테이트 머신
  uint8_t state;                          // MGM_STATE_*
  int32_t lane_low_cnt;
  int32_t lane_high_cnt;
  int32_t wrongway_cnt;                   // 역방향 지속 카운터 (헤딩 신뢰 시에만 셈)
  int32_t wrongway_ok_cnt;                // 정렬 지속 카운터 — 래치 해제용 (같은 조건)
  bool wrongway_latched;                  // 역방향 래치 (§4) — 신뢰 가능한 정렬로만 해제
  int32_t return_hold_left;               // >0이면 waypoint→lane 전이 보류 (avoid 복귀 직후)
  int32_t avoid_ticks;                    // AVOID 지속 틱 (avoid_max_cycles 상한 판정)
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
