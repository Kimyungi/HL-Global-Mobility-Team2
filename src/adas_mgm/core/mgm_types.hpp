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
  // 후진 탈출 전용 — 인지 소스가 아니라 조립 블록이 만드는 **직선 ref**다
  // (2026-08-24). 후진 중에는 조향을 중립으로 두고 곧게 빼는 것이 유일하게
  // 안전한 기하다: 전진용 ref(차 앞의 목표점)를 그대로 두고 v_ref만 음수로
  // 뒤집으면 차가 목표를 등진 채 반대로 꺾인다.
  MGM_SRC_ESCAPE = 4,
};

// 후진 탈출 페이즈 (CoreState.escape_phase) — AVOID 스테이트 **안의** 단계다.
// 스테이트로 승격하지 않는 이유: 후진은 새 횡방향 경로 소스를 만드는 일이 아니라
// 회피를 성립시키기 위한 준비 동작이고, §4가 정지를 스테이트에서 뺀 것과 같은 계열이다.
enum : uint8_t
{
  MGM_ESCAPE_NONE = 0,       // 평시
  MGM_ESCAPE_REVERSING = 1,  // 후진 중 (v_ref < 0, 직선 ref)
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
  float gps_cross_track;                  // [m] 트랙까지 수직거리 (재합류 판정, GpsPath.msg 참조)
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
  // ── stack_gps 지정 구간 (2026-08-18 신설, GpsPath.msg 참조).
  // 0 = 정지 지점 아님, 1~ = 몇 번째 정지 지점인가. **번호**인 이유는 MGM이
  // 지점별로 "이미 정지했다"를 기억해야 하기 때문 — bool이면 언덕에서 정지 중
  // 차가 밀려 구간을 벗어났다 다시 들어올 때 재정지 루프가 된다.
  uint8_t gps_stop_zone;
  bool gps_avoid_zone;       // 회피 허용 구간 안인가 (avoid_zone_only 게이트 입력)
  // GPS 전용 구간 안인가 (2026-08-18). true면 LANE 전이를 하지 않고 WAYPOINT로
  // 고정한다 — 차선을 믿기 어려운 구간을 **구간 단위로** 지정하기 위한 것.
  // launch의 gps_only:=true(임계를 2.0으로 올려 run 전체에서 LANE 불가)와 달리
  // 여기는 그 구간에서만 걸리고 벗어나면 정상 히스테리시스로 돌아온다.
  bool gps_gps_only_zone;
  // 후방 여유 — 후진 탈출(§4)의 전제 (EstopRequest.rear_clear, 2026-08-24).
  // **모르면 false**: 후방 센서 미탑재·스캔 무효·stack_estop staleness 전부 false다.
  // escape_require_rear_clear가 켜져 있으면 이 값이 true인 동안만 후진한다.
  bool estop_rear_clear;
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
  // AVOID 스테이트 전용 속도 상한 [m/s]. 0 이하면 상한 없음(구동작 = v_suggest 그대로).
  //
  // 왜 stack_avoid의 target_speed_mps를 내리지 않고 여기 두는가 (2026-08-17):
  // 그 값은 **두 가지 역할을 겸한다** — ① v_suggest(=AVOID의 v_ref) ② TTC 자차속도
  // 폴백(`/vehicle/vector` 미수신 시. 실측상 0건이라 항상 폴백이 쓰인다).
  // 1.0 m/s로 달리면서 그 값만 0.6으로 내리면 TTC = gap/0.6 이 되어 **1.67배로
  // 부풀고**, avoidable 판정과 MGM의 TTC 안전 바닥이 그만큼 늦게 걸린다.
  // 그래서 "AVOID에서 얼마로 달릴까"는 스테이트별 속도(v_base·v_narrow와 같은 부류)로
  // MGM이 갖고, stack_avoid의 target_speed_mps는 **실제 주행 속도 = v_base**를
  // 유지해 TTC를 정직하게 둔다.
  //
  // ⚠ §3 ① "감속하면서 조향 금지"와의 관계: 이 상한은 진입 시 1.0 → 0.6 감속을
  //   만든다. 0.6은 조향 응답 하한 0.5 위이고 a_down 1.5 m/s²로 0.27s면 끝나므로
  //   run_0812_234253(0.44까지 내려가 조향이 죽은 사례)과는 다르다. 다만 감속과
  //   선회가 겹치는 구간이 생기는 것은 사실이라 회피 시험에서 확인 대상이다.
  float v_avoid;
  // ── 지정 지점 정지 (2026-08-18 신설, §4 우선권 표).
  // 트랙 위 지정 지점(GpsPath.stop_zone)에 도달하면 정지하고 이 틱수만큼 머문 뒤
  // 스스로 재출발한다. 0 이하면 기능 끔(구동작 — 지점을 지나쳐도 아무 일 없음).
  // 카운트다운은 **실제로 멈춘 뒤**(명령 속도 0) 시작한다 — 진입 시점부터 세면
  // 감속에 쓴 시간만큼 정차가 짧아진다.
  int32_t stop_zone_hold_cycles;
  // 0이 아니면 **회피 허용 구간 안에서만** AVOID 전이를 허용한다 (2026-08-18).
  // 구간 밖 장애물은 회피하지 않고 stack_estop 정지로만 대응한다 — 시험 코스에서
  // 회피를 특정 구간에만 쓰고 싶을 때의 운용 스위치다. 0 = 어디서나 회피(구동작).
  // ⚠ TTC 안전 바닥(ttc_stop)은 AVOID 스테이트 안에서만 걸리므로, 이 게이트를
  //   켜면 구간 밖 장애물의 유일한 방어선은 stack_estop 이다.
  int32_t avoid_zone_only;

  // ── 후진 탈출 (2026-08-24 신설, §4 우선권 표 / AVOID 진입 페이즈).
  //
  // 푸는 문제: 회피 불가 장애물 앞에서 estop이 걸리면 차가 영영 못 빠져나온다.
  // estop은 레벨 신호라 장애물이 치워져야 풀리는데, 시험 코스에서는 치워질 일이
  // 없는 경우가 있다(길을 막은 구조물·주차된 차). 그러면 v_ref 0 으로 무한 대기다.
  // 그래서 estop이 충분히 오래 유지되면 **곧게 조금 물러나** 회피가 성립하는
  // 거리를 만들고 AVOID 로 넘어간다.

  // estop이 이 틱수만큼 **연속** 유지되면 후진을 개시한다. 0 이하 = 기능 끔(구동작).
  // 1000틱 = 10s. 카운터는 **실제 EstopRequest 인가**로만 센다(estop_latch_release) —
  // §5.7 watchdog 보정이나 wait_go 대기로 걸린 estop 은 세지 않는다. 이게 없으면
  // 출발 인가 전 대기 중에 차가 스스로 후진한다.
  int32_t escape_after_cycles;

  // 후진 목표 속도 [m/s]. **반드시 음수** — 0 이상이면 기능이 꺼진다(안전 불변식).
  // 이 값이 음수라는 것만으로 "탈출 페이즈는 절대 전진하지 못한다"가 보장된다.
  float v_escape;

  // 후진 최대 틱. 거리 상한 대용이다 (v_escape × escape_max_cycles × 10ms).
  // 예: -0.3 m/s × 200틱 = 0.6m. 0 이하면 기능 끔 — 상한 없는 후진은 만들지 않는다.
  int32_t escape_max_cycles;

  // 0이 아니면 후방 여유(estop_rear_clear)가 확인될 때만 후진한다. **기본 켬.**
  // 후방 센서가 붙기 전에는 이 값이 항상 false 라 기능이 자연히 잠긴다.
  // 끄면 후방을 보지 않고 시간 상한만으로 후진한다 — 관측자를 세운 시험에서만 쓸 것.
  int32_t escape_require_rear_clear;
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
  // 지정 지점 정지 (2026-08-18) — §4 우선권 표의 "지정 정지"
  bool stop_zone_holding;                 // 정지 유지 중 (v_ref 0 요구)
  int32_t stop_hold_left;                 // 남은 정차 틱 (멈춘 뒤에만 줄어든다)
  uint8_t stop_zone_done_id;              // **실제로 정차한** 지점 번호 — 밀림 재정지 방지
  uint8_t stop_zone_boot_id;              // 기동 시점에 이미 안에 있던 지점 번호 (임시 억제)
  bool stop_zone_init;                    // 첫 유효 gps 틱을 지났나 (boot_id 확정용)
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
  // ── 후진 탈출 (2026-08-24)
  int32_t estop_hold_cnt;   // 실제 estop 연속 틱 (watchdog 보정 제외)
  uint8_t escape_phase;     // MGM_ESCAPE_* — AVOID 안의 단계
  int32_t escape_ticks;     // 후진 진행 틱 (escape_max_cycles 상한 판정)
  // 주행을 한 번이라도 시작했는가 (명령 속도가 0을 넘은 적이 있는가).
  // **후진 탈출의 무장 조건**이다. 없으면 다음 함정에 빠진다: 벽을 마주 보고
  // launch 하면 첫 틱부터 실제 estop 이 참이라, 아무도 출발 인가를 하지 않았는데
  // 10초 뒤 차가 스스로 뒤로 물러난다. 한 번이라도 굴러간 뒤에만 "갇혔다"고
  // 말할 수 있다.
  bool escape_armed;
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
