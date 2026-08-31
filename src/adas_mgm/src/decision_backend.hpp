#ifndef ADAS_MGM__SRC__DECISION_BACKEND_HPP_
#define ADAS_MGM__SRC__DECISION_BACKEND_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "core/mgm_types.hpp"

namespace adas_mgm
{

#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
class GeneratedMgmAdapter;
#endif

// Startup-only dispatcher for the production C++ core and the explicitly
// enabled ADAS_MGR2 v1.88 four-state implementation. The generated model does
// not contain the newer production rear-escape extension, so selecting it with
// escape_after_cycles != 0 is rejected at startup.
class DecisionBackend
{
public:
  DecisionBackend(
    const std::string & requested, bool generated_scope_acknowledged,
    const CoreParams & params);
  ~DecisionBackend();

  DecisionBackend(const DecisionBackend &) = delete;
  DecisionBackend & operator=(const DecisionBackend &) = delete;

  CoreOutput step(const CoreSnapshot & input);
  uint8_t activeState() const;
  // 내부 상태 관찰자 (판단 아님 — 전이/정차 로그 전용).
  bool stopZoneHolding() const;
  int32_t stopHoldLeft() const;
  // 차선 히스테리시스 카운터 — 전이 이유 로깅용 관찰자 (판단 아님).
  // core 는 CoreState, generated 는 모델 내부(ADAS_MGR2_DW)에서 읽는다.
  int32_t laneLowCnt() const;
  int32_t laneHighCnt() const;
  int32_t avoidTicks() const;
  int32_t returnHoldLeft() const;
  bool trafficDistanceLatched() const;
  float trafficStoplineDistance() const;
  const CoreParams & params() const {return params_;}
  const std::string & name() const;
  bool faulted() const;
  const std::string & faultReason() const;

private:
  enum class Kind
  {
    kCore,
    kGenerated,
  };

  CoreOutput stepGenerated(const CoreSnapshot & input);
  void latchFault(const std::string & reason);
  CoreOutput failStopOutput() const;

  Kind kind_{Kind::kCore};
  std::string name_{"core"};
  CoreParams params_{};
  CoreState core_state_{};
  uint8_t active_state_{MGM_STATE_LANE};
  bool has_last_valid_output_{false};
  CoreOutput last_valid_output_{};
  bool faulted_{false};
  std::string fault_reason_;
  // PARKING 후진에서 다른 상태로 빠져나올 때 rate limiter가 음수 v_ref를
  // 여러 틱 유지할 수 있다. PARKING에서 시작된 그 감속/가속 꼬리만 허용한다.
  bool parking_reverse_ramp_active_{false};

#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  std::unique_ptr<GeneratedMgmAdapter> generated_;
#endif
};

// Public pure helpers keep fail-closed output checks independently testable.
bool validateGeneratedOutput(
  const CoreOutput & output, const CoreSnapshot & input,
  const CoreParams & params, std::string & reason,
  bool allow_parking_reverse_ramp = false,
  float previous_v_ref = 0.0f);
// Legacy helper name retained for source compatibility. v1.88 covers all four
// states; this now validates generated input safety and rejects rear escape.
bool generatedInputWithinLaneWaypointScope(
  const CoreSnapshot & input, const CoreParams & params,
  std::string & reason);

}  // namespace adas_mgm

#endif  // ADAS_MGM__SRC__DECISION_BACKEND_HPP_
