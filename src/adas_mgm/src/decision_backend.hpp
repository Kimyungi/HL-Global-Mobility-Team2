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
// enabled ADAS_MGR2 v1.68 LANE/WAYPOINT experiment.
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
  // 지정 지점 정차 관찰용 (판단 아님 — 로그 전용). generated backend 는 이
  // 기능이 없으므로 항상 미정차로 보고한다.
  bool stopZoneHolding() const;
  int32_t stopHoldLeft() const;
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

#ifdef ADAS_MGM_HAS_GENERATED_BACKEND
  std::unique_ptr<GeneratedMgmAdapter> generated_;
#endif
};

// Public pure helpers keep fail-closed output checks independently testable.
bool validateGeneratedOutput(
  const CoreOutput & output, const CoreSnapshot & input,
  const CoreParams & params, std::string & reason);
bool generatedInputWithinLaneWaypointScope(
  const CoreSnapshot & input, const CoreParams & params,
  std::string & reason);

}  // namespace adas_mgm

#endif  // ADAS_MGM__SRC__DECISION_BACKEND_HPP_
