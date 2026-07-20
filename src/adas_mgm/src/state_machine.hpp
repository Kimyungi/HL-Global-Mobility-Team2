#ifndef ADAS_MGM__STATE_MACHINE_HPP_
#define ADAS_MGM__STATE_MACHINE_HPP_

#include "types.hpp"

namespace adas_mgm
{

// 시스템 전체에서 판단 로직이 존재하는 유일한 곳 (CLAUDE.md §4).
// 전이 판단과 스테이트 내부 우선권 결정을 모두 수행한다.
// 전역 min/max로 요구를 합치지 말 것 — 우선권은 스테이트마다 다르다.
class StateMachine
{
public:
  explicit StateMachine(const Params & params);

  // 매 10ms 호출. 전이 → 스테이트 내부 우선권 → Decision.
  Decision decide(const Snapshot & s);

  State state() const {return state_;}

private:
  void transition(const Snapshot & s);
  Decision prioritize(const Snapshot & s) const;

  Params p_;
  State state_{State::Lane};
  State avoid_return_{State::Lane};  // 복귀처 변수 1개만 기억 (CLAUDE.md §4)
  int lane_low_cnt_{0};
  int lane_high_cnt_{0};
};

}  // namespace adas_mgm

#endif  // ADAS_MGM__STATE_MACHINE_HPP_
