#ifndef ADAS_MGM__SRC__TRANSITION_LOG_HPP_
#define ADAS_MGM__SRC__TRANSITION_LOG_HPP_

#include <cstdint>
#include <string>

#include "core/mgm_types.hpp"

namespace adas_mgm
{

// 스테이트 전이가 **일어난 뒤** 그 틱의 입력으로 "왜 바뀌었나"를 설명한다.
//
// ★ 판단이 아니다 (CLAUDE.md §5.1). 전이는 이미 코어/생성 모델이 결정했고, 이건
//   그 결정을 사람이 읽을 수 있게 옮겨 적는 관찰자다. 여기서 나온 값이 코어로
//   되먹임되는 경로는 없다.
//
// 그래서 오히려 **스펙 대조**가 된다: §4 가 말하는 조건이 실제로 성립했는지
// 따로 계산해 `spec_match` 에 담는다. 생성 모델(MBD)이 레퍼런스와 다른 조건으로
// 전이하면 여기서 `★ 스펙 불일치`로 드러난다 — 그게 이 시험의 목적이다.
struct TransitionRecord
{
  int64_t tick{0};
  uint8_t from{0};
  uint8_t to{0};
  std::string rule;        // 판정된 §4 규칙 이름
  bool spec_match{false};  // 그 규칙의 조건이 이 틱에 실제로 성립했나
  std::string detail;      // 결정에 관여한 변수들 (key=value, 사람이 읽는 순서)
  std::string csv;         // 같은 내용의 CSV 한 줄 (헤더는 transitionCsvHeader())
};

const char * stateName(uint8_t state);

// 카운터는 **step 직전** 값을 넘긴다 — 전이가 일어나면 코어가 카운터를 리셋하므로
// (2026-08-14 규약) step 이후 값은 0 이라 원인이 안 보인다.
TransitionRecord explainTransition(
  uint8_t from, uint8_t to, const CoreSnapshot & s, const CoreParams & p,
  int32_t lane_low_cnt_before, int32_t lane_high_cnt_before,
  int32_t avoid_ticks_before, int32_t return_hold_left_before,
  float v_ref, int64_t tick);

const char * transitionCsvHeader();

}  // namespace adas_mgm

#endif  // ADAS_MGM__SRC__TRANSITION_LOG_HPP_
