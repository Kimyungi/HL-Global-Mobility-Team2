// core/mgm_step.hpp — MGM 로직 코어 진입점 (CLAUDE.md §5.5)
//
// ROS 의존성 없는 순수 함수: 출력 = mgm_step(인지 스냅샷, 이전 상태).
// 두 구현이 이 인터페이스를 공유한다:
//   ① 이 폴더의 C++ 레퍼런스 구현 (김윤기)
//   ② Simulink/Stateflow → Embedded Coder 생성 C 코드 (김재민, step 함수명 mgm_step)
// 검증: 동일 스냅샷 덤프를 두 구현에 재생 → 출력 비교 (tools/core_replay).
#ifndef ADAS_MGM__CORE__MGM_STEP_HPP_
#define ADAS_MGM__CORE__MGM_STEP_HPP_

#include "mgm_types.hpp"

namespace adas_mgm
{

// 상태 초기화 — 첫 mgm_step 호출 전 1회
void mgm_init(CoreState & st, const CoreParams & params);

// 매 10ms 호출. 판단(스테이트 머신) → 실행(ref 조립·종방향 병합).
CoreOutput mgm_step(const CoreSnapshot & in, CoreState & st);

}  // namespace adas_mgm

#endif  // ADAS_MGM__CORE__MGM_STEP_HPP_
