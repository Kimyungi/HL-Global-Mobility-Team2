// tools/dump_format.hpp — CoreSnapshot 덤프 파일 포맷 (back-to-back 검증용, ROS 무관)
//
// 기록: mgm_node의 snapshot_dump_path 파라미터 (매 틱 1레코드)
// 재생: tools/core_replay — 같은 덤프를 레퍼런스 코어와 생성 코드에 각각 재생 → CSV diff
//
// 주의: 구조체를 컴파일러 기본 정렬 그대로 raw 기록한다 — 같은 머신·같은 ABI에서만
// 호환 (팀 개발 PC 기준). 헤더의 snapshot_size로 레이아웃 불일치를 검출한다.
#ifndef ADAS_MGM__TOOLS__DUMP_FORMAT_HPP_
#define ADAS_MGM__TOOLS__DUMP_FORMAT_HPP_

#include <cstdint>
#include <type_traits>

#include "core/mgm_types.hpp"

namespace adas_mgm
{

constexpr uint32_t kDumpMagic = 0x314D474D;  // little-endian 바이트열 "MGM1"
constexpr uint32_t kDumpVersion = 1;

struct DumpHeader
{
  uint32_t magic;
  uint32_t version;
  uint32_t snapshot_size;  // sizeof(CoreSnapshot) — 읽는 쪽에서 검증
  uint32_t params_size;    // sizeof(CoreParams)
  CoreParams params;       // 기록 당시 파라미터 — 재생 시 동일 조건 보장
};
// 헤더 뒤로 CoreSnapshot 레코드가 EOF까지 연속 (1레코드 = 1틱 = 10ms)

static_assert(std::is_trivially_copyable<CoreSnapshot>::value, "raw dump requires POD");
static_assert(std::is_trivially_copyable<CoreParams>::value, "raw dump requires POD");

}  // namespace adas_mgm

#endif  // ADAS_MGM__TOOLS__DUMP_FORMAT_HPP_
