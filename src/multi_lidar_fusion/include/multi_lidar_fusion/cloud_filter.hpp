// multi_lidar_fusion — 단계 6: 필터링 (ROI / Self / Range / Voxel)
//
// 이 파일의 역할:
//   병합된 점군에서 회피 판단에 쓸모없거나 해로운 점을 제거한다.
//     ① 거리   : base_link 원점 기준 너무 가깝/먼 점 (요구 §12)
//     ② ROI    : 관심 상자 밖 (요구 §11)
//     ③ self   : 차체·구조물이 스스로 찍힌 점 (요구 §11 Self Filtering)
//     ④ voxel  : 시야가 겹치는 곳의 중복점 축약 (요구 §13)
//   모두 in-place 압축이라 새 벡터를 만들지 않는다(요구 §28).
//
//   voxel 은 "각 칸의 대표점 = 원점에서 가장 가까운 점"으로 고른다. 무게중심을 쓰면
//   장애물 경계가 뒤로 밀릴 수 있는데, 회피에서는 가까운 쪽이 안전한 선택이다(§16과 동일 철학).
//
// 입력  : CloudFrame (frame_id = base_link, 병합본)
// 출력  : 같은 CloudFrame in-place
// frame : base_link — 모든 상자 파라미터가 차량 기준이므로 다른 frame 에 쓰면 안 된다.
// 파라미터: filter.* (fusion_params.yaml)
// 관계  : CloudMerger 다음, PointCloud2 직렬화 및 VirtualLaserScan 앞.

#ifndef MULTI_LIDAR_FUSION__CLOUD_FILTER_HPP_
#define MULTI_LIDAR_FUSION__CLOUD_FILTER_HPP_

#include <cstddef>
#include <cstdint>
#include <unordered_map>

#include "multi_lidar_fusion/types.hpp"

namespace multi_lidar_fusion
{

struct FilterParams
{
  // ── 거리 (base_link 원점 기준 수평거리) ──────────────────────────────
  bool range_enabled{true};
  double min_range{0.05};
  double max_range{20.0};

  // ── ROI 상자 ─────────────────────────────────────────────────────────
  bool roi_enabled{true};
  double min_x{-5.0};
  double max_x{10.0};
  double min_y{-5.0};
  double max_y{5.0};
  double min_z{-1.0};
  double max_z{2.0};

  // ── 자기반사 제거 (차체 상자) ────────────────────────────────────────
  bool self_filter_enabled{true};
  double vehicle_length{0.6};    ///< x 방향 전체 길이 [m]
  double vehicle_width{0.4};     ///< y 방향 전체 폭 [m]
  double vehicle_center_x{0.0};  ///< base_link 기준 차체 중심 [m]
  double vehicle_center_y{0.0};
  double self_margin{0.02};      ///< 상자를 이만큼 키워서 판정 (진동·오차 여유)

  // ── voxel ────────────────────────────────────────────────────────────
  bool voxel_enabled{false};
  double voxel_leaf_size{0.03};
  /// true = z 를 무시하고 xy 격자만 (2D 라이다 4대가 서로 다른 높이일 때 유용).
  bool voxel_2d{true};
};

struct FilterStats
{
  std::size_t input{0};
  std::size_t dropped_range{0};
  std::size_t dropped_roi{0};
  std::size_t dropped_self{0};
  std::size_t dropped_voxel{0};
  std::size_t output{0};

  std::size_t droppedTotal() const
  {
    return dropped_range + dropped_roi + dropped_self + dropped_voxel;
  }
};

class CloudFilter
{
public:
  explicit CloudFilter(const FilterParams & params);

  /// frame.points 를 in-place 로 걸러낸다.
  FilterStats apply(CloudFrame & frame) const;

  const FilterParams & params() const {return params_;}
  void setParams(const FilterParams & p) {params_ = p;}

private:
  /// voxel 격자 키 → 출력 배열 인덱스. clear() 해도 버킷이 남아 재할당이 없다.
  mutable std::unordered_map<std::int64_t, std::uint32_t> voxel_map_;
  FilterParams params_;
};

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__CLOUD_FILTER_HPP_
