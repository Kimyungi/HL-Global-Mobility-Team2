// multi_lidar_fusion — 단계 3: 시간 정렬 (Time synchronization)
//
// 이 파일의 역할:
//   센서마다 주기가 다르고(예: A=10Hz, B=20Hz) 콜백 도착 순서도 뒤섞이므로,
//   "지금 도착한 것들을 그냥 합치는"(요구 §29 금지3) 대신 **stamp 기준**으로 한 세트를
//   고른다. 매 융합 주기에 기준시각 t_ref 를 정하고, 센서마다 t_ref 에 가장 가까운
//   프레임 하나를 고른다. 너무 오래된 프레임은 그 주기에서 빠지지만, 그 때문에 융합
//   전체가 멈추지는 않는다(요구 §8, §20 Case1/3, §29 금지4).
//
//   ExactTime 이 아니라 **approximate** 방식이다. 서로 다른 모델의 라이다는 하드웨어
//   트리거가 없어 stamp 가 절대 일치하지 않는다.
//
// 입력  : CloudFrame (센서 frame, 정규화 직후) — push()
// 출력  : SyncResult (선택된 프레임 포인터 + 센서별 사유) — collect()
// frame : 관여하지 않음 (시간만 다룬다)
// 파라미터: sync_tolerance_s, max_cloud_age_s, strict_sync, buffer_size, time_reference
// 관계  : 노드 콜백 → push(), 노드 타이머 → collect() → CloudTransformer/Compensator/Merger
//
// 스레드: **단일 스레드 실행자 전용**. collect() 가 반환하는 포인터는 내부 버퍼를
//         가리키므로(복사 회피 — 요구 §28) 다음 push() 전까지만 유효하다.

#ifndef MULTI_LIDAR_FUSION__CLOUD_SYNCHRONIZER_HPP_
#define MULTI_LIDAR_FUSION__CLOUD_SYNCHRONIZER_HPP_

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include "multi_lidar_fusion/types.hpp"

namespace multi_lidar_fusion
{

struct SyncParams
{
  /// |t_i - t_ref| 가 이 값을 넘으면 "동기 이탈". strict_sync 일 때만 배제한다.
  double sync_tolerance_s{0.05};
  /// now - t_i 가 이 값을 넘으면 그 주기에서 무조건 배제 (요구 §8).
  double max_cloud_age_s{0.10};
  /// true = 동기 이탈 프레임을 버린다. false = 쓰되 경고 카운터만 올린다.
  /// 기본 false — 회피 로직에는 "조금 어긋난 점"이 "점이 아예 없음"보다 낫다.
  bool strict_sync{false};
  /// 센서당 보관할 프레임 수. t_ref 앞뒤로 고를 여지를 준다.
  std::size_t buffer_size{4};
  /// "latest" = 전 센서 중 가장 최근 stamp, "clock" = 노드 시계 now,
  /// 그 외 = 해당 센서 id 의 최신 stamp (기준 센서 고정).
  std::string time_reference{"latest"};
};

/// 한 센서에 대한 한 주기의 결정.
struct SensorSyncResult
{
  std::uint8_t index{0};
  std::string id;
  FrameStatus status{FrameStatus::kNeverReceived};
  double dt_to_ref_s{0.0};      ///< frame.stamp - t_ref (부호 유지)
  double age_s{0.0};            ///< now - frame.stamp
  bool sync_warn{false};        ///< 허용오차 초과인데 strict 가 아니라 살려둔 경우
  const CloudFrame * frame{nullptr};   ///< 내부 버퍼 참조 (복사 없음)
};

struct SyncResult
{
  rclcpp::Time t_ref{0, 0, RCL_ROS_TIME};
  bool valid{false};                     ///< 기여 센서가 1개 이상인가
  std::size_t contributing{0};
  std::size_t sync_warn_count{0};
  std::vector<SensorSyncResult> sensors; ///< registerSensor 순서
};

class CloudSynchronizer
{
public:
  explicit CloudSynchronizer(const SyncParams & params);

  /// 센서 슬롯 등록. index 는 FusionPoint::sensor_id 와 같은 값.
  void registerSensor(std::uint8_t index, const std::string & id, bool enabled);

  /// 정규화된 프레임 투입. 이동 대입이므로 호출측 frame 은 비워진다.
  /// recycled 를 주면 버퍼에서 밀려난 프레임을 그쪽으로 되돌려준다 — 호출측이
  /// 그 벡터 capacity 를 재사용해 매 콜백 재할당을 없애기 위한 것(요구 §28).
  /// `&frame` 과 같은 객체를 넘겨도 안전하다 (이동 후에 대입한다).
  void push(CloudFrame && frame, CloudFrame * recycled = nullptr);

  /// 한 융합 주기의 세트를 고른다. 내부적으로 "이번에 소비했다"를 기록하므로
  /// 같은 프레임이 다음 주기에 다시 뽑히면 kReused 로 표시된다.
  SyncResult collect(const rclcpp::Time & now);

  /// 센서가 마지막으로 데이터를 준 시각 (진단용). 없으면 valid=false 인 Time.
  rclcpp::Time lastStamp(std::uint8_t index) const;

  const SyncParams & params() const {return params_;}
  void setParams(const SyncParams & p) {params_ = p;}

private:
  struct Slot
  {
    std::uint8_t index{0};
    std::string id;
    bool enabled{true};
    std::size_t next_seq{0};
    std::size_t last_consumed_seq{0};
    bool has_consumed{false};
    std::deque<CloudFrame> buffer;
  };

  Slot * findSlot(std::uint8_t index);
  const Slot * findSlot(std::uint8_t index) const;

  SyncParams params_;
  std::vector<Slot> slots_;
};

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__CLOUD_SYNCHRONIZER_HPP_
