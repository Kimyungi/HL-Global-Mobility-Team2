#include "multi_lidar_fusion/cloud_synchronizer.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace multi_lidar_fusion
{

CloudSynchronizer::CloudSynchronizer(const SyncParams & params)
: params_(params)
{
}

void CloudSynchronizer::registerSensor(std::uint8_t index, const std::string & id, bool enabled)
{
  if (findSlot(index) != nullptr) {
    return;
  }
  Slot s;
  s.index = index;
  s.id = id;
  s.enabled = enabled;
  slots_.push_back(std::move(s));
}

CloudSynchronizer::Slot * CloudSynchronizer::findSlot(std::uint8_t index)
{
  for (auto & s : slots_) {
    if (s.index == index) {
      return &s;
    }
  }
  return nullptr;
}

const CloudSynchronizer::Slot * CloudSynchronizer::findSlot(std::uint8_t index) const
{
  for (const auto & s : slots_) {
    if (s.index == index) {
      return &s;
    }
  }
  return nullptr;
}

void CloudSynchronizer::push(CloudFrame && frame, CloudFrame * recycled)
{
  Slot * slot = findSlot(frame.sensor_id);
  if (slot == nullptr || !slot->enabled) {
    return;
  }
  frame.seq = ++slot->next_seq;      // seq 는 여기서만 부여한다 (재사용 판정의 근거).
  slot->buffer.push_back(std::move(frame));

  const std::size_t cap = std::max<std::size_t>(params_.buffer_size, 1U);
  while (slot->buffer.size() > cap) {
    if (recycled != nullptr) {
      *recycled = std::move(slot->buffer.front());   // capacity 를 호출측에 돌려준다
      recycled = nullptr;                            // 한 번만
    }
    slot->buffer.pop_front();
  }
}

rclcpp::Time CloudSynchronizer::lastStamp(std::uint8_t index) const
{
  const Slot * slot = findSlot(index);
  if (slot == nullptr || slot->buffer.empty()) {
    return rclcpp::Time(0, 0, RCL_ROS_TIME);
  }
  return slot->buffer.back().stamp;
}

SyncResult CloudSynchronizer::collect(const rclcpp::Time & now)
{
  SyncResult result;
  result.sensors.reserve(slots_.size());

  // ── ① 기준시각 t_ref 결정 ────────────────────────────────────────────
  rclcpp::Time t_ref = now;
  bool have_ref = false;
  if (params_.time_reference == "clock") {
    have_ref = true;
  } else if (params_.time_reference != "latest") {
    for (const auto & s : slots_) {
      if (s.enabled && s.id == params_.time_reference && !s.buffer.empty()) {
        t_ref = s.buffer.back().stamp;
        have_ref = true;
        break;
      }
    }
  }
  if (!have_ref) {
    // "latest" 또는 지정 기준센서가 아직 말이 없을 때의 안전한 축퇴 경로:
    // 전 센서 중 가장 최근 stamp.
    for (const auto & s : slots_) {
      if (!s.enabled || s.buffer.empty()) {
        continue;
      }
      const rclcpp::Time & st = s.buffer.back().stamp;
      if (!have_ref || st > t_ref) {
        t_ref = st;
        have_ref = true;
      }
    }
  }
  if (!have_ref) {
    t_ref = now;      // 아무 센서도 데이터를 준 적이 없다 (요구 §20 Case6).
  }
  result.t_ref = t_ref;

  // ── ② 센서별로 t_ref 최근접 프레임 선택 ──────────────────────────────
  for (auto & slot : slots_) {
    SensorSyncResult r;
    r.index = slot.index;
    r.id = slot.id;

    if (!slot.enabled) {
      r.status = FrameStatus::kDisabled;
      result.sensors.push_back(r);
      continue;
    }
    if (slot.buffer.empty()) {
      r.status = FrameStatus::kNeverReceived;
      result.sensors.push_back(r);
      continue;
    }

    const CloudFrame * best = nullptr;
    double best_abs_dt = 0.0;
    for (const auto & f : slot.buffer) {
      const double dt = (f.stamp - t_ref).seconds();
      const double adt = std::fabs(dt);
      if (best == nullptr || adt < best_abs_dt) {
        best = &f;
        best_abs_dt = adt;
        r.dt_to_ref_s = dt;
      }
    }

    r.frame = best;
    // ★ stamp 가 아니라 **스캔이 끝난 시각**(stamp + duration) 기준으로 잰다.
    //   LaserScan 의 stamp 는 첫 ray 시각이라, 10Hz 라이다는 도착하는 순간 이미
    //   100ms 과거다. stamp 로 재면 멀쩡한 센서가 전부 too_old 가 된다.
    r.age_s = (now - best->stamp).seconds() - best->duration;

    if (r.age_s > params_.max_cloud_age_s) {
      // 센서가 끊겼거나 심하게 느리다 — 이 주기에서만 뺀다. 노드는 계속 돈다.
      r.status = FrameStatus::kTooOld;
      r.frame = nullptr;
      result.sensors.push_back(r);
      continue;
    }
    if (best_abs_dt > params_.sync_tolerance_s) {
      if (params_.strict_sync) {
        r.status = FrameStatus::kOutOfSync;
        r.frame = nullptr;
        result.sensors.push_back(r);
        continue;
      }
      // 느슨한 모드: 쓰되 표시만 한다. 어긋난 만큼은 motion compensation 이 줄인다.
      r.sync_warn = true;
      ++result.sync_warn_count;
    }

    if (best->points.empty()) {
      r.status = FrameStatus::kEmpty;   // 오류가 아니다 (요구 §20 Case5).
      r.frame = nullptr;
      result.sensors.push_back(r);
      slot.last_consumed_seq = best->seq;
      slot.has_consumed = true;
      continue;
    }

    const bool same_as_last = slot.has_consumed && slot.last_consumed_seq == best->seq;
    r.status = same_as_last ? FrameStatus::kReused : FrameStatus::kUsed;
    slot.last_consumed_seq = best->seq;
    slot.has_consumed = true;

    ++result.contributing;
    result.sensors.push_back(r);
  }

  result.valid = result.contributing > 0;
  return result;
}

}  // namespace multi_lidar_fusion
