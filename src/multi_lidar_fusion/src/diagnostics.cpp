#include "multi_lidar_fusion/diagnostics.hpp"

#include <algorithm>
#include <cstdio>
#include <string>

namespace multi_lidar_fusion
{

namespace
{
constexpr std::uint8_t kOk = diagnostic_msgs::msg::DiagnosticStatus::OK;
constexpr std::uint8_t kWarn = diagnostic_msgs::msg::DiagnosticStatus::WARN;
constexpr std::uint8_t kError = diagnostic_msgs::msg::DiagnosticStatus::ERROR;

diagnostic_msgs::msg::KeyValue kv(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue out;
  out.key = key;
  out.value = value;
  return out;
}

std::string fmt(const char * f, double v)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), f, v);
  return std::string(buf);
}

std::string u64(std::uint64_t v) {return std::to_string(v);}
}  // namespace

Diagnostics::Diagnostics(const Config & config)
: config_(config)
{
}

void Diagnostics::registerSensor(std::uint8_t index, const std::string & id)
{
  if (find(index) != nullptr) {
    return;
  }
  SensorCounters c;
  c.index = index;
  c.id = id;
  sensors_.push_back(c);
}

Diagnostics::SensorCounters * Diagnostics::find(std::uint8_t index)
{
  for (auto & s : sensors_) {
    if (s.index == index) {
      return &s;
    }
  }
  return nullptr;
}

void Diagnostics::recordMessage(
  std::uint8_t index, const rclcpp::Time & stamp, const ConvertStats & conv, bool convert_ok)
{
  SensorCounters * s = find(index);
  if (s == nullptr) {
    return;
  }
  ++s->msgs;
  ++s->total_msgs;
  s->last_stamp = stamp;
  s->ever_received = true;
  s->points_in += conv.input_points;
  s->points_out += conv.output_points;
  s->dropped_invalid += conv.dropped_invalid;
  s->dropped_range += conv.dropped_range;
  s->dropped_fov += conv.dropped_fov;
  if (!convert_ok) {
    ++s->convert_failed;
  }
}

void Diagnostics::recordSync(const SyncResult & sync)
{
  for (const auto & r : sync.sensors) {
    SensorCounters * s = find(r.index);
    if (s == nullptr) {
      continue;
    }
    s->last_status = r.status;
    s->last_dt_to_ref_s = r.dt_to_ref_s;
    if (r.sync_warn) {
      ++s->sync_warn;
    }
    switch (r.status) {
      case FrameStatus::kUsed: ++s->used; break;
      case FrameStatus::kReused: ++s->reused; break;
      case FrameStatus::kTooOld: ++s->too_old; break;
      case FrameStatus::kOutOfSync:
        ++s->out_of_sync;
        ++total_sync_failures_;
        break;
      default: break;
    }
  }
  if (sync.sync_warn_count > 0) {
    ++sync_warn_cycles_;
  }
}

void Diagnostics::recordTfFailure(std::uint8_t index)
{
  ++total_tf_failures_;
  SensorCounters * s = find(index);
  if (s == nullptr) {
    return;
  }
  ++s->tf_failed;
  ++s->total_tf_failed;
  s->last_status = FrameStatus::kTfFailed;
}

void Diagnostics::recordCycle(const CycleReport & report)
{
  ++cycles_;
  ++total_cycles_;
  if (report.published) {
    ++published_;
  }
  if (report.contributing == 0) {
    ++empty_cycles_;
    ++total_empty_cycles_;
  }
  merged_points_ += report.merged_points;
  filtered_points_ += report.published_points;
  dropped_points_ += report.filter.droppedTotal();
  max_spread_s_ = std::max(max_spread_s_, report.max_dt_spread_s);
  last_coverage_ = report.scan.coverage;
  last_active_ = report.contributing;
  min_active_seen_ = std::min(min_active_seen_, report.contributing);
}

bool Diagnostics::due(const rclcpp::Time & now)
{
  if (!window_started_) {
    resetWindow(now);
    return false;
  }
  const double elapsed = (now - window_start_).seconds();
  if (elapsed < config_.report_period_s) {
    return false;
  }
  window_len_s_ = elapsed > 0.0 ? elapsed : 1.0;
  for (auto & s : sensors_) {
    s.fps = static_cast<double>(s.msgs) / window_len_s_;
  }
  computeLevelAndSummary();
  resetWindow(now);
  return true;
}

void Diagnostics::resetWindow(const rclcpp::Time & now)
{
  window_start_ = now;
  window_started_ = true;
  for (auto & s : sensors_) {
    s.msgs = 0;
    s.points_in = 0;
    s.points_out = 0;
    s.dropped_invalid = 0;
    s.dropped_range = 0;
    s.dropped_fov = 0;
    s.convert_failed = 0;
    s.used = 0;
    s.reused = 0;
    s.too_old = 0;
    s.out_of_sync = 0;
    s.sync_warn = 0;
    s.tf_failed = 0;
  }
  cycles_ = 0;
  published_ = 0;
  empty_cycles_ = 0;
  sync_warn_cycles_ = 0;
  merged_points_ = 0;
  filtered_points_ = 0;
  dropped_points_ = 0;
  max_spread_s_ = 0.0;
  min_active_seen_ = kMaxSensors + 1;
}

void Diagnostics::computeLevelAndSummary()
{
  const std::size_t worst_active = min_active_seen_ > kMaxSensors ? last_active_ : min_active_seen_;

  if (last_active_ == 0 || worst_active == 0) {
    level_ = kError;      // 요구 §20 Case6 — 전 센서 두절.
  } else if (worst_active < config_.min_active_sensors) {
    level_ = kWarn;
  } else {
    level_ = kOk;
  }

  std::string s = "active=" + std::to_string(last_active_) + "/" +
    std::to_string(sensors_.size()) +
    " cycles=" + u64(cycles_) + " pub=" + u64(published_);
  if (cycles_ > 0) {
    s += " merged=" + u64(merged_points_ / cycles_) +
      "pt out=" + u64(filtered_points_ / cycles_) + "pt";
  }
  s += " spread=" + fmt("%.1fms", max_spread_s_ * 1e3) +
    " cover=" + fmt("%.0f%%", last_coverage_ * 100.0);
  for (const auto & c : sensors_) {
    s += " | " + c.id + " " + fmt("%.1fHz", c.fps) +
      " dt=" + fmt("%+.1fms", c.last_dt_to_ref_s * 1e3) +
      " " + toString(c.last_status);
    if (c.tf_failed > 0) {
      s += " tf_fail=" + u64(c.tf_failed);
    }
  }
  if (empty_cycles_ > 0) {
    s += " | EMPTY_CYCLES=" + u64(empty_cycles_);
  }
  summary_ = s;
}

std::string Diagnostics::summary() const
{
  return summary_;
}

diagnostic_msgs::msg::DiagnosticArray Diagnostics::toMessage(const rclcpp::Time & now) const
{
  diagnostic_msgs::msg::DiagnosticArray arr;
  arr.header.stamp = now;

  diagnostic_msgs::msg::DiagnosticStatus overall;
  overall.name = "multi_lidar_fusion: fusion";
  overall.hardware_id = config_.hardware_id;
  overall.level = level_;
  overall.message = level_ == kError ?
    "no active lidar" : (level_ == kWarn ? "degraded (missing sensors)" : "ok");
  overall.values.push_back(kv("active_lidars", std::to_string(last_active_)));
  overall.values.push_back(kv("configured_lidars", std::to_string(sensors_.size())));
  overall.values.push_back(kv("cycles_window", u64(cycles_)));
  overall.values.push_back(kv("published_window", u64(published_)));
  overall.values.push_back(kv("empty_cycles_window", u64(empty_cycles_)));
  overall.values.push_back(kv("sync_warn_cycles_window", u64(sync_warn_cycles_)));
  overall.values.push_back(kv("max_stamp_spread_ms", fmt("%.2f", max_spread_s_ * 1e3)));
  overall.values.push_back(
    kv("merged_points_avg", cycles_ ? u64(merged_points_ / cycles_) : "0"));
  overall.values.push_back(
    kv("published_points_avg", cycles_ ? u64(filtered_points_ / cycles_) : "0"));
  overall.values.push_back(
    kv("filtered_out_points_avg", cycles_ ? u64(dropped_points_ / cycles_) : "0"));
  overall.values.push_back(kv("scan_coverage", fmt("%.3f", last_coverage_)));
  overall.values.push_back(kv("total_cycles", u64(total_cycles_)));
  overall.values.push_back(kv("total_empty_cycles", u64(total_empty_cycles_)));
  overall.values.push_back(kv("total_sync_failures", u64(total_sync_failures_)));
  overall.values.push_back(kv("total_tf_failures", u64(total_tf_failures_)));
  arr.status.push_back(overall);

  for (const auto & c : sensors_) {
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.name = "multi_lidar_fusion: lidar_" + c.id;
    st.hardware_id = config_.hardware_id;
    if (!c.ever_received) {
      st.level = kError;
      st.message = "never received";
    } else if (isContributing(c.last_status)) {
      st.level = c.sync_warn > 0 ? kWarn : kOk;
      st.message = c.sync_warn > 0 ? "loose sync" : "ok";
    } else if (c.last_status == FrameStatus::kDisabled) {
      st.level = kOk;
      st.message = "disabled";
    } else {
      st.level = kWarn;
      st.message = toString(c.last_status);
    }
    st.values.push_back(kv("status", toString(c.last_status)));
    st.values.push_back(kv("fps", fmt("%.2f", c.fps)));
    st.values.push_back(
      kv("last_stamp", fmt("%.6f", c.ever_received ? c.last_stamp.seconds() : 0.0)));
    st.values.push_back(kv("dt_to_ref_ms", fmt("%.2f", c.last_dt_to_ref_s * 1e3)));
    st.values.push_back(kv("msgs_window", u64(c.msgs)));
    st.values.push_back(kv("points_in_window", u64(c.points_in)));
    st.values.push_back(kv("points_out_window", u64(c.points_out)));
    st.values.push_back(kv("dropped_invalid_window", u64(c.dropped_invalid)));
    st.values.push_back(kv("dropped_range_window", u64(c.dropped_range)));
    st.values.push_back(kv("dropped_fov_window", u64(c.dropped_fov)));
    st.values.push_back(kv("convert_failed_window", u64(c.convert_failed)));
    st.values.push_back(kv("used_window", u64(c.used)));
    st.values.push_back(kv("reused_window", u64(c.reused)));
    st.values.push_back(kv("too_old_window", u64(c.too_old)));
    st.values.push_back(kv("out_of_sync_window", u64(c.out_of_sync)));
    st.values.push_back(kv("tf_failed_window", u64(c.tf_failed)));
    st.values.push_back(kv("total_msgs", u64(c.total_msgs)));
    st.values.push_back(kv("total_tf_failed", u64(c.total_tf_failed)));
    arr.status.push_back(st);
  }
  return arr;
}

}  // namespace multi_lidar_fusion
