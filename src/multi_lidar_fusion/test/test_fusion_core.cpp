// multi_lidar_fusion — 코어 단위 테스트
//
// ROS 노드 없이 파이프라인 각 단계의 계약을 고정한다. 여기서 깨지면 실차에 나가기 전에
// 걸린다. (좌표변환은 tf2 버퍼가 필요해 여기서 제외 — 통합 검증은 README 의 RViz 절차)

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "multi_lidar_fusion/cloud_filter.hpp"
#include "multi_lidar_fusion/cloud_merger.hpp"
#include "multi_lidar_fusion/cloud_synchronizer.hpp"
#include "multi_lidar_fusion/lidar_converter.hpp"
#include "multi_lidar_fusion/motion_compensator.hpp"
#include "multi_lidar_fusion/virtual_laserscan.hpp"

using namespace multi_lidar_fusion;  // NOLINT

namespace
{

sensor_msgs::msg::LaserScan makeScan(
  double angle_min, double angle_inc, const std::vector<float> & ranges,
  double stamp_s = 1.0)
{
  sensor_msgs::msg::LaserScan s;
  s.header.frame_id = "lidar_a1_link";
  s.header.stamp = rclcpp::Time(static_cast<int64_t>(stamp_s * 1e9), RCL_ROS_TIME);
  s.angle_min = static_cast<float>(angle_min);
  s.angle_increment = static_cast<float>(angle_inc);
  s.angle_max = static_cast<float>(angle_min + angle_inc * (ranges.size() - 1));
  s.range_min = 0.0F;
  s.range_max = 100.0F;
  s.time_increment = 0.0F;
  s.scan_time = 0.1F;
  s.ranges = ranges;
  return s;
}

SensorConfig baseConfig()
{
  SensorConfig c;
  c.id = "a1";
  c.index = 0;
  c.min_range = 0.1;
  c.max_range = 10.0;
  return c;
}

}  // namespace

// ── 단계 1: 정규화 ────────────────────────────────────────────────────────
TEST(LidarConverter, ScanToCartesian)
{
  LidarConverter conv(baseConfig());
  // 0도와 90도에 각각 2m, 3m
  const auto scan = makeScan(0.0, M_PI_2, {2.0F, 3.0F, 4.0F, 5.0F});
  CloudFrame f;
  ConvertStats st;
  ASSERT_TRUE(conv.convert(scan, f, st));
  ASSERT_EQ(f.points.size(), 4U);
  EXPECT_NEAR(f.points[0].x, 2.0, 1e-5);
  EXPECT_NEAR(f.points[0].y, 0.0, 1e-5);
  EXPECT_NEAR(f.points[1].x, 0.0, 1e-5);
  EXPECT_NEAR(f.points[1].y, 3.0, 1e-5);
  EXPECT_EQ(f.frame_id, "lidar_a1_link");
  EXPECT_EQ(f.points[0].sensor_id, 0);
}

TEST(LidarConverter, DropsInvalidAndOutOfRange)
{
  LidarConverter conv(baseConfig());
  const auto nan = std::numeric_limits<float>::quiet_NaN();
  const auto inf = std::numeric_limits<float>::infinity();
  const auto scan = makeScan(0.0, 0.1, {nan, inf, 0.05F, 50.0F, 1.0F});
  CloudFrame f;
  ConvertStats st;
  ASSERT_TRUE(conv.convert(scan, f, st));
  EXPECT_EQ(f.points.size(), 1U);           // 1.0m 만 살아남는다
  EXPECT_EQ(st.dropped_invalid, 2U);        // NaN, Inf
  EXPECT_EQ(st.dropped_range, 2U);          // 0.05m(근접), 50m(초과)
}

TEST(LidarConverter, FovMaskKeepsOnlyRequestedSector)
{
  SensorConfig c = baseConfig();
  c.fov_enabled = true;
  c.fov.min_rad = -M_PI_4;                  // -45도 ~ +45도만
  c.fov.max_rad = M_PI_4;
  LidarConverter conv(c);

  // -90, -45, 0, +45, +90 도
  const auto scan = makeScan(-M_PI_2, M_PI_4, {1.0F, 1.0F, 1.0F, 1.0F, 1.0F});
  CloudFrame f;
  ConvertStats st;
  ASSERT_TRUE(conv.convert(scan, f, st));
  EXPECT_EQ(f.points.size(), 3U);
  EXPECT_EQ(st.dropped_fov, 2U);
}

TEST(LidarConverter, FovMaskWrapsAroundPi)
{
  SensorConfig c = baseConfig();
  c.fov_enabled = true;
  c.fov.min_rad = 3.0 * M_PI_4;             // +135도 ~ -135도 (뒤쪽 90도)
  c.fov.max_rad = -3.0 * M_PI_4;
  LidarConverter conv(c);

  // -180, -90, 0, +90, +180 도  → 살아남아야 할 것은 ±180 두 개
  const auto scan = makeScan(-M_PI, M_PI_2, {1.0F, 1.0F, 1.0F, 1.0F, 1.0F});
  CloudFrame f;
  ConvertStats st;
  ASSERT_TRUE(conv.convert(scan, f, st));
  EXPECT_EQ(f.points.size(), 2U);
}

TEST(LidarConverter, BlindSectorRemovesBracketShadow)
{
  SensorConfig c = baseConfig();
  c.blind_sectors.push_back(AngularSector{-M_PI_4 - 0.01, -M_PI_4 + 0.01});
  LidarConverter conv(c);
  const auto scan = makeScan(-M_PI_2, M_PI_4, {1.0F, 1.0F, 1.0F});
  CloudFrame f;
  ConvertStats st;
  ASSERT_TRUE(conv.convert(scan, f, st));
  EXPECT_EQ(f.points.size(), 2U);
  EXPECT_EQ(st.dropped_fov, 1U);
}

TEST(LidarConverter, RejectsScanWithoutAngleInfo)
{
  LidarConverter conv(baseConfig());
  auto scan = makeScan(0.0, 0.1, {1.0F, 1.0F});
  scan.angle_increment = 0.0F;
  CloudFrame f;
  ConvertStats st;
  EXPECT_FALSE(conv.convert(scan, f, st));
}

// ── 단계 3: 시간 정렬 ─────────────────────────────────────────────────────
namespace
{
CloudFrame frameAt(std::uint8_t idx, double stamp_s, std::size_t npoints = 3)
{
  CloudFrame f;
  f.sensor_id = idx;
  f.stamp = rclcpp::Time(static_cast<int64_t>(stamp_s * 1e9), RCL_ROS_TIME);
  f.frame_id = "base_link";
  f.points.assign(npoints, FusionPoint{});
  for (auto & p : f.points) {
    p.sensor_id = idx;
    p.x = 1.0F;
  }
  return f;
}
rclcpp::Time at(double s) {return rclcpp::Time(static_cast<int64_t>(s * 1e9), RCL_ROS_TIME);}
}  // namespace

TEST(CloudSynchronizer, PicksNearestFrameToReference)
{
  SyncParams p;
  p.sync_tolerance_s = 0.05;
  p.max_cloud_age_s = 0.10;
  CloudSynchronizer sync(p);
  sync.registerSensor(0, "a1", true);
  sync.registerSensor(1, "b1", true);

  // b1(20Hz)이 두 번 들어오는 동안 a1(10Hz)은 한 번
  sync.push(frameAt(0, 10.00));
  sync.push(frameAt(1, 10.00));
  sync.push(frameAt(1, 10.05));

  const SyncResult r = sync.collect(at(10.06));
  EXPECT_TRUE(r.valid);
  EXPECT_EQ(r.contributing, 2U);
  EXPECT_NEAR(r.t_ref.seconds(), 10.05, 1e-6);   // 최신 stamp 가 기준
  ASSERT_EQ(r.sensors.size(), 2U);
  // b1 은 t_ref 와 정확히 일치하는 프레임을 골라야 한다
  EXPECT_NEAR(r.sensors[1].dt_to_ref_s, 0.0, 1e-6);
  // a1 은 50ms 뒤처졌지만 허용치 이내라 살아남는다
  EXPECT_NEAR(r.sensors[0].dt_to_ref_s, -0.05, 1e-6);
}

TEST(CloudSynchronizer, ExcludesStaleSensorButKeepsGoing)
{
  SyncParams p;
  p.max_cloud_age_s = 0.10;
  CloudSynchronizer sync(p);
  sync.registerSensor(0, "a1", true);
  sync.registerSensor(1, "b1", true);

  sync.push(frameAt(0, 10.00));      // 이후 a1 은 끊김
  sync.push(frameAt(1, 10.50));

  const SyncResult r = sync.collect(at(10.51));
  EXPECT_TRUE(r.valid);              // 요구 §29 금지4 — 하나 죽어도 계속 간다
  EXPECT_EQ(r.contributing, 1U);
  EXPECT_EQ(r.sensors[0].status, FrameStatus::kTooOld);
  EXPECT_EQ(r.sensors[1].status, FrameStatus::kUsed);
}

TEST(CloudSynchronizer, MarksReuseOfSlowSensor)
{
  SyncParams p;
  p.max_cloud_age_s = 1.0;
  p.sync_tolerance_s = 1.0;
  CloudSynchronizer sync(p);
  sync.registerSensor(0, "a1", true);

  sync.push(frameAt(0, 10.00));
  EXPECT_EQ(sync.collect(at(10.01)).sensors[0].status, FrameStatus::kUsed);
  EXPECT_EQ(sync.collect(at(10.06)).sensors[0].status, FrameStatus::kReused);
  sync.push(frameAt(0, 10.10));
  EXPECT_EQ(sync.collect(at(10.11)).sensors[0].status, FrameStatus::kUsed);
}

TEST(CloudSynchronizer, StrictModeDropsOutOfSync)
{
  SyncParams p;
  p.sync_tolerance_s = 0.02;
  p.max_cloud_age_s = 1.0;
  p.strict_sync = true;
  CloudSynchronizer sync(p);
  sync.registerSensor(0, "a1", true);
  sync.registerSensor(1, "b1", true);

  sync.push(frameAt(0, 10.00));
  sync.push(frameAt(1, 10.10));

  const SyncResult r = sync.collect(at(10.11));
  EXPECT_EQ(r.sensors[0].status, FrameStatus::kOutOfSync);
  EXPECT_EQ(r.contributing, 1U);
}

TEST(CloudSynchronizer, NoSensorEverReceived)
{
  CloudSynchronizer sync(SyncParams{});
  sync.registerSensor(0, "a1", true);
  const SyncResult r = sync.collect(at(1.0));
  EXPECT_FALSE(r.valid);                                        // 요구 §20 Case6
  EXPECT_EQ(r.sensors[0].status, FrameStatus::kNeverReceived);
}

// ── 단계 4: 운동 보상 ─────────────────────────────────────────────────────
TEST(MotionCompensator, DisabledLeavesCloudUntouched)
{
  MotionCompensator mc{MotionParams{}};
  CloudFrame f = frameAt(0, 10.0, 1);
  f.points[0].x = 2.0F;
  const auto st = mc.compensate(f, at(10.1));
  EXPECT_FALSE(st.applied);
  EXPECT_FLOAT_EQ(f.points[0].x, 2.0F);
}

TEST(MotionCompensator, ForwardMotionPullsPointsCloser)
{
  MotionParams mp;
  mp.enabled = true;
  mp.use_point_dt = false;
  MotionCompensator mc(mp);

  VehicleTwist t;
  t.stamp = at(10.0);
  t.vx = 1.0;               // 1 m/s 전진
  t.valid = true;
  mc.setTwist(t);

  CloudFrame f = frameAt(0, 10.0, 1);
  f.points[0].x = 5.0F;
  f.points[0].y = 0.0F;

  // 0.1s 뒤 기준으로 옮기면 차가 0.1m 전진했으므로 점은 0.1m 가까워진다.
  const auto st = mc.compensate(f, at(10.1));
  EXPECT_TRUE(st.applied);
  EXPECT_NEAR(f.points[0].x, 4.9, 1e-4);
  EXPECT_NEAR(f.points[0].y, 0.0, 1e-4);
}

TEST(MotionCompensator, YawRateRotatesPoints)
{
  MotionParams mp;
  mp.enabled = true;
  mp.use_point_dt = false;
  mp.max_twist_age_s = 2.0;
  MotionCompensator mc(mp);

  VehicleTwist t;
  t.stamp = at(10.0);
  t.yaw_rate = M_PI_2;      // 90 deg/s, 순수 회전
  t.valid = true;
  mc.setTwist(t);

  CloudFrame f = frameAt(0, 10.0, 1);
  f.points[0].x = 1.0F;
  f.points[0].y = 0.0F;

  // 1초 뒤 기준: 차가 +90도 돌았으므로 그 시점 body frame 에서 점은 -90도에 보인다.
  const auto st = mc.compensate(f, at(11.0));
  EXPECT_TRUE(st.applied);
  EXPECT_NEAR(f.points[0].x, 0.0, 1e-4);
  EXPECT_NEAR(f.points[0].y, -1.0, 1e-4);
}

// ── 단계 5: 병합 + 직렬화 ─────────────────────────────────────────────────
TEST(CloudMerger, ConcatenatesAndKeepsSensorId)
{
  CloudMerger m;
  CloudFrame a = frameAt(0, 10.0, 2);
  CloudFrame b = frameAt(3, 10.0, 3);
  CloudFrame out;
  const auto st = m.merge({&a, nullptr, &b}, at(10.0), "base_link", out);
  EXPECT_EQ(st.sources, 2U);
  EXPECT_EQ(out.points.size(), 5U);
  EXPECT_EQ(out.frame_id, "base_link");
  EXPECT_EQ(out.points[0].sensor_id, 0);
  EXPECT_EQ(out.points[4].sensor_id, 3);
}

TEST(CloudMerger, SerializesMinimalXyzLayout)
{
  CloudFrame f = frameAt(2, 10.0, 2);
  f.points[0].x = 1.5F;
  f.points[1].sensor_id = 2;
  sensor_msgs::msg::PointCloud2 msg;
  toPointCloud2(f, false, false, msg);
  ASSERT_EQ(msg.fields.size(), 3U);
  EXPECT_EQ(msg.fields[0].name, "x");
  EXPECT_EQ(msg.point_step, 12U);
  EXPECT_EQ(msg.width, 2U);
  EXPECT_EQ(msg.data.size(), 24U);
  float x = 0.0F;
  std::memcpy(&x, msg.data.data(), 4);
  EXPECT_FLOAT_EQ(x, 1.5F);
}

TEST(CloudMerger, DebugLayoutCarriesSensorId)
{
  CloudFrame f = frameAt(3, 10.0, 1);
  sensor_msgs::msg::PointCloud2 msg;
  toPointCloud2(f, true, true, msg);
  ASSERT_EQ(msg.fields.size(), 5U);
  EXPECT_EQ(msg.fields[4].name, "sensor_id");
  EXPECT_EQ(msg.fields[4].offset, 16U);
  EXPECT_EQ(msg.point_step, 20U);
  EXPECT_EQ(msg.data[16], 3U);
}

// ── 단계 6: 필터 ──────────────────────────────────────────────────────────
namespace
{
CloudFrame cloudOf(const std::vector<std::array<float, 3>> & xyz)
{
  CloudFrame f;
  f.frame_id = "base_link";
  for (const auto & p : xyz) {
    FusionPoint fp;
    fp.x = p[0];
    fp.y = p[1];
    fp.z = p[2];
    f.points.push_back(fp);
  }
  return f;
}
}  // namespace

TEST(CloudFilter, RoiAndSelfFilter)
{
  FilterParams fp;
  fp.range_enabled = false;
  fp.roi_enabled = true;
  fp.min_x = -1.0; fp.max_x = 3.0;
  fp.min_y = -1.0; fp.max_y = 1.0;
  fp.min_z = -1.0; fp.max_z = 1.0;
  fp.self_filter_enabled = true;
  fp.vehicle_length = 0.6;
  fp.vehicle_width = 0.4;
  fp.self_margin = 0.0;
  CloudFilter filter(fp);

  CloudFrame f = cloudOf({
    {2.0F, 0.0F, 0.0F},     // 유지
    {9.0F, 0.0F, 0.0F},     // ROI 밖 (x)
    {1.0F, 5.0F, 0.0F},     // ROI 밖 (y)
    {0.1F, 0.1F, 0.0F},     // 차체 안 → self
  });
  const auto st = filter.apply(f);
  EXPECT_EQ(st.input, 4U);
  EXPECT_EQ(st.dropped_roi, 2U);
  EXPECT_EQ(st.dropped_self, 1U);
  EXPECT_EQ(f.points.size(), 1U);
  EXPECT_FLOAT_EQ(f.points[0].x, 2.0F);
}

TEST(CloudFilter, VoxelKeepsNearestPointPerCell)
{
  FilterParams fp;
  fp.range_enabled = false;
  fp.roi_enabled = false;
  fp.self_filter_enabled = false;
  fp.voxel_enabled = true;
  fp.voxel_leaf_size = 0.10;
  fp.voxel_2d = true;
  CloudFilter filter(fp);

  // 같은 10cm 칸 안의 두 점 + 다른 칸의 점 하나
  CloudFrame f = cloudOf({
    {2.05F, 0.05F, 0.0F},
    {2.01F, 0.01F, 0.0F},   // 더 가깝다 → 이쪽이 남아야 한다
    {5.00F, 0.00F, 0.0F},
  });
  const auto st = filter.apply(f);
  EXPECT_EQ(st.dropped_voxel, 1U);
  ASSERT_EQ(f.points.size(), 2U);
  EXPECT_FLOAT_EQ(f.points[0].x, 2.01F);
}

// ── 단계 7: 가상 LaserScan ────────────────────────────────────────────────
TEST(VirtualLaserScan, NearestWinsInSameBin)
{
  ScanParams sp;
  sp.angle_min = -M_PI;
  sp.angle_max = M_PI;
  sp.angle_increment = 1.0 * M_PI / 180.0;   // 1도
  sp.range_min = 0.1;
  sp.range_max = 20.0;
  VirtualLaserScan builder(sp);

  // 정확히 같은 방향(0도)에 세 센서가 4.2 / 4.1 / 4.4 를 봤다 (요구 §16)
  CloudFrame f = cloudOf({{4.2F, 0.0F, 0.0F}, {4.1F, 0.0F, 0.0F}, {4.4F, 0.0F, 0.0F}});
  f.frame_id = "base_link";
  sensor_msgs::msg::LaserScan scan;
  const auto st = builder.build(f, scan);

  EXPECT_EQ(st.bins, 360U);
  const std::size_t idx = static_cast<std::size_t>((0.0 - sp.angle_min) / sp.angle_increment);
  EXPECT_NEAR(scan.ranges[idx], 4.1F, 1e-4);
  EXPECT_EQ(st.observed_bins, 1U);
  EXPECT_EQ(scan.header.frame_id, "base_link");
}

TEST(VirtualLaserScan, UnobservedBinsAreInf)
{
  ScanParams sp;
  sp.angle_increment = 1.0 * M_PI / 180.0;
  VirtualLaserScan builder(sp);
  CloudFrame f = cloudOf({{3.0F, 0.0F, 0.0F}});
  sensor_msgs::msg::LaserScan scan;
  builder.build(f, scan);
  ASSERT_EQ(scan.ranges.size(), 360U);
  std::size_t inf_count = 0;
  for (float r : scan.ranges) {
    if (std::isinf(r)) {
      ++inf_count;
    }
  }
  EXPECT_EQ(inf_count, 359U);      // 요구 §17
}

TEST(VirtualLaserScan, WrapsExactlyPi)
{
  ScanParams sp;
  sp.angle_increment = 1.0 * M_PI / 180.0;
  VirtualLaserScan builder(sp);
  // atan2(-0, -1) = +pi 인 점. 감아서 0번 bin 에 들어가야 하고 버려지면 안 된다.
  CloudFrame f = cloudOf({{-3.0F, 0.0F, 0.0F}});
  sensor_msgs::msg::LaserScan scan;
  const auto st = builder.build(f, scan);
  EXPECT_EQ(st.points_used, 1U);
  EXPECT_EQ(st.dropped_angle, 0U);
}

TEST(VirtualLaserScan, RespectsRangeWindow)
{
  ScanParams sp;
  sp.angle_increment = 1.0 * M_PI / 180.0;
  sp.range_min = 1.0;
  sp.range_max = 5.0;
  VirtualLaserScan builder(sp);
  CloudFrame f = cloudOf({{0.5F, 0.0F, 0.0F}, {9.0F, 0.0F, 0.0F}, {3.0F, 0.0F, 0.0F}});
  sensor_msgs::msg::LaserScan scan;
  const auto st = builder.build(f, scan);
  EXPECT_EQ(st.dropped_range, 2U);
  EXPECT_EQ(st.points_used, 1U);
}
