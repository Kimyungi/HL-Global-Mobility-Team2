// multi_lidar_fusion — ROS 2 노드 (wrapper)
//
// 이 파일의 역할:
//   파라미터를 읽어 파이프라인 8단계를 **배선만** 한다. 알고리즘은 전부 코어 클래스에
//   있고 여기에는 판단이 없다. 센서가 몇 대인지, 어떤 모델인지, 어떤 메시지 타입인지는
//   전부 YAML 이 정한다(요구 §29 금지5, §33, §34-J).
//
// 입력 topic:
//   sensors.<id>.input_topic  (LaserScan 또는 PointCloud2)  — 4대
//   motion.odom_topic (/odom, nav_msgs/Odometry) 또는
//   motion.twist_topic (/vehicle/twist, geometry_msgs/TwistStamped)
// 출력 topic:
//   /lidar/<id>/cloud        PointCloud2   (센서별 정규화 결과)
//   /lidar/merged_cloud      PointCloud2   base_link
//   /lidar/merged_cloud_debug PointCloud2  base_link + sensor_id 필드 (요구 §18)
//   /lidar/merged_scan       LaserScan     base_link  ← stack_avoid 가 볼 유일한 토픽
//   /diagnostics             DiagnosticArray
// frame: 모든 출력 base_link (target_frame 파라미터)
//
// 스레드: 기본 단일 스레드 실행자. 콜백과 융합 타이머가 직렬화되므로 동기화기의
//         포인터 반환이 안전하다.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include "multi_lidar_fusion/cloud_filter.hpp"
#include "multi_lidar_fusion/cloud_merger.hpp"
#include "multi_lidar_fusion/cloud_synchronizer.hpp"
#include "multi_lidar_fusion/cloud_transformer.hpp"
#include "multi_lidar_fusion/diagnostics.hpp"
#include "multi_lidar_fusion/lidar_converter.hpp"
#include "multi_lidar_fusion/motion_compensator.hpp"
#include "multi_lidar_fusion/types.hpp"
#include "multi_lidar_fusion/virtual_laserscan.hpp"

namespace multi_lidar_fusion
{

namespace
{

rclcpp::QoS makeQoS(const std::string & reliability, int depth)
{
  rclcpp::QoS q(rclcpp::KeepLast(static_cast<std::size_t>(std::max(depth, 1))));
  if (reliability == "reliable") {
    q.reliable();
  } else {
    q.best_effort();      // 라이다 드라이버 기본값 — SensorDataQoS 와 호환(요구 §23)
  }
  q.durability_volatile();
  return q;
}

constexpr double kDeg2Rad = M_PI / 180.0;

}  // namespace

class MultiLidarFusionNode : public rclcpp::Node
{
public:
  MultiLidarFusionNode()
  : rclcpp::Node("multi_lidar_fusion")
  {
    declareCommonParams();
    buildCore();
    setupSensors();
    setupMotionSources();
    setupOutputs();
    publishStaticTransforms();

    const double period = 1.0 / std::max(fusion_rate_hz_, 1.0);
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(period),
      std::bind(&MultiLidarFusionNode::onFusionCycle, this));

    RCLCPP_INFO(
      this->get_logger(),
      "multi_lidar_fusion 기동: 센서 %zu대, 융합 %.1fHz, target_frame=%s, "
      "motion_compensation=%s",
      sensors_.size(), fusion_rate_hz_, target_frame_.c_str(),
      compensator_->params().enabled ? "on" : "off");
  }

private:
  // ── 센서 1대의 런타임 상태 ────────────────────────────────────────────
  struct SensorRuntime
  {
    SensorConfig cfg;
    std::unique_ptr<LidarConverter> converter;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub;
    CloudFrame scratch;                          ///< 콜백 재사용 버퍼
    CloudFrame work;                             ///< 융합 주기 작업 사본
    sensor_msgs::msg::PointCloud2 cloud_msg;     ///< 발행 재사용 버퍼
    // extrinsic (static TF 발행용)
    double x{0.0}, y{0.0}, z{0.0}, roll{0.0}, pitch{0.0}, yaw{0.0};
  };

  // ── 파라미터 ──────────────────────────────────────────────────────────
  template<typename T>
  T param(const std::string & name, const T & def)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, def);
    }
    return this->get_parameter(name).get_value<T>();
  }

  void declareCommonParams()
  {
    sensor_ids_ = param<std::vector<std::string>>(
      "sensor_ids", {"a1", "a2", "b1", "b2"});
    target_frame_ = param<std::string>("target_frame", "base_link");
    fusion_rate_hz_ = param<double>("fusion_rate_hz", 20.0);

    publish_static_tf_ = param<bool>("publish_static_tf", true);
    tf_timeout_s_ = param<double>("tf_timeout_s", 0.02);
    tf_static_fallback_ = param<bool>("tf_allow_static_fallback", true);

    publish_sensor_clouds_ = param<bool>("publish_sensor_clouds", true);
    sensor_cloud_frame_ = param<std::string>("sensor_cloud_frame", "base_link");
    publish_debug_cloud_ = param<bool>("publish_debug_cloud", false);
    publish_intensity_ = param<bool>("publish_intensity", false);

    merged_cloud_topic_ = param<std::string>("merged_cloud_topic", "/lidar/merged_cloud");
    merged_scan_topic_ = param<std::string>("merged_scan_topic", "/lidar/merged_scan");
    debug_cloud_topic_ = param<std::string>("debug_cloud_topic", "/lidar/merged_cloud_debug");

    input_qos_reliability_ = param<std::string>("qos.input_reliability", "best_effort");
    input_qos_depth_ = param<int>("qos.input_depth", 5);
    output_qos_reliability_ = param<std::string>("qos.output_reliability", "best_effort");
    output_qos_depth_ = param<int>("qos.output_depth", 5);
  }

  void buildCore()
  {
    // ── 동기화 ──
    SyncParams sp;
    sp.sync_tolerance_s = param<double>("sync.sync_tolerance_ms", 50.0) * 1e-3;
    sp.max_cloud_age_s = param<double>("sync.max_cloud_age_ms", 100.0) * 1e-3;
    sp.strict_sync = param<bool>("sync.strict", false);
    sp.buffer_size = static_cast<std::size_t>(
      std::max(1, param<int>("sync.buffer_size", 4)));
    sp.time_reference = param<std::string>("sync.time_reference", "latest");
    synchronizer_ = std::make_unique<CloudSynchronizer>(sp);

    // ── TF ──
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    transformer_ = std::make_unique<CloudTransformer>(
      tf_buffer_, target_frame_, tf_timeout_s_, tf_static_fallback_);

    // ── motion compensation ──
    MotionParams mp;
    mp.enabled = param<bool>("motion.enable_motion_compensation", false);
    mp.max_twist_age_s = param<double>("motion.max_twist_age_ms", 200.0) * 1e-3;
    mp.min_speed = param<double>("motion.min_speed", 0.05);
    mp.use_point_dt = param<bool>("motion.use_point_dt", true);
    compensator_ = std::make_unique<MotionCompensator>(mp);

    // ── 필터 ──
    FilterParams fp;
    fp.range_enabled = param<bool>("filter.range_enabled", true);
    fp.min_range = param<double>("filter.min_range", 0.05);
    fp.max_range = param<double>("filter.max_range", 20.0);
    fp.roi_enabled = param<bool>("filter.roi_enabled", true);
    fp.min_x = param<double>("filter.min_x", -5.0);
    fp.max_x = param<double>("filter.max_x", 10.0);
    fp.min_y = param<double>("filter.min_y", -5.0);
    fp.max_y = param<double>("filter.max_y", 5.0);
    fp.min_z = param<double>("filter.min_z", -1.0);
    fp.max_z = param<double>("filter.max_z", 2.0);
    fp.self_filter_enabled = param<bool>("filter.self_filter_enabled", true);
    fp.vehicle_length = param<double>("filter.vehicle_length", 0.6);
    fp.vehicle_width = param<double>("filter.vehicle_width", 0.4);
    fp.vehicle_center_x = param<double>("filter.vehicle_center_x", 0.0);
    fp.vehicle_center_y = param<double>("filter.vehicle_center_y", 0.0);
    fp.self_margin = param<double>("filter.self_margin", 0.02);
    fp.voxel_enabled = param<bool>("filter.enable_voxel_filter", false);
    fp.voxel_leaf_size = param<double>("filter.voxel_leaf_size", 0.03);
    fp.voxel_2d = param<bool>("filter.voxel_2d", true);
    filter_ = std::make_unique<CloudFilter>(fp);

    // ── 가상 스캔 ──
    ScanParams scp;
    scp.angle_min = param<double>("scan.angle_min", -M_PI);
    scp.angle_max = param<double>("scan.angle_max", M_PI);
    scp.angle_increment = param<double>("scan.angle_increment", 0.00872665);
    scp.range_min = param<double>("scan.range_min", 0.1);
    scp.range_max = param<double>("scan.range_max", 20.0);
    scp.z_min = param<double>("scan.z_min", -10.0);
    scp.z_max = param<double>("scan.z_max", 10.0);
    scp.use_inf_for_no_return = param<bool>("scan.use_inf_for_no_return", true);
    scp.no_return_value = param<double>("scan.no_return_value", 0.0);
    scp.publish_intensities = param<bool>("scan.publish_intensities", false);
    scp.scan_time = 1.0 / std::max(fusion_rate_hz_, 1.0);
    scan_builder_ = std::make_unique<VirtualLaserScan>(scp);

    // ── 진단 ──
    Diagnostics::Config dc;
    dc.report_period_s = param<double>("diagnostics.report_period_s", 2.0);
    dc.min_active_sensors = static_cast<std::size_t>(
      std::max(0, param<int>("diagnostics.min_active_sensors", 2)));
    dc.hardware_id = param<std::string>("diagnostics.hardware_id", "multi_lidar_fusion");
    diagnostics_ = std::make_unique<Diagnostics>(dc);
    diag_log_level_warn_only_ = param<bool>("diagnostics.log_warn_only", false);
  }

  /// 센서 하나의 각도 마스크(보는 범위·방향)를 파라미터에서 읽는다.
  void loadAngleMask(const std::string & ns, SensorConfig & cfg)
  {
    cfg.fov_enabled = param<bool>(ns + ".fov_enabled", false);
    cfg.fov.min_rad = param<double>(ns + ".fov_min_deg", -180.0) * kDeg2Rad;
    cfg.fov.max_rad = param<double>(ns + ".fov_max_deg", 180.0) * kDeg2Rad;

    const auto blind = param<std::vector<double>>(ns + ".blind_sectors_deg", {});
    cfg.blind_sectors.clear();
    if (blind.size() % 2 != 0) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s.blind_sectors_deg 원소가 홀수(%zu)라 마지막 값을 버린다 — [min,max] 쌍으로 적을 것",
        ns.c_str(), blind.size());
    }
    for (std::size_t i = 0; i + 1 < blind.size(); i += 2) {
      AngularSector s;
      s.min_rad = normalizeAngle(blind[i] * kDeg2Rad);
      s.max_rad = normalizeAngle(blind[i + 1] * kDeg2Rad);
      cfg.blind_sectors.push_back(s);
    }
  }

  void setupSensors()
  {
    const rclcpp::QoS in_qos = makeQoS(input_qos_reliability_, input_qos_depth_);
    const rclcpp::QoS out_qos = makeQoS(output_qos_reliability_, output_qos_depth_);

    sensors_.reserve(sensor_ids_.size());
    for (std::size_t k = 0; k < sensor_ids_.size(); ++k) {
      const std::string & id = sensor_ids_[k];
      const std::string ns = "sensors." + id;
      const std::string ens = "extrinsics." + id;

      SensorRuntime rt;
      SensorConfig & cfg = rt.cfg;
      cfg.id = id;
      cfg.index = static_cast<std::uint8_t>(k);
      cfg.enabled = param<bool>(ns + ".enabled", true);
      cfg.input_topic = param<std::string>(ns + ".input_topic", "/lidar/" + id + "/scan");
      cfg.input_type = param<std::string>(ns + ".input_type", "scan");
      cfg.cloud_topic = param<std::string>(ns + ".cloud_topic", "/lidar/" + id + "/cloud");
      cfg.frame_id_override = param<std::string>(ns + ".frame_id", "lidar_" + id + "_link");
      cfg.min_range = param<double>(ns + ".min_range", 0.05);
      cfg.max_range = param<double>(ns + ".max_range", 12.0);
      cfg.time_field = param<std::string>(ns + ".time_field", "");
      loadAngleMask(ns, cfg);

      rt.x = param<double>(ens + ".x", 0.0);
      rt.y = param<double>(ens + ".y", 0.0);
      rt.z = param<double>(ens + ".z", 0.0);
      rt.roll = param<double>(ens + ".roll", 0.0);
      rt.pitch = param<double>(ens + ".pitch", 0.0);
      rt.yaw = param<double>(ens + ".yaw", 0.0);

      rt.converter = std::make_unique<LidarConverter>(cfg);
      if (publish_sensor_clouds_ && cfg.enabled) {
        rt.cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(
          cfg.cloud_topic, out_qos);
      }

      sensors_.push_back(std::move(rt));
      synchronizer_->registerSensor(static_cast<std::uint8_t>(k), id, cfg.enabled);
      diagnostics_->registerSensor(static_cast<std::uint8_t>(k), id);
    }

    // 구독은 sensors_ 벡터가 더 이상 재할당되지 않는 시점에 만든다
    // (콜백이 인덱스로 접근하므로 포인터 무효화 걱정은 없지만, 순서를 명확히 둔다).
    for (std::size_t k = 0; k < sensors_.size(); ++k) {
      SensorRuntime & rt = sensors_[k];
      if (!rt.cfg.enabled) {
        RCLCPP_INFO(this->get_logger(), "센서 %s 비활성(파라미터)", rt.cfg.id.c_str());
        continue;
      }
      if (rt.cfg.input_type == "cloud") {
        rt.cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
          rt.cfg.input_topic, in_qos,
          [this, k](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
            this->onCloud(k, *msg);
          });
      } else {
        rt.scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
          rt.cfg.input_topic, in_qos,
          [this, k](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
            this->onScan(k, *msg);
          });
      }
      RCLCPP_INFO(
        this->get_logger(), "센서 %s: %s (%s) frame=%s range=[%.2f, %.2f]%s",
        rt.cfg.id.c_str(), rt.cfg.input_topic.c_str(), rt.cfg.input_type.c_str(),
        rt.cfg.frame_id_override.c_str(), rt.cfg.min_range, rt.cfg.max_range,
        rt.cfg.hasAngleMask() ? " [FOV 제한 있음]" : "");
    }
  }

  void setupMotionSources()
  {
    const std::string source = param<std::string>("motion.twist_source", "none");
    if (source == "odom") {
      const std::string topic = param<std::string>("motion.odom_topic", "/odom");
      odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          VehicleTwist t;
          t.stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
          t.vx = msg->twist.twist.linear.x;
          t.vy = msg->twist.twist.linear.y;
          t.yaw_rate = msg->twist.twist.angular.z;
          t.valid = true;
          compensator_->setTwist(t);
        });
      RCLCPP_INFO(this->get_logger(), "motion 입력: %s (nav_msgs/Odometry)", topic.c_str());
    } else if (source == "twist") {
      const std::string topic = param<std::string>("motion.twist_topic", "/vehicle/twist");
      twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        topic, rclcpp::QoS(rclcpp::KeepLast(10)),
        [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          VehicleTwist t;
          t.stamp = rclcpp::Time(msg->header.stamp, RCL_ROS_TIME);
          t.vx = msg->twist.linear.x;
          t.vy = msg->twist.linear.y;
          t.yaw_rate = msg->twist.angular.z;
          t.valid = true;
          compensator_->setTwist(t);
        });
      RCLCPP_INFO(
        this->get_logger(), "motion 입력: %s (geometry_msgs/TwistStamped)", topic.c_str());
    }
  }

  void setupOutputs()
  {
    const rclcpp::QoS out_qos = makeQoS(output_qos_reliability_, output_qos_depth_);
    merged_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      merged_cloud_topic_, out_qos);
    merged_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      merged_scan_topic_, out_qos);
    if (publish_debug_cloud_) {
      debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        debug_cloud_topic_, out_qos);
    }
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", rclcpp::QoS(rclcpp::KeepLast(10)));
  }

  /// base_link → lidar_xx_link 를 **평평하게** 발행한다 (요구 §4 — 연쇄 금지).
  /// 외부에서 URDF/robot_state_publisher 로 이미 내보내고 있으면
  /// publish_static_tf:=false 로 끄면 된다.
  void publishStaticTransforms()
  {
    if (!publish_static_tf_) {
      RCLCPP_INFO(
        this->get_logger(),
        "static TF 발행 안 함 (publish_static_tf=false) — 외부 TF 소스를 쓴다");
      return;
    }
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    std::vector<geometry_msgs::msg::TransformStamped> tfs;
    tfs.reserve(sensors_.size());
    const rclcpp::Time stamp = this->now();
    for (const auto & rt : sensors_) {
      if (!rt.cfg.enabled) {
        continue;
      }
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = target_frame_;              // 부모는 언제나 base_link
      t.child_frame_id = rt.cfg.frame_id_override;
      t.transform.translation.x = rt.x;
      t.transform.translation.y = rt.y;
      t.transform.translation.z = rt.z;
      tf2::Quaternion q;
      q.setRPY(rt.roll, rt.pitch, rt.yaw);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();
      tfs.push_back(t);
      RCLCPP_INFO(
        this->get_logger(),
        "static TF %s -> %s : xyz=(%.3f, %.3f, %.3f) rpy=(%.3f, %.3f, %.3f)",
        target_frame_.c_str(), rt.cfg.frame_id_override.c_str(),
        rt.x, rt.y, rt.z, rt.roll, rt.pitch, rt.yaw);
    }
    if (!tfs.empty()) {
      static_broadcaster_->sendTransform(tfs);
    }
  }

  // ── 센서 콜백 ─────────────────────────────────────────────────────────
  void onScan(std::size_t k, const sensor_msgs::msg::LaserScan & msg)
  {
    SensorRuntime & rt = sensors_[k];
    ConvertStats cs;
    const bool ok = rt.converter->convert(msg, rt.scratch, cs);
    ingest(rt, ok, cs);
  }

  void onCloud(std::size_t k, const sensor_msgs::msg::PointCloud2 & msg)
  {
    SensorRuntime & rt = sensors_[k];
    ConvertStats cs;
    const bool ok = rt.converter->convert(msg, rt.scratch, cs);
    ingest(rt, ok, cs);
  }

  /// 정규화된 프레임의 공통 후처리: 센서별 cloud 발행 → base_link 변환 → 버퍼 투입.
  /// 변환을 여기서 하는 이유: 그 프레임 자신의 stamp 로 TF 를 조회하는 것이 가장
  /// 정확하고, 융합 주기에서 같은 프레임을 두 번 변환하는 일도 없어진다.
  /// 실패하면 센서 frame 그대로 넣어 두고, 융합 주기가 다시 시도한다.
  void ingest(SensorRuntime & rt, bool convert_ok, const ConvertStats & cs)
  {
    diagnostics_->recordMessage(rt.cfg.index, rt.scratch.stamp, cs, convert_ok);
    if (!convert_ok) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "센서 %s: 메시지를 해석할 수 없다 (빔 0 / 각도 정보 없음 / x,y,z 필드 없음)",
        rt.cfg.id.c_str());
      return;
    }
    rt.converter->setCapacityHint(std::max<std::size_t>(rt.scratch.points.size(), 64));

    const bool want_sensor_frame = publish_sensor_clouds_ && rt.cloud_pub &&
      sensor_cloud_frame_ == "sensor";
    if (want_sensor_frame) {
      toPointCloud2(rt.scratch, publish_intensity_, false, rt.cloud_msg);
      rt.cloud_pub->publish(rt.cloud_msg);
    }

    std::string err;
    const bool tf_ok = transformer_->transformInPlace(rt.scratch, err);
    if (!tf_ok) {
      diagnostics_->recordTfFailure(rt.cfg.index);
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "센서 %s: TF %s -> %s 실패 (%s). 이 프레임은 융합 주기에서 재시도한다.",
        rt.cfg.id.c_str(), rt.scratch.frame_id.c_str(), target_frame_.c_str(), err.c_str());
    } else if (publish_sensor_clouds_ && rt.cloud_pub && !want_sensor_frame) {
      toPointCloud2(rt.scratch, publish_intensity_, false, rt.cloud_msg);
      rt.cloud_pub->publish(rt.cloud_msg);
    }

    // scratch 를 넘기고, 밀려난 프레임의 벡터 capacity 를 돌려받는다.
    synchronizer_->push(std::move(rt.scratch), &rt.scratch);
  }

  // ── 융합 주기 ─────────────────────────────────────────────────────────
  void onFusionCycle()
  {
    const rclcpp::Time now = this->now();
    SyncResult sync = synchronizer_->collect(now);

    selected_.clear();
    CycleReport report;
    report.t_ref = sync.t_ref;

    double t_min = 0.0;
    double t_max = 0.0;
    bool spread_init = false;

    for (auto & r : sync.sensors) {
      if (r.frame == nullptr || !isContributing(r.status)) {
        continue;
      }
      SensorRuntime & rt = sensors_[r.index];
      rt.work = *r.frame;      // 작업 사본 (버퍼 원본은 다음 주기에 또 뽑힐 수 있다)

      // 콜백에서 이미 변환됐으면 no-op, 그때 실패했으면 여기서 재시도.
      std::string err;
      if (!transformer_->transformInPlace(rt.work, err)) {
        diagnostics_->recordTfFailure(r.index);
        r.status = FrameStatus::kTfFailed;
        r.frame = nullptr;
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "센서 %s: TF 재시도 실패 (%s) — 이번 주기에서 제외",
          rt.cfg.id.c_str(), err.c_str());
        continue;
      }

      compensator_->compensate(rt.work, sync.t_ref);

      if (!spread_init) {
        t_min = t_max = r.dt_to_ref_s;
        spread_init = true;
      } else {
        t_min = std::min(t_min, r.dt_to_ref_s);
        t_max = std::max(t_max, r.dt_to_ref_s);
      }
      selected_.push_back(&rt.work);
    }

    report.contributing = selected_.size();
    report.max_dt_spread_s = spread_init ? (t_max - t_min) : 0.0;
    diagnostics_->recordSync(sync);

    if (selected_.empty()) {
      // 요구 §20 Case6 — 아무 센서도 못 봤다. 조용히 넘어가지 않는다.
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "기여 센서 0 — merged 출력 없음. 드라이버/TF/max_cloud_age 확인 필요.");
      diagnostics_->recordCycle(report);
      maybeReport(now);
      return;
    }

    // ── 병합 → 필터 ──
    const MergeStats ms = merger_.merge(selected_, sync.t_ref, target_frame_, merged_);
    report.merged_points = ms.output_points;
    report.filter = filter_->apply(merged_);
    report.published_points = merged_.points.size();

    // ── 발행 ──
    toPointCloud2(merged_, publish_intensity_, false, merged_cloud_msg_);
    merged_cloud_pub_->publish(merged_cloud_msg_);

    if (debug_cloud_pub_) {
      toPointCloud2(merged_, true, true, debug_cloud_msg_);
      debug_cloud_pub_->publish(debug_cloud_msg_);
    }

    report.scan = scan_builder_->build(merged_, merged_scan_msg_);
    merged_scan_pub_->publish(merged_scan_msg_);

    report.published = true;
    diagnostics_->recordCycle(report);
    maybeReport(now);
  }

  void maybeReport(const rclcpp::Time & now)
  {
    if (!diagnostics_->due(now)) {
      return;
    }
    diag_pub_->publish(diagnostics_->toMessage(now));

    const std::uint8_t lvl = diagnostics_->level();
    if (lvl == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
      RCLCPP_ERROR(this->get_logger(), "%s", diagnostics_->summary().c_str());
    } else if (lvl == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      RCLCPP_WARN(this->get_logger(), "%s", diagnostics_->summary().c_str());
    } else if (!diag_log_level_warn_only_) {
      RCLCPP_INFO(this->get_logger(), "%s", diagnostics_->summary().c_str());
    }
  }

  // ── 파라미터 캐시 ────────────────────────────────────────────────────
  std::vector<std::string> sensor_ids_;
  std::string target_frame_;
  double fusion_rate_hz_{20.0};
  bool publish_static_tf_{true};
  double tf_timeout_s_{0.02};
  bool tf_static_fallback_{true};
  bool publish_sensor_clouds_{true};
  std::string sensor_cloud_frame_{"base_link"};
  bool publish_debug_cloud_{false};
  bool publish_intensity_{false};
  std::string merged_cloud_topic_;
  std::string merged_scan_topic_;
  std::string debug_cloud_topic_;
  std::string input_qos_reliability_;
  int input_qos_depth_{5};
  std::string output_qos_reliability_;
  int output_qos_depth_{5};
  bool diag_log_level_warn_only_{false};

  // ── 코어 ─────────────────────────────────────────────────────────────
  std::vector<SensorRuntime> sensors_;
  std::unique_ptr<CloudSynchronizer> synchronizer_;
  std::unique_ptr<CloudTransformer> transformer_;
  std::unique_ptr<MotionCompensator> compensator_;
  std::unique_ptr<CloudFilter> filter_;
  std::unique_ptr<VirtualLaserScan> scan_builder_;
  std::unique_ptr<Diagnostics> diagnostics_;
  CloudMerger merger_;

  // ── 재사용 버퍼 ──────────────────────────────────────────────────────
  std::vector<const CloudFrame *> selected_;
  CloudFrame merged_;
  sensor_msgs::msg::PointCloud2 merged_cloud_msg_;
  sensor_msgs::msg::PointCloud2 debug_cloud_msg_;
  sensor_msgs::msg::LaserScan merged_scan_msg_;

  // ── ROS 핸들 ─────────────────────────────────────────────────────────
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr merged_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr merged_scan_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace multi_lidar_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multi_lidar_fusion::MultiLidarFusionNode>());
  rclcpp::shutdown();
  return 0;
}
