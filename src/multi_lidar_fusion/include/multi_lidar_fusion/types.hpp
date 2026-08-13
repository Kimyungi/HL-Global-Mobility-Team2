// multi_lidar_fusion — 내부 데이터 계약
//
// 이 파일의 역할:
//   파이프라인 전 단계(정규화→변환→동기→보상→병합→필터→가상스캔)가 공유하는
//   **유일한** 데이터 표현을 정의한다. 각 단계가 ROS 메시지를 직렬화/역직렬화하며
//   주고받으면 매 주기 수 MB 복사가 생기므로, 콜백에서 한 번 이 표현으로 바꾼 뒤
//   발행 직전에 한 번만 PointCloud2 로 되돌린다.
//
// 입력 topic / 출력 topic: 없음 (순수 자료구조)
// frame: CloudFrame::frame_id 가 그 시점의 좌표계를 들고 다닌다.
//        정규화 직후 = 센서 frame, CloudTransformer 통과 후 = base_link.

#ifndef MULTI_LIDAR_FUSION__TYPES_HPP_
#define MULTI_LIDAR_FUSION__TYPES_HPP_

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "rclcpp/time.hpp"

namespace multi_lidar_fusion
{

/// 센서 슬롯 최대 개수 (a1/a2/b1/b2). 늘려도 코어 로직은 그대로다.
constexpr std::size_t kMaxSensors = 8;

/// 각도를 (-pi, pi] 로 정규화.
inline double normalizeAngle(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a <= -M_PI) {a += 2.0 * M_PI;}
  return a;
}

/// 경계 판정 여유 [rad]. LaserScan 의 각도는 float 로 누적되므로
/// fov_min_deg: -45.0 처럼 딱 떨어지는 값을 줘도 경계 빔이 미세하게 밖으로 계산된다.
/// 0.006° 는 어떤 2D 라이다의 각분해능보다도 훨씬 작아 판정에 영향이 없다.
constexpr double kAngleEps = 1e-4;

/// 각도 구간 [min, max]. **min > max 이면 ±pi 를 가로지르는 구간**으로 해석한다.
/// 예) min=+150°, max=-150° → 후방 ±30° 만 유효.
/// 라이다의 "보는 범위·방향"을 센서 좌표계에서 잘라내는 데 쓴다(요구: 사용자 지정 FOV).
struct AngularSector
{
  double min_rad{-M_PI};
  double max_rad{M_PI};

  bool wraps() const {return min_rad > max_rad;}

  bool contains(double a) const
  {
    const double n = normalizeAngle(a);
    const double lo = min_rad - kAngleEps;
    const double hi = max_rad + kAngleEps;
    if (wraps()) {
      return n >= lo || n <= hi;
    }
    return n >= lo && n <= hi;
  }
};

/// 융합 파이프라인이 다루는 최소 점 표현.
///
///  - x/y/z        : 현재 frame_id 기준 좌표 [m]
///  - intensity    : 센서가 주지 않으면 0. 병합 시 필드 불일치를 흡수하는 자리.
///  - dt           : **프레임 stamp 기준 이 점의 상대 획득시각 [s]**.
///                   LaserScan 은 header.stamp = 첫 ray 시각이므로 dt = i*time_increment.
///                   드라이버가 time_increment=0 을 주면 0 (프레임 단위 보상으로 축퇴).
///                   Motion compensation 이 이 값을 쓴다.
///  - sensor_id    : 0=a1, 1=a2, 2=b1, 3=b2 … 디버그/추적용. 병합 후에도 유지된다.
struct FusionPoint
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
  float dt{0.0F};
  std::uint8_t sensor_id{0};
};

/// 한 센서의 한 스캔(= 한 프레임) 전체.
struct CloudFrame
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};   ///< header.stamp (첫 ray 시각)
  std::string frame_id;                     ///< 현재 좌표계
  std::uint8_t sensor_id{0};
  std::size_t seq{0};                       ///< 이 센서에서 몇 번째로 받은 프레임인가
  std::vector<FusionPoint> points;

  bool empty() const {return points.empty();}
  void clear()
  {
    points.clear();
    frame_id.clear();
    seq = 0;
  }
};

/// 센서 1대의 설정. 전부 YAML/파라미터에서 온다 (코드 하드코딩 금지 — 요구 §29-5).
struct SensorConfig
{
  std::string id;                  ///< "a1" … 파라미터 네임스페이스 키
  std::uint8_t index{0};           ///< 0..3, FusionPoint::sensor_id 와 동일
  std::string input_topic;         ///< 예: /lidar/a1/scan
  std::string input_type{"scan"};  ///< "scan"(LaserScan) | "cloud"(PointCloud2)
  std::string cloud_topic;         ///< 정규화 결과 발행 토픽 (예: /lidar/a1/cloud)
  /// TF 조회에 쓸 frame. 비우면 수신 메시지의 header.frame_id 를 그대로 쓴다.
  /// 드라이버가 엉뚱한 frame_id 를 박아 넣는 경우를 파라미터로 덮어쓰기 위한 탈출구.
  std::string frame_id_override;
  double min_range{0.05};          ///< 이보다 가까운 측정은 신뢰하지 않음 [m]
  double max_range{12.0};          ///< 이보다 먼 측정은 버림 [m]
  /// PointCloud2 입력에서 점별 시각을 담고 있는 필드 이름 ("" = 없음).
  std::string time_field;
  bool enabled{true};

  // ── 센서가 "보는 범위·방향" (센서 좌표계 기준, 정규화 단계에서 적용) ──────────
  /// false 면 전방위 사용. YAML 에서 켜고 min/max 만 고치면 된다.
  bool fov_enabled{false};
  AngularSector fov;
  /// 브래킷·차체에 가려 못 믿는 구간들. fov 안이라도 여기 들어오면 버린다.
  std::vector<AngularSector> blind_sectors;

  /// 이 센서가 fov/blind 를 통과시키는지.
  bool angleAccepted(double a) const
  {
    if (fov_enabled && !fov.contains(a)) {
      return false;
    }
    for (const auto & b : blind_sectors) {
      if (b.contains(a)) {
        return false;
      }
    }
    return true;
  }

  bool hasAngleMask() const {return fov_enabled || !blind_sectors.empty();}
};

/// 한 융합 주기에서 센서 하나에 무슨 일이 있었는지.
enum class FrameStatus : std::uint8_t
{
  kUsed = 0,        ///< 정상 사용
  kReused,          ///< 새 데이터가 없어 직전 프레임 재사용 (느린 센서)
  kNeverReceived,   ///< 기동 후 한 번도 못 받음
  kTooOld,          ///< max_cloud_age 초과 → 배제
  kOutOfSync,       ///< sync_tolerance 초과 → strict_sync 일 때만 배제
  kTfFailed,        ///< TF 없음 → 배제
  kEmpty,           ///< 유효점 0 → 무해하게 무시
  kDisabled,        ///< 파라미터로 꺼둠
};

inline const char * toString(FrameStatus s)
{
  switch (s) {
    case FrameStatus::kUsed: return "used";
    case FrameStatus::kReused: return "reused";
    case FrameStatus::kNeverReceived: return "never_received";
    case FrameStatus::kTooOld: return "too_old";
    case FrameStatus::kOutOfSync: return "out_of_sync";
    case FrameStatus::kTfFailed: return "tf_failed";
    case FrameStatus::kEmpty: return "empty";
    case FrameStatus::kDisabled: return "disabled";
  }
  return "unknown";
}

/// 그 상태의 프레임이 융합에 들어갔는가.
inline bool isContributing(FrameStatus s)
{
  return s == FrameStatus::kUsed || s == FrameStatus::kReused;
}

/// 차량 순간 운동 상태 (body frame = base_link).
/// Motion compensation 인터페이스의 유일한 입력 — /odom 이든 /vehicle/twist 든
/// dSPACE vehicle_vector 든 이 구조체로 환원해서 넣는다.
struct VehicleTwist
{
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  double vx{0.0};        ///< longitudinal velocity [m/s]
  double vy{0.0};        ///< lateral velocity [m/s]
  double yaw_rate{0.0};  ///< [rad/s]
  bool valid{false};
};

/// 유효하지 않은 거리(미관측)를 나타내는 값 — LaserScan 관례상 +inf.
constexpr float kNoReturn = std::numeric_limits<float>::infinity();

}  // namespace multi_lidar_fusion

#endif  // MULTI_LIDAR_FUSION__TYPES_HPP_
