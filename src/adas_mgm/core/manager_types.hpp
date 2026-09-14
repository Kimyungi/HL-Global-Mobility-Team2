#ifndef ADAS_MGM__CORE__MANAGER_TYPES_HPP_
#define ADAS_MGM__CORE__MANAGER_TYPES_HPP_

#include <cstdint>
#include <limits>

namespace adas_mgm
{
enum class TopState : uint8_t {AUTONOMOUS_ENABLE, AUTONOMOUS_DRIVE, FINISH};
enum class NavState : uint8_t {LINE, GPS_BACKUP, GPS_ONLY_NAV};
enum class AvoidState : uint8_t {INACTIVE, AVOID_ACTIVE, CLEAR_CONFIRM, GPS_RETURN};
enum class SignalState : uint8_t {SIGNAL_IDLE, RED_DETECTED, APPROACH_STOP_LINE, STOPPED_WAIT};
enum class SafetyState : uint8_t {NORMAL, AUTO_ESTOP, REVERSE_RECOVERY, SAFE_STOP};
enum class MissionState : uint8_t {MISSION_IDLE=0, MISSION_ACTIVE=1, MISSION_PREPARE=2};
enum class MissionType : uint8_t {NONE, T_PARKING, PARALLEL_PARKING};
enum class SpeedOwner : uint8_t {NAVIGATION, AVOIDANCE, TRAFFIC, MISSION, SAFETY, FINISH};
enum class ZoneType : uint8_t {NORMAL_ZONE, GPS_ONLY_ZONE, MISSION_ZONE};

enum class CalibrationState : uint8_t {UNCALIBRATED=0, CALIBRATED=1, INVALID_CONFIG=2, NOT_REQUIRED=3};
enum class RearCorridorState : uint8_t {UNKNOWN=0, CLEAR=1, BLOCKED=2};
enum class RecoveryBlockReason : uint8_t {
  NONE, CONFIG_DISABLED, REAR_UNKNOWN, REAR_BLOCKED, REAR_INVALID,
  NOT_DRIVING, MISSION_ACTIVE, FORCED_STOP, SIGNAL_STOP, NOT_ARMED, NO_DANGER, WAIT_DELAY
};
enum class RecoveryReason : uint8_t {
  NONE, ENTERED, TIME_LIMIT, REAR_LOST, DANGER_CLEARED, CONFIG_DISABLED,
  AUTHORITY_LOST, REFERENCE_INVALID
};
struct RecoveryDiagnostics {
  bool configured, rear_sensor_valid;
  RearCorridorState rear_corridor_state;
  bool eligible;
  RecoveryBlockReason block_reason;
  uint64_t attempt_count;
  double command_time_s, measured_distance_m;
  bool measured_distance_complete;
  RecoveryReason last_reason;
};

constexpr int MGM_REFERENCE_PROVIDERS = 5;  // existing MGM_SRC_* order
enum SafeStopReason : uint32_t
{
  SAFE_STOP_GPS_ONLY_GPS_LOSS = 1u << 0,
  SAFE_STOP_ALL_SENSORS_LOST = 1u << 1,
  SAFE_STOP_REFERENCE_INVALID = 1u << 2,
  SAFE_STOP_EXTERNAL = 1u << 3,
  SAFE_STOP_MISSION_FEEDBACK = 1u << 4,
  SAFE_STOP_TRAFFIC_INPUT = 1u << 5,
  SAFE_STOP_VEHICLE_SPEED = 1u << 6,
  SAFE_STOP_REAR_UNAVAILABLE = 1u << 7,
  SAFE_STOP_ZONE_CONTEXT_UNAVAILABLE = 1u << 8,
  SAFE_STOP_ROUTE_SEQUENCE = 1u << 9,
  SAFE_STOP_MISSION_ZONE_UNKNOWN = 1u << 10,
  SAFE_STOP_LIDAR_INPUT = 1u << 11,
};
struct ReferenceSample
{
  uint64_t generation;  // actual input generation stamp, never publication count
  float age_s;         // monotonic elapsed age; -1 if no known generation
  float timeout_s;     // existing provider stale_timeout_sec, supplied by wrapper
};
struct ReferenceStatus
{
  uint8_t source;
  bool available;
  bool valid;
  bool fresh;
  float age_s;
  uint64_t generation;
};

// Representation capacity (uint8 mission IDs), not a driving/timing threshold.
constexpr int MGM_MISSION_CAPACITY = std::numeric_limits<uint8_t>::max() + 1;
constexpr int MGM_CLEAR_CYCLES = 200;  // user requirement: 2s at 100Hz
constexpr int MGM_ZONE_CAPACITY = std::numeric_limits<uint8_t>::max() + 1;
struct ZoneObservation
{
  uint8_t zone_id;  // 1..255, 0 is implicit NORMAL_ZONE outside all definitions
  ZoneType zone_type;
  MissionType mission_type;
  uint8_t mission_id;  // distinct from zone_id; multiple zones can share a mission
  bool in_zone;
  float boundary_distance_m;  // Euclidean distance to nearest configured endpoint, telemetry only
};
struct ZoneSnapshot
{
  bool zone_valid;
  int32_t count;
  ZoneObservation observations[MGM_ZONE_CAPACITY];
  uint64_t generation;  // independent GNSS fix, held across timer publications
};
struct ZoneContext
{
  bool zone_valid;
  uint8_t zone_id;
  ZoneType zone_type;
  MissionType mission_type;
  uint8_t mission_id;
  bool in_zone;
  bool zone_entered;
  bool zone_exited;
  bool mission_entry_suppressed;  // reset inside a zone requires confirmed exit before another mission entry
  bool raw_in_zone;
  bool stable_in_zone;  // in_zone is the compatibility alias of this value
  int32_t enter_count, exit_count;
  float boundary_distance_m;
};
struct ZoneState
{
  ZoneContext contexts[MGM_ZONE_CAPACITY];  // indexed by stable Zone ID
  ZoneContext selected;  // display priority only; never suppresses other memberships
  bool in_gps_only_zone;
  uint64_t last_generation;
  CalibrationState calibration;
  bool definitions_seen;
};

enum class MissionCancelReason : uint8_t
{
  NONE, SEARCH_TIMEOUT, TRAVEL_DISTANCE, EXPLICIT, FINISH, SESSION_RESET,
  CALIBRATION_REQUIRED, MOTION_UNAVAILABLE, MODULE_ABORT, ZONE_EXIT, ROUTE_END
};
enum MissionEvent : uint32_t
{
  MISSION_EVENT_ZONE_ENTRY=1u, MISSION_EVENT_SEARCH_START=2u,
  MISSION_EVENT_SPACE_FOUND=4u, MISSION_EVENT_READY=8u,
  MISSION_EVENT_HANDOFF=16u, MISSION_EVENT_CANCEL=32u, MISSION_EVENT_DONE=64u
};
struct MissionObservation
{
  bool recorded;
  int64_t time_ns;  // ROS time, for correlation with logs
  bool position_valid;
  double x, y;     // existing GPS ENU map frame
  int32_t track_index;  // -1 if unavailable
  bool speed_valid;
  float actual_speed;
  uint8_t zone_id;
  double travel_distance;
};
struct MissionRequest
{
  bool active;
  uint64_t request_id;
  uint8_t mission_id;
  MissionType mission_type;
  uint8_t source_zone_id;
  int64_t start_time_ns;  // monotonic lifetime clock
  int64_t last_update_ns;
  double elapsed_s;
  double travel_distance;
  bool search_acknowledged;
  bool space_found;
  bool preparation_ready;
  MissionCancelReason cancel_reason;
  MissionObservation zone_entry, search_start, space, ready, handoff;
};

enum class RoutePhase : uint8_t {DISABLED=0, RUNNING=1, WAIT_MISSION=2, WAIT_STOP=3, WAIT_ACK=4, FINISHED=5, FAULT=6};
enum class RouteCompletion : uint8_t {ENDPOINT_AND_MISSIONS=0, MISSIONS_COMPLETE=1};
struct RouteFeedback {
  RouteCompletion completion;
  bool enabled;
  bool connecting, next_connecting;
  uint64_t sequence_id, instance_id, acknowledged_request;
  int32_t index, count, required_count;
  uint8_t required_missions[256];
};
struct RouteControl {
  bool enabled;
  RoutePhase phase;
  RouteCompletion completion;
  uint64_t sequence_id, instance_id, request_id;
  int32_t index, count, requested_index;
  bool seen_nonterminal, end_reached, changed, session_reset_pending;
  bool connecting, next_connecting, requested_connecting;
  uint64_t last_generation, request_generation;
  bool required_missions[256];
};
struct ManagerState
{
  TopState top;
  NavState nav;
  AvoidState avoid;
  SignalState signal;
  SafetyState safety;
  MissionState mission;
  MissionType mission_type;
  bool gps_only_context;
  ZoneState zones;
  int32_t clear_count;
  bool avoid_fallback_only;
  // Zone-only avoidance uses independent GPS fixes, not manager timer ticks.
  bool avoid_zone_inside;
  bool avoid_zone_maneuver_seen;
  bool avoid_zone_completed;
  int32_t avoid_zone_enter_count, avoid_zone_exit_count;
  uint64_t avoid_zone_generation;
  bool mission_completed[MGM_MISSION_CAPACITY];
  bool mission_failed[MGM_MISSION_CAPACITY];  // terminal zone-exit failure, distinct from success
  uint8_t active_mission;
  bool mission_feedback_seen;
  bool mission_start;
  bool mission_cancel;
  bool mission_prepare;
  MissionRequest request;
  uint64_t last_request_id;  // survives explicit session reset
  uint32_t mission_events;
  bool recovery_waiting_reference;
  bool avoid_episode_reference_seen;
  uint32_t safe_stop_reasons;
  RecoveryDiagnostics recovery;
  int64_t previous_tick_ns;
  bool previous_tick_known, previous_reverse_command;
  RouteControl route;
};
}  // namespace adas_mgm
#endif
