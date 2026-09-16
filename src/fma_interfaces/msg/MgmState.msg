# Parallel manager diagnostics, published once per MGM tick. These fields are
# not additional control commands; /adas/target_ref remains the sole output.
std_msgs/Header header
RouteControl route
uint32 SAFE_STOP_ROUTE_SEQUENCE=512
uint8 MISSION_IDLE=0
uint8 MISSION_ACTIVE=1
uint8 MISSION_PREPARE=2
# 0=AUTONOMOUS_ENABLE, 1=AUTONOMOUS_DRIVE, 2=FINISH
uint8 top
# 0=LINE, 1=GPS_BACKUP, 2=GPS_ONLY_NAV
uint8 navigation
# 0=INACTIVE, 1=AVOID_ACTIVE, 2=CLEAR_CONFIRM (historical), 3=GPS_RETURN (still AVOID)
uint8 avoidance
float32 avoid_return_cross_track_m
float32 avoid_return_yaw_error_rad
bool avoid_return_error_valid
# 0=SIGNAL_IDLE, 1=RED_DETECTED, 2=APPROACH_STOP_LINE, 3=STOPPED_WAIT
uint8 signal
# 0=NORMAL, 1=AUTO_ESTOP (legacy), 2=REVERSE_RECOVERY (legacy), 3=SAFE_STOP, 4=ESTOP
uint8 safety
# 0=MISSION_IDLE, 1=MISSION_ACTIVE, 2=MISSION_PREPARE
uint8 mission
# 0=NONE, 1=T_PARKING, 2=PARALLEL_PARKING
uint8 mission_type
# Existing MGM_SRC_* values
uint8 reference_source
# 0=NAVIGATION, 1=AVOIDANCE, 2=TRAFFIC, 3=MISSION, 4=SAFETY, 5=FINISH
uint8 speed_owner
bool reference_available
bool immediate_stop
ZoneContext zone       # display priority: MISSION > GPS_ONLY > NORMAL
ZoneContext[] zones    # all current memberships plus exit edges; overlap is preserved
bool in_gps_only_zone
uint8 active_mission_id

# Reference safety diagnostics; no extra control command topic.
bool selected_reference_valid
bool selected_reference_fresh
float32 selected_reference_age_s  # -1: no known actual generation
uint64 selected_reference_generation
bool line_ref_valid
bool gps_ref_valid
bool avoid_ref_valid
bool mission_ref_valid
bool recovery_ref_valid
bool avoid_episode_reference_seen
uint32 SAFE_STOP_GPS_ONLY_GPS_LOSS=1
uint32 SAFE_STOP_ALL_SENSORS_LOST=2
uint32 SAFE_STOP_REFERENCE_INVALID=4
uint32 SAFE_STOP_EXTERNAL=8
uint32 SAFE_STOP_MISSION_FEEDBACK=16
uint32 SAFE_STOP_TRAFFIC_INPUT=32
uint32 SAFE_STOP_VEHICLE_SPEED=64
uint32 SAFE_STOP_REAR_UNAVAILABLE=128
uint32 active_safe_stop_reasons

# Last request is retained for diagnostics after active clears.
bool mission_request_active
uint64 mission_request_id
uint8 mission_source_zone_id
uint8 request_mission_id
uint8 request_mission_type
int64 request_start_monotonic_ns
float64 search_elapsed_s
float64 search_travel_distance
bool parking_search_acknowledged
bool parking_space_found
bool parking_ready
bool mission_completed
bool mission_failed  # Zone-exit (historical) or route-end failure; never a success
bool parking_search_zone_only
bool parking_zone_entry_active
uint8 CANCEL_NONE=0
uint8 CANCEL_SEARCH_TIMEOUT=1
uint8 CANCEL_TRAVEL_DISTANCE=2
uint8 CANCEL_EXPLICIT=3
uint8 CANCEL_FINISH=4
uint8 CANCEL_SESSION_RESET=5
uint8 CANCEL_CALIBRATION_REQUIRED=6
uint8 CANCEL_MOTION_UNAVAILABLE=7
uint8 CANCEL_MODULE_ABORT=8
uint8 CANCEL_ZONE_EXIT=9
uint8 CANCEL_ROUTE_END=10
uint8 mission_cancel_reason
uint32 mission_events
MissionObservation zone_entry_observation
MissionObservation search_start_observation
MissionObservation space_found_observation
MissionObservation ready_observation
MissionObservation handoff_observation

uint8 UNCALIBRATED=0
uint8 CALIBRATED=1
uint8 INVALID_CONFIG=2
uint8 NOT_REQUIRED=3  # time/distance calibration not used by zone-only search
uint8 parking_calibration_state
uint8 zone_calibration_state
uint32 SAFE_STOP_ZONE_CONTEXT_UNAVAILABLE=256
uint32 SAFE_STOP_MISSION_ZONE_UNKNOWN=1024
uint64 zone_generation
int32 zone_enter_confirm_samples
int32 zone_exit_confirm_samples
bool gps_position_valid
float64 gps_x
float64 gps_y
int32 gps_track_index
int32 gps_previous_track_index
float64 gps_position_step_m
float64 gps_vehicle_heading_rad
bool gps_vehicle_heading_valid
bool recovery_configured
bool rear_sensor_valid
uint8 REAR_UNKNOWN=0
uint8 REAR_CLEAR=1
uint8 REAR_BLOCKED=2
uint8 rear_corridor_state
bool recovery_eligible
uint8 recovery_block_reason
uint64 recovery_attempt_count
float64 cumulative_reverse_command_time
float64 cumulative_reverse_measured_distance
bool reverse_measured_distance_complete
uint8 last_recovery_reason
bool traffic_distance_known
float32 traffic_remaining_m
bool traffic_stop_in_success_region

# Start authorization requires live LiDAR AND (camera frame OR fresh RTK FIXED), independent
# of lane detection. Driving reference validity is reported separately above.
bool camera_available
bool gps_fixed_ready
bool lidar_ready
string[] lidar_missing_topics
bool start_ready
bool go_authorized

# bit 0 lane camera, 1 traffic camera, 2..5 raw LiDAR a1/a2/b1/b2, 6 GPS.
# Any live bit prevents sensor-only SAFE_STOP. Availability != a usable driving path.
uint8 sensor_alive_mask
bool safe_stop_all_sensors_only
bool reference_motion_blocked

# Revised v2: transition-only recovery contract and traffic module lifecycle.
bool revised_v2
bool estop_active
uint64 estop_request_id
bool traffic_zone_active
