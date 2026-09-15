# Wall planner output. MGM consumes only control_enabled plans with unexpired leases.
bool control_enabled
RouteFeedback route
uint8 READY=0
uint8 PASS=1
uint8 REJOIN=2
uint8 EXIT_FOLLOW=3
uint8 HOLD=4
uint8 COMPLETE=5
uint8 WALL_FOLLOW=6
std_msgs/Header header
string route_id
uint64 episode_id
uint64 plan_id
uint64 observation_generation
uint8 phase
bool episode_active
bool perception_active

# Legacy diagnostics: occupancy near GPS, not a trigger. Wall following does not overtake.
bool obstacle_detected
bool maneuver_active
bool gps_follow
bool plan_valid
bool complete
bool deadline_hit
string reason
float64 processing_ms
float64 planner_ms
uint32 expanded_nodes
float64 station_m
float64 stop_distance_m
float64 min_clearance_m
builtin_interfaces/Time observation_stamp
builtin_interfaces/Time pose_stamp
builtin_interfaces/Time valid_until
# Diagnostic global path in header.frame_id; reference points are vehicle-frame.
RefPoint[] path
AvoidStatus reference
# Requested speed 100ms along the current plan and per-point speed diagnostics.
# Path starts at measured velocity; these do not imply instantaneous deceleration.
float64 speed_limit_mps
float64[] path_speed_mps
