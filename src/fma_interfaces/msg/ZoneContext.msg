# Spatial membership only: zones never produce a driving reference.
uint8 NORMAL_ZONE=0
uint8 GPS_ONLY_ZONE=1
uint8 MISSION_ZONE=2
uint8 MISSION_NONE=0
uint8 T_PARKING=1
uint8 PARALLEL_PARKING=2
bool zone_valid
uint8 zone_id          # 1..255 configured IDs; 0 is implicit normal/outside
uint8 zone_type
uint8 mission_type
uint8 mission_id       # independent completion-memory key, 0..255
bool in_zone
# GPS publishes membership; GPS_ONLY uses station OR nearest preview waypoint,
# while MISSION uses station only. MGM alone computes these edges per zone_id.
bool zone_entered
bool zone_exited

# Raw input versus MGM-confirmed membership. in_zone aliases stable_in_zone on MGM output.
bool raw_in_zone
bool stable_in_zone
int32 enter_count
int32 exit_count
# Telemetry only: Euclidean distance to nearest configured track endpoint (not signed boundary clearance).
float32 boundary_distance_m -1.0

bool mission_entry_suppressed

uint8 LAST_MISSION_ZONE=3
