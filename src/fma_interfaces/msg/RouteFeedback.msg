# Immutable plan identity and currently applied GPS route (indices are zero-based).
uint8 ENDPOINT_AND_MISSIONS=0
uint8 MISSIONS_COMPLETE=1
uint8 completion
bool enabled
bool connecting
bool next_connecting
uint64 sequence_id
uint64 instance_id
uint64 acknowledged_request
int32 index
int32 count
uint8[] required_missions
string route_id
string waypoint_csv
string zones_file

# Optional final exit branches, preloaded under the same immutable sequence ID.
bool terminal
bool last_mission_enabled
int32 left_index
int32 right_index
