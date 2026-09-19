# One MGM request's stop-then-five-frame wall acquisition.
std_msgs/Header header
uint64 request_id
bool mission_active
bool vehicle_speed_valid
bool stopped
float32 actual_speed
uint8 frame_count
bool complete
string phase
