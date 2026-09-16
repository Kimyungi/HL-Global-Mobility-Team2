# PR108 measured recovery executor -> MGM. No direct CAN/TargetRef publisher.
# MGM publishes the current episode ID in MgmState.estop_request_id.
std_msgs/Header header
builtin_interfaces/Time reference_stamp
uint64 request_id
bool done
float32 v_suggest
RefPoint[] points
