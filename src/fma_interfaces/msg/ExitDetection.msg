# A single camera frame; class 0=Right/07, class 1=Left/06, -1=no unambiguous detection.
# reference_stamp is capture time, never inference completion/publication time.
builtin_interfaces/Time reference_stamp
uint64 request_id
int32 class_id -1
float32 confidence
