# CLAUDE.md §3 RX — dSPACE 상태 추정 결과 → PC localization 보정 입력
std_msgs/Header header
float32 x       # [m]
float32 y       # [m]
float32 yaw     # [rad]
float32 v       # [m/s]
float32 str     # [rad] 조향각
uint32 counter  # dSPACE 측 패킷 카운터
