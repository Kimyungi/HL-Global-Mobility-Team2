# CLAUDE.md §3 RX — dSPACE 상태 추정 결과 → PC localization 보정 입력
std_msgs/Header header
float32 x       # [m]
float32 y       # [m]
float32 yaw     # [rad]
float32 v       # [m/s]
float32 str     # [rad] 실제 조향각
float32 str_ref # [rad] MPC 명령 조향각 (v5/PR #52 신규. v3 프레임 수신 시 0)
uint32 counter  # dSPACE 측 패킷 카운터
