# stack_lane 출력 — 차선 기반 경로 (vehicle frame)
std_msgs/Header header
RefPoint[] points
float32 confidence   # 0.0~1.0 — lane↔waypoint 전이 판정 입력 (판정은 MGM 스테이트 머신에서)
