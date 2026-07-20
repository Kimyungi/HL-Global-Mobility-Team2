# stack_traffic 출력 — 신호등·정지선 정지 요구
# 요구일 뿐 명령이 아님: 적용 여부·우선권은 MGM 스테이트 머신이 결정 (parking에서는 비활성)
std_msgs/Header header
bool stop_required
float32 stop_distance   # [m] 정지선까지 거리 (모르면 음수)
