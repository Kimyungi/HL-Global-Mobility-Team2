# stack_lane 출력 — 정지선 검출 (이현준 → stack_traffic 김재민 입력)
# OAK-D 장착 각도상 신호등이 안 보여 stack_lane은 정지선만 검출하고,
# 신호등 적색 판정은 stack_traffic이 별도 웹캠으로 수행해 둘을 결합한다.
# header.stamp 필수 — 수신 측 stale 판정(신선도)은 이 스탬프 기준.
std_msgs/Header header
bool detected      # 정지선 검출 여부. false면 distance 무효
float32 distance   # [m] 정지선까지 전방 거리 (vehicle frame x)
