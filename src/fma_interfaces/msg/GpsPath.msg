# stack_gps 출력 — waypoint 기반 경로 (vehicle frame으로 변환 완료 상태)
std_msgs/Header header
RefPoint[] points
bool accel_zone      # 가속구간 (종방향 ↑ 요구의 원천 — 적용 여부는 스테이트 우선권이 결정)
bool parking_zone    # GPS 주차구간 (lane→parking 전이 조건의 절반)
uint8 fix_quality    # 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
