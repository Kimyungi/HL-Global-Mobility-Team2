# stack_gps 출력 — waypoint 기반 경로 (vehicle frame으로 변환 완료 상태)
std_msgs/Header header
RefPoint[] points
bool accel_zone      # 가속구간 (종방향 ↑ 요구의 원천 — 적용 여부는 스테이트 우선권이 결정)
bool parking_zone    # GPS 주차구간 (lane→parking 전이 조건의 절반)
bool at_end          # 트랙 종점 도달 — waypoint 스테이트의 종점 정지 요구 원천 (§4)
uint8 fix_quality    # 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
# 트랙 최근접점까지의 거리 [m] — "지금 트랙 위에 있나"의 측정값.
# 판단은 MGM이 한다: waypoint→lane 전이는 회피 복귀 후 **실제로 트랙에 재합류한
# 뒤에만** 허용돼야 하는데(2026-08-14), 시간 게이트만으로는 이탈한 채 lane으로
# 넘어가 버린다. stack_gps가 이미 내부에서 쓰던 값을 계약에 노출한 것.
float32 cross_track_m
