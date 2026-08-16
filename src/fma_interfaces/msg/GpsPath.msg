# stack_gps 출력 — waypoint 기반 경로 (vehicle frame으로 변환 완료 상태)
std_msgs/Header header
RefPoint[] points
bool accel_zone      # 가속구간 (종방향 ↑ 요구의 원천 — 적용 여부는 스테이트 우선권이 결정)
bool parking_zone    # GPS 주차구간 (lane→parking 전이 조건의 절반)
bool at_end          # 트랙 종점 도달 — waypoint 스테이트의 종점 정지 요구 원천 (§4)
uint8 fix_quality    # 0=invalid, 1=GPS, 2=DGPS, 4=RTK fixed, 5=RTK float
# 트랙까지의 **수직거리** [m] — "지금 트랙 위에 있나"의 측정값.
# 2026-08-17: 최근접 **꼭짓점**까지의 점-대-점 거리 → 최근접 **선분**에 내린
# 수선의 길이로 정정. 종전 값은 트랙이 이산 웨이포인트라, 차가 트랙과 완벽히
# 나란히 가도 간격 L 에 맞춰 h ↔ √(h²+(L/2)²) 로 진동했다 — run_0817_020112
# 실측(L=0.342m, 0.6m/s): 주기 0.583s·진폭 0.081m 로 실제 오차 0.104m 의 78%.
# 아래 전이 게이트가 0.5m 문턱이라 그대로 둘 수 없었다(속도·간격에 비례해 커진다).
# 판단은 MGM이 한다: waypoint→lane 전이는 회피 복귀 후 **실제로 트랙에 재합류한
# 뒤에만** 허용돼야 하는데(2026-08-14), 시간 게이트만으로는 이탈한 채 lane으로
# 넘어가 버린다. stack_gps가 이미 내부에서 쓰던 값을 계약에 노출한 것.
float32 cross_track_m

# 헤딩 추정 소스 (2026-08-16 신설). MGM이 "이 헤딩을 믿어도 되는가"를 알아야 하는데
# 종전 계약엔 그 정보가 없었다 — fix_quality가 위치 신뢰도만 알려주듯, 헤딩에도
# 같은 것이 필요하다.
#   HEADING_TANGENT: 헤딩을 모를 때의 폴백 = "최근접 트랙 접선 = 차량 헤딩" 가정.
#     차가 트랙 위에 정렬돼 있을 때만 유효하며, **이탈 복귀 중에는 가정이 깨진다**
#     (path_engine.snapshot() docstring). 이 값이면 헤딩 기반 판단을 하면 안 된다.
#   HEADING_COG: GNSS 이동방향. 저속·정지에서 무작위가 된다.
#   HEADING_FUSED: IMU 융합 — 정지 중에도 유효한 유일한 소스.
# 실사례(2026-08-16 run_0816_184505): IMU 미연결로 융합이 죽고 COG↔접선을 오가며
# 트랙 기준 헤딩오차가 0.1초 단위로 0°↔130° 반전 → 재합류 목표 방위가 ±25°로
# 좌우 뒤집히고 역방향 가드가 걸려 5m 이탈 지점에서 영구 정지했다.
uint8 HEADING_TANGENT = 0
uint8 HEADING_COG = 1
uint8 HEADING_FUSED = 2
uint8 heading_source
