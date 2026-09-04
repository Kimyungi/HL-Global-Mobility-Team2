# stack_traffic 출력 — 신호등·정지선 정지 요구
# 요구일 뿐 명령이 아님: 적용 여부·우선권은 MGM 스테이트 머신이 결정 (parking에서는 비활성)
std_msgs/Header header
bool stop_required
float32 stop_distance   # [m] 정지선까지 카메라 optical-Z 거리 (모르면 음수).
                        # 진단용이며 MGM의 제동 거리는 정지선 소실 edge에서 시드한다.

# 신호등 상태 머신 입력. red_active와 stopline_detected가 동시에 true일 때만
# MGM은 TRAFFIC에 진입한다. 진입 후에는 정지선이 화면에서 사라져도 확정 초록
# 전까지 TRAFFIC 상태를 유지한다.
# stopline_detected는 안정 영상 검출과 유효 depth가 함께 성립한 현재 관측이다.
# 이 값이 true에서 false로 바뀌면 MGM이 남은 거리를 시드하고, 이후 dSPACE
# 실차속도 적분으로 거리를 추정한다. green_active는 TRAFFIC을 즉시 해제한다.
bool red_active
bool green_active
bool stopline_detected

# 카메라/노드 고장 및 startup hold. 기존 stop_required는 구버전 소비자를 위해
# 이 값과 적색 정지 래치의 OR를 유지한다. 새 MGM은 적색 제동과 고장 정지를 구분한다.
bool fail_safe_stop
