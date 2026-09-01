# stack_traffic 출력 — 신호등·정지선 정지 요구
# 요구일 뿐 명령이 아님: 적용 여부·우선권은 MGM 스테이트 머신이 결정 (parking에서는 비활성)
std_msgs/Header header
bool stop_required
float32 stop_distance   # [m] 정지선까지 거리 (모르면 음수). PR #21부터 카메라
                        # optical-Z 기준 — MGM은 stop_required만 사용하므로 무해.

# 신호등 상태 머신 입력. red_active가 true가 되는 즉시 MGM은 TRAFFIC에 진입한다.
# stopline_detected가 안정 검출되는 매 틱마다 MGM은 stop_distance를 새로 신뢰해
# 거리를 갱신한다(2026-09-01 개정 — 최초 한 번만 래치하지 않음, 정지선 인식이
# 간헐적으로만 성공하는 조건에서 첫 관측 하나에 전체 제동을 고정하지 않기
# 위해서다). 정지선이 영상에서 사라지면 dSPACE 실차속도 적분으로 남은 거리를
# 추정한다. green_active는 TRAFFIC 즉시 해제.
bool red_active
bool green_active
bool stopline_detected

# 카메라/노드 고장 및 startup hold. 기존 stop_required는 구버전 소비자를 위해
# 이 값과 적색 정지 래치의 OR를 유지한다. 새 MGM은 적색 제동과 고장 정지를 구분한다.
bool fail_safe_stop
