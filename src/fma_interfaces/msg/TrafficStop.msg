# stack_traffic 출력 — 신호등·정지선 정지 요구
# 요구일 뿐 명령이 아님: 적용 여부·우선권은 MGM 스테이트 머신이 결정 (parking에서는 비활성)
std_msgs/Header header
bool stop_required
float32 stop_distance   # [m] 정지선까지 거리 (모르면 음수). PR #21부터 카메라
                        # optical-Z 기준 — MGM은 stop_required만 사용하므로 무해.

# 아래 3개는 진단·테스트 전용 raw 신호 (2026-09-01, 카메라 CAN 테스트 하네스용).
# stop_required와 달리 근접·해제 래치를 거치지 않은 이번 프레임의 원시 판정이다.
bool red_active         # 최근 vote_window 중 적색 투표가 minimum_red_votes 이상
bool green_active       # 최근 vote_window 중 초록 투표가 minimum_green_votes 이상
bool stopline_detected  # 이번 프레임에 정지선 라인이 검출됐는지 (stable 아님)
