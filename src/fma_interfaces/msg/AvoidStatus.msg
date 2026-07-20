# stack_avoid 출력 — 장애물 상황 + 회피 경로 (vehicle frame)
# "회피 가능" 판정 재료(TTC·측방 여유)는 여기서 제공, 전이 결정은 MGM 스테이트 머신에서.
std_msgs/Header header
bool obstacle_detected
bool avoidable          # TTC·측방 여유 충분 → 회피 기동 성립 가능
float32 ttc             # [s] time-to-collision (장애물 없으면 +inf 대신 큰 값)
bool narrow_gap         # 여유 폭 좁음 → avoid 스테이트에서 감속 근거
bool maneuver_done      # 회피 기동 완료 → 복귀 트리거
RefPoint[] points       # 회피 경로
float32 v_suggest       # [m/s] 회피 기하로 결정한 권장 속도
