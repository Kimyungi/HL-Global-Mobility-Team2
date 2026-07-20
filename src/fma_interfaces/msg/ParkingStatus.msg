# stack_parking 출력 — 주차공간 인식 + 주차 경로 (vehicle frame)
# 정적 경계(콘·연석)는 로컬맵 입력일 뿐 정지 트리거가 아님 (CLAUDE.md §4)
std_msgs/Header header
bool space_found        # 주차공간 인식 → lane→parking 전이 조건의 절반
bool path_blocked       # 경로 침범(동적) → parking 스테이트에서 정지 근거
bool done               # 주차 완료 → parking→lane 복귀
RefPoint[] points       # 주차 경로
float32 v_suggest       # [m/s] 주차 진행 권장 속도 (후진 = 음수)
