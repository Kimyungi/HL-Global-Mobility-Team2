# stack_estop 출력 — 돌발 장애물 긴급 정지 요구
# 정지는 스테이트가 아니다: 각 스테이트 우선권 표의 최상위 요구, v_ref=0으로만 반영 (CLAUDE.md §4)
std_msgs/Header header
bool estop

# 후방 여유 — 후진 탈출(§4)의 전제 조건. "지금 뒤로 빼도 되는가"만 답한다.
# estop과 같은 레벨 신호다: 매 스캔 재평가, 래치 금지.
# **후방 센서가 없거나 스캔이 무효면 false** — 모르면 false 가 안전측이다.
# MGM은 escape_require_rear_clear 가 켜져 있으면 이 값이 true 일 때만 후진한다.
# 판정은 stack_estop 소관 — 어느 라이다를 쓸지(후방 단독 / 4개 통합)도 그쪽 결정이다.
bool rear_clear
