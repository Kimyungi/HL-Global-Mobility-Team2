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

# False when scan timeout/invalid scan caused estop. Allows MGM to distinguish
# LiDAR failure from actual danger without changing emergency-brake thresholds.
bool scan_valid

# Dedicated escape corridor certification. No current producer certifies CLEAR.
# rear_clear remains a legacy bool and an additional veto; it alone never enables base Recovery.
uint8 REAR_UNKNOWN=0
uint8 REAR_CLEAR=1
uint8 REAR_BLOCKED=2
uint8 rear_corridor_state
bool rear_sensor_valid
builtin_interfaces/Time rear_reference_stamp
