# CLAUDE.md §3 TX — ADAS MGM → bridge_dspace. 10ms 주기.
# packet counter는 bridge가 송신 시점에 부여한다 (여기 포함하지 않음).
uint8 STATE_LANE=0
uint8 STATE_WAYPOINT=1
uint8 STATE_AVOID=2
uint8 STATE_PARKING=3

std_msgs/Header header
RefPoint[] ref_points   # vehicle frame, 모든 모드 동일 포맷 (dSPACE는 소스 구분 불필요)
float32 v_ref           # [m/s] 종방향 병합 최종 목표 속도. 정지 = 0
uint8 state             # 현재 주행 스테이트 (flags의 state 부분)
