# stack_parking 출력 — 주차공간 인식 + 주차 경로 (vehicle frame)
# 정적 경계(콘·연석)는 로컬맵 입력일 뿐 정지 트리거가 아님 (CLAUDE.md §4)
std_msgs/Header header
bool space_found        # 주차공간 인식 → lane→parking 전이 조건의 절반
bool path_blocked       # 경로 침범(동적) → parking 스테이트에서 정지 근거
bool done               # 주차 완료 → parking→lane 복귀
RefPoint[] points       # 주차 경로
float32 v_suggest       # [m/s] 주차 진행 권장 속도 (후진 = 음수)

# Latest LiDAR SLAM pose delta. The translation is expressed in the previous
# vehicle frame and held between 10Hz localization samples for MGM's 100Hz
# TargetRef publication. First valid sample: zero delta with update=1.
float64 dx
float64 dy
float64 dyaw
uint64 update

# Existing mission state, exposed for MGM command acknowledgement. COMPLETE
# still supplies done; no new completion/failure judgment is made here.
bool mission_active
# Same values as GpsPath.PARKING_*: 0=none, 1=perpendicular/T, 2=parallel
uint8 mission_mode

# Actual sensor-input generation backing these points, not publication time.
# Held on republish; zero means unproven/no generated reference. MGM applies
# the existing provider timeout to this stamp, including transport delay.
# Exception: RC T-reference adapter uses planning time, recomputing from latest
# received pose without an acquisition-age gate; this is not sensor freshness.
builtin_interfaces/Time reference_stamp

# Current MGM request only. 0 denotes the legacy/manual interface.
uint64 request_id
bool wall_acquisition_complete
uint8 wall_acquisition_frames
bool search_active
# Existing detector + planner produced a space/plan; localization may still run.
bool search_space_found
# Existing pipeline PARKING + plan + fresh localization, before execution starts.
bool preparation_ready
# Actual accepted localization input backing preparation_ready (not heartbeat).
# RC T-reference adapter: planning time under the same exception as above.
builtin_interfaces/Time preparation_stamp
