# bridge_dspace → MGM · 운용자 — CAN 링크 건전성 (CLAUDE.md §5.7 ⑥)
#
# "정지 명령이 실제로 버스에 나가고 있는가" 하나에만 답한다. 판단은 하지 않는다 —
# 문턱 판정과 estop 보정은 MGM wrapper 가 한다.
#
# ⚠ 이 메시지는 **CAN 프레임을 하나도 유발하지 않는다** (ROS 토픽일 뿐).
#    PROTOCOL.md "브리지에 keep-alive 를 넣지 말 것" 을 지키기 위한 전제다 —
#    송신은 오직 /adas/target_ref 를 받았을 때만 일어난다.
std_msgs/Header header

# 소켓이 살아 있는가. 치명 errno 로 닫고 재오픈을 기다리는 중이면 false.
bool link_up

# 마지막 **송신 시도**의 성공 여부. 보낼 것이 없어 시도하지 않은 주기는 직전 값을
# 유지한다 — 송신 없음은 고장이 아니다(MGM 이 멈춘 것은 dSPACE counter watchdog 소관).
bool tx_ok

# 연속 송신 실패 주기 수 (성공하면 0). 문턱 판정은 구독자 몫.
uint32 consecutive_tx_fail

# 마지막 실패의 errno (0 = 없음).
#   치명: ENETDOWN(100)·ENODEV(19)·ENXIO(6)·EBADF(9) → 소켓 재오픈 대상
#   일시: ENOBUFS(105)·EAGAIN(11) → 버스가 밀리는 중, 재오픈하지 않는다
int32 last_errno

# 불건전이 시작된 뒤 흐른 시간 [s]. 건전하면 0.0.
# MGM 의 재인가 문턱(can_relatch_sec) 판정 입력 — 짧은 두절은 자동 복귀,
# 긴 고장은 사람이 다시 인가해야 재출발한다 (PROTOCOL.md:78 "래치 없음" 규정과
# 갈리는 유일한 지점 — INTEGRATION_TEST_0826.md §6 ① 참조).
float32 down_duration_s

# 마지막으로 수신한 CAN 에러 프레임의 종류 (CAN_RAW_ERR_FILTER). 0 = 없음.
# bus-off 는 드라이버가 restart-ms 100 으로 자동 복구하지만, **복구했다는 사실과
# 무관하게 그 구간의 송신은 실패했다** — 그것을 사람이 알게 하는 것이 목적이다.
uint32 last_err_class
bool bus_off
