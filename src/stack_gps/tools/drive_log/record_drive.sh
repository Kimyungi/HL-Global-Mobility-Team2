#!/usr/bin/env bash
# 주행 로깅 — rosbag(전 토픽) + CAN 원시 프레임(candump) + 메타데이터를 한 폴더에 기록.
#
# 사용법:
#   ./record_drive.sh <주행이름> [웨이포인트CSV]
#   (주행 시작 전에 실행 → 주행 끝나면 Ctrl-C)
#
# 출력: ~/FMA_ws/drive_logs/<이름>_<타임스탬프>/
#   ├── bag/            rosbag2 (아래 토픽 전부)
#   ├── candump-*.log   CAN 원시 프레임 (can0 활성 시 — canplayer로 재생 가능)
#   ├── meta.txt        일시·git 해시·베이스 좌표·트랙 파일명
#   └── <트랙>.csv      사용한 웨이포인트 사본 (재현용)
#
# 분석 방법은 stack_gps/DRIVE_GUIDE.md §7 참조.
set -u

NAME=${1:-drive}
CSV=${2:-}
STAMP=$(date +%Y%m%d_%H%M%S)
DIR="$HOME/FMA_ws/drive_logs/${NAME}_${STAMP}"
mkdir -p "$DIR"

# ── 메타데이터 (문제 검토 시 "그때 코드/좌표가 뭐였나"를 답해주는 파일) ──
{
    echo "date: $(date -Is)"
    echo "host: $(hostname)"
    echo "git:  $(git -C "$HOME/FMA_ws" rev-parse --short HEAD 2>/dev/null || echo '?')"
    echo "track: ${CSV:-(미지정)}"
    echo "--- base coords (README 헤더) ---"
    head -10 "$HOME/FMA_ws/src/stack_gps/tools/base_station/README.md" 2>/dev/null | grep -E "lat|지점"
} > "$DIR/meta.txt"
[ -n "$CSV" ] && cp "$CSV" "$DIR/" 2>/dev/null

# ── CAN 원시 로깅 (can0 있으면) — 하위(dSPACE) 피드백 원본 그대로 남긴다 ──
CANPID=""
if ip link show can0 &>/dev/null; then
    if command -v candump &>/dev/null; then
        (cd "$DIR" && exec candump -l can0) &
        CANPID=$!
        echo "[log] candump 시작 (can0 → $DIR/candump-*.log)"
    else
        echo "[log] ⚠ can-utils 미설치 — sudo apt install can-utils (CAN 원시 로깅 생략)"
    fi
else
    echo "[log] can0 없음 — CAN 원시 로깅 생략 (벤치/GPS 단독 시 정상)"
fi

# ── rosbag — 판단·인지·하위 피드백 전 계층 ──
TOPICS=(
    /perception/gps_path        # stack_gps 출력 (vehicle frame ref) — MGM 입력
    /perception/gps_fix         # 로버 절대좌표 (NavSatFix) — 궤적 복원용
    /perception/gps_path_viz    # RViz 경로
    /perception/gps_track_viz   # 전체 트랙 (latched)
    /perception/estop           # 긴급정지 요구
    /perception/traffic_stop    # 신호등 정지 요구
    /perception/stopline        # 정지선
    /adas/target_ref            # MGM 최종 판단 (dSPACE로 나가는 내용)
    /vehicle/vector             # dSPACE 상태 추정 회신 (하위 피드백 — CAN RX)
    /scan                       # 라이다 (estop 입력)
    /tf /tf_static              # 좌표 변환 (RViz 재생용)
    /rosout                     # 전 노드 로그 (경고·오류 타임라인)
)
echo "[log] rosbag 기록 시작 → $DIR/bag"
echo "[log] 주행 끝나면 Ctrl-C 한 번 (bag 마감까지 몇 초 기다릴 것)"
ros2 bag record -o "$DIR/bag" "${TOPICS[@]}"

# ── 마감 ──
[ -n "$CANPID" ] && { kill -INT "$CANPID" 2>/dev/null; wait "$CANPID" 2>/dev/null; }
echo ""
echo "[log] 저장 완료: $DIR"
du -sh "$DIR" 2>/dev/null
echo "[log] 확인: ros2 bag info $DIR/bag"
