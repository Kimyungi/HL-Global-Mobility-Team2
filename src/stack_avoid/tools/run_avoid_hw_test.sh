#!/bin/bash
# 실물(LiDAR + CAN) 회피 테스트 + 로깅 실행 스크립트.
#   - 워크스페이스 소싱 → can0 활성화 → 타임스탬프 로그 디렉터리 → 런치(+rosbag)
#   - Ctrl+C 한 번으로 전체 정리, 콘솔·rosbag 로그 저장
#
# 사용:  bash src/stack_avoid/tools/run_avoid_hw_test.sh            # CAN+LiDAR+RViz+로그
#        CAN=false bash src/stack_avoid/tools/run_avoid_hw_test.sh  # CAN 없이 라이다만
# (주의: ROS setup.bash가 set -u와 비호환이라 set -u 사용 안 함)

source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
source "$HOME/ydlidar_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export DISPLAY="${DISPLAY:-:0}"

TS=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/avoid_logs/$TS"
mkdir -p "$LOG"
echo "════════════════════════════════════════════"
echo " 로그 디렉터리: $LOG"
echo "   - console.log : 전체 노드 콘솔"
echo "   - bag/        : rosbag (스캔·회피출력·CAN RX·TF)"
echo "════════════════════════════════════════════"

# ── 하드웨어 점검 ───────────────────────────────
if ! ls /dev/ttyUSB* >/dev/null 2>&1; then
  echo "⚠ 라이다(/dev/ttyUSB*) 미검출 — USB 연결 확인"
fi

WANT_CAN="${CAN:-true}"
if [ "$WANT_CAN" = "true" ]; then
  if ip -br link show can0 2>/dev/null | grep -q DOWN; then
    echo "can0 DOWN → 1Mbps 활성화 시도 (sudo)"
    sudo ip link set can0 up type can bitrate 1000000 restart-ms 100 \
      || echo "⚠ can0 활성화 실패 — 수동: sudo ip link set can0 up type can bitrate 1000000  (또는 udev 자동활성화 설치)"
  fi
  ip -br link show can0 2>/dev/null | grep -q "UP\|UNKNOWN" && echo "can0 UP ✓" || echo "⚠ can0 아직 DOWN"
fi

# ── 실행 (Ctrl+C 시 launch가 자식 전부 정리) ──────
echo "실행 시작 — 종료하려면 이 창에서 Ctrl+C"
ros2 launch stack_avoid avoid_hw_log.launch.py \
    bag_dir:="$LOG/bag" can:="$WANT_CAN" rviz:=true 2>&1 | tee "$LOG/console.log"

echo "종료됨. 로그: $LOG"
echo "재생:  ros2 bag play $LOG/bag"
echo "요약:  ros2 bag info $LOG/bag"
