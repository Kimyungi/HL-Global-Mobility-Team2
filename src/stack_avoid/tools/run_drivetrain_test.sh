#!/bin/bash
# 구동계 검증 (dummy ref → CAN → dSPACE) + 로깅.  ★★ 실차가 움직인다 ★★
#   VREF=0.2 CURV=0.0 bash src/stack_avoid/tools/run_drivetrain_test.sh
#   - 바퀴 들고(스탠드) 먼저 · 물리 비상정지 준비 · 주변 통제
#   - Ctrl+C 종료 → 30ms 후 dSPACE watchdog가 v_ref=0 (안전 정지)
# (주의: ROS setup.bash가 set -u와 비호환이라 set -u 사용 안 함)

source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
export ROS_DOMAIN_ID=0

VREF="${VREF:-0.2}"
CURV="${CURV:-0.0}"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/avoid_logs/drivetrain_$TS"
mkdir -p "$LOG"

cat <<BANNER
════════════════════════════════════════════════════════════
 ★★ 실차 구동계 검증 — 차가 실제로 움직입니다 ★★
   v_ref = $VREF m/s   curvature = $CURV
   □ 바퀴 들고(스탠드) 먼저 확인했는가?
   □ 물리 비상정지 손 닿는 곳에 있는가?
   □ 주변/바닥 통제됐는가?
   로그: $LOG   (console.log + bag/)
════════════════════════════════════════════════════════════
BANNER
read -r -p "위 확인했으면 Enter, 취소는 Ctrl+C: " _

# CAN 활성화
if ip -br link show can0 2>/dev/null | grep -q DOWN; then
  echo "can0 DOWN → 1Mbps 활성화 (sudo)"
  sudo ip link set can0 up type can bitrate 1000000 restart-ms 100 \
    || { echo "⚠ can0 활성화 실패 — 중단"; exit 1; }
fi
ip -br link show can0 2>/dev/null | grep -qE "UP|UNKNOWN" && echo "can0 UP ✓" || { echo "⚠ can0 DOWN — 중단"; exit 1; }

echo "시작 — 정지하려면 Ctrl+C (watchdog가 30ms 후 v_ref=0)"
ros2 launch stack_avoid drivetrain_test.launch.py \
    v_ref:="$VREF" curvature:="$CURV" bag_dir:="$LOG/bag" 2>&1 | tee "$LOG/console.log"

echo "종료. 로그: $LOG"
echo "  회신 확인:  ros2 bag play $LOG/bag  /  ros2 bag info $LOG/bag"
