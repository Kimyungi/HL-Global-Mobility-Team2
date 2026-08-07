#!/bin/bash
# 라이다 회피 통합 — LiDAR→회피점→dSPACE 실제 조향/구동 + RViz + 로깅.  ★★ 실차가 회피 기동한다 ★★
#   VREF=0.2 bash src/stack_avoid/tools/run_avoid_drive.sh
#   - 반드시 dummy_ref 구동계 테스트(run_drivetrain_test.sh)를 먼저 Ctrl+C로 종료할 것
#     (안 그러면 /adas/target_ref·CAN 이 충돌)
#   - 바퀴 들고(스탠드) 먼저 · 물리 비상정지 준비 · 주변 통제
# (ROS setup.bash 비호환 때문에 set -u 사용 안 함)

source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
source "$HOME/ydlidar_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export DISPLAY="${DISPLAY:-:0}"

VREF="${VREF:-0.2}"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/avoid_logs/avoiddrive_$TS"
mkdir -p "$LOG"

cat <<BANNER
════════════════════════════════════════════════════════════
 ★★ 라이다 회피 실주행 — 장애물 보면 실제로 조향/회피한다 ★★
   v_ref = $VREF m/s  (장애물 없으면 정지, 감지+통과가능하면 회피 주행)
   □ mgm_node 미실행 확인했는가?         (/adas/target_ref 이중 발행)
   □ dummy_ref 구동계 테스트 종료했는가? (충돌 방지)
   □ 바퀴 들고(스탠드) 먼저 확인했는가?
   □ 물리 비상정지 손 닿는 곳?  □ 주변/바닥 통제?
   로그: $LOG
════════════════════════════════════════════════════════════
BANNER
read -r -p "확인했으면 Enter, 취소 Ctrl+C: " _

if ! ls /dev/ttyUSB* >/dev/null 2>&1; then echo "⚠ 라이다 미검출"; fi
if ip -br link show can0 2>/dev/null | grep -q DOWN; then
  echo "can0 DOWN → 1Mbps 활성화 (sudo)"
  sudo ip link set can0 up type can bitrate 1000000 restart-ms 100 || { echo "⚠ can0 실패 — 중단"; exit 1; }
fi
ip -br link show can0 2>/dev/null | grep -qE "UP|UNKNOWN" && echo "can0 UP ✓" || { echo "⚠ can0 DOWN — 중단"; exit 1; }

echo "시작 — 정지 Ctrl+C (watchdog가 30ms 후 v_ref=0)"
ros2 launch stack_avoid avoid_drive_full.launch.py \
    drive:=true v_ref:="$VREF" bag_dir:="$LOG/bag" 2>&1 | tee "$LOG/console.log"

echo "종료. 로그: $LOG   (ros2 bag info $LOG/bag)"
