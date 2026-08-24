#!/usr/bin/env bash
# GPS 단독 주행 — 상시 3종을 한 터미널에서 (V1 RTCM중계 + V3 브리지 + V4 MGM)
# 종료(Ctrl-C) 시 can_zero 로 dSPACE 목표값 0 복귀까지 보장한다.
#   ⚠ bridge.launch.py 단독에는 이 가드가 없다 — dSPACE watchdog 미구현(HANDOVER §3.6)
set -u
source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"

LOG="$HOME/FMA_ws/drive_logs/session_$(date +%m%d_%H%M)"
mkdir -p "$LOG"
echo "로그: $LOG"

cleanup() {
  trap - EXIT INT TERM
  echo ""
  echo "[가드] 브리지 종료 대기..."
  kill "$MGM" 2>/dev/null
  kill "$BRIDGE" 2>/dev/null
  wait "$BRIDGE" 2>/dev/null
  echo "[가드] dSPACE 목표값 0 복귀 송신..."
  ros2 run stack_avoid can_zero --once
  kill "$RTCM" 2>/dev/null
  echo "[가드] 완료 — 안전 종료"
  exit 0
}
trap cleanup EXIT INT TERM

python3 -u "$HOME/FMA_ws/src/stack_gps/tools/base_station/rtcm_server.py" \
    --port /dev/ttyRadio --tcp-port 2101 > "$LOG/rtcm.log" 2>&1 &
RTCM=$!
ros2 launch bridge_dspace bridge.launch.py > "$LOG/bridge.log" 2>&1 &
BRIDGE=$!
ros2 launch adas_mgm mgm.launch.py > "$LOG/mgm.log" 2>&1 &
MGM=$!

echo "상시 3종 기동 — RTCM($RTCM) 브리지($BRIDGE) MGM($MGM)"
echo "Ctrl-C 로 안전 종료 (목표값 0 복귀 포함)"
echo "----- RTCM 수신 상태 (10초마다) -----"
tail -f "$LOG/rtcm.log"
