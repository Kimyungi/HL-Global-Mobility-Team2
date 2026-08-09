#!/bin/bash
# 회피↔긴급정지 경계 시험 (ⓐⓑⓒ) — stack_avoid + stack_estop 합동.  이기돈 · 박찬미
#   DRIVE=false bash src/stack_avoid/tools/run_avoid_estop_joint.sh   # ← 먼저 (차 안 움직임)
#   DRIVE=true VREF=0.2 bash src/stack_avoid/tools/run_avoid_estop_joint.sh
#
#   ⓐ 3m 전방 콘 회피 중 estop 미발동   ⓑ 1m 급투입 시 estop 발동   ⓒ 연석 접근 시 avoid 미진입+정지
#
# ★★ DRIVE=true 에서는 clear 구간에 차가 계속 전진한다 (straight_when_clear=true).
#    ⓐ가 "접근하다 비켜 간다"를 보는 시험이라 그래야 성립하지만, 기존
#    run_avoid_drive.sh(장애물 없으면 정지)보다 위험하다. 반드시 통제된 공간에서.
# (ROS setup.bash 비호환 때문에 set -u 사용 안 함)

source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
source "$HOME/ydlidar_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export DISPLAY="${DISPLAY:-:0}"

DRIVE="${DRIVE:-false}"
VREF="${VREF:-0.2}"
DYNAMIC="${DYNAMIC:-true}"
ESTOP_ON="${ESTOP_ON:-0.70}"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/avoid_logs/joint_$TS"
mkdir -p "$LOG"

cat <<BANNER
════════════════════════════════════════════════════════════
 회피↔긴급정지 경계 시험 (ⓐⓑⓒ)   drive=$DRIVE
   v_ref=$VREF m/s · estop 정지거리=$ESTOP_ON m · 동적기준=$DYNAMIC
BANNER
if [ "$DRIVE" = "true" ]; then
cat <<BANNER
 ★★ 실차 조향/구동 — 장애물 없으면 계속 전진한다 ★★
   □ mgm_node 미실행 확인했는가?          (/adas/target_ref 이중 발행)
   □ dummy_ref 구동계 테스트 종료했는가?  (/adas/target_ref·CAN 충돌)
   □ 찬미 estop launch 별도 실행 중 아닌가? (stack_estop_node 중복)
   □ 바퀴 들고(스탠드) 먼저 확인했는가?
   □ 물리 비상정지 손 닿는 곳?  □ 주변/바닥 통제?
   □ dSPACE watchdog 확인됐는가? — CAN 끊김 시 정지가 미확인 상태면 ⓑ 보류 (손상민)
BANNER
fi
cat <<BANNER
   로그: $LOG
════════════════════════════════════════════════════════════
BANNER
read -r -p "확인했으면 Enter, 취소 Ctrl+C: " _

if ! ls /dev/ttyUSB* >/dev/null 2>&1; then echo "⚠ 라이다 미검출"; fi

# 이중 발행 방지 — 같은 토픽을 내는 노드가 이미 떠 있으면 중단 (PR #27 리뷰 반영:
# ① avoid_to_ref·step_injector 포함 — 이 launch 가 avoid_to_ref 를 띄우므로 좀비 잔존
#   시 field_session 과 동일한 이중 발행 위험. ② node list 실패 시 페일-클로즈).
if ! NODES=$(ros2 node list 2>/dev/null); then
  echo "⚠ ros2 node list 실패 — 그래프 상태를 확인할 수 없어 중단 (ros2 daemon stop 후 재시도)"
  exit 1
fi
if echo "$NODES" | grep -qE "mgm_node|dummy_ref_publisher|avoid_to_ref|step_injector"; then
  echo "⚠ /adas/target_ref 를 내는 노드가 이미 실행 중 — 이중 발행:"
  echo "$NODES" | grep -E "mgm_node|dummy_ref_publisher|avoid_to_ref|step_injector" | sed 's/^/    /'
  echo "  정리:  pkill -f 'avoid_to_ref|step_injector|dummy_ref_publisher|mgm_node'"
  exit 1
fi
if echo "$NODES" | grep -q "stack_estop_node"; then
  echo "⚠ stack_estop_node 이미 실행 중 — 이 launch가 또 띄운다. 종료 후 재시도."; exit 1
fi

if [ "$DRIVE" = "true" ]; then
  if ip -br link show can0 2>/dev/null | grep -q DOWN; then
    echo "can0 DOWN → 1Mbps 활성화 (sudo)"
    sudo ip link set can0 up type can bitrate 1000000 restart-ms 100 || { echo "⚠ can0 실패 — 중단"; exit 1; }
  fi
  ip -br link show can0 2>/dev/null | grep -qE "UP|UNKNOWN" && echo "can0 UP ✓" || { echo "⚠ can0 DOWN — 중단"; exit 1; }
fi

echo "시작 — 정지 Ctrl+C"
ros2 launch stack_avoid avoid_estop_joint.launch.py \
    drive:="$DRIVE" v_ref:="$VREF" dynamic:="$DYNAMIC" \
    estop_on_distance_m:="$ESTOP_ON" bag_dir:="$LOG/bag" 2>&1 | tee "$LOG/console.log"

echo "종료. 로그: $LOG   (ros2 bag info $LOG/bag)"
echo "판정: console.log 의 'v_ref=0 ←' 사유 + /perception/static_estop·/perception/dynamic_estop"
