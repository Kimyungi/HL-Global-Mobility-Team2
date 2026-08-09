#!/bin/bash
# 실차 측정 세션 — stage2 실측 ①②③ + 경계시험 ⓐⓑⓒ.  이기돈
#   MODE=perception bash src/stack_avoid/tools/run_field_session.sh    # ③ 감지 (차 안 움직임)
#   MODE=step VREF=0.3 bash ...                                        # ① 조향응답 ★스탠드에서
#   MODE=step VREF=0.3 HOLD=6.0 bash ...                               # ② 측방이동 (지상)
#   MODE=step VREF=0.5 HOLD=6.0 bash ...                               # ② 측방이동 (지상, 고속)
#   MODE=avoid VREF=0.2 bash ...                                       # ⓐⓑⓒ 경계 시험
#
# 별도 터미널에서 구간 표시기를 띄울 것:  ros2 run stack_avoid mark
# (ROS setup.bash 비호환 때문에 set -u 사용 안 함)

source /opt/ros/humble/setup.bash
source "$HOME/FMA_ws/install/setup.bash"
source "$HOME/ydlidar_ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=0
export DISPLAY="${DISPLAY:-:0}"

MODE="${MODE:-perception}"
VREF="${VREF:-0.3}"
HOLD="${HOLD:-3.0}"
REPEATS="${REPEATS:-3}"
OFFSETS="${OFFSETS:-[0.46, -0.46, 0.30, -0.30]}"
DYNAMIC="${DYNAMIC:-true}"
ESTOP_ON="${ESTOP_ON:-0.70}"
TS=$(date +%Y%m%d_%H%M%S)
LOG="$HOME/avoid_logs/field_${MODE}_$TS"
mkdir -p "$LOG"

case "$MODE" in
  perception) PURPOSE="③ 감지 신뢰 거리 — 명령 없음, 차 안 움직임" ; MOVES=false ;;
  step)       PURPOSE="① 조향 응답 / ② 측방 이동 곡선 — 측방 스텝 주입" ; MOVES=true ;;
  avoid)      PURPOSE="ⓐⓑⓒ 경계 시험 — 회피 하네스 + estop" ; MOVES=true ;;
  *) echo "MODE는 perception|step|avoid 중 하나여야 함 (받은 값: $MODE)"; exit 1 ;;
esac

cat <<BANNER
════════════════════════════════════════════════════════════
 실차 측정 세션   mode=$MODE
   $PURPOSE
   v_ref=$VREF m/s · estop 정지거리=$ESTOP_ON m · 동적기준=$DYNAMIC
BANNER
if [ "$MODE" = "step" ]; then
cat <<BANNER
   스텝 $OFFSETS · 유지 ${HOLD}s · 반복 ${REPEATS}회
   ★ v_ref=0 이면 MPC 지평(0.2×v_ref)이 0이라 조향이 안 움직인다 — 반드시 >0
   ★ ① 은 스탠드(바퀴 듦)에서. ② 는 지상 직선 구간에서.
BANNER
fi
if [ "$MOVES" = "true" ]; then
cat <<BANNER
 ★★ 실차 조향/구동 ★★
   □ mgm_node 미실행 확인했는가?          (/adas/target_ref 이중 발행)
   □ dummy_ref 구동계 테스트 종료했는가?
   □ 찬미 estop launch 별도 실행 중 아닌가? (stack_estop_node 중복)
   □ 조이스틱 전원 ON?  ← 꺼져 있으면 액추에이션이 죽어 str이 고정된다 (8/6 사례)
   □ 물리 비상정지 손 닿는 곳?  □ 주변/바닥 통제?
   □ dSPACE watchdog 확인됐는가? — 미확인이면 ⓑ 보류 (손상민)
BANNER
fi
cat <<BANNER
   구간 표시기: 다른 터미널에서  ros2 run stack_avoid mark
   로그: $LOG
════════════════════════════════════════════════════════════
BANNER
read -r -p "확인했으면 Enter, 취소 Ctrl+C: " _

if ! ls /dev/ttyUSB* >/dev/null 2>&1; then echo "⚠ 라이다 미검출"; fi

# 이중 발행 방지 — 같은 토픽을 내는 노드가 이미 떠 있으면 중단.
# avoid_to_ref·step_injector 도 반드시 본다: 앞선 세션이나 단독 테스트의 잔여
# 프로세스가 남아 있는 일이 실제로 있었다(ros2 run 자식이 Ctrl+C로 안 죽는 경우).
# node list 실패 시 페일-클로즈 — 빈 검사 결과로 조용히 통과하지 않는다 (PR #27 리뷰).
if ! NODES=$(ros2 node list 2>/dev/null); then
  echo "⚠ ros2 node list 실패 — 그래프 상태를 확인할 수 없어 중단 (ros2 daemon stop 후 재시도)"
  exit 1
fi
if [ "$MOVES" = "true" ] && echo "$NODES" | grep -qE "mgm_node|dummy_ref_publisher|avoid_to_ref|step_injector"; then
  echo "⚠ /adas/target_ref 를 내는 노드가 이미 실행 중 — 이중 발행:"
  echo "$NODES" | grep -E "mgm_node|dummy_ref_publisher|avoid_to_ref|step_injector" | sed 's/^/    /'
  echo "  정리:  pkill -f 'avoid_to_ref|step_injector|dummy_ref_publisher'"
  exit 1
fi
if echo "$NODES" | grep -q "stack_estop_node"; then
  echo "⚠ stack_estop_node 이미 실행 중 — 이 launch가 또 띄운다. 종료 후 재시도."; exit 1
fi

if [ "$MOVES" = "true" ]; then
  if ip -br link show can0 2>/dev/null | grep -q DOWN; then
    echo "can0 DOWN → 1Mbps 활성화 (sudo)"
    sudo ip link set can0 up type can bitrate 1000000 restart-ms 100 || { echo "⚠ can0 실패 — 중단"; exit 1; }
  fi
  ip -br link show can0 2>/dev/null | grep -qE "UP|UNKNOWN" && echo "can0 UP ✓" \
    || { echo "⚠ can0 DOWN — 중단"; exit 1; }
fi

echo "시작 — 정지 Ctrl+C"
ros2 launch stack_avoid field_session.launch.py \
    mode:="$MODE" v_ref:="$VREF" hold_s:="$HOLD" repeats:="$REPEATS" \
    offsets:="$OFFSETS" dynamic:="$DYNAMIC" estop_on_distance_m:="$ESTOP_ON" \
    bag_dir:="$LOG/bag" 2>&1 | tee "$LOG/console.log"

echo "종료. 로그: $LOG"
echo "분석: python3 src/stack_avoid/tools/analyze_field_bag.py $LOG/bag"
