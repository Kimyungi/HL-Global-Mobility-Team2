#!/usr/bin/env bash
# CAN 송수신 감시 — 1초마다 TX/RX 프레임 수를 보여준다.
#   TX = PC → dSPACE   /   RX = dSPACE → PC
# USB 재열거(인터페이스 재생성)도 감지한다 (HANDOVER §3.7).
#
#   사용: ~/FMA_ws/can_watch.sh          (기본 can0)
#         ~/FMA_ws/can_watch.sh can1
IF=${1:-can0}
S=/sys/class/net/$IF/statistics
pt=0; pr=0; pidx=""; first=1

printf "%-8s │ %8s %8s │ %10s %10s │ %s\n" "시각" "TX/s" "RX/s" "누적TX" "누적RX" "상태"
printf "─────────┼──────────────────┼───────────────────────┼──────────────────\n"

while true; do
  if [ ! -d "$S" ]; then
    printf "%-8s │ %8s %8s │ %10s %10s │ ⛔ %s 없음 (USB 빠짐?)\n" "$(date +%H:%M:%S)" - - - - "$IF"
    pidx=""; first=1
    sleep 1; continue
  fi
  idx=$(cat /sys/class/net/$IF/ifindex 2>/dev/null)
  t=$(cat $S/tx_packets 2>/dev/null || echo 0)
  r=$(cat $S/rx_packets 2>/dev/null || echo 0)
  st=$(ip -details link show $IF 2>/dev/null | grep -oP 'can state \K[A-Z-]+')
  err=$(ip -details link show $IF 2>/dev/null | grep -oP 'berr-counter tx \K[0-9]+')

  if [ -n "$pidx" ] && [ "$idx" != "$pidx" ]; then
    printf "⚠⚠⚠  %s 가 다시 만들어졌다 (ifindex %s→%s) — USB 재열거! 실행 중이던 노드가 죽었을 수 있다\n" "$IF" "$pidx" "$idx"
    pt=0; pr=0
  fi
  pidx=$idx

  if [ $first -eq 1 ]; then dt=0; dr=0; first=0; else dt=$((t-pt)); dr=$((r-pr)); fi
  [ $dt -lt 0 ] && dt=0; [ $dr -lt 0 ] && dr=0

  mark=""
  [ "$dt" -gt 0 ] && mark="${mark}▶송신 "
  [ "$dr" -gt 0 ] && mark="${mark}◀수신 "
  [ -z "$mark" ] && mark="— 조용함"

  printf "%-8s │ %8d %8d │ %10d %10d │ %s %s(TEC %s)\n" \
      "$(date +%H:%M:%S)" "$dt" "$dr" "$t" "$r" "$mark" "${st:-?}" "${err:-?}"
  pt=$t; pr=$r
  sleep 1
done
