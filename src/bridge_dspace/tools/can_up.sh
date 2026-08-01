#!/bin/sh
# CAN 인터페이스 자동 활성화 — udev가 호출 (70-can-auto.rules)
# 팀 표준: classic CAN, 1 Mbps (PROTOCOL.md / CAN_BRINGUP.md)
# 수동 실행도 가능:  sudo /usr/local/bin/can_up.sh can0
IFACE="${1:-can0}"
ip link set "$IFACE" down 2>/dev/null
ip link set "$IFACE" type can bitrate 1000000 restart-ms 100
ip link set "$IFACE" up
logger -t can_up "$IFACE up @ 1Mbps (restart-ms 100)"
