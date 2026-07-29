#!/usr/bin/env bash
# CAN 자동 셋업 설치 — 이후로는 PCAN을 꽂기만 하면 can0이 1 Mbps로 자동 up.
#
#   sudo ./install.sh          # can0 자동 셋업만
#   sudo ./install.sh --vcan   # + vcan0 상시 생성 (루프백 테스트용 개발 머신)
#
# 제거: sudo systemctl disable --now can-iface@can0 vcan0 2>/dev/null;
#       sudo rm /etc/systemd/system/{can-iface@.service,vcan0.service}
set -euo pipefail
cd "$(dirname "$0")"

if [[ $EUID -ne 0 ]]; then
  echo "sudo로 실행할 것: sudo $0 $*" >&2
  exit 1
fi

install -m 644 can-iface@.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable can-iface@can0.service
echo "✔ can-iface@can0 설치 — PCAN을 꽂으면 자동으로 1 Mbps up"

# 이미 꽂혀 있으면 즉시 적용
if [[ -d /sys/class/net/can0 ]]; then
  systemctl start can-iface@can0.service
  echo "✔ can0 지금 up: $(ip -br link show can0)"
fi

if [[ "${1:-}" == "--vcan" ]]; then
  install -m 644 vcan0.service /etc/systemd/system/
  systemctl daemon-reload
  systemctl enable --now vcan0.service
  echo "✔ vcan0 상시 생성: $(ip -br link show vcan0)"
fi
