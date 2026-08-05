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

# systemd-networkd가 활성이면 udev·서비스가 올린 설정을 networkd가 마지막에
# 덮어쓴다 (실사례 2026-08-03: 잔재 80-can0.network의 500k가 1 Mbps를 뒤집어
# ERROR-PASSIVE 유발). 같은 이름으로 팀 표준 파일을 설치해 그 경로도 1 Mbps로.
if systemctl is-active --quiet systemd-networkd; then
  conflict=$(grep -rlZ "Name=can0" /etc/systemd/network/ 2>/dev/null \
             | tr '\0' '\n' | grep -v "80-can0.network" || true)
  [[ -n "$conflict" ]] && echo "⚠ can0을 건드리는 다른 networkd 설정 발견 — 확인 필요: $conflict"
  install -m 644 80-can0.network /etc/systemd/network/
  networkctl reload 2>/dev/null || true
  echo "✔ networkd 80-can0.network 설치 (1 Mbps — networkd 덮어쓰기 경로 차단)"
fi

# 이미 꽂혀 있으면 즉시 적용 (networkd 관리 중이면 새 설정으로 재구성부터)
if [[ -d /sys/class/net/can0 ]]; then
  networkctl reconfigure can0 2>/dev/null || true
  systemctl start can-iface@can0.service
  echo "✔ can0 지금 up: $(ip -br link show can0)"
  ip -details link show can0 | grep -o "bitrate [0-9]*"
fi

if [[ "${1:-}" == "--vcan" ]]; then
  install -m 644 vcan0.service /etc/systemd/system/
  systemctl daemon-reload
  systemctl enable --now vcan0.service
  echo "✔ vcan0 상시 생성: $(ip -br link show vcan0)"
fi
