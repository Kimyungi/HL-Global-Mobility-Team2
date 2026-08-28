#!/usr/bin/env bash
# FMA CAN 셋업 — 새 PC 에서 이거 하나만 돌리면 CAN 통신이 준비된다.
#
#   sudo ./install.sh            # 실차 PC
#   sudo ./install.sh --vcan     # + vcan0 (dSPACE 없이 루프백 시험하는 개발 머신)
#   ./install.sh --check         # 설치 상태 점검 (sudo 불필요, 아무것도 안 바꿈)
#
# 설치 후에는 어댑터를 **뺐다 꽂아도** can0 가 CAN FD 로 자동으로 올라온다.
#
# 제거: sudo systemctl disable --now can-iface@can0 vcan0 2>/dev/null
#       sudo rm -f /etc/systemd/system/{can-iface@.service,vcan0.service} \
#                  /etc/udev/rules.d/70-can-auto.rules /usr/local/bin/can_up.sh
#       sudo rm -rf /usr/local/lib/fma-can
set -euo pipefail
cd "$(dirname "$0")"

. ./can_params.sh
NOM_SP_PCT="$(awk "BEGIN{printf \"%d%%\", $CAN_NOM_SAMPLE_POINT*100}")"
DATA_SP_PCT="$(awk "BEGIN{printf \"%d%%\", $CAN_DATA_SAMPLE_POINT*100}")"

# 템플릿 → 실제 파일. 값이 한 곳(can_params.sh)에서만 오므로 세 경로가 어긋날 수 없다.
render() {
  sed -e "s|@NOM_BITRATE@|$CAN_NOM_BITRATE|g" \
      -e "s|@NOM_SAMPLE_POINT@|$CAN_NOM_SAMPLE_POINT|g" \
      -e "s|@DATA_BITRATE@|$CAN_DATA_BITRATE|g" \
      -e "s|@DATA_SAMPLE_POINT@|$CAN_DATA_SAMPLE_POINT|g" \
      -e "s|@NOM_SAMPLE_POINT_PCT@|$NOM_SP_PCT|g" \
      -e "s|@DATA_SAMPLE_POINT_PCT@|$DATA_SP_PCT|g" \
      -e "s|@RESTART_MS@|$CAN_RESTART_MS|g" \
      -e "s|@TXQUEUELEN@|$CAN_TXQUEUELEN|g" "$1"
}

# ──────────────────────────────────────────────────────────── --check
if [[ "${1:-}" == "--check" ]]; then
  fail=0
  say() { printf '  %s %s\n' "$1" "$2"; }
  chk() { if [[ -e "$2" ]]; then say "✔" "$3"; else say "✘" "$3 — 없음"; fail=1; fi; }

  echo "── 설치 파일"
  chk f /usr/local/bin/can_up.sh              "/usr/local/bin/can_up.sh"
  chk f /usr/local/lib/fma-can/can_params.sh  "/usr/local/lib/fma-can/can_params.sh"
  chk f /etc/udev/rules.d/70-can-auto.rules   "udev 규칙"
  chk f /etc/systemd/system/can-iface@.service "can-iface@.service"

  echo "── 값 일치 (★ 드리프트 검사 — 예전 사고의 원인)"
  # 설치본이 저장소의 현재 값으로 만들어진 것과 같은가
  for pair in \
    "/usr/local/bin/can_up.sh:can_up.sh:../can_up.sh" \
    "/usr/local/lib/fma-can/can_params.sh:can_params.sh:can_params.sh"; do
    IFS=: read -r inst name src <<<"$pair"
    if [[ -e "$inst" ]] && diff -q "$inst" "$src" >/dev/null 2>&1; then
      say "✔" "$name 저장소와 동일"
    else
      say "✘" "$name ★저장소와 다름 — sudo ./install.sh 재실행할 것"; fail=1
    fi
  done
  for pair in "/etc/systemd/system/can-iface@.service:can-iface@.service.in" \
              "/etc/systemd/network/80-can0.network:80-can0.network.in"; do
    IFS=: read -r inst tpl <<<"$pair"
    if [[ ! -e "$inst" ]]; then
      say "–" "$(basename "$inst") 미설치 (networkd 비활성이면 정상)"
    elif diff -q "$inst" <(render "$tpl") >/dev/null 2>&1; then
      say "✔" "$(basename "$inst") 값 일치"
    else
      say "✘" "$(basename "$inst") ★값이 다름 — sudo ./install.sh 재실행할 것"; fail=1
    fi
  done

  echo "── 다른 networkd 설정이 can0 를 건드리는가"
  other=$(grep -rlZ "Name=can0" /etc/systemd/network/ 2>/dev/null | tr '\0' '\n' \
          | grep -v "80-can0.network" || true)
  if [[ -n "$other" ]]; then say "✘" "★충돌: $other"; fail=1; else say "✔" "충돌 없음"; fi

  echo "── 현재 can0 상태"
  if [[ -d /sys/class/net/can0 ]]; then
    mtu=$(cat /sys/class/net/can0/mtu)
    [[ "$mtu" -ge 72 ]] && say "✔" "MTU $mtu = CAN FD" || { say "✘" "MTU $mtu = classic 전용"; fail=1; }
    ip -details link show can0 | grep -oE "state ERROR[A-Z-]*|bitrate [0-9]+|dbitrate [0-9]+" \
      | sed 's/^/    /'
  else
    say "–" "can0 없음 (어댑터 미연결이면 정상)"
  fi

  echo
  [[ $fail -eq 0 ]] && echo "✔ 점검 통과 — 어댑터를 뺐다 꽂아도 자동으로 올라온다" \
                    || echo "✘ 문제 있음 — 위 ✘ 항목 확인 후 sudo ./install.sh"
  exit $fail
fi

# ──────────────────────────────────────────────────────────── 설치
if [[ $EUID -ne 0 ]]; then
  echo "sudo로 실행할 것: sudo $0 $*" >&2
  exit 1
fi

# ① 값 + 활성화 스크립트 (udev 가 부른다)
install -d /usr/local/lib/fma-can
install -m 644 can_params.sh /usr/local/lib/fma-can/can_params.sh
install -m 755 ../can_up.sh /usr/local/bin/can_up.sh
echo "✔ can_up.sh + can_params.sh 설치"

# ② udev — 어댑터 삽입 시 자동 활성화 (핫플러그의 주 경로)
install -m 644 ../70-can-auto.rules /etc/udev/rules.d/70-can-auto.rules
udevadm control --reload
echo "✔ udev 규칙 설치 — 어댑터를 꽂으면 자동으로 CAN FD up"

# ③ systemd 유닛 — 부팅 시·장치 유닛 기준 (udev 가 안 걸리는 경우 대비)
render can-iface@.service.in > /etc/systemd/system/can-iface@.service
systemctl daemon-reload
systemctl enable can-iface@can0.service >/dev/null
echo "✔ can-iface@can0 설치"

# ④ networkd — 활성이면 이 경로가 ①②를 **마지막에 덮어쓴다**. 반드시 같은 값으로.
if systemctl is-active --quiet systemd-networkd; then
  conflict=$(grep -rlZ "Name=can0" /etc/systemd/network/ 2>/dev/null \
             | tr '\0' '\n' | grep -v "80-can0.network" || true)
  [[ -n "$conflict" ]] && echo "⚠ can0을 건드리는 다른 networkd 설정 발견 — 확인 필요: $conflict"
  render 80-can0.network.in > /etc/systemd/network/80-can0.network
  networkctl reload 2>/dev/null || true
  echo "✔ networkd 80-can0.network 설치"
else
  echo "· systemd-networkd 비활성 — networkd 경로 건너뜀"
fi

# ⑤ 이미 꽂혀 있으면 즉시 적용
if [[ -d /sys/class/net/can0 ]]; then
  networkctl reconfigure can0 2>/dev/null || true
  systemctl restart can-iface@can0.service || /usr/local/bin/can_up.sh can0
  sleep 1
  mtu=$(cat /sys/class/net/can0/mtu)
  if [[ "$mtu" -ge 72 ]]; then
    echo "✔ can0 up — MTU $mtu = CAN FD"
  else
    echo "⚠ can0 MTU $mtu = classic 전용 — FD 설정이 안 먹었다. CAN_BRINGUP.md §1 참조"
  fi
  ip -details link show can0 |
    grep -oE "bitrate [0-9]+|dbitrate [0-9]+|sample-point [0-9.]+" | paste -sd" " || true
fi

# ⑥ vcan0 (개발 머신) — dSPACE 없이 루프백 시험용. MTU 72 여야 FD 프레임이 실린다.
if [[ "${1:-}" == "--vcan" ]]; then
  install -m 644 vcan0.service /etc/systemd/system/
  systemctl daemon-reload
  systemctl enable --now vcan0.service
  echo "✔ vcan0: $(ip -br link show vcan0) MTU $(cat /sys/class/net/vcan0/mtu)"
  [[ $(cat /sys/class/net/vcan0/mtu) -ge 72 ]] ||
    echo "⚠ vcan0 MTU가 72가 아니다 — 루프백 FD 시험이 기동에서 실패한다"
fi

echo
echo "설치 완료. 확인:  $PWD/install.sh --check"
