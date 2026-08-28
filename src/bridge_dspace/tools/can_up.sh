#!/bin/sh
# CAN 인터페이스 활성화 — udev(70-can-auto.rules)가 어댑터 삽입 시 호출한다.
# 수동 실행도 가능:  sudo /usr/local/bin/can_up.sh can0
#
# ★ 값은 여기 없다. can_params.sh 가 단일 진실 원천이다 (PROTOCOL.md §공통).
#   설치본은 /usr/local/lib/fma-can/can_params.sh, 저장소에서는 can_setup/can_params.sh.
IFACE="${1:-can0}"

for P in /usr/local/lib/fma-can/can_params.sh \
         "$(dirname "$0")/can_setup/can_params.sh" \
         "$(dirname "$0")/can_params.sh"; do
  if [ -r "$P" ]; then . "$P"; PARAMS="$P"; break; fi
done
if [ -z "${PARAMS:-}" ]; then
  logger -t can_up "can_params.sh 를 못 찾음 — 설치 확인: sudo .../can_setup/install.sh"
  echo "can_params.sh 를 못 찾았다. sudo src/bridge_dspace/tools/can_setup/install.sh 실행할 것" >&2
  exit 1
fi

ip link set "$IFACE" down 2>/dev/null
# fd on = CAN FD 활성화 (MTU 16 → 72). PC 코드는 이 MTU 로 FD 가능 여부를 판정한다.
if ip link set "$IFACE" type can \
     bitrate "$CAN_NOM_BITRATE" sample-point "$CAN_NOM_SAMPLE_POINT" \
     dbitrate "$CAN_DATA_BITRATE" dsample-point "$CAN_DATA_SAMPLE_POINT" \
     fd on restart-ms "$CAN_RESTART_MS"
then
  ip link set "$IFACE" txqueuelen "$CAN_TXQUEUELEN" 2>/dev/null
  ip link set "$IFACE" up
  logger -t can_up "$IFACE up @ CAN FD ${CAN_NOM_BITRATE}/${CAN_DATA_BITRATE} (params: $PARAMS)"
else
  # 컨트롤러가 샘플포인트를 정확히 못 맞추면 여기로 온다 — 샘플포인트 없이 재시도.
  # 이 경로로 올라오면 dSPACE 와 샘플포인트가 어긋날 수 있으니 CAN_BRINGUP.md §1 확인.
  logger -t can_up "$IFACE: sample-point 지정 실패 — 기본 샘플포인트로 재시도"
  ip link set "$IFACE" type can bitrate "$CAN_NOM_BITRATE" \
     dbitrate "$CAN_DATA_BITRATE" fd on restart-ms "$CAN_RESTART_MS"
  ip link set "$IFACE" txqueuelen "$CAN_TXQUEUELEN" 2>/dev/null
  ip link set "$IFACE" up
  logger -t can_up "$IFACE up @ CAN FD ${CAN_NOM_BITRATE}/${CAN_DATA_BITRATE} (기본 샘플포인트)"
fi
