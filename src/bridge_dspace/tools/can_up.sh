#!/bin/sh
# CAN 인터페이스 자동 활성화 — udev가 호출 (70-can-auto.rules)
# 팀 표준: CAN FD, nominal 1 Mbps / data 2 Mbps (PROTOCOL.md / CAN_BRINGUP.md)
# 수동 실행도 가능:  sudo /usr/local/bin/can_up.sh can0
#
# ★ 아래 4개 값은 dSPACE 측(RTI CAN FD 블록셋)과 **반드시 같아야 한다**.
#   한쪽만 바꾸면 버스가 ERROR-PASSIVE/BUS-OFF 로 떨어진다. 변경 시 PROTOCOL.md
#   §공통의 FD 파라미터 표도 함께 갱신할 것.
#   - data 를 2 Mbps 로 잡은 이유: 배선·종단이 1 Mbps classic 으로만 검증돼 있고,
#     8바이트 페이로드에서는 그 이상 올려도 얻는 게 없다 (PROTOCOL.md 버스 부하).
IFACE="${1:-can0}"
NOM_BITRATE=1000000
NOM_SAMPLE_POINT=0.8
DATA_BITRATE=2000000
DATA_SAMPLE_POINT=0.8

ip link set "$IFACE" down 2>/dev/null
# fd on = CAN FD 활성화 (인터페이스 MTU 16 → 72). PC 코드는 이 MTU 로 FD 가능 여부를 판정한다.
if ip link set "$IFACE" type can \
     bitrate "$NOM_BITRATE" sample-point "$NOM_SAMPLE_POINT" \
     dbitrate "$DATA_BITRATE" dsample-point "$DATA_SAMPLE_POINT" \
     fd on restart-ms 100
then
  ip link set "$IFACE" up
  logger -t can_up "$IFACE up @ CAN FD ${NOM_BITRATE}/${DATA_BITRATE} (restart-ms 100)"
else
  # sample-point 를 컨트롤러가 정확히 못 맞추면 여기로 온다 — 샘플포인트 없이 재시도.
  # 이 경로로 올라오면 dSPACE 와 샘플포인트가 어긋날 수 있으니 CAN_BRINGUP.md §1 확인.
  logger -t can_up "$IFACE: sample-point 지정 실패 — 기본 샘플포인트로 재시도"
  ip link set "$IFACE" type can \
     bitrate "$NOM_BITRATE" dbitrate "$DATA_BITRATE" fd on restart-ms 100
  ip link set "$IFACE" up
  logger -t can_up "$IFACE up @ CAN FD ${NOM_BITRATE}/${DATA_BITRATE} (기본 샘플포인트)"
fi
