#!/usr/bin/env bash
# multi_lidar_fusion — 어느 by-id 포트가 어느 라이다인지 찾아내는 도구.
#
# 왜 필요한가:
#   YDLiDAR T-mini Plus 와 SLAMTEC RPLiDAR C1M1 은 둘 다 Silicon Labs CP210x 를 쓴다.
#   이 PC 의 /etc/udev/rules.d/99-ydlidar.rules 는 10c4:ea60 전체를 /dev/ydlidar 로
#   묶어버려서, 4대를 동시에 쓰면 어느 것이 어느 것인지 알 수 없다.
#   /dev/serial/by-id/ 는 칩 시리얼이 들어가 있어 개체마다 유일하다.
#
# 사용법:
#   ./identify_lidars.sh              # 후보 포트 목록만
#   ./identify_lidars.sh probe        # 한 포트씩 실제로 열어 어느 드라이버가 붙는지 확인

set -u

echo "=== /dev/serial/by-id 목록 ==="
if [ ! -d /dev/serial/by-id ]; then
  echo "  (없음) USB 시리얼 장치가 하나도 안 붙어 있다."
  exit 1
fi

for p in /dev/serial/by-id/*; do
  [ -e "$p" ] || continue
  tty=$(readlink -f "$p")
  vidpid=$(udevadm info -q property -n "$tty" 2>/dev/null |
    awk -F= '/^ID_VENDOR_ID=/{v=$2} /^ID_MODEL_ID=/{m=$2} END{print v":"m}')
  hint="?"
  case "$p" in
    *HandsFree_IMU*) hint="IMU (라이다 아님)" ;;
    *CP2102N*)       hint="CP2102N — RPLiDAR C1M1 유력" ;;
    *CP2102_*)       hint="CP2102  — YDLiDAR T-mini Plus 유력" ;;
    *FTDI*)          hint="FTDI    — 어댑터 종류 확인 필요" ;;
  esac
  printf '  %-100s -> %-12s [%s] %s\n' "$(basename "$p")" "$tty" "$vidpid" "$hint"
done

echo
echo "=== 현재 심볼릭 링크 ==="
for l in /dev/ydlidar /dev/rplidar; do
  [ -e "$l" ] && printf '  %-14s -> %s\n' "$l" "$(readlink -f "$l")"
done
echo "  ※ udev 규칙이 CP210x 전체를 한 이름으로 묶으므로 위 링크는 신뢰하지 말 것."

if [ "${1:-}" != "probe" ]; then
  echo
  echo "실제로 어느 포트가 어느 라이다인지 확인하려면: $0 probe"
  exit 0
fi

echo
echo "=== 포트별 실측 (한 대씩 띄워서 /scan 이 나오는지 본다) ==="
echo "각 포트에 대해 아래를 직접 실행해 보는 것이 가장 확실하다:"
cat <<'EOF'

  # YDLiDAR 후보
  ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
      -p port:=<BY_ID_PATH> -p baudrate:=230400 -p lidar_type:=0 \
      -p device_type:=0 -p isSingleChannel:=false -p frequency:=10.0

  # RPLiDAR C1M1 후보
  ros2 run rplidar_ros rplidar_node --ros-args \
      -p serial_port:=<BY_ID_PATH> -p serial_baudrate:=460800 \
      -p scan_mode:=Standard

  # 다른 터미널에서
  ros2 topic hz /scan
  ros2 topic echo /scan --field angle_increment --once

한 대만 동작하면 그 포트가 그 모델이다. 확인된 값을
launch/multi_lidar_drivers.launch.py 의 DEFAULT_PORTS 에 적어 둘 것.
EOF
