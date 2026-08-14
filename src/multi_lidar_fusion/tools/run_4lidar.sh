#!/usr/bin/env bash
# 라이다 4대 융합을 **명령 하나로** 띄운다.
#
# 이 파일의 역할:
#   드라이버 launch + 융합 launch + RViz 를 순서대로 띄우고, 종료할 때 안전하게 내린다.
#   터미널을 여러 개 열 필요도, 워크스페이스를 source 할 필요도 없다.
#
#     ~/FMA_ws/src/multi_lidar_fusion/tools/run_4lidar.sh
#
# 인자:
#   --no-rviz     RViz 없이 (rosbag 기록·원격 접속용)
#   --build       띄우기 전에 colcon build
#   --a1 <포트>   앞 YD 포트 수동 지정 (자동 탐지 실패 시)
#   --a2 <포트>   뒤 YD 포트 수동 지정
#
# ★ YD 2대는 **by-path** 로 잡는다. CP210x 시리얼이 둘 다 `0001` 이라 by-id 가 겹쳐
#   나중에 붙은 쪽이 먼저 것의 링크를 덮어쓴다. 그런데 by-path 는 "허브의 그 구멍"이
#   주소라, USB 를 다른 포트에 옮기거나 허브 전원을 껐다 켜면 **조용히 바뀐다**
#   (2026-08-14: 0:1.2.x -> 0:3.x 로 바뀌어 YD 2대가 무발행이었다).
#   그래서 이 스크립트는 매번 **자동 탐지**하고 무엇을 골랐는지 찍는다.
#   RPLiDAR 2대는 시리얼이 고유해 by-id 로 충분하므로 launch 기본값을 그대로 쓴다.
#
# ★ 종료는 반드시 SIGINT(Ctrl-C) 로. SIGKILL 로 죽이면 드라이버가 라이다에 정지
#   명령을 못 보내고, RPLiDAR 가 모터·스캔 상태를 물고 있어 다음 기동이
#   `SL_RESULT_OPERATION_TIMEOUT` / `Can not start scan` 으로 실패한다(실측).
#   이 스크립트의 trap 이 그 순서를 지킨다.
set -u

RVIZ=true
BUILD=false
A1_PORT=""
A2_PORT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --no-rviz) RVIZ=false; shift ;;
    --build)   BUILD=true; shift ;;
    --a1)      A1_PORT="${2:-}"; shift 2 ;;
    --a2)      A2_PORT="${2:-}"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    *) echo "모르는 인자: $1"; exit 2 ;;
  esac
done

# 워크스페이스 루트 = 이 스크립트에서 세 단계 위 (src/multi_lidar_fusion/tools/)
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
cd "$WS" || exit 1

# ★ ROS 의 setup.bash 는 미설정 변수를 참조하므로 `set -u` 를 켠 채로 source 하면
#   "AMENT_TRACE_SETUP_FILES: 바인딩 해제한 변수" 로 즉시 죽는다. source 구간만 해제한다.
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
set -u
if [ "$BUILD" = true ]; then
  echo "== 빌드 =="
  colcon build --packages-select multi_lidar_fusion stack_parking || exit 1
fi
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "! install/setup.bash 가 없습니다. --build 로 한 번 빌드하세요."
  exit 1
fi
set +u
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

# ── YD 2대 포트 자동 탐지 ──────────────────────────────────────────────
# CP2102(구형, 비-N)가 T-mini Plus 다. RPLiDAR·IMU 는 CP2102N(신형)이라 갈린다.
if [ -z "$A1_PORT" ] || [ -z "$A2_PORT" ]; then
  mapfile -t YD < <(
    for d in /sys/bus/usb/devices/*/; do
      p=$(cat "$d/product" 2>/dev/null) || continue
      [ "$p" = "CP2102 USB to UART Bridge Controller" ] || continue
      # sysfs 구조는 <장치>/<인터페이스>/ttyUSBn 이다 (중간에 /tty/ 단계는 없다).
      for t in "$d"*/ttyUSB*; do
        [ -e "$t" ] || continue
        tty=$(basename "$t")
        for bp in /dev/serial/by-path/*; do
          [ "$(readlink -f "$bp")" = "/dev/$tty" ] && echo "$bp"
        done
      done
    done | sort
  )
  if [ "${#YD[@]}" -ne 2 ]; then
    echo "! YDLiDAR(CP2102) 를 2대 찾지 못했습니다 (찾은 수: ${#YD[@]})."
    echo "  USB 연결과 허브 전원을 확인하거나 --a1 / --a2 로 직접 지정하세요:"
    ls -1 /dev/serial/by-path/ 2>/dev/null | sed 's/^/    /'
    exit 1
  fi
  # 허브 포트 번호가 큰 쪽이 앞(a1). 2026-08-13/14 배선 기준이며, 케이블을 옮겨
  # 꽂았다면 view_one_lidar 로 다시 확인해야 한다.
  A2_PORT="${A2_PORT:-${YD[0]}}"
  A1_PORT="${A1_PORT:-${YD[1]}}"
fi
echo "== 포트 =="
echo "   a1 앞 : $A1_PORT"
echo "   a2 뒤 : $A2_PORT"
echo "   (b1/b2 RPLiDAR 는 by-id 고정이라 launch 기본값을 쓴다)"
echo "   ★ 앞/뒤가 바뀐 것 같으면: ros2 launch multi_lidar_fusion view_one_lidar.launch.py unit:=yd0"

PIDS=()
cleanup() {
  echo
  echo "== 종료 (SIGINT 로 순서대로) =="
  for pid in "${PIDS[@]}"; do kill -INT "$pid" 2>/dev/null; done
  sleep 4                      # 드라이버가 라이다에 정지 명령을 보낼 시간
  for pid in "${PIDS[@]}"; do kill -9 "$pid" 2>/dev/null; done
  wait 2>/dev/null
  echo "   내렸습니다."
}
trap cleanup INT TERM EXIT

echo
echo "== 드라이버 4대 =="
ros2 launch multi_lidar_fusion multi_lidar_drivers.launch.py \
  a1_port:="$A1_PORT" a2_port:="$A2_PORT" &
PIDS+=("$!")

# 4대가 다 올라올 때까지 기다린다 (RPLiDAR 는 기동에 2~3초 걸린다)
echo "   스캔 수신 대기..."
for _ in $(seq 1 20); do
  sleep 1
  ok=0
  for t in a1 a2 b1 b2; do
    timeout 1 ros2 topic echo "/lidar/$t/scan" --once >/dev/null 2>&1 && ok=$((ok + 1))
  done
  echo "   $ok/4"
  [ "$ok" -eq 4 ] && break
done
if [ "${ok:-0}" -lt 4 ]; then
  echo "! 4대가 다 올라오지 않았습니다. 허브 전원(외장 허브는 반드시 전원 연결)과"
  echo "  포트를 확인하세요. 그대로 진행합니다 — 융합 노드는 살아 있는 센서만 씁니다."
fi

echo
echo "== 융합 + RViz =="
ros2 launch multi_lidar_fusion multi_lidar_fusion.launch.py rviz:="$RVIZ" &
PIDS+=("$!")

echo
echo "   /lidar/merged_scan  ← 회피 로직이 구독할 토픽"
echo "   Ctrl-C 로 전부 종료"
wait
