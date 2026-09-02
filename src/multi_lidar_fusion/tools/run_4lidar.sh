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
#   --a1 <포트>   앞 YD 포트 수동 지정
#   --a2 <포트>   뒤 YD 포트 수동 지정
#   --b1 <포트>   좌 YD 포트 수동 지정
#   --b2 <포트>   우 YD 포트 수동 지정
#
# ★ 네 기본 포트는 실기에서 확인한 udev 위치 링크로 고정한다.
#   케이블은 다른 USB 허브 포트로 옮기지 않는다.
#
# ★ 종료는 반드시 SIGINT(Ctrl-C) 로. SIGKILL 로 죽이면 드라이버가 라이다에 정지
#   명령을 못 보내므로 다음 기동 때 포트를 정상적으로 열지 못할 수 있다.
#   이 스크립트의 trap 이 그 순서를 지킨다.
set -u

RVIZ=true
BUILD=false
A1_PORT=""
A2_PORT=""
B1_PORT=""
B2_PORT=""
while [ $# -gt 0 ]; do
  case "$1" in
    --no-rviz) RVIZ=false; shift ;;
    --build)   BUILD=true; shift ;;
    --a1)      A1_PORT="${2:-}"; shift 2 ;;
    --a2)      A2_PORT="${2:-}"; shift 2 ;;
    --b1)      B1_PORT="${2:-}"; shift 2 ;;
    --b2)      B2_PORT="${2:-}"; shift 2 ;;
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

# 실기 확인 위치 링크를 네 대 모두 기본값으로 고정한다.
A1_PORT="${A1_PORT:-/dev/lidar_front}"
A2_PORT="${A2_PORT:-/dev/lidar_rear}"
B1_PORT="${B1_PORT:-/dev/lidar_left}"
B2_PORT="${B2_PORT:-/dev/lidar_right}"

for port in "$A1_PORT" "$A2_PORT" "$B1_PORT" "$B2_PORT"; do
  if [ ! -e "$port" ]; then
    echo "! 라이다 포트가 없습니다: $port"
    echo "  tools/99-fma-lidars.rules 설치와 USB 연결을 확인하세요."
    exit 1
  fi
done
echo "== 포트 =="
echo "   a1 앞 : $A1_PORT"
echo "   a2 뒤 : $A2_PORT"
echo "   b1 좌 : $B1_PORT"
echo "   b2 우 : $B2_PORT"
echo "   ★ 위치가 다르면 케이블을 옮기지 말고 udev 설치 상태를 확인하세요."

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
  a1_port:="$A1_PORT" a2_port:="$A2_PORT" \
  b1_port:="$B1_PORT" b2_port:="$B2_PORT" &
PIDS+=("$!")

# 4대가 모두 올라올 때까지 기다린다.
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
