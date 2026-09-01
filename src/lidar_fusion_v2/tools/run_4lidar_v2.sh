#!/usr/bin/env bash
# Start the four verified YDLiDAR drivers and only the v2 fusion pipeline.

set -eu

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [ -d "${SCRIPT_DIR}/../../../src/lidar_fusion_v2" ]; then
  # Running from ws/src/lidar_fusion_v2/tools.
  WORKSPACE=$(cd -- "${SCRIPT_DIR}/../../.." && pwd)
elif [ -d "${SCRIPT_DIR}/../../../../../src/lidar_fusion_v2" ]; then
  # Running from ws/install/lidar_fusion_v2/share/lidar_fusion_v2/tools.
  WORKSPACE=$(cd -- "${SCRIPT_DIR}/../../../../.." && pwd)
else
  echo "cannot locate the colcon workspace from ${SCRIPT_DIR}" >&2
  exit 2
fi
RVIZ=true
BUILD=false
EXTRA_ARGS=()

while [ "$#" -gt 0 ]; do
  case "$1" in
    --no-rviz) RVIZ=false; shift ;;
    --build) BUILD=true; shift ;;
    --a1|--a2|--b1|--b2)
      SENSOR="${1#--}"
      [ "$#" -ge 2 ] || { echo "$1 requires a device path" >&2; exit 2; }
      EXTRA_ARGS+=("${SENSOR}_port:=$2")
      shift 2
      ;;
    -h|--help)
      echo "usage: $0 [--build] [--no-rviz] [--a1 DEV] [--a2 DEV] [--b1 DEV] [--b2 DEV]"
      exit 0
      ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done

for DEVICE in /dev/lidar_front /dev/lidar_rear /dev/lidar_left /dev/lidar_right; do
  if [ ! -e "$DEVICE" ]; then
    echo "missing $DEVICE; install tools/99-fma-lidars.rules or pass a port override" >&2
  fi
done

set +u
source /opt/ros/humble/setup.bash
set -u
if [ "$BUILD" = true ]; then
  colcon build --base-paths "${WORKSPACE}/src" \
    --packages-select lidar_fusion_v2 --symlink-install
fi
set +u
source "${WORKSPACE}/install/setup.bash"
set -u

exec ros2 launch lidar_fusion_v2 bringup.launch.py \
  "rviz:=${RVIZ}" "${EXTRA_ARGS[@]}"
