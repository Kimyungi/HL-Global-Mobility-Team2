#!/usr/bin/env bash
# Default: offline preparation only. --start launches with wait_go=true.
set -eo pipefail
TASK_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$TASK_ROOT/install/setup.bash"
MODE="${1:---check}"
if [[ "$MODE" != --check && "$MODE" != --start ]]; then
  echo "Usage: $0 [--check|--start]" >&2
  exit 2
fi
COURSE="$TASK_ROOT/src/stack_gps/waypoints/waypoints_halla_univ_20260819_182657.csv"
ARGS=(
  REAL_VEHICLE_CONFIRM:=I_UNDERSTAND_THIS_ENABLES_REAL_CAN_TX
  "waypoint_csv:=$COURSE"
  v_base:=2.0 v_accel_zone:=2.0 avoid_target_speed_mps:=2.0
  ttc_stop:=2.0 escape_after_cycles:=0
  estop_on_distance_m:=3.6 estop_off_distance_m:=4.0
  estop_corridor_max_x_m:=4.2 dynamic_stop_distance_m:=4.0
  dynamic_roi_max_x_m:=4.2 dynamic_tracking_max_distance_m:=5.0
  lane_enabled:=true gps_only:=false traffic_enabled:=true parking_enabled:=false
  usb_speed:=high camera_fps:=10 "lane_device:=${LANE_DEVICE:-xpu}"
  traffic_yolo_image_size:=800 traffic_stopline_yolo_image_size:=320
  traffic_stop_y_ratio:=0.0
  record:=true lane_csv:=true lane_debug:=false
)
/usr/bin/python3 - "$COURSE" <<'PY'
import csv, os, sys
from ament_index_python.packages import get_package_share_directory
for package in ('adas_mgm','stack_lane','stack_gps','stack_estop','stack_avoid','bridge_dspace','ydlidar_ros2_driver'):
    print(package, get_package_share_directory(package))
with open(sys.argv[1]) as f:
    rows = list(csv.DictReader(f))
assert len(rows) >= 10
assert all(-90 <= float(r['lat']) <= 90 and -180 <= float(r['lon']) <= 180 for r in rows)
for path in ('~/FMA_ws/src/stack_lane/config/homography.json', '~/FMA_ws/src/stack_lane/models/yolopv2.pt'):
    assert os.path.isfile(os.path.expanduser(path)), path
print(f'Course: {len(rows)} points; model/calibration present')
PY
printf 'Launch arguments: %s\n' "${ARGS[*]}"
if [[ "$MODE" == --check ]]; then
  ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py --show-args >/dev/null
  echo 'Offline check passed. USB/CAN/RTK/camera throughput still unverified. No nodes started.'
  exit 0
fi
for port in /dev/ttyRadio /dev/ttyRover /dev/ttyUSB_LIDAR; do
  [[ -e "$port" ]] || { echo "Missing device: $port" >&2; exit 1; }
done
[[ "$(cat /sys/class/net/can0/mtu)" == 72 ]] || { echo 'can0 must be CAN FD (MTU 72)' >&2; exit 1; }
echo 'Starting with wait_go=true. Departure requires ros2 run adas_mgm go in a separate terminal.'
exec ros2 launch adas_mgm REAL_VEHICLE_lane_gps_can.launch.py "${ARGS[@]}"
