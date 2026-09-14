"""GPS producer -> ROS message -> avoidance consumer, with no hardware."""
import math
from pathlib import Path

import pytest
from fma_interfaces.msg import GpsPath
from rclpy.time import Time

from stack_gps.node import StackGpsNode
from stack_gps.path_engine import PathEngine, load_waypoints_csv
from test_surface_node import node, scene_scan  # noqa: F401


def test_real_csv_window_drives_avoidance_station_in_shared_enu_frame(node):
    root = Path(__file__).resolve().parents[2]
    points = load_waypoints_csv(root/'stack_gps/waypoints/waypoints_halla_reference_path_03.csv')
    engine = PathEngine(points, station_tracking=True)
    lat, lon = points[10]
    yaw = engine.yaw[10]
    snap = engine.snapshot(lat, lon, yaw, generation=1., v_ref=1.)
    msg = GpsPath(fix_quality=4, position_valid=True, vehicle_heading_valid=True,
                  heading_source=GpsPath.HEADING_FUSED, vehicle_heading_rad=yaw)
    msg.reference_stamp = Time(seconds=99.9).to_msg()
    # An arbitrary common-frame shift reproduces sequence ENU vs route-local ENU.
    ev, nv = engine.to_enu(lat, lon)
    msg.position_x, msg.position_y = ev+123., nv-57.
    StackGpsNode._fill_station_reference(None, msg, snap)
    node._on_gps_path(msg)
    node.on_scan(scene_scan([]))
    target = node.messages[-1].points[0]
    world = engine.station_path.sample(snap['station_m']+1.)
    expected = (math.cos(yaw)*(world[0]-ev)+math.sin(yaw)*(world[1]-nv),
                -math.sin(yaw)*(world[0]-ev)+math.cos(yaw)*(world[1]-nv))
    assert (target.x, target.y) == pytest.approx(expected, abs=2e-6)
    assert node._planner.gps_station == pytest.approx(snap['station_m'])
    assert node._pose_source == 'gps'
