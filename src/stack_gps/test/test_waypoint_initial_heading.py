"""Initial course assumption uses the selected route, then yields to the IMU."""
from types import SimpleNamespace as NS

import pytest
from stack_gps.node import StackGpsNode
from stack_gps.path_engine import PathEngine
from stack_gps.heading_fusion import HeadingFusion


def test_initial_heading_at_current_waypoint_without_forward_motion():
    engine = PathEngine([(37., 127.), (37.00001, 127.), (37.00002, 127.),
                         (37.00002, 127.00001), (37.00002, 127.00002)])
    logs = []
    node = NS(initial_heading_from_waypoint=True, engine=engine,
              fusion=HeadingFusion(), get_logger=lambda: NS(info=logs.append))
    node.fusion.update_imu(.7, 1.)
    initialize = StackGpsNode._initialize_waypoint_heading
    initialize(node, 37.00002, 127.00001, 5, 1.)
    assert not node.fusion.aligned  # RTK FLOAT cannot choose the initial station.
    initialize(node, 37.00002, 127.00001, 4, 1.)
    assert node.fusion.heading(1.) == pytest.approx(engine.yaw[3])
    assert 'idx 3' in logs[0]
    node.fusion.update_imu(.9, 1.1)
    initialize(node, 37., 127., 4, 1.1)
    assert node.fusion.heading(1.1) == pytest.approx(engine.yaw[3] + .2)
    assert len(logs) == 1


def test_disabled_option_preserves_cog_initialization():
    node = NS(initial_heading_from_waypoint=False, fusion=HeadingFusion())
    StackGpsNode._initialize_waypoint_heading(node, 37., 127., 4, 1.)
    assert not node.fusion.aligned
