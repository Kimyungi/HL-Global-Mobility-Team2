#!/usr/bin/env python3
"""Exercise the real GPS geometry publisher without starting sensor drivers."""
import ast
import math
import os
from pathlib import Path as FilePath
from types import SimpleNamespace as NS

from fma_interfaces.msg import GpsRoute, RefPoint
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time


source = FilePath(__file__).parents[2]/'stack_gps/stack_gps/node.py'
tree = ast.parse(source.read_text())
methods = [m for c in tree.body if isinstance(c, ast.ClassDef) for m in c.body
           if isinstance(m, ast.FunctionDef) and m.name in ('_publish_track', '_fill_route')]
scope = dict(Path=Path, PoseStamped=PoseStamped, GpsRoute=GpsRoute, RefPoint=RefPoint, math=math, os=os)
exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), scope)


class Publisher:
    def publish(self, msg):
        self.msg = msg


class Fixture:
    _publish_track = scope['_publish_track']
    _fill_route = scope['_fill_route']
    _route_instance = 7

    def __init__(self, plan=None):
        self._route_plan = plan
        self.engine = NS(e=[0., 1., 2.], n=[0., .2, .3], yaw=[.1, .2, .3])
        self.pub_track = Publisher()
        self.pub_route_geometry = Publisher()

    def get_clock(self):
        return NS(now=lambda: NS(to_msg=lambda: Time(sec=3)))

    def get_parameter(self, name):
        assert name == 'waypoint_csv'
        return NS(value='/tmp/track.csv')


single = Fixture()
single._publish_track()
assert single.pub_route_geometry.msg.route_id == 'track.csv'
assert not single.pub_route_geometry.msg.route.enabled
plan = NS(active_points=[(3., 4.), (4., 5.), (5., 6.)], position=lambda lat, lon: (lat+10., lon+20.),
          connecting=False, next_connecting=False, sequence_id=11, index=0,
          acknowledged_request=9, required=[[]], files=[NS(id='route-a', csv='a.csv', zones='a.yaml', completion=0)])
sequenced = Fixture(plan)
sequenced._publish_track()
msg = sequenced.pub_route_geometry.msg
assert msg.route_id == 'route-a' and msg.route.enabled and msg.route.instance_id == 7
assert msg.route.acknowledged_request == 9
assert msg.points[0].x == 13. and msg.points[0].y == 24.
for fixture in (single, sequenced):
    a, b = fixture.pub_track.msg, fixture.pub_route_geometry.msg
    assert a.header == b.header and b.header.frame_id == 'map'
    assert len(a.poses) == len(b.points) == 3
    for i, (pose, point) in enumerate(zip(a.poses, b.points)):
        assert pose.pose.position.x == point.x and pose.pose.position.y == point.y
        assert point.yaw == fixture.engine.yaw[i]
print('PASS: GPS route geometry matches active ENU track in standalone and route-sequence modes')
