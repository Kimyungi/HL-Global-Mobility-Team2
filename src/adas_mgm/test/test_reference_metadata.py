"""Exercise producer wrapper methods without opening sensors/loading inference.

Methods are compiled from the actual source AST to avoid importing camera/GPU
backends. Only clocks, input samples and publishers are mocked; metadata and
publication method bodies are the production code.
"""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import math
import numpy as np
import pytest
from rclpy.duration import Duration
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from fma_interfaces.msg import LanePath, GpsPath, AvoidStatus, ParkingStatus, RefPoint

ROOT = Path(__file__).resolve().parents[3]


def method(path, name, namespace):
    tree = ast.parse((ROOT / path).read_text())
    function = next(item for cls in tree.body if isinstance(cls, ast.ClassDef)
                    for item in cls.body if isinstance(item, ast.FunctionDef) and item.name == name)
    namespace = dict(namespace)
    module = ast.Module(body=[ast.ImportFrom(module='__future__',
                        names=[ast.alias(name='annotations')], level=0), function], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(ROOT/path), 'exec'), namespace)
    return namespace[name]


class Pub:
    def __init__(self): self.messages = []
    def publish(self, msg): self.messages.append(msg)


@pytest.fixture
def environment():
    clock = NS(ros=100., mono=10.)
    context = dict(math=math, time=NS(monotonic=lambda:clock.mono), Duration=Duration, Time=Time)
    node = NS(get_clock=lambda:NS(now=lambda:Time(seconds=clock.ros)))
    return clock, context, node


def stamp(msg): return msg.reference_stamp.sec * 1_000_000_000 + msg.reference_stamp.nanosec


def test_line_held_search_does_not_refresh_generation(environment):
    clock, ns, node = environment
    fn = method('src/stack_lane/stack_lane/node.py', '_set_reference_stamp', ns)
    node._reference_stamp = None
    first = LanePath(points=[RefPoint(x=1.)]); fn(node,first,9.9,True)
    assert stamp(first) > 0
    for _ in range(10):
        clock.ros += .1; clock.mono += .1
        held = LanePath(points=first.points); fn(node,held,clock.mono,False)
        assert stamp(held) == stamp(first)
    actual = LanePath(points=first.points); fn(node,actual,clock.mono,True)
    assert stamp(actual) > stamp(first)  # same coordinates on a real new estimate are legitimate


def test_line_unknown_capture_and_old_capture_are_not_freshened(environment):
    clock, ns, node = environment
    fn = method('src/stack_lane/stack_lane/node.py', '_set_reference_stamp', ns)
    node._reference_stamp = None
    msg = LanePath(points=[RefPoint(x=1.)]); fn(node,msg,None,True)
    assert stamp(msg) == 0
    old = LanePath(points=msg.points); fn(node,old,8.,True)
    assert stamp(old) == 98_000_000_000
    empty = LanePath(); fn(node,empty,clock.mono,False)
    assert stamp(empty) == 0


def test_gps_same_fix_republication_keeps_generation(environment):
    clock, ns, node = environment
    fn = method('src/stack_gps/stack_gps/node.py', '_set_reference_stamp', ns)
    node._reference_fix_time = node._reference_stamp = None
    first = GpsPath(); fn(node,first,9.9)
    clock.ros += 1.; clock.mono += 1.
    repeated = GpsPath(); fn(node,repeated,9.9)
    assert stamp(first) == stamp(repeated)
    new = GpsPath(); fn(node,new,10.9)
    assert stamp(new) > stamp(first)


def test_parking_timer_uses_actual_localization_stamp(environment):
    clock, ns, node = environment
    fn = method('src/stack_parking/stack_parking/node.py', '_set_reference_stamp', ns)
    node.reference_input_stamp_s = 99.9
    node._localization_valid = lambda now:True
    first = ParkingStatus(points=[RefPoint(x=-1.)]); fn(node,first,clock.ros)
    clock.ros += 1.
    repeated = ParkingStatus(points=first.points); fn(node,repeated,clock.ros)
    assert stamp(first) == stamp(repeated)
    node.reference_input_stamp_s = 100.9
    new = ParkingStatus(points=first.points); fn(node,new,clock.ros)
    assert stamp(new) > stamp(first)
    node._localization_valid = lambda now:False
    invalid = ParkingStatus(points=first.points); fn(node,invalid,clock.ros)
    assert stamp(invalid) == 0


def test_avoid_generation_comes_from_scan_not_heartbeat(environment):
    clock, ns, node = environment
    ns.update(AvoidStatus=AvoidStatus, EPS_SPEED=1e-3, TTC_INF=1e9)
    fn = method('src/stack_avoid/stack_avoid/node.py','on_scan',ns)
    node.front_scan_pub = Pub(); node.pub = Pub()
    node._front_only_scan = lambda scan:scan
    node._nearest_front_obstacle = lambda scan:(2.,0.)
    node._gap_target = lambda scan,gap:RefPoint(x=1.5,y=.3)
    node._ego_speed = lambda:.6
    node.detect_range=3.; node.detect_hysteresis=.2; node._detected_prev=False
    node.ttc_stop=1.3; node.target_speed=.6; node._maneuver_armed=False
    node._done_until=0.; node._clear_since=None; node._prev_center=None
    node.clear_gap_max=2.; node.vehicle_len=.5; node.clear_margin=.3
    node._rp=lambda x,y:RefPoint(x=float(x),y=float(y))
    scan=LaserScan(angle_min=-1.,angle_increment=.1,range_min=.1,range_max=12.,ranges=[2.]*20)
    scan.header.stamp=Time(seconds=99.9).to_msg()
    fn(node,scan); first=node.pub.messages[-1]
    clock.ros+=.2; clock.mono+=.2
    fn(node,scan); repeat=node.pub.messages[-1]
    assert first.header.stamp != repeat.header.stamp and stamp(first)==stamp(repeat)
    node._nearest_front_obstacle=lambda scan:None
    scan.header.stamp=Time(seconds=100.1).to_msg()
    fn(node,scan); clear=node.pub.messages[-1]
    assert clear.points and stamp(clear)>stamp(first)  # existing clearance hold point recomputed on new scan
    node._maneuver_armed=False
    fn(node,scan)
    assert not node.pub.messages[-1].points and stamp(node.pub.messages[-1])==0


def test_fusion_timer_does_not_launder_cached_sensor_stamps(environment):
    clock, ns, node = environment
    ns.update(np=np, Header=Header, LaserScan=LaserScan, point_cloud2=point_cloud2,
              scan_to_base=lambda *args:np.array([[1.,0.]],dtype=np.float32),
              points_to_virtual_scan=lambda *args:np.ones(10,dtype=np.float32))
    fn=method('src/lidar_fusion_v2/lidar_fusion_v2/fusion_node.py','_publish',ns)
    node.sensor_ids=['a1','a2']; node.latest={}; node.received_monotonic={}
    for sid, when in [('a1',99.9),('a2',99.8)]:
        node.latest[sid]=LaserScan()
        node.latest[sid].header.stamp=Time(seconds=when).to_msg()
        node.received_monotonic[sid]=clock.mono
    node.geometry={'a1':None,'a2':None}; node.max_age=.3
    node.scan_increment=.1; node.scan_range_min=.1; node.scan_range_max=12.; node.publish_rate=10.
    node.base_frame='base_link'; node.last_active=('a1','a2')
    node.fields=[PointField(name=name,offset=4*i,datatype=PointField.FLOAT32,count=1)
                 for i,name in enumerate(('x','y','z'))]
    node.raw_pubs={'a1':Pub(),'a2':Pub()}; node.cloud_pub=Pub(); node.scan_pub=Pub()
    fn(node); original=node.scan_pub.messages[-1].header.stamp
    clock.ros+=.1; clock.mono+=.1; fn(node)
    assert node.scan_pub.messages[-1].header.stamp==original
    assert original==node.latest['a1'].header.stamp  # latest actual contributing input
    assert node.raw_pubs['a1'].messages[-1].header.stamp==node.latest['a1'].header.stamp
    node.latest['a2'].header.stamp=Time(seconds=100.).to_msg(); fn(node)
    assert node.scan_pub.messages[-1].header.stamp==node.latest['a2'].header.stamp
    node.latest['a1'].header.stamp=Time(seconds=100.1).to_msg(); fn(node)
    assert node.scan_pub.messages[-1].header.stamp==node.latest['a1'].header.stamp
