"""Execute production ROS adapter callbacks with message/transport-only stubs."""
import importlib
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np

from stack_parking.geometry import PathPoint
from stack_parking.t_reference_parking import Scan
from test_t_reference_parking import path, left, rear


def stamp(t=0.):
    sec,ns=divmod(round(t*1e9),1_000_000_000)
    return NS(sec=sec,nanosec=ns)


class Status:
    def __init__(self):
        self.header=NS(stamp=stamp(),frame_id='')
        self.preparation_stamp=stamp();self.reference_stamp=stamp()
        self.points=[];self.v_suggest=0.;self.done=False
        self.mission_active=False;self.search_active=False


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.command=NS(PREPARE=1,ACTIVATE=2,CANCEL=3)
        stubs={'yaml':NS(), 'ament_index_python':NS(),
               'ament_index_python.packages':NS(get_package_share_directory=lambda _:''),
               'rclpy':NS(), 'rclpy.qos':NS(qos_profile_sensor_data=None),
               'fma_interfaces':NS(), 'fma_interfaces.msg':NS(GpsPath=NS(HEADING_FUSED=2),
                   MgmState=NS(MISSION_ACTIVE=1),ParkingCommand=cls.command,
                   ParkingStatus=Status,RefPoint=lambda **kw:NS(**kw),VehicleVector=NS),
               'sensor_msgs':NS(), 'sensor_msgs.msg':NS(LaserScan=NS),
               'std_msgs':NS(), 'std_msgs.msg':NS(String=lambda **kw:NS(**kw))}
        with patch.dict(sys.modules,stubs):
            cls.module=importlib.import_module('stack_parking.t_parking_adapter')
        # Avoid leaking stubbed ROS modules into other tests in ROS installations.
        sys.modules.pop('stack_parking.t_parking_adapter',None)

    def setUp(self):
        self.now=1.;self.messages=[]
        self.node=NS(search_request_id=0,search_mission_mode=0,_cancel_search=lambda:None,
                     _clock_s=lambda:self.now,get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:stamp(self.now))),
                     status_pub=NS(publish=self.messages.append),get_logger=lambda:NS(info=lambda _:None))
        self.adapter=self.module.ReferenceParkingAdapter.__new__(self.module.ReferenceParkingAdapter)
        a=self.adapter;a.node=self.node
        a.course=((path('a',1),path('b',-1)),tuple(PathPoint(x,0,0,0,1) for x in np.linspace(-2,0,41)),np.linspace(0,2,41))
        a.route_csv=Path('route03.csv').resolve();a.core=None;a.request_id=0
        a.active=a.authorized=False;a.started=-math.inf;a.route_identity=None;a.last_phase=None
        a.gps=a.vehicle=a.mgm=None;a.scans={k:None for k in ('a1','a2','b1')}
        a.phase_pub=NS(publish=lambda _:None)
        self.send(1,100)

    def send(self,action,request=100,mode=1):
        return self.adapter.command(NS(action=action,request_id=request,mission_mode=mode))

    def supply(self,dt=.1):
        self.now+=dt;a=self.adapter;t=self.now
        a.on_gps(NS(reference_stamp=stamp(t),position_x=-2.,position_y=0.,vehicle_heading_rad=0.,
                    position_valid=True,vehicle_heading_valid=True,heading_source=2,fix_quality=4,at_end=False,
                    dx=.01,dy=0.,dyaw=0.,update=round(t*10),
                    route=NS(enabled=True,connecting=False,route_id='03',waypoint_csv=str(a.route_csv),
                             sequence_id=7,instance_id=8,index=2)))
        a.on_vehicle(NS(header=NS(stamp=stamp(t)),v=0.))
        a.on_mgm(NS(header=NS(stamp=stamp(t)),mission_request_active=True,mission_request_id=100,
                    mission=1,mission_type=1,top=1,active_safe_stop_reasons=4,mission_completed=False))
        a.scans={'a1':Scan(t,np.c_[np.full(21,5.),np.linspace(-.3,.3,21)],(.76,0.)),
                 'a2':rear(t),'b1':left(t,[[-2,-.8]])}

    def ready(self):
        for _ in range(15): self.supply();self.adapter.tick()
        self.assertTrue(self.messages[-1].preparation_ready)
        self.assertEqual(self.adapter.core.phase,'ADVANCE_3')

    def test_prepare_selects_but_cannot_drive_until_activate(self):
        self.ready()
        self.assertTrue(all(m.v_suggest==0 and not m.done for m in self.messages))
        self.send(2);self.supply();self.adapter.tick()
        out=self.messages[-1]
        self.assertGreater(out.v_suggest,0);self.assertEqual(len(out.points),1)
        self.assertTrue(out.mission_active);self.assertFalse(out.done)

    def test_activate_before_selection_cannot_authorize(self):
        self.send(2);self.assertFalse(self.adapter.authorized)

    def test_wrong_raw_sensor_frame_is_rejected(self):
        self.supply()
        self.adapter.on_scan('b1',NS(header=NS(frame_id='base_link')))
        self.assertIsNone(self.adapter.scans['b1'])

    def test_prepare_retry_is_idempotent(self):
        core=self.adapter.core;self.send(1);self.assertIs(core,self.adapter.core)

    def test_stale_and_wrong_mode_commands_cannot_take_ownership(self):
        self.send(3,99);self.send(2,100,2)
        self.assertTrue(self.adapter.active);self.assertFalse(self.adapter.authorized)

    def test_cancel_tombstone_blocks_delayed_prepare(self):
        self.send(3,101);self.send(1,101)
        self.assertFalse(self.adapter.active)
        self.assertEqual(self.node.search_request_id,101)

    def test_new_parallel_prepare_releases_to_original_pipeline(self):
        self.assertFalse(self.send(1,101,2));self.assertFalse(self.adapter.active)

    def test_tangent_heading_is_not_real_reverse_heading(self):
        for _ in range(15):
            self.supply();self.adapter.gps.heading_source=0;self.adapter.tick()
        self.assertFalse(self.messages[-1].preparation_ready)

    def test_pre_request_frames_do_not_make_ready(self):
        self.supply();self.adapter.gps.reference_stamp=stamp(.9);self.adapter.tick()
        self.assertFalse(self.messages[-1].preparation_ready)

    def test_heartbeat_preserves_actual_input_generation(self):
        self.ready();first=self.messages[-1].reference_stamp
        self.now+=.01;self.adapter.tick()
        last=self.messages[-1].reference_stamp
        self.assertEqual((first.sec,first.nanosec),(last.sec,last.nanosec))

    def test_route_datum_identity_change_stops_without_done(self):
        self.ready();self.send(2);self.supply();self.adapter.gps.route.instance_id+=1
        self.adapter.tick()
        self.assertEqual(self.adapter.core.phase,'FAULT')
        self.assertEqual(self.messages[-1].v_suggest,0);self.assertFalse(self.messages[-1].done)

    def test_missing_actual_owner_cannot_drive(self):
        self.ready();self.send(2);self.supply();self.adapter.mgm.mission_request_id=99
        self.adapter.tick();self.assertEqual(self.messages[-1].v_suggest,0)

    def test_done_not_emitted_at_wall_or_wait(self):
        self.ready();self.send(2);self.adapter.core.phase='WAIT_10';self.adapter.core.wait_since=self.now
        self.supply();self.adapter.tick()
        self.assertFalse(self.messages[-1].done);self.assertEqual(self.messages[-1].v_suggest,0)


if __name__ == '__main__': unittest.main()
