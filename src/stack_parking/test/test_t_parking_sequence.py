"""Offline full-sequence and real-CSV frame tests; no ROS/CAN or vehicle."""
import math
from pathlib import Path
import unittest
import numpy as np

from stack_parking.geometry import PathPoint, Pose2
from stack_parking.t_reference_parking import Config, Scan, inspect_candidate
from stack_parking.t_parking_sequence import TParkingSequence, load_course, csv_rows, metric_path
from test_t_reference_parking import path, left, rear


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.paths = (path('ref1',1),path('ref2',-1))
        xs = np.linspace(-2,0,41)
        self.approach = tuple(PathPoint(x,0,0,0,1) for x in xs)
        self.core = TParkingSequence(self.paths,self.approach,xs+2,Config(min_radius=.1))
        self.now = 1.
        self.pose = Pose2(-2,0,0)
        self.free = 0
        self.history = []

    def tick(self,dt=.1,**changes):
        self.now += dt
        obstacle = self.paths[1-self.free].path[-1]
        delta = np.array([obstacle.x-self.pose.x,obstacle.y-self.pose.y])
        c,s = math.cos(self.pose.yaw),math.sin(self.pose.yaw)
        local = [c*delta[0]+s*delta[1],-s*delta[0]+c*delta[1]]
        args = dict(now=self.now,pose=self.pose,pose_stamp=self.now,speed=0.,speed_stamp=self.now,
                    left=left(self.now,[local]),rear=rear(self.now),
                    front=Scan(self.now,np.c_[np.full(21,5.),np.linspace(-.3,.3,21)],(.76,0.)),
                    owned=True,route_at_end=False)
        args.update(changes)
        out = self.core.tick(**args)
        if not self.history or self.history[-1] != out.phase:
            self.history.append(out.phase)
        return out

    def select(self):
        for _ in range(12):
            out = self.tick()
            if out.phase == 'ADVANCE_3':
                break
        self.assertEqual(out.phase,'ADVANCE_3')
        self.assertEqual(out.selected,self.free)
        self.assertEqual(out.speed,0)

    def to_reverse(self):
        self.select()
        for p in self.approach:
            self.pose = Pose2(p.x,p.y,p.yaw)
            out = self.tick(route_at_end=p is self.approach[-1])
            self.assertGreaterEqual(out.speed,0)
        self.assertEqual(out.phase,'STOP_REVERSE')
        for _ in range(7): out = self.tick(route_at_end=True)
        self.assertEqual(out.phase,'REVERSE')

    def to_wait(self):
        self.to_reverse()
        candidate = self.paths[self.free]
        for i,p in enumerate(candidate.path):
            self.pose = Pose2(p.x,p.y,p.yaw)
            clearance = candidate.s[-1]-candidate.s[i]+.49
            out = self.tick(rear=rear(self.now+.1,clearance))
            self.assertLessEqual(out.speed,0)
            self.assertNotEqual(out.phase,'FAULT',out.reason)
        for _ in range(8):
            out = self.tick(rear=rear(self.now+.1,.49))
            if out.phase == 'WAIT_10': break
        self.assertEqual(out.phase,'WAIT_10')
        self.assertFalse(out.done)

    def test_full_sequence_both_slots_and_exact_hold(self):
        for free in (0,1):
            with self.subTest(free=free):
                self.setUp(); self.free=free; self.to_wait()
                entered = self.core.wait_since
                for _ in range(99):
                    out = self.tick(rear=rear(self.now+.1,.49))
                    self.assertEqual(out.phase,'WAIT_10')
                    self.assertEqual(out.speed,0)
                    self.assertFalse(out.done)
                while self.core.phase == 'WAIT_10':
                    out = self.tick(rear=rear(self.now+.1,.49))
                self.assertGreaterEqual(self.now-entered,10.)
                self.assertEqual(out.speed,0)
                for p in self.core.exit_path:
                    self.pose=Pose2(p.x,p.y,p.yaw)
                    out=self.tick()
                    self.assertGreaterEqual(out.speed,0)
                    self.assertFalse(out.done)
                self.assertEqual(out.phase,'EXIT_STOP')
                for _ in range(7): out=self.tick()
                self.assertTrue(out.done)
                self.assertEqual(self.history,['STOP_SELECT','ADVANCE_3','STOP_REVERSE','REVERSE','WAIT_10','EXIT','EXIT_STOP','DONE'])

    def test_selection_is_at_state_trigger_not_reverse_start(self):
        self.select()
        self.assertGreater(math.hypot(self.pose.x,self.pose.y),1)
        self.assertEqual(self.tick().phase,'ADVANCE_3')

    def test_endpoint_requires_metric_and_gps_agreement(self):
        self.select()
        self.assertEqual(self.tick(route_at_end=True).phase,'ADVANCE_3')
        for p in self.approach:
            self.pose=Pose2(p.x,p.y,p.yaw); out=self.tick()
        self.assertEqual(out.phase,'ADVANCE_3')
        self.assertEqual(out.speed,0)
        self.assertEqual(self.tick(route_at_end=True).phase,'STOP_REVERSE')

    def test_stale_pose_faults_during_advance_and_never_done(self):
        self.select()
        out=self.tick(pose_stamp=0)
        self.assertEqual(out.phase,'FAULT'); self.assertFalse(out.done)
        self.assertEqual(self.tick().phase,'FAULT')

    def test_stop_feedback_required_for_selection(self):
        for _ in range(20): self.assertEqual(self.tick(speed=.2).phase,'STOP_SELECT')

    def test_duplicate_scan_cannot_vote(self):
        for _ in range(8): self.tick()
        # Reset votes then repeatedly reuse one acquisition generation.
        self.setUp(); fixed=left(1.1,[[-1,.8]])
        for _ in range(20): self.tick(left=fixed)
        self.assertIsNone(self.core.selected)

    def test_front_obstacle_stops_forward(self):
        self.select()
        front=Scan(self.now+.1,np.c_[np.full(21,1.),np.linspace(-.3,.3,21)],(.76,0.))
        self.assertEqual(self.tick(front=front).reason,'front_obstacle')

    def test_no_front_returns_are_not_clear(self):
        self.select()
        front=Scan(self.now+.1,np.c_[np.full(21,5.),np.full(21,2.)],(.76,0.))
        self.assertEqual(self.tick(front=front).reason,'front_corridor_unknown')

    def test_authority_loss_during_wait_never_exits(self):
        self.to_wait()
        self.assertEqual(self.tick(owned=False).phase,'FAULT')
        self.assertEqual(self.tick(dt=20.).speed,0)

    def test_clock_rollback_latches_stop(self):
        self.select(); self.assertEqual(self.tick(dt=-1).reason,'clock_reversed')

    def test_rolling_during_wait_restarts_ten_seconds(self):
        self.to_wait(); self.tick(dt=5.)
        self.tick(speed=.1)
        self.assertIsNone(self.core.wait_since)
        for _ in range(8): self.tick()
        self.assertLess(self.now-self.core.wait_since,1)

    def test_wrong_direction_feedback_faults(self):
        self.select(); self.assertEqual(self.tick(speed=-.2).reason,'wrong_direction_forward')


class RealCsvTests(unittest.TestCase):
    def test_actual_left_fov_selects_each_free_slot_at_route3_marker(self):
        root=Path(__file__).resolve().parents[2]
        wp=root/'stack_gps/waypoints'
        candidates,approach,_=load_course(wp/'waypoints_halla_20260916_path_01.csv',
            wp/'waypoints_halla_20260916_path_03.csv',
            [root/'stack_parking/config'/f'parking_ref_{i:02d}.csv' for i in (1,2)])
        rows=csv_rows(wp/'waypoints_halla_20260916_path_03.csv')
        trigger=next(i for i,r in enumerate(rows) if int(r['state'])==1)
        origin=csv_rows(wp/'waypoints_halla_20260916_path_01.csv')[0]
        pose=metric_path(rows[trigger:trigger+2],(float(origin['lat']),float(origin['lon'])))[0][0]
        def transform(points,p):
            c,s=math.cos(p.yaw),math.sin(p.yaw)
            return np.asarray(points)@np.array([[c,s],[-s,c]])+[p.x,p.y]
        body=np.array([[.76,.31],[.76,-.31],[-.09,-.31],[-.09,.31]])
        origin=transform([[.215329,.211549]],pose)[0]
        walls=[]
        for candidate in candidates:
            p=candidate.path[-1]
            centre=transform([[-.110354-.5,.002473]],p)[0]
            side=np.array([-math.sin(p.yaw),math.cos(p.yaw)])*.58
            walls.append((centre-side,centre+side))
        for free in (0,1):
            occupied=transform(body,candidates[1-free].path[-1])
            segments=walls+list(zip(occupied,np.roll(occupied,-1,axis=0)))
            points=[]
            for angle in np.deg2rad(np.arange(35,145.01,.5)+1.703610)+pose.yaw:
                direction=np.array([math.cos(angle),math.sin(angle)])
                distance=math.inf
                for a,b in segments:
                    matrix=np.column_stack((direction,a-b))
                    if abs(np.linalg.det(matrix))<1e-9:continue
                    t,u=np.linalg.solve(matrix,a-origin)
                    if t>=0 and 0<=u<=1:distance=min(distance,t)
                if distance<=12:points.append(origin+distance*direction)
            c,s=math.cos(pose.yaw),math.sin(pose.yaw)
            local=(np.array(points)-[pose.x,pose.y])@np.array([[c,-s],[s,c]])
            scan=Scan(1.,local,(.215329,.211549))
            self.assertGreaterEqual(len(local),Config().min_scan_points)
            findings=[inspect_candidate(candidate,Pose2(pose.x,pose.y,pose.yaw),scan,Config())
                      for candidate in candidates]
            self.assertTrue(findings[1-free][0]);self.assertFalse(findings[free][0])

    def test_common_frame_for_every_supported_start(self):
        root=Path(__file__).resolve().parents[2]
        waypoints=root/'stack_gps/waypoints'
        parking=[root/'stack_parking/config'/f'parking_ref_{i:02d}.csv' for i in (1,2)]
        for start in ('01','02','03'):
            candidates,approach,_=load_course(waypoints/f'waypoints_halla_20260916_path_{start}.csv',
                                             waypoints/'waypoints_halla_20260916_path_03.csv',parking)
            for c in candidates:
                self.assertLess(math.hypot(c.path[0].x-approach[-1].x,c.path[0].y-approach[-1].y),.01)
                self.assertTrue(all(p.gear == -1 for p in c.path))


if __name__ == '__main__': unittest.main()
