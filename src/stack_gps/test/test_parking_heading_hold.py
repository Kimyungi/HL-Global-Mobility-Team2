"""No ROS/pytest: keep the IMU datum through a long reverse parking maneuver."""
import math
import ast
from pathlib import Path
import time
from types import SimpleNamespace as NS
import unittest
from stack_gps.heading_fusion import HeadingFusion


class ParkingHeadingTests(unittest.TestCase):
    def test_real_target_callback_requires_fresh_nonparking_to_release(self):
        # Execute the actual callback source with a transport/clock fixture.
        # ROS is deliberately not imported or represented as tested here.
        source=Path(__file__).parents[1]/'stack_gps/node.py'
        tree=ast.parse(source.read_text(encoding='utf-8'))
        method=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='_on_target_ref')
        namespace={'math':math,'time':time}
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        node=NS(fusion=HeadingFusion(),_station_target_stamp=0,stale_timeout=.5,
                get_clock=lambda:NS(now=lambda:NS(nanoseconds=2_000_000_000)))
        def send(sec,ns,state):
            namespace['_on_target_ref'](node,NS(header=NS(stamp=NS(sec=sec,nanosec=ns)),state=state,v_ref=0.))
        send(1,800_000_000,3);self.assertTrue(node.fusion.cog_hold)
        send(1,0,1);self.assertTrue(node.fusion.cog_hold)  # stale
        send(1,800_000_000,1);self.assertTrue(node.fusion.cog_hold)  # duplicate
        send(3,0,1);self.assertTrue(node.fusion.cog_hold)  # future
        send(1,900_000_000,1);self.assertFalse(node.fusion.cog_hold)

    def test_reverse_cog_never_reseeds_held_alignment(self):
        fusion=HeadingFusion(seed_n=1,reseed_after=3)
        fusion.update_imu(0.,0.);fusion.update_cog(0.,0.)
        fusion.cog_hold=True
        for i in range(1,301):
            t=i*.1;yaw=.5*math.sin(t)
            fusion.update_imu(yaw,t,gyro_z=0.)
            fusion.update_cog(yaw+math.pi,t,speed=.55)
            self.assertAlmostEqual(fusion.heading(t),yaw)
        self.assertTrue(fusion.aligned);self.assertEqual(fusion.reseeds,0)

    def test_hold_neither_seeds_missing_alignment_nor_masks_imu_loss(self):
        fusion=HeadingFusion(seed_n=1);fusion.cog_hold=True
        fusion.update_imu(0.,0.);fusion.update_cog(math.pi,0.)
        self.assertFalse(fusion.aligned);self.assertIsNone(fusion.heading(0.))
        fusion.cog_hold=False;fusion.update_cog(0.,1.);fusion.update_imu(0.,1.)
        fusion.update_cog(0.,1.1);self.assertTrue(fusion.aligned)
        fusion.cog_hold=True;self.assertIsNone(fusion.heading(2.))

    def test_forward_navigation_releases_cog_correction(self):
        fusion=HeadingFusion(seed_n=1,alpha=.5)
        fusion.update_imu(0.,0.);fusion.update_cog(0.,0.)
        fusion.cog_hold=True;fusion.update_imu(0.,1.);fusion.update_cog(.2,1.)
        self.assertAlmostEqual(fusion.heading(1.),0.)
        fusion.cog_hold=False;fusion.update_imu(0.,2.);fusion.update_cog(.2,2.)
        self.assertAlmostEqual(fusion.heading(2.),.1)


if __name__=='__main__':unittest.main()
