import math
import unittest
from estop_recovery_core import Recovery, rear_corridor_clear

class RecoveryTest(unittest.TestCase):
    def setUp(self):
        self.r=Recovery()
        self.call(0,active=False,authorized=False,speed=0.)
    def call(self,t,**kw):
        args=dict(now=t,stamp_now=t,request=1,active=True,authorized=True,
                  speed=0.,speed_stamp=t,rear_clear=True,rear_stamp=t)
        args.update(kw)
        return self.r.step(**args)
    def reverse(self):
        self.assertEqual(self.call(1),(0.,False))
        self.assertEqual(self.call(10.99),(0.,False))
        self.assertEqual(self.call(11),(-.3,False))
    def test_full_sequence_and_actual_stop(self):
        self.reverse()
        for n in range(1,36): self.call(11+n*.1,speed=-.3)
        self.assertEqual(self.r.phase,'SETTLE')
        self.assertEqual(self.call(14.6),(0.,True))
    def test_restart_inside_estop_does_not_repeat_reverse(self):
        self.r=Recovery();self.call(1);self.call(12)
        self.assertEqual(self.r.phase,'HOLD')
        self.assertFalse(self.r.armed)
    def test_estop_entry_arms_without_forward_history(self):
        self.assertFalse(self.r.armed)
        self.assertEqual(self.call(1),(0.,False))
        self.assertTrue(self.r.armed)
        self.assertEqual(self.call(10.99),(0.,False))
        self.assertEqual(self.call(11),(-.3,False))
    def test_missing_mgm_at_start_is_not_an_inactive_observation(self):
        self.r=Recovery()
        self.call(0,active=False,authorized=False,state_valid=False)
        self.call(1);self.call(12)
        self.assertFalse(self.r.armed)
        self.assertEqual(self.r.phase,'HOLD')
    def test_arming_does_not_bypass_authority_or_fresh_speed(self):
        self.call(1,authorized=False)
        self.assertTrue(self.r.armed)
        self.assertEqual(self.call(12,authorized=False),(0.,False))
        self.assertEqual(self.call(13,speed_stamp=1),(0.,False))
        self.assertEqual(self.call(14),(0.,False))
        self.assertEqual(self.call(23.99),(0.,False))
        self.assertEqual(self.call(24),(-.3,False))
    def test_movement_resets_hold(self):
        self.call(1);self.call(10,speed=.1);self.call(11)
        self.assertEqual(self.call(20),(0.,False))
    def test_duplicate_speed_does_not_integrate(self):
        self.reverse();self.call(11.1,speed=-.3,speed_stamp=11)
        self.assertEqual(self.r.distance,0)
        self.call(11.3,speed=-.3,speed_stamp=11)
        self.assertEqual(self.r.phase,'FAULT')
    def test_rear_loss_sticky(self):
        self.reverse();self.call(11.1,rear_clear=False)
        self.assertEqual(self.call(11.2), (0.,False))
        self.assertEqual(self.r.phase,'FAULT')
    def test_authority_loss(self):
        self.reverse();self.call(11.1,authorized=False)
        self.assertEqual(self.r.phase,'FAULT')
    def test_five_second_timeout(self):
        self.reverse()
        for n in range(1,51):self.call(11+n*.1)
        self.assertEqual(self.r.phase,'FAULT')
    def test_wrong_direction(self):
        self.reverse();self.call(11.1,speed=.1)
        self.assertEqual(self.r.phase,'FAULT')
    def test_backward_clock(self):
        self.reverse();self.call(10.9)
        self.assertEqual(self.r.phase,'FAULT')
    def test_rear_required_to_start(self):
        self.call(1);self.call(12,rear_clear=False)
        self.assertEqual(self.r.phase,'HOLD')
    def test_a2_corridor(self):
        mount=[-.110354,.002473,93.410120,20.,160.,.069,.15,12.]
        def clear(r):return rear_corridor_clear(r,0.,math.tau/1000,.1,12.,mount)
        self.assertTrue(clear([10.]*1000))
        self.assertFalse(clear([math.inf]*1000))
        self.assertFalse(clear([math.nan]*1000))
        ranges=[10.]*1000;ranges[240]=.5
        self.assertFalse(clear(ranges))
        self.assertFalse(clear([]))

if __name__=='__main__':unittest.main()
