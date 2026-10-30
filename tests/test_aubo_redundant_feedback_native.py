"""Two SDK instances can publish the same waypoint stream into one cache."""
import importlib.util
from pathlib import Path
import time
import unittest

runtime = Path('/data/VR_teleoperation_runtime')
binary = next((runtime/'build-follow/fake-python').glob('_aubo_follow*.so'))
def load(name):
    spec=importlib.util.spec_from_file_location(name,binary)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
native=load('_aubo_follow');controls=load('_aubo_follow_controls')

class RedundantFeedbackTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a=native.J6Follower(six_axis=True,follow_hz=25)
        cls.b=native.RobotStateObserver()
        cls.a.login('offline',1,'test','test');cls.b.login('offline',1,'test','test')
        cls.a.start_waypoint_feedback();cls.b.query_state()
        cls.a.attach_state_observer(cls.b,True)
        if not cls.a.prepare()['prepared']:raise AssertionError('Offline prepare failed')

    def setUp(self):
        controls.push_enabled(True);controls.push_enabled_b(True)
        controls.state(0);controls.joint(0.)
        self.a.revoke();self.b.query_state()

    @classmethod
    def tearDownClass(cls):
        controls.push_enabled(True);controls.push_enabled_b(True)
        cls.a.logout();cls.b.logout()

    def test_b_keeps_shared_feedback_fresh_when_a_stalls(self):
        controls.push_enabled(False);controls.push_enabled_b(True)
        try:
            before=self.a.waypoint_feedback();time.sleep(.25);after=self.a.waypoint_feedback()
            self.assertGreater(after['channels']['B']['total_received'],before['channels']['B']['total_received'])
            self.assertEqual(after['samples'][-1]['connection'],'B')
            self.assertLess(time.monotonic()-after['samples'][-1]['host_received_monotonic_s'],.2)
        finally:
            controls.push_enabled(True);controls.push_enabled_b(True)

    def test_a_keeps_shared_feedback_fresh_when_b_push_stalls(self):
        controls.push_enabled(True);controls.push_enabled_b(False)
        try:
            before=self.a.waypoint_feedback();time.sleep(.25);after=self.a.waypoint_feedback()
            self.assertGreater(after['channels']['A']['total_received'],before['channels']['A']['total_received'])
            self.assertEqual(after['samples'][-1]['connection'],'A')
            self.assertLess(time.monotonic()-after['samples'][-1]['host_received_monotonic_s'],.2)
        finally:controls.push_enabled_b(True)

    def test_native_arm_and_send_accept_fresh_b_with_a_stale(self):
        controls.push_enabled(False)
        try:
            time.sleep(.23);self.b.query_state()
            snap=self.a.snapshot()
            self.assertFalse(snap.get('observation_pending',False))
            token=self.a.revision();self.assertTrue(self.a.arm(token))
            sent=self.a.send_joints([.01]*6,token)
            self.assertTrue(sent['sdk_invoked'],sent)
            self.assertEqual(sent['return_code'],0)
        finally:self.a.revoke();controls.push_enabled(True)

    def test_both_pushes_stale_still_refuse_motion(self):
        token=self.a.revision();self.assertTrue(self.a.arm(token))
        controls.push_enabled(False);controls.push_enabled_b(False)
        try:
            time.sleep(.23);self.b.query_state()
            snap=self.a.snapshot()
            self.assertTrue(snap.get('observation_pending'))
            self.assertFalse(self.a.arm(token))
            sent=self.a.send_joints([.01]*6,token)
            self.assertFalse(sent['sdk_invoked'],sent)
        finally:controls.push_enabled(True);controls.push_enabled_b(True)

    def test_incremental_read_returns_new_samples_and_keeps_old_timestamp_if_quiet(self):
        before=self.a.waypoint_feedback()['total_received']
        time.sleep(.01)
        controls.push_enabled(False);controls.push_enabled_b(False)
        try:
            new=self.a.waypoint_feedback_since(before)
            self.assertTrue(new['samples'])
            self.assertTrue(all(s['sequence']>before for s in new['samples']))
            self.assertLess(len(new['samples']),len(self.a.waypoint_feedback()['samples']))
            quiet=self.a.waypoint_feedback_since(new['total_received'])
            self.assertEqual(quiet['samples'],[new['samples'][-1]])
            self.assertEqual(quiet['total_received'],new['total_received'])
        finally:controls.push_enabled(True);controls.push_enabled_b(True)

if __name__=='__main__':unittest.main()
