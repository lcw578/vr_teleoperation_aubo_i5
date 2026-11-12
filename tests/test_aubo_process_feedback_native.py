"""Actual C++ binding refuses stale/invalid cross-process joint observations."""
import importlib.util
import os
from pathlib import Path
import time
import unittest

runtime=Path(os.environ.get('AUBO_RUNTIME_DIR','/data/VR_teleoperation_runtime'))
binary=next((runtime/'build-follow/fake-python').glob('_aubo_follow*.so'))
def load(name):
    spec=importlib.util.spec_from_file_location(name,binary)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
native=load('_aubo_follow');controls=load('_aubo_follow_controls')


class ProcessFeedbackNativeTests(unittest.TestCase):
    source = 'B'
    def setUp(self):
        self.a=native.J6Follower(six_axis=True)
        self.a.login('offline',1,'test','test')
        self.state=native.RobotStateCache();self.sg=self.state.invalidate()
        self.joints=native.WaypointFeedbackCache(self.source);self.jg=self.joints.invalidate()
        self.sequence=0
        self.publish()
        self.a.attach_process_feedback(self.state,self.joints)
        self.assertTrue(self.a.prepare()['prepared'])
        self.publish()

    def tearDown(self):
        self.a.revoke();self.a.logout()

    def publish(self,stamp=None):
        now=time.monotonic();self.sequence+=1
        self.assertTrue(self.state.publish(0,0,now-.001,now,self.sg))
        self.assertTrue(self.joints.publish([0.]*6,now if stamp is None else stamp,
                                          self.sequence,self.jg))

    def test_native_sender_reads_B_cache_without_querying_or_subscribing_A(self):
        before=controls.counts()[:2]
        snap=self.a.snapshot()
        self.assertFalse(snap.get('observation_pending',False))
        feedback=self.a.waypoint_feedback()
        self.assertEqual(feedback['channels']['A']['total_received'],0)
        self.assertEqual(feedback['samples'][-1]['connection'],self.source)
        token=self.a.revision();self.assertTrue(self.a.arm(token))
        sent=self.a.send_joints([.001]*6,token)
        self.assertTrue(sent['sdk_invoked'])
        self.assertEqual(controls.counts()[:2],before)

    def test_stale_joints_refuse_send_even_when_state_is_fresh(self):
        token=self.a.revision();self.assertTrue(self.a.arm(token))
        time.sleep(.21)
        now=time.monotonic();self.state.publish(0,0,now-.001,now,self.sg)
        self.assertTrue(self.a.snapshot().get('observation_pending'))
        before=controls.counts()[2]
        self.assertFalse(self.a.send_joints([.001]*6,token)['sdk_invoked'])
        self.assertEqual(controls.counts()[2],before)

    def test_joint_generation_invalidation_immediately_removes_motion_feedback(self):
        token=self.a.revision();self.assertTrue(self.a.arm(token))
        self.joints.invalidate()
        self.assertFalse(self.joints.publish([0.]*6,time.monotonic(),99,self.jg))
        self.assertFalse(self.a.send_joints([.001]*6,token)['sdk_invoked'])
        self.assertFalse(self.a.arm(token))

    def test_future_nonfinite_regressed_and_repeated_packets_are_rejected(self):
        now=time.monotonic()
        invalid=[([0.]*6,now+1,self.sequence+1),([float('nan')]*6,now,self.sequence+1),
                 ([0.]*6,now,self.sequence),([0.]*6,now-1,self.sequence+1)]
        original=self.joints.feedback()['samples'][-1]
        for q,stamp,sequence in invalid:
            self.assertFalse(self.joints.publish(q,stamp,sequence,self.jg))
        self.assertEqual(self.joints.feedback()['samples'][-1],original)


class ProcessFeedbackNativeCTests(ProcessFeedbackNativeTests):
    source = 'C'


if __name__=='__main__':unittest.main()
