"""Graded protection through the production C++ binding and simulated SDK only."""
import time
import unittest
from test_aubo_process_feedback_native import native,controls

class HotFeedbackNativeTests(unittest.TestCase):
    def setUp(self):
        self.a=native.J6Follower(six_axis=True,passthrough=True)
        self.a.login('offline',1,'test','test')
        self.state=native.RobotStateCache();self.joints=native.WaypointFeedbackCache('C')
        self.publish()
        self.a.attach_process_feedback(self.state,self.joints)
        self.assertTrue(self.a.prepare()['prepared'])
        self.token=self.a.revision();self.assertTrue(self.a.arm(self.token))
        self.a.enter_passthrough(self.token)
        for _ in range(12):
            self.publish();self.a.snapshot();self.a.send_priming(self.token)
        self.publish();self.a.snapshot();self.assertTrue(self.a.finish_priming(self.token))
    def tearDown(self):
        controls.send_delay_s(0)
        self.a.stop();self.a.logout()
    def publish(self,joint_age=0,state_age=0):
        now=time.monotonic()
        sg=self.state.invalidate();jg=self.joints.invalidate()
        self.assertTrue(self.state.publish(0,0,now-state_age-.001,now-state_age,sg))
        self.assertTrue(self.joints.publish([0.]*6,now-joint_age,1,jg))
    def test_single_tail_and_hard_age_boundaries(self):
        for ja,sa in ((.201,.201),(.30258,.3189),(.49,.49)):
            self.publish(ja,sa)
            self.assertFalse(self.a.snapshot().get('observation_pending',False))
            result=self.a.send_joints([0.]*6,self.token)
            self.assertTrue(result['sdk_invoked']);self.assertFalse(result['revoked_during_call'])
        self.publish(.501,0)
        self.assertFalse(self.a.send_joints([0.]*6,self.token)['sdk_invoked'])
        with self.assertRaisesRegex(RuntimeError,'permission revoked'):
            self.a.send_joints([0.]*6,self.token)
    def test_state_501ms_refuses_and_revokes(self):
        self.publish(0,.501)
        self.assertFalse(self.a.send_joints([0.]*6,self.token)['sdk_invoked'])
    def test_three_slow_calls_across_hot_stream_revoke_and_fast_call_resets(self):
        for duration,expected in ((.26,1),(.26,2),(0.,0),(.26,1),(.26,2),(.26,3)):
            self.publish();controls.send_delay_s(duration)
            result=self.a.send_joints([0.]*6,self.token)
            self.assertEqual(result['consecutive_slow_sends'],expected)
            self.assertEqual(result['revoked_during_call'],expected==3)
        with self.assertRaisesRegex(RuntimeError,'permission revoked'):
            self.a.send_joints([0.]*6,self.token)
    def test_native_30ms_priming_admits_after_41ms_reset_and_five_fast_calls(self):
        b=native.J6Follower(six_axis=True,passthrough=True)
        try:
            b.login('offline',1,'test','test');self.publish()
            b.attach_process_feedback(self.state,self.joints)
            self.assertTrue(b.prepare()['prepared'])
            token=b.revision();self.assertTrue(b.arm(token));b.enter_passthrough(token)
            for index,delay in enumerate([.03]*11+[.041]+[.03]*5):
                self.publish();b.snapshot();controls.send_delay_s(delay)
                result=b.send_priming(token)
                if index==11:self.assertEqual(result['consecutive_fast'],0)
                self.publish();b.snapshot()
                self.assertEqual(b.finish_priming(token),index==16)
        finally:
            controls.send_delay_s(0);b.stop();b.logout()

    def test_single_call_over_500ms_revokes(self):
        self.publish();controls.send_delay_s(.501)
        result=self.a.send_joints([0.]*6,self.token)
        self.assertTrue(result['revoked_during_call'])

if __name__=='__main__':unittest.main()
