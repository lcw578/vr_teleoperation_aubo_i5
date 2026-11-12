"""Production binding checked against simulated SDK, without network or motion."""
import time
import unittest
from test_aubo_hot_feedback_native import HotFeedbackNativeTests,native,controls

class BatchNativeTests(unittest.TestCase):
    setUp=HotFeedbackNativeTests.setUp
    publish=HotFeedbackNativeTests.publish
    def tearDown(self):
        controls.watermark_raw(0)
        HotFeedbackNativeTests.tearDown(self)
    def test_vector_call_preserves_order_and_count(self):
        self.publish();self.a.query_diagnosis();before=controls.batch_counts()
        points=[[i*.0001]*6 for i in range(1,9)]
        r=self.a.send_batch(points,self.token)
        self.assertEqual(r['points_rad'],points)
        self.assertEqual(r['target_rad'],points[-1])
        self.assertEqual(controls.batch_counts(),[before[0]+1,before[1]+8])
        self.assertEqual(r['pending_upper_after_submit'],8)
        with self.assertRaisesRegex(RuntimeError,'reservation'):
            self.a.send_batch([points[-1]],self.token)
    def test_one_reservation_supports_two_four_point_calls_and_refuses_the_ninth_point(self):
        self.publish();self.a.query_diagnosis()
        self.a.send_batch([[.0001]*6]*4,self.token)
        self.a.send_batch([[.0002]*6]*4,self.token)
        with self.assertRaisesRegex(RuntimeError,'reservation'):
            self.a.send_batch([[.0003]*6],self.token)

    def test_queue_plus_batch_not_greater_than_eight(self):
        controls.watermark_raw(36);self.publish();self.a.query_diagnosis()
        with self.assertRaisesRegex(RuntimeError,'40ms'):
            self.a.send_batch([[.0001]*6]*3,self.token)
    def test_step_limit_checks_every_point_and_previous_submitted_endpoint(self):
        self.publish();self.a.query_diagnosis()
        before=controls.batch_counts()
        with self.assertRaisesRegex(RuntimeError,'step'):
            self.a.send_batch([[.0001]*6,[.002]*6],self.token)
        self.assertEqual(controls.batch_counts(),before)
    def test_invalid_count_and_stale_queue_reservation(self):
        for points in ([],[[0.]*6]*9):
            with self.assertRaises(ValueError):self.a.send_batch(points,self.token)
        self.a.query_diagnosis();time.sleep(.06);self.publish()
        with self.assertRaisesRegex(RuntimeError,'reservation'):
            self.a.send_batch([[0.]*6],self.token)

    def test_three_slow_batch_calls_revoke_without_changing_step_bound(self):
        for count in range(1,4):
            self.publish();self.a.query_diagnosis();controls.send_delay_s(.26)
            result=self.a.send_batch([[0.]*6]*4,self.token)
            self.assertEqual(result['consecutive_slow_sends'],count)
            self.assertEqual(result['revoked_during_call'],count==3)

    def test_batch_uses_same_500ms_feedback_and_call_hard_limits(self):
        self.publish(.501,0);self.a.query_diagnosis()
        with self.assertRaisesRegex(RuntimeError,'age'):
            self.a.send_batch([[0.]*6],self.token)

    def test_clock_plan_prefills_eight_then_waits_and_does_not_need_diagnosis(self):
        p=self.a.clock_batch_plan(self.token,8)
        self.assertEqual(p['point_count'],8)
        r=self.a.send_batch([[0.]*6]*8,self.token,True)
        self.assertEqual(r['clock_sent_points'],8)
        self.assertIsNone(r['pending_upper_after_submit'])
        self.assertEqual(self.a.clock_batch_plan(self.token,8)['point_count'],0)
        with self.assertRaisesRegex(RuntimeError,'wall schedule'):
            self.a.send_batch([[0.]*6]*8,self.token,True)

    def test_clock_long_stall_discards_excess_time_instead_of_replaying_large_debt(self):
        self.a.clock_batch_plan(self.token,8);self.a.send_batch([[0.]*6]*8,self.token,True)
        time.sleep(.085);self.publish()
        p=self.a.clock_batch_plan(self.token,8)
        self.assertEqual(p['point_count'],8)
        self.assertGreaterEqual(p['discarded_wall_ticks'],8)
        r=self.a.send_batch([[0.]*6]*8,self.token,True)
        self.assertEqual(r['clock_sent_points'],16)
        self.assertEqual(self.a.clock_batch_plan(self.token,8)['point_count'],0)

    def test_clock_steady_stream_tracks_wall_rate_with_bounded_cushion(self):
        first=self.a.clock_batch_plan(self.token,8);count=0;last=None
        deadline=time.monotonic()+.35
        while time.monotonic()<deadline:
            self.publish();p=self.a.clock_batch_plan(self.token,8)
            if p['point_count']:
                last=self.a.send_batch([[0.]*6]*p['point_count'],self.token,True)
                count+=p['point_count']
                ahead=count-(last['host_started_monotonic_s']-first['epoch_monotonic_s'])/.005
                self.assertLessEqual(ahead,8.01)
                self.assertGreater(ahead,3.)
            time.sleep(.002)
        self.assertEqual(last['discarded_wall_ticks'],0)
        self.assertGreater(count,65)

if __name__=='__main__':unittest.main()
