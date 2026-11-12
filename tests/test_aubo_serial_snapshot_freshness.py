"""Replay an aged serial snapshot against independently received SDK push."""
import copy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_aubo_follow import Clock, result
from test_aubo_six_axis import SixClient
from teleop.aubo.follow import J6FollowWorker


class SerialSnapshotFreshnessTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=SixClient(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=[[-3.,3.]]*6,clock=self.clock,waypoint_feedback=True,diagnostics=True)
        self.snapshot=dict(joints=result(self.clock,data=[0.]*6),state=result(self.clock,data=0))
        # Observed field ages: snapshot joints 204.613ms, state 183.810ms;
        # a push had actually been received only 25.197ms before completion.
        self.snapshot['joints'].update(source='sdk_waypoint_push',host_received_monotonic_s=self.clock()-.204613)
        self.snapshot['state']['host_received_monotonic_s']=self.clock()-.183810
        self.push=dict(q_rad=[.01]*6,sequence=1,host_received_monotonic_s=self.clock()-.025197)
        self.client.snapshot=lambda:copy.deepcopy(self.snapshot)
        self.client.waypoint_feedback=lambda:dict(samples=[copy.deepcopy(self.push)])
        self.time_patch=patch('teleop.aubo.j6_trial.time.monotonic',self.clock)
        self.time_patch.start();self.addCleanup(self.time_patch.stop)

    def test_idle_field_sequence_uses_real_new_push_without_arm_send_stop_or_fatal(self):
        with patch.object(self.client,'arm',wraps=self.client.arm) as arm:
            self.worker._cycle();arm.assert_not_called()
        state=self.worker.status()
        self.assertFalse(state['fatal']);self.assertTrue(state['feedback_ready'])
        self.assertEqual(state['state'],'idle')
        self.assertEqual(self.client.sent,[]);self.assertEqual(self.client.stops,0)
        snap=state['last_snapshot']
        self.assertEqual(snap['joints']['data'],[.01]*6)
        self.assertEqual(snap['joints']['host_received_monotonic_s'],self.push['host_received_monotonic_s'])
        self.assertEqual(snap['state']['host_received_monotonic_s'],self.snapshot['state']['host_received_monotonic_s'])
        self.assertEqual(snap['snapshot_joints_before_refresh'],self.snapshot['joints'])
        self.assertTrue(any(e['event']=='serial_snapshot_joint_refresh' for e in self.worker.diagnostic_report()['events']))

    def test_actually_stale_push_remains_rejected(self):
        self.push['host_received_monotonic_s']=self.clock()-.21
        with self.assertRaisesRegex(RuntimeError,'feedback_reception_expired'):self.worker._cycle()
        self.assertEqual(self.client.sent,[])
        self.assertFalse(self.worker.status()['feedback_ready'])

    def test_stale_state_cannot_be_repaired_by_a_new_joint_push(self):
        self.snapshot['state']['host_received_monotonic_s']=self.clock()-.201
        with patch.object(self.client,'waypoint_feedback',wraps=self.client.waypoint_feedback) as read:
            with self.assertRaisesRegex(RuntimeError,'feedback_reception_expired'):self.worker._cycle()
            read.assert_not_called()

    def test_no_push_reader_keeps_original_stale_snapshot_rejection(self):
        del self.client.waypoint_feedback
        with self.assertRaisesRegex(RuntimeError,'feedback_reception_expired'):self.worker._cycle()

    def test_future_joint_timestamp_remains_invalid(self):
        self.snapshot['joints']['host_received_monotonic_s']=self.clock()+.001
        with self.assertRaisesRegex(RuntimeError,'feedback_reception_expired'):self.worker._cycle()

    def test_invalid_old_joint_or_controller_state_not_hidden_by_new_push(self):
        self.snapshot['joints']['data']=[4.]*6
        with self.assertRaisesRegex(RuntimeError,'outside_controller_ranges'):self.worker._cycle()
        self.snapshot['joints']['data']=[0.]*6
        self.snapshot['state']['data']=2
        with self.assertRaisesRegex(RuntimeError,'Unexpected controller state'):self.worker._cycle()

    def test_state_aging_out_during_refresh_is_still_rejected(self):
        def delayed_read():
            self.clock.advance(.03)
            return dict(samples=[copy.deepcopy(self.push)])
        self.client.waypoint_feedback=delayed_read
        with self.assertRaisesRegex(RuntimeError,'feedback_reception_expired'):self.worker._cycle()

    def test_fresh_snapshot_needs_no_extra_push_read(self):
        self.snapshot['joints']['host_received_monotonic_s']=self.clock()-.01
        with patch.object(self.client,'waypoint_feedback',wraps=self.client.waypoint_feedback) as read:
            self.worker._cycle();read.assert_not_called()
        self.assertEqual(self.worker.status()['last_snapshot']['joints']['data'],[0.]*6)

    def test_stop_confirmation_preserves_existing_strict_feedback_path(self):
        with patch.object(self.client,'waypoint_feedback',wraps=self.client.waypoint_feedback) as read:
            with self.assertRaisesRegex(RuntimeError,'stop_feedback_reception_expired'):
                self.worker._observe(stop_deadline=self.clock()+1.)
            read.assert_not_called()


if __name__=='__main__':unittest.main()
