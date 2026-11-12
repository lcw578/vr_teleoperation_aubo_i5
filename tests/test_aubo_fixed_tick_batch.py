"""Uniform sample continuity, queue bound enforcement, and hot release behavior."""
import math
import unittest
from unittest.mock import Mock
from unittest.mock import patch
from pathlib import Path
import tempfile
from test_aubo_hot_clutch import CountClient
from test_aubo_follow import Clock,result
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.trajectory import JointTrajectory
from teleop.aubo.offline_session import Candidate

class FixedTickTrajectoryTests(unittest.TestCase):
    def test_batches_are_identical_to_each_uniform_tick_across_replans_and_braking(self):
        a=JointTrajectory([0.]*6,[[-3,3]]*6)
        b=JointTrajectory([0.]*6,[[-3,3]]*6)
        for index in range(100):
            goal=[.08*math.sin(index*.11)]*6
            points,detail=a.sample_fixed_ticks(goal,8)
            expected=[b.advance(goal,.005)[0] for _ in range(8)]
            self.assertEqual(points,tuple(expected))
            self.assertEqual((a.q,a.v,a.a),(b.q,b.v,b.a))
        ga,_=a.brake();gb,_=b.brake();self.assertEqual(ga,gb)
        while not a.stopped:
            points,_=a.sample_fixed_ticks(ga,8)
            self.assertEqual(points,tuple(b.advance(gb,.005)[0] for _ in range(8)))
    def test_step_bound_and_first_frame_exact_anchor(self):
        t=JointTrajectory([0.]*6,[[-3,3]]*6);previous=t.q
        for n in (1,3,8,7,8)*20:
            points,detail=t.sample_fixed_ticks([.2]*6,n,first_anchor=previous==(0.,)*6)
            for point in points:
                self.assertLessEqual(max(abs(a-b) for a,b in zip(point,previous)),t.velocity_rad_s*.005+1e-10)
                previous=point
        for n in (0,9,-1):
            with self.assertRaises(ValueError):t.sample_fixed_ticks([0.]*6,n)

class ProgramSineTests(unittest.TestCase):
    def test_high_flow_reference_and_brake_handoff_preserve_derivatives(self):
        from scripts.aubo_program_sine import ProgramSineTrajectory
        t=ProgramSineTrajectory([0.]*6,[[-3,3]]*6,axis=0,center=[0.]*6)
        previous=(t.q,t.v,t.a);peak=0.;low=0.;high=0.
        for _ in range(1450):
            t.advance([0.]*6,.005)
            for old,new,limit in zip(previous,(t.q,t.v,t.a),(t.velocity_rad_s,t.acceleration_rad_s2,t.jerk_rad_s3)):
                self.assertLessEqual(max(abs(a-b) for a,b in zip(old,new)),limit*.005+1e-9)
            previous=(t.q,t.v,t.a);peak=max(peak,math.degrees(abs(t.v[0])))
            low=min(low,math.degrees(t.q[0]));high=max(high,math.degrees(t.q[0]))
        self.assertGreater(peak,8.9);self.assertLess(peak,9.1)
        self.assertLess(low,-5.9);self.assertGreater(high,5.9)
        before=(t.q,t.v,t.a);goal,_=t.brake()
        self.assertEqual(before,(t.q,t.v,t.a))
        for _ in range(600):
            t.advance(goal,.005)
            if t.stopped:break
        self.assertTrue(t.stopped)

class BatchClient(CountClient):
    def __init__(self,clock):super().__init__(clock);self.raw=0;self.batches=[]
    def query_diagnosis(self):return dict(result(self.clock),data=dict(buffer_data_raw=self.raw))
    def send_batch(self,points,token):
        self.batches.append(points)
        r=self.send_joints(points[-1],token)
        return dict(r,point_count=len(points),points_rad=points)

class FixedTickWorkerTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=BatchClient(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,clock=self.clock,command_mode='passthrough',
            waypoint_feedback=True,state_observer=Mock(),fixed_tick_batch=True,batch_point_limit=8)
        w=self.worker;self.client.arm(0);w._token=0;w._active_session=1;w._may_move=True
        w._priming_anchor=(0.,)*6;w._status.update(state='active',active_since_monotonic_s=self.clock()-.1)
        w._first_active_hold=True;w._last_sent_q=(0.,)*6;w._last_send_s=self.clock()-.01
        w._input_s=self.clock();w.offer(Candidate((.1,)*6,self.clock(),1))
    def test_observed_pending_slots_bound_total_not_only_single_batch(self):
        self.client.raw=30
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(len(self.client.batches[-1]),3)
        self.assertEqual(self.client.batches[-1][0],(0.,)*6)
        self.clock.advance(.04)
        self.client.raw=48;before=self.worker._trajectory.q
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(len(self.client.batches),1)
        self.assertEqual(self.worker._trajectory.q,before)
    def test_capacity_credit_can_be_split_without_another_query_but_never_invented(self):
        self.worker.batch_point_limit=4
        self.client.query_diagnosis=Mock(wraps=self.client.query_diagnosis)
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(self.client.query_diagnosis.call_count,1)
        self.assertEqual([len(b) for b in self.client.batches],[4,4])
        self.client.raw=48
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(self.client.query_diagnosis.call_count,2)
        self.assertEqual(len(self.client.batches),2)
        self.clock.advance(.04);self.client.raw=24
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(len(self.client.batches),3)

    def test_hard_watermark_fault_revokes_and_release_during_query_emits_no_active_batch(self):
        self.client.raw=54;self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertTrue(self.worker.status()['fatal'])

        self.assertEqual(self.client.batches,[])
    def test_release_during_capacity_request_cannot_send_old_active_goal(self):
        original=self.client.query_diagnosis
        def query():self.worker.revoke('grip_released');return original()
        self.client.query_diagnosis=query
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(self.client.batches,[])
        self.assertEqual(self.worker.status()['state'],'braking')

    def test_intentional_stop_during_batch_does_not_become_fatal(self):
        original=self.client.send_batch
        def send(points,token):
            result=original(points,token)
            self.worker.revoke('staged_acceptance_complete')
            return dict(result,revoked_during_call=True)
        self.client.send_batch=send
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertTrue(self.worker._stop_needed)
        self.assertFalse(self.worker.status()['fatal'])

    def test_unexpected_revoke_during_batch_is_fatal(self):
        original=self.client.send_batch
        self.client.send_batch=lambda points,token:dict(original(points,token),revoked_during_call=True)
        self.worker._fixed_tick_send((.1,)*6,'active')
        self.assertTrue(self.worker.status()['fatal'])

    def test_wall_clock_path_never_queries_stale_watermark_and_passes_clock_guard(self):
        w=self.worker;w.wall_clock_batch=True
        self.client.query_diagnosis=Mock(side_effect=AssertionError('No motion diagnosis RPC'))
        self.client.clock_batch_plan=Mock(return_value=dict(point_count=8,discarded_wall_ticks=0,
            next_due_monotonic_s=self.clock()+.02))
        self.client.send_batch=Mock(return_value=dict(result(self.clock),clock_epoch_monotonic_s=self.clock(),
            clock_sent_points=8,point_count=8))
        w._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(self.client.send_batch.call_args.args[2],True)
        self.assertEqual(len(self.client.send_batch.call_args.args[0]),8)
        self.client.query_diagnosis.assert_not_called()
        self.assertFalse(w.status()['fatal'])
        w._wanted=None;w._status['state']='braking'
        self.assertAlmostEqual(w._wait_timeout_s(),.02)

    def test_release_during_wall_plan_cannot_submit_old_active_points(self):
        w=self.worker;w.wall_clock_batch=True
        def plan(*args):
            w.revoke('grip_released')
            return dict(point_count=8,discarded_wall_ticks=0,next_due_monotonic_s=self.clock())
        self.client.clock_batch_plan=plan
        w._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(self.client.batches,[])
        self.assertEqual(w.status()['state'],'braking')

class FixedTickQualificationTests(unittest.TestCase):
    def test_unqualified_batch_cannot_reach_native_login_from_vr(self):
        from teleop.aubo.quest_follow import main
        with tempfile.TemporaryDirectory() as folder:
            Path(folder,'follow-build.json').write_text('{"fixed_tick_batch_candidate":{"execution_quality_qualified":false}}')
            native=Mock()
            with patch('teleop.aubo.quest_follow.load_follow_native',return_value=native), \
                 patch('teleop.aubo.quest_follow.check_relay_port'):
                with self.assertRaisesRegex(RuntimeError,'physical smoothness'):
                    main(['--execute-six-axis','--fixed-tick-batch','--waypoint-feedback',
                          '--separate-waypoint-connection','--runtime',folder,
                          '--host','127.0.0.1','--port','8899','--user','offline','--usb','--headless'])
            native.J6Follower.assert_not_called()

if __name__=='__main__':unittest.main()
