"""Bounded warmup, ACTIVE protection and actual-time reference regressions."""
import math
import random
import unittest
from pathlib import Path
import sys
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_aubo_follow import Clock,result
from test_aubo_six_axis import SixClient
from teleop.aubo.follow import J6FollowWorker,ObservationPending
from teleop.aubo.offline_session import Candidate
from teleop.aubo.trajectory import JointTrajectory
from teleop.aubo.quest_follow import J6QuestRuntime
from teleop.aubo.quest_preview import make_shadow
from teleop.aubo.feedback import FeedbackCache


class PrimingClient(SixClient):
    def enter_passthrough(self,token):
        self.fixed=tuple(self.q)
        return dict(result(self.clock),anchor_rad=self.fixed)
    def send_priming(self,token):return self.send_joints(self.fixed,token)
    def finish_priming(self,token):return self.permitted and token==self.rev


class PrimingTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=PrimingClient(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,clock=self.clock,command_mode='passthrough',diagnostics=True)
    def tick(self,dt=.006):
        self.clock.advance(dt);self.worker.note_input(self.clock())
        self.worker.offer(Candidate((.5,)*6,self.clock(),1));self.worker._cycle()
    def test_targets_during_warmup_never_escape_static_anchor_and_reanchor_ack_gates_active(self):
        self.worker._priming_reanchor_required=True
        for _ in range(13):self.tick()
        self.assertEqual(self.worker.status()['state'],'priming')
        self.assertTrue(self.worker.status()['priming_ready'])
        self.assertTrue(all(q==(0.,)*6 for q in self.client.sent))
        self.assertTrue(self.worker.acknowledge_priming_reanchor(1,self.clock()))
        self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'active')
        self.assertEqual(self.worker._trajectory.v,(0.,)*6)
        self.assertEqual(self.worker._trajectory.a,(0.,)*6)
        self.tick()
        self.assertLess(max(abs(q) for q in self.client.sent[-1]),math.radians(.01))
    def test_slow_frame_resets_fast_streak_even_after_minimum_steps(self):
        self.worker._priming_reanchor_required=True
        for _ in range(11):self.tick()
        self.client.send_hook=lambda:self.clock.advance(.097)
        self.tick();self.client.send_hook=None
        self.tick()
        self.assertFalse(self.worker.status()['priming_ready'])
        for _ in range(5):self.tick()
        self.assertTrue(self.worker.status()['priming_ready'])
    def test_priming_deadline_requests_recoverable_stop(self):
        self.tick();self.clock.advance(2.501);self.worker.watchdog_tick()
        self.assertFalse(self.worker.status()['fatal'])
        self.assertTrue(self.worker._stop_needed)
    def test_400ms_call_is_fatal_and_release_during_hold_cannot_activate(self):
        self.client.send_hook=lambda:self.clock.advance(.401)
        self.tick();self.assertTrue(self.worker.status()['fatal'])
    def test_priming_400ms_limit_remains_separate_from_active_500ms(self):
        self.tick()
        self.worker.parallel_state_query=True
        for age in (.35,.45):
            self.clock.advance(.6)
            snap=self.client.snapshot()
            for channel in ('joints','state'):
                snap[channel]['host_received_monotonic_s']=self.clock()-age
            snap['observation_pending']=True
            with patch.object(self.client,'snapshot',return_value=snap):
                self.worker._status['state']='priming'
                if age>.4:
                    with self.assertRaises(ObservationPending):self.worker._observe()
                else:self.worker._observe()
                self.worker._status['state']='active'
                self.worker._observe()

    def test_continuous_30ms_calls_pass_but_41ms_call_resets_fast_gate(self):
        self.worker._priming_reanchor_required=True
        self.client.send_hook=lambda:self.clock.advance(.03)
        for _ in range(14):self.tick()
        self.assertTrue(self.worker.status()['priming_ready'])
        self.client.send_hook=lambda:self.clock.advance(.041)
        self.tick();self.tick()
        self.assertFalse(self.worker.status()['priming_ready'])
        self.client.send_hook=lambda:self.clock.advance(.03)
        for _ in range(6):self.tick()
        self.assertTrue(self.worker.status()['priming_ready'])

    def test_pose_deviation_during_warmup_stops(self):
        self.tick();self.client.q=[math.radians(.11)]*6
        self.tick();self.assertTrue(self.worker.status()['fatal'])


class TrajectoryTests(unittest.TestCase):
    def test_irregular_intervals_bound_whole_segments_and_do_not_overshoot_fixed_goal(self):
        t=JointTrajectory([0.]*6,[[-3,3]]*6)
        previous=0.
        for i in range(600):
            dt=(.0078,.028,.097)[i%3]
            q,d=t.advance([math.radians(2)]*6,dt)
            self.assertGreaterEqual(q[0]+1e-12,previous)
            self.assertLessEqual(q[0],math.radians(2)+1e-12)
            self.assertLessEqual(q[0]-previous,t.velocity_rad_s*min(dt,.05)+1e-12)
            for p in t.controls:
                v=t.derivative(p,t.duration);a=t.derivative(v,t.duration);j=t.derivative(a,t.duration)
                self.assertLessEqual(max(abs(x) for _,x in t.extrema(v)),t.velocity_rad_s+1e-12)
                self.assertLessEqual(max(abs(x) for _,x in t.extrema(a)),t.acceleration_rad_s2+1e-12)
                self.assertLessEqual(max(abs(x) for _,x in t.extrema(j)),t.jerk_rad_s3+1e-12)
            previous=q[0]
        self.assertAlmostEqual(q[0],math.radians(2),places=10)
    def test_noisy_reversals_do_not_reset_position_velocity_or_acceleration(self):
        t=JointTrajectory([0.]*6,[[-3,3]]*6);rng=random.Random(42)
        for i in range(1000):
            dt=(.008,.028,.097)[i%3];oldq,oldv,olda=t.q,t.v,t.a
            q,d=t.advance([rng.uniform(-.04,.04)]*6,dt)
            effective=min(dt,.05)
            self.assertLessEqual(abs(q[0]-oldq[0]),t.velocity_rad_s*effective+1e-12)
            self.assertLessEqual(abs(t.v[0]-oldv[0]),t.acceleration_rad_s2*effective+1e-12)
            self.assertLessEqual(abs(t.a[0]-olda[0]),t.jerk_rad_s3*effective+1e-12)
    def test_long_interval_advances_at_most_50ms(self):
        a=JointTrajectory([0.]*6,[[-3,3]]*6);b=JointTrajectory([0.]*6,[[-3,3]]*6)
        self.assertEqual(a.advance([.1]*6,.19)[0],b.advance([.1]*6,.05)[0])


class ReanchorTests(unittest.TestCase):
    def test_warmup_hand_motion_is_discarded_and_no_ik_runs_until_reanchored(self):
        clock=Clock();client=PrimingClient(clock)
        anchor=(.18,.26,2.37,2.04,1.51,.8);client.q=list(anchor)
        bounds=[[-3,3]]*6
        worker=J6FollowWorker(client,anchor,j6_range_rad=(-3,3),joint_ranges_rad=bounds,
            clock=clock,command_mode='passthrough',diagnostics=True)
        shadow=make_shadow(anchor,bounds)
        shadow.clock=shadow.session.clock=clock;shadow.session.cache=FeedbackCache(clock)
        runtime=J6QuestRuntime(shadow,worker,control_dt=.005,clock=clock)
        def tick(grip,position):
            clock.advance(.006)
            runtime.submit_feedback(anchor,received_monotonic_s=clock())
            frame=dict(type='xr_frame',epoch='priming-test',seq=int(clock()*10000),t_client=clock()*1000,
                controllers={'right':dict(position=position,orientation=[0.,0.,0.,1.],
                    buttons=[dict(p=False,v=0.),dict(p=grip,v=float(grip))])})
            runtime.submit_xr_frame(frame,received_monotonic_s=clock())
            runtime.step();worker._cycle()
        with patch('vr_teleop_kit.core.quest_teleop.time.monotonic',clock):
            tick(False,[0.,0.,0.]);tick(True,[0.,0.,0.])
            self.assertEqual(worker.status()['state'],'priming')
            with patch.object(shadow.core,'get_action',side_effect=AssertionError('IK ran in warmup')):
                for _ in range(15):
                    if worker.status()['state']=='active':break
                    tick(True,[.3,.2,.1])
            self.assertEqual(worker.status()['state'],'active')
            self.assertTrue(all(q==anchor for q in client.sent))
            tick(True,[.3,.2,.1])
            self.assertLess(max(abs(a-b) for a,b in zip(client.sent[-1],anchor)),1e-5)
        runtime.close()


if __name__=='__main__':unittest.main()
