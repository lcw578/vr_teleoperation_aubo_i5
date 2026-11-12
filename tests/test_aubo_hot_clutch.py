"""Hot clutch lifecycle, whole-curve stop bounds, and release/stop races."""
import math
import random
import unittest
from unittest.mock import patch, Mock
from test_aubo_priming import PrimingClient
from test_aubo_follow import Clock
from teleop.aubo.follow import J6FollowWorker, ObservationPending
from teleop.aubo.offline_session import Candidate
from teleop.aubo.trajectory import JointTrajectory
from teleop.aubo.quest_follow import J6QuestRuntime
from teleop.aubo.quest_preview import make_shadow
from teleop.aubo.feedback import FeedbackCache


class CountClient(PrimingClient):
    entries=0
    def enter_passthrough(self,token):
        self.entries+=1
        return super().enter_passthrough(token)


class HotTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=CountClient(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,clock=self.clock,command_mode='passthrough',diagnostics=True)
    def tick(self,target=None,session=1,dt=.008):
        self.clock.advance(dt);self.worker.note_input(self.clock())
        if target is not None:self.worker.offer(Candidate(tuple(target),self.clock(),session))
        self.worker._cycle()
    def active(self):
        for _ in range(30):
            self.tick([0.]*6)
            if self.worker.status()['state']=='active':return
        self.fail(self.worker.status())
    def holding(self):
        for _ in range(375):
            self.tick()
            if self.worker.status()['state']=='holding':return
        self.fail(self.worker.status())
    def test_five_moving_clutches_enter_once_and_reanchor_at_zero_velocity(self):
        self.active()
        for cycle in range(5):
            for _ in range(150):self.tick([.03*(-1)**cycle]*6,cycle+1)
            self.worker.revoke('grip_released')
            self.assertEqual(self.worker.status()['state'],'braking')
            self.assertTrue(self.client.permitted)
            self.assertFalse(self.worker.offer(Candidate((.8,)*6,self.clock(),99)))
            self.assertFalse(self.worker.resume_clutch(cycle+2,self.clock()))
            self.holding()
            for _ in range(80):self.tick()
            held=self.worker._hold_q
            self.assertTrue(all(q==held for q in self.client.sent[-20:]))
            self.assertTrue(self.worker.resume_clutch(cycle+2,self.clock()))
            self.assertEqual(self.worker._trajectory.q,held)
            self.assertEqual(self.worker._trajectory.v,(0.,)*6)
            self.assertEqual(self.worker._trajectory.a,(0.,)*6)
            self.tick([.2]*6,cycle+2)
            self.assertEqual(self.client.sent[-1],held)
            self.assertEqual(self.client.stops,0)
        self.assertEqual(self.client.entries,1)
    def test_holding_drift_is_fatal(self):
        self.active();self.worker.revoke('grip_released');self.holding()
        self.client.q[0]+=.002
        self.tick(dt=.11)
        self.assertTrue(self.worker.status()['fatal'])
        self.assertEqual(self.worker.status()['fault'],'hot_holding_pose_deviation')
    def test_small_holding_motion_blocks_resume_until_new_quiet_window(self):
        self.active();self.worker.revoke('grip_released');self.holding()
        self.client.q[0]+=math.radians(.03)
        self.tick(dt=.11)
        self.assertFalse(self.worker.status()['hot_ready'])
        self.assertFalse(self.worker.resume_clutch(2,self.clock()))
        self.assertFalse(self.worker.status()['fatal'])

    def test_safety_revoke_during_brake_send_is_never_reauthorized(self):
        self.active()
        for _ in range(12):self.tick([.06]*6)
        self.worker.revoke('grip_released')
        self.client.send_hook=lambda:self.worker.revoke('input_disconnected')
        self.tick()
        self.assertTrue(self.worker._stop_needed)
        self.assertFalse(self.client.permitted)
        self.assertFalse(self.worker.resume_clutch(2,self.clock()))

    def test_hot_phases_tolerate_transient_ages_and_latch_hard_limits(self):
        self.active()
        for phase in ('active','braking','holding'):
            for joint_age,state_age,accepted in ((.201,.201,True),(.30258,.3189,True),(.499,.499,True),
                    (.501,.1,False),(.1,.501,False)):
                with self.subTest(phase=phase,joint_age=joint_age,state_age=state_age):
                    self.worker._status['state']=phase
                    self.clock.advance(.6)
                    snap=self.client.snapshot()
                    snap['joints']['host_received_monotonic_s']=self.clock()-joint_age
                    snap['state']['host_received_monotonic_s']=self.clock()-state_age
                    snap['observation_pending']=True
                    self.worker.parallel_state_query=True
                    with patch.object(self.client,'snapshot',return_value=snap):
                        if accepted:
                            self.worker._observe()
                            self.assertTrue(self.client.permitted)
                        else:
                            with self.assertRaises(ObservationPending):self.worker._observe()
        self.worker._handle_pending_observation(ObservationPending('old',snap))
        self.assertTrue(self.worker.status()['fatal'])
        self.assertFalse(self.client.permitted)

    def test_slow_calls_tolerated_streak_resets_and_third_slow_latches(self):
        self.active()
        for duration,streak in ((.26,1),(.4,2),(.02,0),(.26,1),(.26,2),(.26,3)):
            self.client.send_hook=lambda:self.clock.advance(duration)
            self.tick([.02]*6)
            self.assertEqual(self.worker.status()['consecutive_slow_sends'],streak)
            self.assertEqual(self.worker.status()['fatal'],streak==3)
            self.assertEqual(self.client.permitted,streak!=3)

    def test_single_500ms_exceeded_call_latches(self):
        self.active()
        self.client.send_hook=lambda:self.clock.advance(.501)
        self.tick([.02]*6)
        self.assertTrue(self.worker.status()['fatal'])
        self.assertFalse(self.client.permitted)

    def test_held_press_waits_for_fresh_state_without_cold_restart(self):
        self.active();self.worker.revoke('grip_released');self.holding()
        for _ in range(80):self.tick()
        original=self.client.snapshot
        def delayed_snapshot():
            snap=original()
            snap['state']['host_received_monotonic_s']=self.clock()-.28
            return snap
        with patch.object(self.client,'snapshot',side_effect=delayed_snapshot):
            self.tick(dt=.11)
            self.assertFalse(self.worker.status()['hot_ready'])
            self.assertFalse(self.worker.resume_clutch(2,self.clock()))
            self.assertTrue(self.client.permitted)
            self.assertFalse(self.worker.status()['fatal'])
        self.tick()
        self.assertTrue(self.worker.resume_clutch(2,self.clock()))
        self.assertEqual(self.client.entries,1)

    def test_30ms_hot_calls_admit_and_feedback_gates_use_250ms_120ms(self):
        self.active();self.worker.revoke('grip_released');self.holding()
        for _ in range(80):self.tick()
        self.client.send_hook=lambda:self.clock.advance(.041)
        self.tick()
        self.assertEqual(self.worker._hot_fast,0)
        self.client.send_hook=lambda:self.clock.advance(.03)
        for _ in range(5):self.tick()
        self.assertEqual(self.worker._hot_fast,5)
        now=self.clock();snap=self.worker.status()['last_snapshot']
        for state_age,joint_age,ready in ((.24,.11,True),(.251,.11,False),(.24,.121,False)):
            adjusted={key:dict(value) for key,value in snap.items() if key in ('state','joints')}
            adjusted['state']['host_received_monotonic_s']=now-state_age
            adjusted['joints']['host_received_monotonic_s']=now-joint_age
            self.assertEqual(self.worker._hot_admissible(now,adjusted),ready)

    def test_B_uses_200ms_only_in_hot_phases_and_50ms_for_cold_and_stop(self):
        observer=Mock()
        worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,clock=self.clock,command_mode='passthrough',
            waypoint_feedback=True,state_observer=observer)
        query=worker.state_query
        for phase in ('idle','arming','priming','active','braking','holding','stopping'):
            worker._status['state']=phase
            hot=phase in ('active','braking','holding')
            self.assertEqual(query.is_active(),hot)
            self.assertEqual(query.active_period_s if query.is_active() else query.idle_period_s,
                             .2 if hot else .05)
        # Run the real scheduler to verify start-to-start timing for a hot phase.
        worker._status['state']='active'
        def query_once():
            start=self.clock();self.clock.advance(.017)
            query._last_result={'host_started_monotonic_s':start}
        query.query_once=query_once
        def wait(delay):
            self.assertAlmostEqual(delay,.183)
            query._quit.set()
        query._wake=Mock();query._wake.wait.side_effect=wait
        query._run()

    def test_input_disconnect_exits_hot_instead_of_resuming(self):
        self.active();self.worker.revoke('grip_released');self.holding()
        self.worker.revoke('input_disconnected')
        self.assertFalse(self.client.permitted)
        self.assertTrue(self.worker._stop_needed)
        self.assertFalse(self.worker.resume_clutch(2,self.clock()))
    def test_holding_idle_exits_and_cold_cooldown_blocks_entry(self):
        self.active();self.worker.revoke('grip_released');self.holding()
        self.worker.holding_idle_s=.04
        for _ in range(6):
            if self.worker._stop_needed:break
            self.tick()
        self.assertTrue(self.worker._stop_needed)
        with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
            self.worker._cycle()
        self.assertTrue(self.worker.status()['stop_confirmed'])
        self.assertFalse(self.worker.status()['fatal'])
        self.tick([0.]*6,2)
        self.assertEqual(self.client.entries,1)
        self.clock.advance(.5);self.tick([0.]*6,2)
        self.assertEqual(self.client.entries,2)
    def test_timeout_recovers_only_after_stop_and_cooldown(self):
        self.tick([0.]*6)
        self.clock.advance(2.501);self.worker.note_input(self.clock())
        self.worker.watchdog_tick()
        self.assertFalse(self.worker.status()['fatal'])
        self.assertTrue(self.worker._stop_needed)
        with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
            self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'idle')
        self.clock.advance(.5);self.tick([0.]*6,2)
        self.assertEqual(self.worker.status()['state'],'priming')


class BrakeTests(unittest.TestCase):
    def test_moving_multi_axis_brakes_preserve_qva_and_certify_whole_curve(self):
        rng=random.Random(34)
        for _ in range(80):
            t=JointTrajectory([0.]*6,[[-3,3]]*6)
            for _ in range(rng.randrange(3,100)):
                t.advance([rng.uniform(-.1,.1) for _ in range(6)],.008)
            q,v,a=t.q,t.v,t.a
            goal,duration=t.brake()
            self.assertEqual((q,v,a),(t.q,t.v,t.a))
            for i,p in enumerate(t.controls):
                self.assertAlmostEqual(t.evaluate(p,0),q[i],places=11)
                dv=t.derivative(p,duration);da=t.derivative(dv,duration);dj=t.derivative(da,duration)
                self.assertAlmostEqual(t.evaluate(dv,0),v[i],places=10)
                self.assertAlmostEqual(t.evaluate(da,0),a[i],places=9)
                self.assertLessEqual(max(abs(x) for _,x in t.extrema(dv)),t.velocity_rad_s+1e-9)
                self.assertLessEqual(max(abs(x) for _,x in t.extrema(da)),t.acceleration_rad_s2+1e-9)
                self.assertLessEqual(max(abs(x) for _,x in t.extrema(dj)),t.jerk_rad_s3+1e-8)
            for _ in range(450):
                t.advance(goal,.008)
                if t.stopped:break
            self.assertTrue(t.stopped)
            self.assertEqual(t.v,(0.,)*6);self.assertEqual(t.a,(0.,)*6)
    def test_no_stop_within_joint_bounds_raises_without_changing_curve(self):
        t=JointTrajectory([.999]*6,[[-1,1]]*6)
        t.v=(.1,)*6
        with self.assertRaises(RuntimeError):t.brake()
        self.assertEqual(t.q,(.999,)*6)


class HotVRTests(unittest.TestCase):
    def test_refined_rapid_regrip_during_braking_reanchors_without_ik_or_wait(self):
        clock=Clock();client=CountClient(clock)
        anchor=(.18,.26,2.37,2.04,1.51,.8);client.q=list(anchor);bounds=[[-3,3]]*6
        worker=J6FollowWorker(client,anchor,j6_range_rad=(-3,3),joint_ranges_rad=bounds,
            clock=clock,command_mode='passthrough',diagnostics=True)
        shadow=make_shadow(anchor,bounds)
        shadow.clock=shadow.session.clock=clock;shadow.session.cache=FeedbackCache(clock)
        runtime=J6QuestRuntime(shadow,worker,control_dt=.005,clock=clock)
        def tick(grip,position):
            clock.advance(.008)
            runtime.submit_feedback(client.q,received_monotonic_s=clock())
            frame=dict(type='xr_frame',epoch='immediate-hot',seq=int(clock()*10000),t_client=clock()*1000,
                controllers={'right':dict(position=position,orientation=[0.,0.,0.,1.],
                    buttons=[dict(p=False,v=0.),dict(p=grip,v=float(grip))])})
            runtime.submit_xr_frame(frame,received_monotonic_s=clock());runtime.step();worker._cycle()
        with patch('vr_teleop_kit.core.quest_teleop.time.monotonic',clock):
            tick(False,[0.,0.,0.]);tick(True,[0.,0.,0.])
            for _ in range(40):
                if worker.status()['state']=='active':break
                tick(True,[0.,0.,0.])
            worker.refined_stream=True
            for cycle in range(12):
                for j in range(8):tick(True,[.001*j,0.,0.])
                tick(False,[.008,0.,0.])
                self.assertEqual(worker.status()['state'],'braking')
                before=(worker._trajectory.q,worker._trajectory.v,worker._trajectory.a)
                worker._hot_fast=0;worker._status['hot_ready']=False
                with patch.object(shadow.core,'get_action',side_effect=AssertionError('No IK before reanchor')):
                    tick(True,[.012,0.,0.])
                self.assertEqual(worker.status()['state'],'active')
                event=next(e for e in reversed(worker.diagnostic_report()['events']) if e['event']=='hot_clutch_resumed')
                self.assertEqual(tuple(event['q_anchor_rad']),before[0])
                self.assertEqual(tuple(event['v_anchor_rad_s']),before[1])
                self.assertEqual(tuple(event['a_anchor_rad_s2']),before[2])
                self.assertFalse(worker.status()['fatal'])
            self.assertEqual(client.entries,1);self.assertEqual(client.stops,0)
        runtime.close()

    def test_hand_motion_during_braking_is_discarded_without_ik(self):
        clock=Clock();client=CountClient(clock)
        anchor=(.18,.26,2.37,2.04,1.51,.8);client.q=list(anchor);bounds=[[-3,3]]*6
        worker=J6FollowWorker(client,anchor,j6_range_rad=(-3,3),joint_ranges_rad=bounds,
            clock=clock,command_mode='passthrough',diagnostics=True)
        shadow=make_shadow(anchor,bounds)
        shadow.clock=shadow.session.clock=clock;shadow.session.cache=FeedbackCache(clock)
        runtime=J6QuestRuntime(shadow,worker,control_dt=.005,clock=clock)
        def tick(grip,position):
            clock.advance(.008)
            runtime.submit_feedback(client.q,received_monotonic_s=clock())
            frame=dict(type='xr_frame',epoch='hot-test',seq=int(clock()*10000),t_client=clock()*1000,
                controllers={'right':dict(position=position,orientation=[0.,0.,0.,1.],
                    buttons=[dict(p=False,v=0.),dict(p=grip,v=float(grip))])})
            runtime.submit_xr_frame(frame,received_monotonic_s=clock());runtime.step();worker._cycle()
        with patch('vr_teleop_kit.core.quest_teleop.time.monotonic',clock):
            tick(False,[0.,0.,0.]);tick(True,[0.,0.,0.])
            for _ in range(40):
                if worker.status()['state']=='active':break
                tick(True,[.1,.2,.3])
            self.assertEqual(worker.status()['state'],'active')
            for cycle in range(5):
                tick(False,[.1,.2,.3])
                if cycle==0:
                    original=client.snapshot
                    def delayed_state():
                        snap=original()
                        snap['state']['host_received_monotonic_s']=clock()-.28
                        return snap
                    with patch.object(client,'snapshot',side_effect=delayed_state):
                        for _ in range(80):tick(True,[.4,.5,.6])
                    self.assertEqual(worker.status()['state'],'holding')
                    self.assertTrue(client.permitted)
                    self.assertFalse(worker.status()['fatal'])
                with patch.object(shadow.core,'get_action',side_effect=AssertionError('IK during hot clutch')):
                    for _ in range(80):
                        if worker.status()['state']=='active':break
                        tick(True,[.4,.5,.6])
                self.assertEqual(worker.status()['state'],'active')
                self.assertEqual(client.entries,1);self.assertEqual(client.stops,0)
                self.assertEqual(client.sent[-1],tuple(worker.status()['active_anchor_rad']))
                tick(True,[.4,.5,.6])
                self.assertLess(max(abs(a-b) for a,b in zip(client.sent[-1],anchor)),1e-12)
        runtime.close()


if __name__=='__main__':unittest.main()
