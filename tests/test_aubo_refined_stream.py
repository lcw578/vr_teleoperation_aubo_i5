"""No hardware: stream pause barrier, underrun handshake and native freshness."""
import threading,time,unittest,math
from unittest.mock import Mock
from teleop.aubo.state_query import StateQueryWorker
from teleop.aubo.follow import J6FollowWorker
from test_aubo_fixed_tick_batch import FixedTickWorkerTests
from test_aubo_hot_feedback_native import native,controls
from test_aubo_state_query import result

class PausedStateTests(unittest.TestCase):
    def test_pause_waits_for_inflight_and_blocks_all_new_queries_until_resume(self):
        entered=threading.Event();release=threading.Event();paused=threading.Event()
        client=Mock()
        def call():
            start=time.monotonic();entered.set();release.wait(.3);return result(start)
        client.query_state.side_effect=call
        q=StateQueryWorker(client,on_failure=Mock(),is_active=lambda:True,diagnose=Mock(),active_period_s=.01)
        q._thread.start();self.assertTrue(entered.wait(.2))
        t=threading.Thread(target=lambda:(q.pause(),paused.set()));t.start()
        try:
            self.assertFalse(paused.wait(.02));release.set();self.assertTrue(paused.wait(.3))
            count=client.query_state.call_count;time.sleep(.06)
            q.query_once();self.assertEqual(client.query_state.call_count,count)
            stamp=q.status()['last_result']['host_received_monotonic_s'];time.sleep(.02)
            self.assertEqual(q.status()['last_result']['host_received_monotonic_s'],stamp)
            q.resume();deadline=time.monotonic()+.2
            while client.query_state.call_count==count and time.monotonic()<deadline:time.sleep(.002)
            self.assertGreater(client.query_state.call_count,count)
        finally:release.set();t.join(.5);q.close()

class RefinedNativeTests(unittest.TestCase):
    def setUp(self):
        controls.send_delay_s(0);self.a=native.J6Follower(six_axis=True,passthrough=True)
        self.a.login('offline',1,'test','test')
        self.state=native.RobotStateCache();self.joints=native.WaypointFeedbackCache('C');self.publish()
        self.a.attach_process_feedback(self.state,self.joints);self.assertTrue(self.a.prepare()['prepared'])
        self.a.configure_refined_stream();self.token=self.a.revision();self.assertTrue(self.a.arm(self.token))
        self.a.enter_passthrough(self.token)
        for _ in range(12):self.publish();self.a.send_priming(self.token)
        self.publish();self.assertTrue(self.a.finish_priming(self.token))
    def publish(self,ja=0,sa=0):
        now=time.monotonic();sg=self.state.invalidate();jg=self.joints.invalidate()
        self.state.publish(0,0,now-sa-.001,now-sa,sg);self.joints.publish([0.]*6,now-ja,1,jg)
    def tearDown(self):
        controls.send_delay_s(0);self.a.stop();self.a.logout()
    def test_twelve_point_cushion_two_point_cadence(self):
        p=self.a.clock_batch_plan(self.token,8);self.assertEqual(p['cushion_points'],12);self.assertEqual(p['minimum_points'],2)
        self.a.send_batch([[0.]*6]*8,self.token,True)
        self.assertGreaterEqual(self.a.clock_batch_plan(self.token,8)['point_count'],4)
        self.a.send_batch([[0.]*6]*4,self.token,True)
        time.sleep(.011);self.publish()
        self.assertGreaterEqual(self.a.clock_batch_plan(self.token,8)['point_count'],2)
    def test_stale_state_allowed_only_in_stream_stale_joints_still_revoke(self):
        self.publish(sa=10);s=self.a.snapshot();self.assertFalse(s.get('observation_pending',False));self.assertTrue(s['state_polling_suppressed'])
        self.a.clock_batch_plan(self.token,8);self.a.send_batch([[0.]*6]*8,self.token,True)
        self.publish(ja=.501,sa=10)
        with self.assertRaisesRegex(RuntimeError,'age'):self.a.send_batch([[0.]*6]*2,self.token,True)
    def test_stop_restores_state_age_and_old_state_cannot_arm(self):
        self.publish(sa=10);self.a.stop();self.assertTrue(self.a.snapshot().get('observation_pending'))
        self.assertFalse(self.a.arm(self.a.revision()))
    def test_refined_priming_uses_requested_500ms_joint_limit(self):
        b=native.J6Follower(six_axis=True,passthrough=True)
        try:
            b.login('offline',1,'test','test');self.publish()
            b.attach_process_feedback(self.state,self.joints);self.assertTrue(b.prepare()['prepared'])
            b.configure_refined_stream();token=b.revision();self.assertTrue(b.arm(token));b.enter_passthrough(token)
            self.publish(ja=.45,sa=10)
            self.assertFalse(b.snapshot().get('observation_pending',False))
            self.assertEqual(b.send_priming(token)['return_code'],0)
            self.publish(ja=.501,sa=10)
            with self.assertRaisesRegex(RuntimeError,'Priming age'):
                b.send_priming(token)
        finally:b.stop();b.logout()
    def test_underrun_requires_explicit_restart_and_fresh_endpoint(self):
        self.a.clock_batch_plan(self.token,8);self.a.send_batch([[0.]*6]*8,self.token,True)
        self.a.clock_batch_plan(self.token,8);self.a.send_batch([[0.]*6]*4,self.token,True)
        time.sleep(.09);self.publish(sa=10)
        p=self.a.clock_batch_plan(self.token,8);self.assertTrue(p['recovery_required']);self.assertEqual(p['point_count'],0)
        with self.assertRaisesRegex(RuntimeError,'recovery required'):self.a.send_batch([[0.]*6]*2,self.token,True)
        self.publish(ja=.501,sa=10)
        with self.assertRaisesRegex(RuntimeError,'not valid or fresh'):self.a.restart_clock_from_rest(self.token)
        self.publish(ja=.2,sa=10);ack=self.a.restart_clock_from_rest(self.token)
        self.assertEqual(ack['restart_count'],1);self.assertEqual(ack['q_restart_rad'],[0.]*6)
        p=self.a.clock_batch_plan(self.token,8);self.assertFalse(p['recovery_required']);self.assertEqual(p['point_count'],2)

    def test_stationary_debt_rebases_to_two_without_recovery(self):
        self.a.clock_batch_plan(self.token,8,True)
        self.a.send_batch([[0.]*6]*8,self.token,True)
        self.a.clock_batch_plan(self.token,8,True)
        self.a.send_batch([[0.]*6]*4,self.token,True)
        time.sleep(.09);self.publish(sa=10)
        p=self.a.clock_batch_plan(self.token,8,True)
        self.assertFalse(p['recovery_required']);self.assertTrue(p['stationary_rebased'])
        self.assertEqual(p['point_count'],2)
        self.a.send_batch([[0.]*6]*2,self.token,True)

    def test_delayed_partial_prefill_does_not_repeat_recovery(self):
        self.a.clock_batch_plan(self.token,8)
        controls.send_delay_s(.065)
        self.a.send_batch([[0.]*6]*8,self.token,True)
        controls.send_delay_s(0);self.publish(sa=10)
        p=self.a.clock_batch_plan(self.token,8)
        self.assertGreater(p['due_points'],12);self.assertFalse(p['recovery_required'])
        self.assertFalse(p['warmup_complete'])
        self.a.send_batch([[0.]*6]*4,self.token,True)
        time.sleep(.07);self.publish(sa=10)
        self.assertTrue(self.a.clock_batch_plan(self.token,8)['recovery_required'])
        self.a.restart_clock_from_rest(self.token)
        for _ in range(5):
            self.publish(sa=10);p=self.a.clock_batch_plan(self.token,8)
            self.assertFalse(p['recovery_required']);self.assertLessEqual(p['point_count'],4)
            controls.send_delay_s(.045)
            self.a.send_batch([[0.]*6]*p['point_count'],self.token,True)
        controls.send_delay_s(0)

class RefinedWorkerTests(unittest.TestCase):
    def test_confirmed_exit_discards_old_recovery_before_new_cold_session(self):
        from test_aubo_hot_clutch import HotTests
        from unittest.mock import patch
        fixture=HotTests();fixture.setUp();fixture.active()
        w=fixture.worker;clock=fixture.clock
        w._underrun=dict(started_s=clock(),endpoint=(0.,)*6)
        w._next_batch_s=clock()-1;w._status['underrun_recovering']=True
        w.revoke('invalid_xr_frame')
        with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:clock.advance(.051)):
            w._cycle()
        self.assertTrue(w.status()['stop_confirmed']);self.assertFalse(w.status()['fatal'])
        self.assertIsNone(w._underrun);self.assertIsNone(w._next_batch_s)
        self.assertFalse(w.status()['underrun_recovering'])
        clock.advance(1.);fixture.active()
        self.assertEqual(fixture.client.entries,2)
        self.assertIsNone(w._underrun)
        self.assertFalse(w.status()['fatal'])

    def test_refined_resume_during_braking_preserves_q_v_a_and_ignores_fast_gate(self):
        from test_aubo_hot_clutch import HotTests
        fixture=HotTests();fixture.setUp();fixture.active()
        for _ in range(100):fixture.tick([.2]*6)
        w=fixture.worker;clock=fixture.clock;w.refined_stream=True
        w.revoke('grip_released')
        self.assertEqual(w.status()['state'],'braking')
        before=(w._trajectory.q,w._trajectory.v,w._trajectory.a)
        self.assertGreater(max(map(abs,before[1])),0)
        w._hot_fast=0;w._hot_quiet_s=None;w._status['hot_ready']=False
        snap=fixture.client.snapshot();snap['joints']['host_received_monotonic_s']=clock()-.3
        w._status['last_snapshot']=snap
        anchor=Mock(return_value=True)
        self.assertTrue(w.resume_clutch(2,clock(),reanchor=anchor))
        anchor.assert_called_once_with(before[0])
        self.assertEqual((w._trajectory.q,w._trajectory.v,w._trajectory.a),before)
        self.assertEqual(w.status()['state'],'active');self.assertFalse(w._brake_pending)
        q,_=w._trajectory.advance([.15]*6,.005)
        self.assertLessEqual(max(abs(a-b) for a,b in zip(q,before[0])),math.radians(.05)+1e-10)

    def test_refined_resume_still_rejects_hard_joint_expiry_and_revoked_stop(self):
        from test_aubo_hot_clutch import HotTests
        fixture=HotTests();fixture.setUp();fixture.active()
        w=fixture.worker;clock=fixture.clock;w.refined_stream=True;w.revoke('grip_released')
        snap=fixture.client.snapshot();snap['joints']['host_received_monotonic_s']=clock()-.501
        w._status['last_snapshot']=snap
        self.assertFalse(w.resume_clutch(2,clock()))
        w._status['last_snapshot']=fixture.client.snapshot();w.revoke('input_lost')
        self.assertFalse(w.resume_clutch(2,clock()))

    def test_idle_pending_feedback_yields_and_still_revokes_a_new_intent(self):
        from teleop.aubo.follow import ObservationPending
        fixture=FixedTickWorkerTests();fixture.setUp();w=fixture.worker;clock=fixture.clock
        w._may_move=False;w._active_session=None;w._wanted=None
        w._status.update(state='idle',feedback_ready=True)
        snap={key:dict(return_code=0,data=0 if key=='state' else [0.]*6,
            host_received_monotonic_s=clock()-.3) for key in ('state','joints')}
        w._observe=Mock(side_effect=ObservationPending('waiting',snap))
        w._cycle();self.assertTrue(w._wake.is_set())
        w._wake.clear()
        for _ in range(100):
            w._cycle()
            self.assertFalse(w._wake.is_set())
            self.assertGreaterEqual(w._wait_timeout_s(),.005)
        self.assertFalse(w._fatal)
        from teleop.aubo.offline_session import Candidate
        w.offer(Candidate((.1,)*6,clock(),2));w._wake.clear();w._cycle()
        self.assertIsNone(w._wanted)
        self.assertTrue(w._wake.is_set())

    def test_stopped_worker_ignores_expired_batch_deadline(self):
        fixture=FixedTickWorkerTests();fixture.setUp();w=fixture.worker
        w.wall_clock_batch=True;w._next_batch_s=fixture.clock()-1
        w._may_move=False
        self.assertGreaterEqual(w._wait_timeout_s(),.005)
        w._may_move=True;w._stop_needed=True
        self.assertGreaterEqual(w._wait_timeout_s(),.005)

    def test_watchdog_rechecks_latest_input_after_descheduling(self):
        from teleop.aubo.offline_session import Candidate
        fixture=FixedTickWorkerTests();fixture.setUp();w=fixture.worker;clock=fixture.clock
        def updated_status():
            clock.advance(.2);w.note_input(clock());w.offer(Candidate((.1,)*6,clock(),1))
            return dict(sdk_inflight=None,last_result=dict(host_received_monotonic_s=clock()))
        w.state_query=Mock();w.state_query.status.side_effect=updated_status
        w.measured_feedback=Mock(return_value=Mock(q_rad=(0.,)*6))
        w.watchdog_tick()
        self.assertFalse(w._stop_needed)
        self.assertFalse(w.status()['fatal'])

    def test_sine_recovery_preserves_phase_and_zero_speed_blends_into_program(self):
        from scripts.aubo_program_sine import ProgramSineTrajectory
        t=ProgramSineTrajectory([0.]*6,[[-3,3]]*6,axis=0,center=[0.]*6)
        for _ in range(1000):t.advance([0.]*6,.005)
        old_time=t.program_time;q=t.q;t.restart_from_rest(q)
        self.assertEqual(t.program_time,old_time);self.assertEqual(t.v,(0.,)*6);self.assertEqual(t.a,(0.,)*6)
        self.assertIsNotNone(t.recovery_bridge)
        previous=(t.q,t.v,t.a)
        for _ in range(1800):
            t.advance([0.]*6,.005)
            for old,new,limit in zip(previous,(t.q,t.v,t.a),(t.velocity_rad_s,t.acceleration_rad_s2,t.jerk_rad_s3)):
                self.assertLessEqual(max(abs(a-b) for a,b in zip(old,new)),limit*.005+1e-9)
            self.assertLessEqual(abs(t.q[0]),math.radians(6)+1e-9)
            previous=(t.q,t.v,t.a)
        self.assertIsNone(t.recovery_bridge);self.assertGreater(t.program_time,old_time+5.)
    def test_recovery_immediately_resets_reference_and_brake_goal_without_quiet_wait(self):
        fixture=FixedTickWorkerTests();fixture.setUp();w=fixture.worker;c=fixture.client;clock=fixture.clock
        w.refined_stream=True;w.wall_clock_batch=True;w._status['state']='braking'
        w._trajectory.q=(.02,)*6;w._trajectory.v=(.15,)*6;w._trajectory.a=(.2,)*6
        c.restart_clock_from_rest=Mock(return_value=dict(q_restart_rad=[.02]*6))
        w.measured_feedback=Mock(side_effect=AssertionError('No quiet polling'))
        before=clock();w._recover_underrun(dict(recovery_required=True))
        self.assertEqual(clock(),before);c.restart_clock_from_rest.assert_called_once()
        self.assertEqual(w._trajectory.v,(0.,)*6);self.assertEqual(w._trajectory.a,(0.,)*6)
        self.assertEqual(w._hold_q,(.02,)*6);self.assertFalse(w.status()['underrun_recovering'])

    def test_native_expiry_between_sampling_and_send_recovers_unsent_curve(self):
        fixture=FixedTickWorkerTests();fixture.setUp();w=fixture.worker;c=fixture.client
        w.refined_stream=True;w.wall_clock_batch=True
        c.clock_batch_plan=Mock(return_value=dict(recovery_required=False,next_due_monotonic_s=fixture.clock(),
            discarded_wall_ticks=0,point_count=4))
        c.send_batch=Mock(side_effect=RuntimeError('Clock recovery required before sending'))
        c.restart_clock_from_rest=Mock(return_value=dict(q_restart_rad=[0.]*6))
        w._fixed_tick_send((.1,)*6,'active')
        self.assertEqual(w._trajectory.q,(0.,)*6);self.assertEqual(w._trajectory.v,(0.,)*6)
        self.assertEqual(w._trajectory.a,(0.,)*6);self.assertFalse(w.status()['fatal'])
        c.restart_clock_from_rest.assert_called_once();self.assertEqual(w.status()['submitted'],0)

    def test_restarted_curve_starts_at_zero_velocity_and_respects_whole_curve_bounds(self):
        from teleop.aubo.trajectory import JointTrajectory
        t=JointTrajectory([0.]*6,[[-3,3]]*6)
        for _ in range(2000):
            t.advance([.3]*6,.005)
            if max(t.v)>.03:break
        self.assertGreater(max(t.v),.03);q=t.q;t.restart_from_rest(q)
        self.assertEqual(t.v,(0.,)*6);self.assertEqual(t.a,(0.,)*6)
        points,d=t.sample_fixed_ticks([.3]*6,8,first_anchor=True)
        self.assertEqual(points[0],q)
        for a,b in zip((q,)+points[:-1],points):self.assertLess(max(abs(x-y) for x,y in zip(a,b)),math.radians(.05))
        for p in t.controls:
            v=t.derivative(p,t.duration);a=t.derivative(v,t.duration);j=t.derivative(a,t.duration)
            self.assertLessEqual(max(abs(x) for _,x in t.extrema(v)),t.velocity_rad_s)
            self.assertLessEqual(max(abs(x) for _,x in t.extrema(a)),t.acceleration_rad_s2)
            self.assertLessEqual(max(abs(x) for _,x in t.extrema(j)),t.jerk_rad_s3)
if __name__=='__main__':unittest.main()
