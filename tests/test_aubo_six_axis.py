"""Six-axis submission and push feedback regressions; no robot connection."""
import math
import copy
import io
import json
import threading
import time
import tempfile
import unittest
from unittest.mock import Mock, patch
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parent))
from test_aubo_follow import Client, Clock, result
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.offline_session import Candidate
from teleop.aubo.quest_follow import J6QuestRuntime, main, validate_prepare_profile
from teleop.aubo.quest_preview import make_shadow
from teleop.aubo.feedback import FeedbackCache


class SixClient(Client):
    def send_joints(self,q,token):
        if not self.permitted or token!=self.rev:
            raise RuntimeError('Send permission revoked')
        started=self.clock()
        self.sent.append(tuple(q))
        if self.send_hook:self.send_hook()
        out=result(self.clock);out.update(target_rad=list(q),sdk_invoked=True,
            host_started_monotonic_s=started,duration_s=self.clock()-started)
        self.q=list(q)
        return out


class SixAxisTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=SixClient(self.clock)
        self.bounds=((-3.,3.),)*6
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=self.bounds[5],
            joint_ranges_rad=self.bounds,clock=self.clock,diagnostics=True)
    def offer(self,q,session=1):
        self.worker.note_input(self.clock())
        return self.worker.offer(Candidate(tuple(q),self.clock(),session))
    def test_explicit_50hz_cadence_and_live_metrics_exclude_release_gap(self):
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=self.bounds,clock=self.clock,follow_hz=50)
        self.assertEqual(self.worker.period_s,.02)
        self.arm()
        self.clock.advance(.021);self.offer([.1]*6);self.worker._cycle()
        self.clock.advance(.010);self.offer([.1]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),1)
        self.clock.advance(.011);self.offer([.1]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),2)
        self.clock.advance(1.)
        metrics=self.worker.status()['send_metrics']
        self.assertEqual(metrics['calls'],2)
        self.assertAlmostEqual(metrics['actual_within_engagement_hz'],1/.021)
        self.worker._send_samples.append(dict(started_s=self.clock(),duration_s=.007,session=999,accepted=True))
        self.clock.advance(1.)
        self.assertEqual(self.worker.status()['send_metrics']['send_interval']['count'],1)
    def test_explicit_25hz_does_not_send_before_40ms_or_catch_up(self):
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=self.bounds,clock=self.clock,follow_hz=25)
        self.arm()
        self.clock.advance(.039);self.offer([.1]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),0)
        self.clock.advance(.002);self.offer([.1]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),1)
        self.clock.advance(.12);self.offer([.2]*6);self.worker._cycle()
        self.worker._cycle()
        self.assertEqual(len(self.client.sent),2)
        self.assertEqual(self.worker.status()['requested_frequency_hz'],25.)
    def test_50hz_rejected_for_other_backends_and_invalid_rate(self):
        for kwargs in [dict(follow_hz=50),dict(follow_hz=50,joint_ranges_rad=self.bounds,command_mode='passthrough'),
                       dict(follow_hz=100,joint_ranges_rad=self.bounds)]:
            with self.assertRaises(ValueError):
                J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),clock=self.clock,**kwargs)
    def test_200hz_cadence_does_not_burst_after_slow_sdk_call(self):
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=self.bounds,clock=self.clock,follow_hz=200,diagnostics=True)
        self.assertEqual(self.worker.period_s,.005)
        self.assertEqual(self.worker._trace.maxlen,100000)
        self.arm()
        self.clock.advance(.004);self.offer([.1]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),0)
        self.clock.advance(.002);self.offer([.1]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),1)
        self.client.send_hook=lambda:self.clock.advance(.040)
        self.clock.advance(.006);self.offer([.2]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),2)
        self.client.send_hook=None
        self.offer([.3]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),3)
        self.worker._cycle()
        self.assertEqual(len(self.client.sent),3)
    def test_follow_rate_cli_rejects_wrong_scope_before_native_loading(self):
        with patch('teleop.aubo.quest_follow.load_follow_native') as load,patch('sys.stderr',io.StringIO()):
            for args in [['--follow-hz','50'],['--execute-six-axis','--follow-hz','50'],
                         ['--execute-j6','--command-mode','follow','--follow-hz','50']]:
                with self.assertRaises(SystemExit):main(args)
            load.assert_not_called()
    def arm(self):
        self.offer([.1]*6);self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'active')
    def send(self,q):
        self.clock.advance(.101);self.offer(q);self.worker._cycle()
    def test_six_axis_forwards_exact_ik_positions_without_host_rate_shaping(self):
        self.arm();self.send([.1,-.2,.3,-.4,.5,-.6])
        self.assertEqual(len(self.client.sent),1)
        q=self.client.sent[0]
        self.assertEqual(q,(.1,-.2,.3,-.4,.5,-.6))
        self.assertGreater(max(abs(v) for v in q),math.radians(1.))
        self.assertEqual(self.worker.status()['execution_scope'],'six_axis')
        self.assertEqual(self.worker.status()['target_policy'],'direct_ik')
        self.assertIsNone(self.worker.status()['host_target_rate_limit_deg_s'])
        self.assertIsNone(self.worker.status()['sdk_client_speed_deg_s'])
        self.assertIsNone(self.worker.status()['client_acceleration_deg_s2'])
        self.assertFalse(self.worker.status()['sdk_profile_writes'])
        self.assertEqual(self.worker.status()['sdk_profile_policy'],'sdk_unmodified')

    def test_stalled_ik_cannot_offer_new_candidate_after_return(self):
        from teleop.aubo.runtime import OfflineRuntime
        self.arm()
        runtime=J6QuestRuntime(Mock(), self.worker, control_dt=.005, clock=self.clock)
        def slow_step():
            self.clock.advance(.120)
            return Candidate((.2,)*6, self.clock(), 1)
        with patch.object(OfflineRuntime, "step", side_effect=slow_step), patch.object(self.worker, "offer") as offer:
            self.assertIsNone(runtime.step())
            offer.assert_not_called()
        self.assertIsNone(self.worker._wanted)
        self.assertFalse(self.client.permitted)
        self.assertEqual(self.worker.status()["first_stop_trigger"]["reason"], "preview_control_stall")
        self.assertEqual(self.client.sent, [])

    def test_control_timing_diagnostics_use_existing_opt_in_trace(self):
        runtime=J6QuestRuntime(Mock(), self.worker, control_dt=.005, clock=self.clock)
        runtime.record_control_timing({"cycles": 200, "skipped_deadlines": 3})
        event=self.worker.diagnostic_report()["events"][-1]
        self.assertEqual(event["event"], "control_loop_timing")
        self.assertEqual(event["timing"]["skipped_deadlines"], 3)
    def test_reengagement_starts_at_current_six_axis_feedback(self):
        self.client.q=[.25,-.3,.4,-.2,.1,.7]
        self.arm()
        self.assertEqual(self.client.sent,[])
        self.assertEqual(self.worker._last_sent_q,tuple(self.client.q))
        self.send([.5]*6)
        self.assertEqual(self.client.sent[0],(.5,)*6)
    def test_each_controller_limit_is_applied(self):
        self.assertEqual(self.worker.project_target([-4.,4.,-5.,5.,-6.,6.]),(-3.,3.,-3.,3.,-3.,3.))
    def test_lag_above_three_degrees_does_not_reintroduce_host_target_limits(self):
        self.assertIsNone(self.worker.status()['following_error_limit_deg'])
        self.arm()
        for _ in range(10):
            self.send([1.]*6)
            self.client.q=[0.]*6  # Valid stationary feedback, not a follower simulation.
        self.assertEqual(len(self.client.sent),10)
        self.assertGreater(min(self.client.sent[-1]),math.radians(3.))
        self.assertTrue(all(target==(1.,)*6 for target in self.client.sent))
        self.assertFalse(self.worker.status()['fatal'])
        self.worker.revoke('grip_released')
        with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
            self.worker._cycle()
        self.assertTrue(self.worker.status()['stop_confirmed'])
    def test_out_of_range_measured_joint_rejects_before_send(self):
        self.arm();self.client.q[2]=3.01;self.clock.advance(.101);self.offer([.1]*6)
        with self.assertRaisesRegex(RuntimeError,'outside_controller_ranges'):self.worker._cycle()
        self.assertEqual(self.client.sent,[])
    def test_range_validation_covers_all_axes_and_anchor(self):
        for bounds in (self.bounds[:5],((1.,2.),)+self.bounds[1:],((0.,0.),)+self.bounds[1:],
                       ((float('nan'),3.),)+self.bounds[1:]):
            with self.assertRaises(ValueError):
                J6FollowWorker(self.client,[0.]*6,j6_range_rad=self.bounds[5],joint_ranges_rad=bounds)
    def test_release_discards_all_pending_joints_and_requires_stop(self):
        self.arm();self.worker.revoke('grip_released')
        self.assertIsNone(self.worker._wanted)
        self.assertFalse(self.offer([1.]*6,2))
        with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
            self.worker._cycle()
        self.assertTrue(self.worker.status()['stop_confirmed'])
        self.assertEqual(self.client.sent,[])
        self.offer([1.]*6,2);self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'active')
    def test_delayed_cycle_sends_only_latest_ik_without_interpolation_or_queue(self):
        self.arm();self.clock.advance(.4);self.offer([1.]*6);self.worker._cycle()
        self.assertEqual(self.client.sent,[(1.,)*6])
        self.clock.advance(.651);self.offer([1.]*6);self.worker._cycle()
        self.assertEqual(len(self.client.sent),1)
        self.assertTrue(self.worker._stop_needed)
    def test_reversal_forwards_latest_angles_without_chasing_previous_target(self):
        self.arm();self.send([.2,-.2,.3,-.3,.4,-.4])
        self.send([-.2,.2,-.3,.3,-.4,.4])
        self.assertEqual(self.client.sent,[(.2,-.2,.3,-.3,.4,-.4),(-.2,.2,-.3,.3,-.4,.4)])
    def test_latest_candidate_replaces_old_before_next_send(self):
        self.arm();self.clock.advance(.002);self.offer([.2]*6);self.worker._cycle()
        self.assertEqual(self.client.sent,[])
        self.clock.advance(.049);self.offer([-.1]*6);self.worker._cycle()
        self.assertEqual(self.client.sent,[(-.1,)*6])
    def test_follow_twenty_hz_forwards_latest_targets_without_catchup(self):
        self.arm()
        self.assertEqual(self.worker.status()['requested_frequency_hz'],20.)
        for target in (.1,.2,.3):
            self.clock.advance(.0501);self.offer([target]*6);self.worker._cycle()
        self.assertEqual(self.client.sent,[(.1,)*6,(.2,)*6,(.3,)*6])
        self.clock.advance(.3);self.offer([.4]*6);self.worker._cycle()
        self.assertEqual(self.client.sent,[(.1,)*6,(.2,)*6,(.3,)*6,(.4,)*6])
    def test_blocked_call_revoke_cannot_authorize_new_six_axis_send(self):
        self.arm()
        def blocked():
            self.clock.advance(.51);self.worker.watchdog_tick()
        self.client.send_hook=blocked;self.send([.1]*6)
        self.assertTrue(self.worker.status()['fatal'])
        self.assertFalse(self.offer([.2]*6,2));self.assertEqual(len(self.client.sent),1)
    def test_explicit_six_axis_cli_requires_push_and_full_device_parameters(self):
        with patch('teleop.aubo.quest_follow.load_follow_native') as load,patch('sys.stderr'):
            for args in (['--execute-six-axis'],['--execute-six-axis','--waypoint-feedback'],
                         ['--execute-six-axis','--execute-j6']):
                with self.assertRaises(SystemExit):main(args)
            load.assert_not_called()

    def test_author_cartesian_input_reaches_multiple_physical_joint_targets(self):
        q=(.18,.26,2.37,2.04,1.51,.8)
        self.client.q=list(q)
        self.worker=J6FollowWorker(self.client,q,j6_range_rad=self.bounds[5],
            joint_ranges_rad=self.bounds,clock=self.clock)
        shadow=make_shadow(q,self.bounds)
        shadow.clock=shadow.session.clock=self.clock
        shadow.session.cache=FeedbackCache(self.clock)
        runtime=J6QuestRuntime(shadow,self.worker,control_dt=.01,clock=self.clock)
        try:
            with patch('vr_teleop_kit.core.quest_teleop.time.monotonic',self.clock):
                for i in range(100):
                    self.clock.advance(.01)
                    runtime.submit_feedback(self.client.q,received_monotonic_s=self.clock())
                    frame=dict(type='xr_frame',epoch='six-test',seq=i,t_client=self.clock()*1000,
                        controllers={'right':dict(position=[min(.05,max(0,i-3)*.001),0.,0.],
                            orientation=[0.,0.,0.,1.],buttons=[dict(p=False,v=0.),dict(p=i>1,v=float(i>1))])})
                    runtime.submit_xr_frame(frame)
                    candidate=runtime.step();self.worker._cycle()
                    if candidate:
                        self.assertEqual(candidate.q_rad,self.worker._wanted[0])
            self.assertGreater(len(self.client.sent),0)
            last=self.client.sent[-1]
            self.assertGreater(sum(abs(a-b)>1e-5 for a,b in zip(last[:5],q[:5])),1)
            self.assertFalse(self.worker.status()['fatal'])
            self.assertEqual(shadow.session.backend.submitted,[])
        finally:runtime.close()


class StateQuerySchedulingTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=SixClient(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=((-3.,3.),)*6,clock=self.clock,waypoint_feedback=True)
    def offer(self):
        self.worker.note_input(self.clock())
        self.worker.offer(Candidate((.1,)*6,self.clock(),1))
    def arm(self):
        self.offer();self.worker._cycle()
    def test_active_push_sends_at_fifty_ms_without_query_and_queries_at_hundred_ms(self):
        with patch.object(self.client,'snapshot',wraps=self.client.snapshot) as query:
            self.arm();self.assertEqual(query.call_count,1)
            state_stamp=self.worker.status()['last_snapshot']['state']['host_received_monotonic_s']
            self.clock.advance(.051);self.offer();self.worker._cycle()
            self.assertEqual(query.call_count,1);self.assertEqual(len(self.client.sent),1)
            self.assertEqual(self.worker.status()['last_snapshot']['state']['host_received_monotonic_s'],state_stamp)
            self.clock.advance(.051);self.offer();self.worker._cycle()
            self.assertEqual(query.call_count,2);self.assertEqual(len(self.client.sent),2)
    def test_idle_and_nonpush_keep_original_query_cadence(self):
        with patch.object(self.client,'snapshot',wraps=self.client.snapshot) as query:
            self.worker._cycle();self.clock.advance(.051);self.worker._cycle()
            self.assertEqual(query.call_count,2);self.assertEqual(self.client.sent,[])
        self.worker.waypoint_feedback_enabled=False
        with patch.object(self.client,'snapshot',wraps=self.client.snapshot) as query:
            self.arm();self.clock.advance(.051);self.offer();self.worker._cycle()
            self.assertEqual(query.call_count,2)
    def test_engagement_requires_new_query_even_with_recent_idle_cache(self):
        self.worker._cycle();self.clock.advance(.01)
        self.client.state=1
        self.offer();self.worker._cycle()
        self.assertTrue(self.worker.status()['fatal']);self.assertEqual(self.client.sent,[])
    def test_failed_or_stale_query_never_sends_the_pending_target(self):
        for fail in ('return_code','old_state'):
            with self.subTest(fail=fail):
                self.setUp();self.arm();self.clock.advance(.101);self.offer()
                snap=self.client.snapshot()
                if fail=='return_code':snap['state']['return_code']=-1
                else:snap['state']['host_received_monotonic_s']=self.clock()-.201
                with patch.object(self.client,'snapshot',return_value=snap):
                    with self.assertRaises(RuntimeError):self.worker._cycle()
                self.assertEqual(self.client.sent,[])
    def test_release_during_query_prevents_send(self):
        self.arm();self.clock.advance(.101);self.offer()
        snapshot=self.client.snapshot
        def release():
            self.worker.revoke('grip_released')
            return snapshot()
        with patch.object(self.client,'snapshot',side_effect=release):self.worker._cycle()
        self.assertEqual(self.client.sent,[])
        self.assertEqual(self.worker.status()['state'],'stop_requested')
    def test_slow_query_is_not_inserted_before_first_due_send(self):
        snapshot=self.client.snapshot
        def slow_query():
            self.clock.advance(.04)
            return snapshot()
        with patch.object(self.client,'snapshot',side_effect=slow_query) as query:
            self.arm();self.clock.advance(.051);self.offer()
            due=self.clock();self.worker._cycle()
            self.assertEqual(query.call_count,1)
            self.assertEqual(self.worker.status()['last_attempt']['sdk_result']['host_started_monotonic_s'],due)
            self.clock.advance(.051);self.offer();self.worker._cycle()
            self.assertEqual(query.call_count,2)
    def test_target_updated_during_query_replaces_old_target(self):
        self.arm();self.clock.advance(.101);self.offer()
        snapshot=self.client.snapshot
        def update_target():
            self.clock.advance(.04)
            self.worker.note_input(self.clock())
            self.worker.offer(Candidate((-.2,)*6,self.clock(),1))
            return snapshot()
        with patch.object(self.client,'snapshot',side_effect=update_target):self.worker._cycle()
        self.assertEqual(self.client.sent,[(-.2,)*6])
    def test_stop_path_runs_before_any_periodic_query_or_send(self):
        self.arm();self.clock.advance(.101);self.worker.revoke('grip_released')
        with patch.object(self.worker,'_stop_and_confirm') as stop,patch.object(self.client,'snapshot') as query:
            self.worker._cycle();stop.assert_called_once();query.assert_not_called()
        self.assertEqual(self.client.sent,[])
    def test_wait_ends_at_send_deadline_and_does_not_catch_up(self):
        self.arm();self.clock.advance(self.worker.period_s-.002)
        self.assertAlmostEqual(self.worker._wait_timeout_s(),.002)
        self.clock.advance(.2);self.offer();self.worker._cycle()
        self.assertEqual(len(self.client.sent),1)
        self.worker._cycle();self.assertEqual(len(self.client.sent),1)

    def test_expired_joint_snapshot_refreshes_even_with_recent_robot_state(self):
        self.arm()
        # Reproduce the field timing: joints 202ms old, state only 93ms old.
        self.clock.advance(.110)
        snapshot=self.client.snapshot()
        snapshot['joints']['host_received_monotonic_s']=self.clock()-.109
        self.worker._status['last_snapshot']=snapshot
        self.worker.cache.update([0.]*6, source='sdk_waypoint_push',
            received_monotonic_s=snapshot['joints']['host_received_monotonic_s'])
        self.clock.advance(.093)
        self.offer()
        with patch.object(self.client,'snapshot',wraps=self.client.snapshot) as query:
            self.worker._cycle()
        query.assert_called_once()
        self.assertFalse(self.worker.status()['fatal'])
        self.assertEqual(len(self.client.sent),1)

    def test_refresh_for_expired_joints_still_rejects_stale_query_result(self):
        self.arm();self.clock.advance(.21);self.offer()
        snapshot=self.client.snapshot()
        snapshot['joints']['host_received_monotonic_s']=self.clock()-.201
        with patch.object(self.client,'snapshot',return_value=snapshot):
            with self.assertRaisesRegex(RuntimeError,'feedback_reception_expired'):
                self.worker._cycle()
        self.assertEqual(self.client.sent,[])


class PassthroughTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=SixClient(self.clock)
        self.events=[]
        self.entry_rc=0
        self.entry_hook=None
        self.client.enter_passthrough=self.enter
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=((-3.,3.),)*6,clock=self.clock,
            command_mode='passthrough',diagnostics=True)
    def enter(self,token):
        self.assertTrue(self.client.permitted);self.assertEqual(token,self.client.rev)
        self.events.append('enter')
        if self.entry_hook:self.entry_hook()
        return result(self.clock,op='enter_tcp2canbus',rc=self.entry_rc)
    def offer(self,session=1):
        self.worker.note_input(self.clock())
        self.worker.offer(Candidate((.2,-.2,.3,-.3,.4,-.4),self.clock(),session))
    def stop(self):
        self.worker.revoke('grip_released')
        with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
            self.worker._cycle()
    def test_mode_entry_only_on_engagement_then_exact_six_joint_send(self):
        self.worker._cycle();self.assertEqual(self.events,[])
        self.offer();self.worker._cycle()
        self.assertEqual(self.events,['enter']);self.assertEqual(self.client.sent,[])
        self.clock.advance(.051);self.offer();self.worker._cycle()
        self.assertEqual(self.client.sent,[(.2,-.2,.3,-.3,.4,-.4)])
        self.assertEqual(self.worker.status()['command_mode'],'passthrough')
        self.assertFalse(self.worker.status()['sdk_profile_controls_passthrough'])
    def test_passthrough_two_hundred_hz_uses_latest_target_and_never_catches_up(self):
        self.offer();self.worker._cycle()
        self.clock.advance(.002);self.offer();self.worker._cycle()
        self.assertEqual(self.client.sent,[])
        self.clock.advance(.0031)
        self.worker.note_input(self.clock())
        self.worker.offer(Candidate((-.1,)*6,self.clock(),1));self.worker._cycle()
        self.assertEqual(self.client.sent,[(-.1,)*6])
        self.clock.advance(.08);self.offer();self.worker._cycle()
        self.assertEqual(len(self.client.sent),2)
        self.worker._cycle();self.assertEqual(len(self.client.sent),2)
        self.assertEqual(self.worker.status()['requested_frequency_hz'],200.)
    def test_release_during_enter_never_allows_a_target(self):
        self.entry_hook=lambda:self.worker.revoke('grip_released')
        self.offer();self.worker._cycle()
        self.assertEqual(self.client.sent,[]);self.assertEqual(self.worker.status()['state'],'stop_requested')
        self.stop();self.assertTrue(self.worker.status()['stop_confirmed'])
    def test_failed_enter_stops_and_stays_latched(self):
        self.entry_rc=1;self.offer();self.worker._cycle()
        self.assertTrue(self.worker.status()['fatal']);self.assertEqual(self.client.sent,[])
        self.stop();self.assertTrue(self.worker.status()['stop_confirmed'])
        self.assertEqual(self.events,['enter'])
    def test_failed_exit_or_stop_does_not_allow_reengagement(self):
        self.offer();self.worker._cycle();self.client.stop_rc=1
        self.stop()
        self.assertTrue(self.worker.status()['stop_result_unknown'])
        self.assertTrue(self.worker.status()['fatal'])
        self.offer(session=2);self.worker._cycle()
        self.assertEqual(self.events,['enter']);self.assertEqual(self.client.sent,[])
    def test_entry_exception_never_sends_and_leaves_stop_path_available(self):
        def fail():raise RuntimeError('lost enter response')
        self.entry_hook=fail;self.offer()
        with self.assertRaisesRegex(RuntimeError,'lost enter'):self.worker._cycle()
        self.assertTrue(self.worker._may_move);self.assertEqual(self.client.sent,[])
        self.stop();self.assertTrue(self.worker.status()['stop_confirmed'])
    def test_normal_stop_reengages_mode_from_new_measured_anchor(self):
        self.offer();self.worker._cycle();self.stop()
        self.client.q=[.1]*6
        self.offer(session=2);self.worker._cycle()
        self.assertEqual(self.events,['enter','enter'])
        self.assertEqual(self.worker._last_sent_q,(.1,)*6)
        self.assertEqual(self.client.sent,[])
    def test_legacy_j6_cannot_use_passthrough(self):
        with self.assertRaises(ValueError):
            J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),command_mode='passthrough')


class ProfileTests(unittest.TestCase):
    def prepared(self, six_axis=True):
        return dict(six_axis=six_axis, command_mode="follow",
            target_policy='direct_ik' if six_axis else 'j6_rate_limited',
            host_target_rate_limit_deg_s=None if six_axis else 10.,
            client_speed_deg_s=None if six_axis else 10.,
            client_acceleration_deg_s2=None if six_axis else 5.,
            sdk_profile_policy='sdk_unmodified' if six_axis else 'legacy_j6_explicit',
            sdk_profile_writes=not six_axis, following_error_limit_deg=None,
            send_period_s=.05 if six_axis else .1, send_max_gap_s=.65, sdk_budget_s=.5,
            sdk_profile_readback=dict(source='sdk_client_memory_not_controller_settings',
                max_velocity_rad_s=[.8,.9,1.,1.1,1.2,1.3],
                max_acceleration_rad_s2=[.3,.4,.5,.6,.7,.8]))
    def test_six_axis_accepts_unmodified_nonuniform_sdk_profile_and_reports_it(self):
        prepared=self.prepared()
        readback=validate_prepare_profile(prepared,six_axis=True)
        self.assertIs(readback,prepared['sdk_profile_readback'])
        worker=J6FollowWorker(SixClient(Clock()),[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=((-3.,3.),)*6,sdk_profile_readback=readback)
        self.assertEqual(worker.status()['sdk_profile_readback'],readback)
        self.assertIsNone(worker.status()['speed_deg_s'])
    def test_passthrough_profile_rejects_old_follow_binding(self):
        prepared=self.prepared()
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            validate_prepare_profile(prepared,six_axis=True,command_mode='passthrough')
        prepared['command_mode']='passthrough'
        prepared['send_period_s']=.005
        validate_prepare_profile(prepared,six_axis=True,command_mode='passthrough')
    def test_selected_50hz_profile_must_match_native(self):
        prepared=self.prepared()
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            validate_prepare_profile(prepared,six_axis=True,follow_hz=50)
        prepared['send_period_s']=.02
        validate_prepare_profile(prepared,six_axis=True,follow_hz=50)
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            validate_prepare_profile(prepared,six_axis=True)
    def test_selected_25hz_profile_must_match_native(self):
        prepared=self.prepared();prepared['send_period_s']=.04
        validate_prepare_profile(prepared,six_axis=True,follow_hz=25)
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            validate_prepare_profile(prepared,six_axis=True,follow_hz=50)
    def test_selected_200hz_follow_profile_matches_without_switching_backend(self):
        prepared=self.prepared();prepared['send_period_s']=.005
        validate_prepare_profile(prepared,six_axis=True,follow_hz=200)
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            validate_prepare_profile(prepared,six_axis=True)
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            validate_prepare_profile(prepared,six_axis=True,command_mode='passthrough')
    def test_old_native_profile_or_wrong_execution_scope_rejected(self):
        for updates in (dict(client_speed_deg_s=10.),dict(client_acceleration_deg_s2=5.),
                        dict(sdk_profile_writes=True),dict(sdk_profile_policy='legacy_j6_explicit'),
                        dict(target_policy='j6_rate_limited'),dict(six_axis=False),dict(send_period_s=.1),dict(send_period_s=.005)):
            prepared=self.prepared();prepared.update(updates)
            with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
                validate_prepare_profile(prepared,six_axis=True)
        prepared=self.prepared();del prepared['client_speed_deg_s']
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            validate_prepare_profile(prepared,six_axis=True)
    def test_invalid_or_mislabeled_readback_rejected(self):
        for changes in (dict(source='controller_settings'),dict(max_velocity_rad_s=[1.]*5),
                        dict(max_velocity_rad_s=[float('nan')]*6),
                        dict(max_acceleration_rad_s2=[0.]*6),dict(max_velocity_rad_s=None)):
            prepared=copy.deepcopy(self.prepared());prepared['sdk_profile_readback'].update(changes)
            with self.assertRaisesRegex(RuntimeError,'memory profile readback'):
                validate_prepare_profile(prepared,six_axis=True)
        for readback in (None,{}):
            prepared=self.prepared();prepared['sdk_profile_readback']=readback
            with self.assertRaisesRegex(RuntimeError,'memory profile readback'):
                validate_prepare_profile(prepared,six_axis=True)
    def test_legacy_j6_policy_is_separate_and_unchanged(self):
        validate_prepare_profile(self.prepared(False),six_axis=False)
        with self.assertRaisesRegex(RuntimeError,'profile mismatch'):
            validate_prepare_profile(self.prepared(),six_axis=False)
    def test_default_plan_does_not_claim_trial_speed_or_controller_inheritance(self):
        output=io.StringIO()
        with patch('sys.stdout',output),patch('teleop.aubo.quest_follow.load_follow_native') as load:
            self.assertEqual(main([]),0);load.assert_not_called()
        plan=json.loads(output.getvalue())
        self.assertIsNone(plan['sdk_client_speed_deg_s'])
        self.assertIsNone(plan['client_acceleration_deg_s2'])
        self.assertIsNone(plan['physical_command_speed_deg_s'])
        self.assertFalse(plan['six_axis_sdk_profile_writes'])
        self.assertEqual(plan['six_axis_sdk_profile_policy'],'sdk_unmodified')
        self.assertEqual(plan['legacy_j6_profile']['speed_deg_s'],10.)
        self.assertEqual(plan['six_axis_send_hz'],200.)
        self.assertEqual(plan['legacy_j6_send_hz'],10.)
        self.assertEqual(plan['six_axis_command_mode'],'passthrough')
    def test_inspection_never_constructs_worker_or_calls_arm_send_stop(self):
        from unittest.mock import Mock
        client=Mock(spec=['login','prepare','logout'])
        client.login.return_value={'return_code':0}
        client.logout.return_value={'return_code':0}
        client.prepare.return_value=dict(self.prepared(),return_code=0,prepared=True,
            anchor_rad=[0.]*6,joint_ranges_rad=[[-3.,3.]]*6)
        native=Mock();native.J6Follower.return_value=client
        with tempfile.TemporaryDirectory() as isolated_root, \
             patch('teleop.aubo.quest_follow.ROOT',Path(isolated_root)), \
             patch('teleop.aubo.quest_follow.load_follow_native',return_value=native), \
             patch('teleop.aubo.quest_follow.getpass.getpass',return_value='test-only'), \
             patch('teleop.aubo.quest_follow.J6FollowWorker') as worker, \
             patch('teleop.aubo.quest_follow.make_shadow') as shadow, \
             patch('teleop.aubo.quest_follow.socket.socket') as sock, \
             patch('sys.stdout',io.StringIO()) as output:
            # Profile validation uses the real constants, independent of the
            # worker-construction sentinel above.
            worker.period_s=.1;worker.six_axis_period_s=.005;worker.six_axis_follow_period_s=.05;worker.speed_deg_s=10.
            worker.client_acceleration_deg_s2=5.;worker.following_error_limit_deg=None
            worker.send_max_gap_s=.65;worker.sdk_budget_s=.5
            self.assertEqual(main(['--inspect-profile','--host','192.0.2.1','--port','8899','--user','test']),0)
            worker.assert_not_called();shadow.assert_not_called();sock.assert_not_called()
        report=json.loads(output.getvalue())
        self.assertFalse(report['motion_available']);self.assertFalse(report['sdk_profile_writes'])
        self.assertAlmostEqual(report['sdk_profile_readback']['max_acceleration_deg_s2'][0],math.degrees(.3))
        native.J6Follower.assert_called_once_with(six_axis=True, follow_hz=20, inspect_profile=True)
        self.assertEqual([c[0] for c in client.method_calls],['login','prepare','logout'])


class NormalClutchTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=SixClient(self.clock)
        self.q=(.18,.26,2.37,2.04,1.51,.8);self.client.q=list(self.q)
        self.bounds=((-3.,3.),)*6
        self.worker=J6FollowWorker(self.client,self.q,j6_range_rad=self.bounds[5],
            joint_ranges_rad=self.bounds,clock=self.clock,diagnostics=True,
            command_mode='follow',waypoint_feedback=True)
        self.push_sequence=0
        self.shadow=make_shadow(self.q,self.bounds)
        self.shadow.clock=self.shadow.session.clock=self.clock
        self.shadow.session.cache=FeedbackCache(self.clock)
        self.runtime=J6QuestRuntime(self.shadow,self.worker,control_dt=.01,clock=self.clock)
        self.seq=0
        self.patch=patch('vr_teleop_kit.core.quest_teleop.time.monotonic',self.clock)
        self.patch.start()
        self.tick(False);self.tick(True);self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'active')
    def tearDown(self):
        self.runtime.close();self.patch.stop()
    def frame(self,grip):
        self.seq+=1
        return dict(type='xr_frame',epoch='clutch-test',seq=self.seq,t_client=self.clock()*1000,
            controllers={'right':dict(position=[0.,0.,0.],orientation=[0.,0.,0.,1.],
                buttons=[dict(p=False,v=0.),dict(p=grip,v=float(grip))])})
    def tick(self,grip,position=None):
        self.clock.advance(.01)
        self.push_sequence+=1
        push=dict(sequence=self.push_sequence,host_received_monotonic_s=self.clock(),q_rad=list(self.client.q))
        self.client.waypoint_feedback=lambda:dict(samples=[push])
        self.runtime.refresh_feedback()
        frame=self.frame(grip)
        if position is not None:frame['controllers']['right']['position']=position
        self.runtime.submit_xr_frame(frame,received_monotonic_s=self.clock())
        return self.runtime.step()
    def stop_complete(self):
        with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
            self.worker._cycle()
        self.assertTrue(self.worker.status()['stop_confirmed'])
        self.assertIsNotNone(self.worker.status()['stop_confirmed_monotonic_s'])
    def test_held_grip_input_timeout_freezes_evidence_through_later_missing_hand(self):
        self.clock.advance(.21)
        self.runtime.submit_feedback(self.q, received_monotonic_s=self.clock())
        self.assertIsNone(self.runtime.step())
        self.assertEqual(self.worker.status()['first_stop_trigger']['reason'], 'core_input_unavailable')
        frozen = self.runtime.status()['last_input_stop']
        self.assertEqual(frozen['core_interruption']['core_paused'], 'input_timeout')
        self.assertTrue(frozen['core_interruption']['grip'])
        self.assertTrue(frozen['last_received']['grip'])
        self.assertEqual(frozen['last_processed']['reason'], 'accepted')
        self.assertTrue(self.worker.status()['stop_requires_post_confirm_release'])
        self.assertIsNone(self.worker._wanted)
        invalid = self.frame(True); invalid['controllers'] = {}
        self.assertFalse(self.runtime.submit_xr_frame(invalid))
        after = self.runtime.status()
        self.assertFalse(after['input_timing']['last_received']['right_present'])
        self.assertEqual(after['last_input_stop'], frozen)
        self.assertFalse(self.worker.status()['fatal'])
    def test_normal_release_preserved_and_first_post_stop_press_reanchors(self):
        self.tick(False)
        for _ in range(3):self.tick(False)
        self.assertFalse(self.worker.status()['stop_requires_post_confirm_release'])
        self.client.q=[v+.01 for v in self.q]
        self.stop_complete()
        self.assertIsNotNone(self.tick(True))
        self.assertEqual(self.runtime.status()['clutch_state'],'engaged')
        self.assertEqual(self.shadow.session.last_reference,tuple(self.client.q))
        self.assertEqual(self.client.sent,[])
    def test_press_while_stopping_resumes_from_fresh_frame_after_confirmation(self):
        self.tick(False);self.assertIsNone(self.tick(True))
        self.assertIsNone(self.worker._wanted)
        self.client.q=[v+.01 for v in self.q]
        self.stop_complete()
        self.assertIsNotNone(self.tick(True))
        self.assertEqual(self.shadow.session.last_reference,tuple(self.client.q))
        self.assertEqual(self.client.sent,[])
        self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'active')
    def test_stop_completed_between_control_ticks_keeps_early_press(self):
        self.clock.advance(.01)
        self.runtime.submit_xr_frame(self.frame(False))
        self.runtime.submit_xr_frame(self.frame(True))
        self.stop_complete()  # No runtime.step observed the stopping state.
        self.assertIsNotNone(self.tick(True))
        self.assertEqual(self.client.sent,[])
    def test_long_stop_does_not_expire_the_release_fact(self):
        self.tick(False)
        self.client.stop_hook=lambda:self.clock.advance(.58)
        self.stop_complete()
        self.assertIsNotNone(self.tick(True))
        self.assertIsNone(self.shadow.session.fault)
    def test_long_stop_with_early_press_does_not_need_second_press(self):
        self.tick(False);self.tick(True)
        self.client.stop_hook=lambda:self.clock.advance(.58)
        self.stop_complete()
        self.assertIsNotNone(self.tick(True))
        self.assertEqual(self.runtime.status()['clutch_state'],'engaged')
    def test_pre_confirmation_pose_cannot_generate_a_restart_candidate(self):
        self.tick(False);self.tick(True)
        self.stop_complete()
        self.assertIsNone(self.runtime.step())
        self.assertIsNone(self.worker._wanted)
        self.assertFalse(self.shadow.session.engaged)
        self.assertIsNotNone(self.tick(True))
    def test_hand_motion_during_stop_is_not_replayed_after_reengagement(self):
        self.tick(False);self.tick(True,position=[.2,0.,0.])
        self.client.q=[v+.01 for v in self.q]
        self.client.stop_hook=lambda:self.clock.advance(.58)
        self.stop_complete()
        candidate=self.tick(True,position=[.4,.1,.2])
        self.assertIsNotNone(candidate)
        self.worker._cycle()
        self.tick(False);self.stop_complete()
        stationary=self.tick(True,position=[0.,0.,0.])
        self.assertIsNotNone(stationary)
        # IK regularization can make a small correction at engagement. The
        # hand's travel while stopped must not affect that first target.
        for moved,still in zip(candidate.q_rad,stationary.q_rad):
            self.assertAlmostEqual(moved,still,places=8)
        self.assertEqual(self.client.sent,[])
    def test_stop_with_only_one_release_frame_between_ticks(self):
        self.clock.advance(.01)
        self.runtime.submit_xr_frame(self.frame(False))
        self.client.stop_hook=lambda:self.clock.advance(.58)
        self.stop_complete()
        self.assertIsNotNone(self.tick(True))
    def test_fault_after_confirmation_before_gate_processing_requires_new_release(self):
        self.tick(False);self.stop_complete()
        self.runtime.submit_fault('websocket_disconnected')
        self.assertIsNone(self.tick(True))
        self.assertTrue(self.worker.status()['stop_requires_post_confirm_release'])
        self.tick(False);self.assertIsNotNone(self.tick(True))
    def test_release_already_consumed_before_stop_keeps_first_post_stop_press(self):
        from teleop.aubo.runtime import OfflineRuntime
        self.clock.advance(.01)
        self.runtime.submit_xr_frame(self.frame(False))
        OfflineRuntime.step(self.runtime)  # Receiver wins during an active tick.
        self.assertFalse(self.shadow.session.engaged)
        self.assertFalse(self.shadow.session.require_release)
        self.assertIsNone(self.runtime._release_frame)
        self.stop_complete()
        self.assertIsNotNone(self.tick(True))
    def test_fault_stop_keeps_strict_post_confirmation_release_rule(self):
        self.runtime.submit_fault('websocket_disconnected')
        self.tick(False);self.stop_complete()
        self.assertTrue(self.worker.status()['stop_requires_post_confirm_release'])
        for _ in range(3):self.assertIsNone(self.tick(True))
        self.tick(False);self.assertIsNotNone(self.tick(True))
    def test_fault_during_normal_stop_cannot_be_cleared_by_later_releases(self):
        self.tick(False);self.runtime.submit_fault('invalid_xr_frame')
        self.tick(False);self.tick(False);self.stop_complete()
        self.assertTrue(self.worker.status()['stop_requires_post_confirm_release'])
        for _ in range(2):self.assertIsNone(self.tick(True))
        self.tick(False);self.assertIsNotNone(self.tick(True))
    def test_fatal_stop_remains_latched_even_after_fresh_release(self):
        self.tick(False);self.worker.revoke('sdk_call_over_budget',fatal=True)
        self.stop_complete();self.tick(False)
        self.assertIsNone(self.tick(True));self.assertEqual(self.runtime.status()['clutch_state'],'fault')
    def test_stop_diagnostics_contain_edges_call_start_and_measured_confirmation(self):
        self.tick(False);self.stop_complete()
        events=self.worker.diagnostic_report()['events']
        wanted=['grip_edge','stop_requested','slow_stop_call_started','stop_returned','stop_confirmed']
        for name in wanted:self.assertIn(name,[e['event'] for e in events])
        ordered=[next(e for e in events if e['event']==name) for name in wanted[1:]]
        self.assertEqual([e['host_monotonic_s'] for e in ordered],sorted(e['host_monotonic_s'] for e in ordered))
        self.assertEqual(ordered[-1]['q_rad'],list(self.client.q))
        self.assertEqual(self.client.sent,[])


class PushTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=SixClient(self.clock)
        self.frames=[]
        self.client.waypoint_feedback=lambda:dict(samples=list(self.frames),total_received=len(self.frames))
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=((-3.,3.),)*6,clock=self.clock,waypoint_feedback=True,diagnostics=True)
    def push(self,seq=1,age=0.,q=None):
        self.frames.append(dict(sequence=seq,host_received_monotonic_s=self.clock()-age,q_rad=q or [0.]*6))
    def test_push_remains_readable_while_serial_owner_is_blocked(self):
        self.worker._observe();old=self.worker.cache.latest().received_monotonic_s
        self.worker._inflight=dict(operation='follow',started_s=self.clock(),budget_s=.5)
        self.clock.advance(.23);self.push(q=[.1]*6)
        sample=self.worker.measured_feedback()
        self.assertEqual(sample.q_rad,(.1,)*6)
        self.assertEqual(sample.received_monotonic_s,self.clock())
        self.assertEqual(self.worker.cache.latest().received_monotonic_s,old)
        self.assertEqual(self.worker._status['last_snapshot']['state']['host_received_monotonic_s'],old)
        event=self.worker.diagnostic_report()['events'][-1]
        self.assertEqual(event['event'],'push_observed')
        self.assertEqual(event['sdk_inflight']['operation'],'follow')
    def test_repeated_frame_does_not_refresh_age(self):
        self.push();self.worker.measured_feedback();self.clock.advance(.201)
        with self.assertRaisesRegex(RuntimeError,'expired'):self.worker.measured_feedback()
    def test_diagnostic_keeps_callback_samples_between_control_ticks(self):
        self.push(seq=1,q=[.1]*6);self.clock.advance(.01)
        self.push(seq=2,q=[.2]*6);self.clock.advance(.01)
        self.push(seq=3,q=[.3]*6)
        self.worker.measured_feedback()
        event=self.worker.diagnostic_report()['events'][-1]
        self.assertEqual(event['previous_sequence'],0)
        self.assertEqual(event['samples'],self.frames)
        self.assertEqual(event['samples'][0]['host_received_monotonic_s'],
                         self.frames[0]['host_received_monotonic_s'])
        count=len(self.worker.diagnostic_report()['events'])
        self.worker.measured_feedback()
        self.assertEqual(len(self.worker.diagnostic_report()['events']),count)
    def test_newer_snapshot_cannot_overwrite_independent_push_cache(self):
        self.push();self.worker.measured_feedback();self.clock.advance(.05);self.push(seq=2,q=[.2]*6)
        self.worker.measured_feedback()
        self.worker.cache.update([0.]*6,source='snapshot',received_monotonic_s=self.clock()-.02)
        self.assertEqual(self.worker.measured_feedback().q_rad,(.2,)*6)
    def test_bad_pushes_do_not_reuse_valid_feedback(self):
        for values in (dict(age=-.01),dict(age=.21),dict(q=[float('nan')]*6),dict(q=[4.]*6),dict(seq=0)):
            self.setUp();self.push(**values)
            with self.assertRaises((RuntimeError,ValueError)):self.worker.measured_feedback()
    def test_missing_and_regressing_push_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'missing'):self.worker.measured_feedback()
        self.clock.advance(.01);self.push(seq=2);self.worker.measured_feedback();self.push(seq=1)
        with self.assertRaisesRegex(RuntimeError,'regressed'):self.worker.measured_feedback()
    def test_read_cache_during_real_blocking_call_uses_no_sdk_query(self):
        entered=threading.Event();release=threading.Event()
        def blocked():entered.set();release.wait(1.);return result(self.clock)
        thread=threading.Thread(target=lambda:self.worker._call('follow',blocked));thread.start()
        try:
            self.assertTrue(entered.wait(.5));self.clock.advance(.23);self.push()
            with patch.object(self.client,'snapshot',side_effect=AssertionError('Unexpected SDK query')):
                self.assertTrue(self.worker.measured_feedback().valid)
                self.assertEqual(self.worker.status()['sdk_inflight']['operation'],'follow')
        finally:release.set();thread.join()



if __name__=='__main__':unittest.main(verbosity=2)
