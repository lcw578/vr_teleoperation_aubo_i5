"""Hardware follower ordering, stop and blocking races; never connects a robot."""
import asyncio
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.feedback import FeedbackCache
from teleop.aubo.offline_session import Candidate
from teleop.aubo.quest_follow import J6QuestRuntime, main
from teleop.aubo.quest_preview import PreviewStatus, QuestReceiver, make_shadow, main as preview_main


class Clock:
    def __init__(self): self.now=100.
    def __call__(self): return self.now
    def advance(self,dt=.05): self.now+=dt


def result(clock, op="ok", rc=0, data=None):
    return dict(operation=op,return_code=rc,duration_s=0.,
        host_started_monotonic_s=clock(),host_received_monotonic_s=clock(),data=data)


class Client:
    def __init__(self,clock):
        self.clock=clock;self.rev=0;self.sent=[];self.stops=0
        self.q=[0.]*6;self.state=0;self.stop_rc=0;self.stop_hook=None;self.send_hook=None
        self.permitted=False
    def revoke(self): self.rev+=1;self.permitted=False;return self.rev
    def revision(self): return self.rev
    def arm(self,token):self.permitted=token==self.rev;return self.permitted
    def snapshot(self):return dict(joints=result(self.clock,data=list(self.q)),state=result(self.clock,data=self.state))
    def send_offset(self,offset,token):
        if not self.permitted or token!=self.rev:raise RuntimeError("Send permission revoked")
        self.sent.append(offset)
        if self.send_hook:self.send_hook()
        out=result(self.clock);out.update(target_rad=[0.]*5+[math.radians(offset)],offset_deg=offset)
        return out
    def stop(self):
        self.stops+=1;self.permitted=False
        if self.stop_hook:self.stop_hook()
        return result(self.clock,rc=self.stop_rc)


class FollowTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=Client(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.05,3.05),clock=self.clock)
    def offer(self,session=1,deg=5.):
        self.worker.note_input(self.clock())
        return self.worker.offer(Candidate((0.,)*5+(math.radians(deg),),self.clock(),session))
    def arm(self):
        self.offer();self.worker._cycle();self.assertEqual(self.worker.status()["state"],"active")
    def stop_confirm(self):
        with patch("teleop.aubo.follow.time.sleep",side_effect=lambda _:self.clock.advance(.051)):
            self.worker._cycle()
    def test_default_and_invalid_arguments_never_load_motion_sdk(self):
        with patch("teleop.aubo.quest_follow.load_follow_native") as load,patch("builtins.print"):
            self.assertEqual(main([]),0);load.assert_not_called()
        with patch("sys.stderr"),patch("teleop.aubo.quest_follow.load_follow_native") as load:
            for args in (["--host","192.168.1.5"],["--execute-j6"],["--preview-speed-deg-s","nan"]):
                with self.assertRaises(SystemExit):main(args)
            load.assert_not_called()
    def test_preview_speed_changes_only_candidate_policy(self):
        q=[.18,.26,2.37,2.04,1.51,.8]
        slow=make_shadow(q,[[-3.05,3.05]]*6,preview_speed_deg_s=.5)
        fast=make_shadow(q,[[-3.05,3.05]]*6,preview_speed_deg_s=5.)
        self.assertAlmostEqual(fast.session.policy.velocity_rad_s[0]/slow.session.policy.velocity_rad_s[0],10.)
        self.assertEqual(self.worker.speed_deg_s,10.)
        self.assertEqual(self.worker.client_acceleration_deg_s2,5.)
        from teleop.mujoco_session import MuJoCoFollower
        baseline=MuJoCoFollower()
        restored=make_shadow(q,[[-3.05,3.05]]*6)
        self.assertEqual(tuple(restored.session.policy.velocity_rad_s),tuple(baseline.vmax))
        self.assertEqual(restored.core.config.control_dt,baseline.dt)
        self.assertEqual(restored.core.config.max_dq_per_joint,baseline.teleop.config.max_dq_per_joint)
        with patch("teleop.aubo.quest_preview.load_native") as load,patch("builtins.print"):
            preview_main(["--preview-speed-deg-s","5"]);load.assert_not_called()
        slow.close();fast.close();restored.close()
    def test_diagnostics_are_opt_in_and_never_open_hardware_by_default(self):
        self.worker._cycle()
        report=self.worker.diagnostic_report()
        self.assertFalse(report['enabled']);self.assertEqual(report['events'],[])
        with patch('sys.stderr'),patch('teleop.aubo.quest_follow.load_follow_native') as load:
            for args in (['--diagnostic-output','/data/test.jsonl'],
                         ['--execute-j6','--diagnostic-output','/tmp/test.jsonl','--duration','10'],
                         ['--execute-j6','--diagnostic-output','/data/test.jsonl','--duration','301']):
                with self.assertRaises(SystemExit):main(args)
            load.assert_not_called()
    def test_diagnostics_preserve_feedback_and_each_send_without_extra_queries(self):
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.05,3.05),clock=self.clock,diagnostics=True)
        with patch.object(self.client,'snapshot',wraps=self.client.snapshot) as queries:
            self.arm();self.clock.advance(.101);self.offer();self.worker._cycle()
            self.assertEqual(queries.call_count,2)
        report=self.worker.diagnostic_report()
        self.assertEqual([e['event'] for e in report['events']
                          if e['event'] not in ('sdk_call_started','sdk_call_finished')],
                         ['feedback','feedback','send_attempt','send_returned'])
        attempt=report['events'][-1]['attempt']
        self.assertEqual(attempt['outcome'],'sdk_accepted')
        self.assertEqual(attempt['feedback_snapshot']['joints']['data'],[0.]*6)
        self.assertIsNotNone(attempt['sdk_result']['host_started_monotonic_s'])
        self.assertEqual(report['dropped_events'],0)
    def test_local_guard_refusal_is_not_reported_as_sdk_submission(self):
        self.arm();self.clock.advance(.101);self.offer()
        def refuse(offset,token):
            out=result(self.clock,rc=None)
            out.update(sdk_invoked=False,local_refusal_reason='J6 target age/range/step budget violated',
                       pre_send=dict(command_feedback_error_deg=3.1,following_error_limit_deg=None))
            return out
        self.client.send_offset=refuse;self.worker._cycle()
        status=self.worker.status();attempt=status['last_attempt']
        self.assertEqual(attempt['outcome'],'not_submitted_local_guard')
        self.assertEqual(status['submitted'],0)
        self.assertEqual(attempt['sdk_result']['pre_send']['command_feedback_error_deg'],3.1)
        self.assertTrue(status['fatal']);self.assertTrue(self.worker._stop_needed)
        self.stop_confirm();self.assertTrue(self.worker.status()['stop_confirmed'])
        self.assertFalse(self.offer(session=2));self.assertEqual(self.client.sent,[])
    def test_diagnostic_memory_is_bounded_and_reports_dropped_events(self):
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.05,3.05),clock=self.clock,diagnostics=True)
        for i in range(20005):self.worker._diagnose('test',sequence=i)
        report=self.worker.diagnostic_report()
        self.assertEqual(len(report['events']),20000)
        self.assertEqual(report['total_events'],20005);self.assertEqual(report['dropped_events'],5)
        self.assertEqual(report['events'][0]['sequence'],5)
    def test_idle_feedback_never_sends_or_stops(self):
        self.worker._cycle();self.assertEqual(self.client.sent,[]);self.assertEqual(self.client.stops,0)
    def test_latest_target_is_rate_limited_with_no_catch_up(self):
        self.arm();self.clock.advance(.101);self.offer(deg=9.);self.worker._cycle()
        self.assertEqual(len(self.client.sent),1);self.assertLessEqual(self.client.sent[0],1.)
        self.assertAlmostEqual(self.client.sent[0],.98)
        self.clock.advance(.7);self.offer();self.worker._cycle()
        self.assertEqual(len(self.client.sent),1);self.assertTrue(self.worker._stop_needed)
        self.assertFalse(self.worker.status()['fatal'])
    def test_227ms_gap_with_fresh_inputs_does_not_catch_up_or_fault(self):
        self.arm();self.clock.advance(.227);self.offer(deg=90.)
        self.worker._cycle()
        self.assertEqual(len(self.client.sent),1)
        self.assertLessEqual(self.client.sent[0],10.*self.worker.period_s)
        self.assertFalse(self.worker.status()['fatal']);self.assertFalse(self.worker._stop_needed)
    def test_10hz_waits_for_100ms_before_next_target(self):
        self.arm();self.clock.advance(.051);self.offer();self.worker._cycle()
        self.assertEqual(self.client.sent,[])
        self.clock.advance(.05);self.offer();self.worker._cycle()
        self.assertEqual(len(self.client.sent),1)
        self.assertLessEqual(self.client.sent[0],1.)
        self.assertEqual(self.worker.status()['requested_frequency_hz'],10.)
    def test_push_source_and_reception_time_survive_feedback_cache(self):
        original=self.client.snapshot
        def pushed():
            snap=original();snap['joints']['source']='sdk_waypoint_push'
            snap['joints']['host_received_monotonic_s']=self.clock()-.03
            return snap
        self.client.snapshot=pushed;self.worker._cycle()
        sample=self.worker.cache.require_recent(.2)
        self.assertEqual(sample.source,'sdk_waypoint_push')
        self.assertAlmostEqual(self.clock()-sample.received_monotonic_s,.03)
        self.assertEqual(self.client.sent,[])
    def test_expired_push_validation_cannot_arm_or_send(self):
        original=self.client.snapshot
        def expired():
            snap=original();snap['validation_error']=dict(type='RuntimeError',message='Waypoint feedback missing or expired')
            return snap
        self.client.snapshot=expired;self.offer()
        with self.assertRaisesRegex(RuntimeError,'Waypoint feedback missing or expired'):
            self.worker._cycle()
        self.assertFalse(self.client.permitted);self.assertEqual(self.client.sent,[])
    def test_excessive_gap_requires_stop_before_reengagement_but_is_not_device_fault(self):
        self.arm();self.clock.advance(.651);self.offer();self.worker._cycle()
        self.assertEqual(self.worker.status()['fault'],'send_period_over_budget')
        self.assertFalse(self.offer(session=2));self.stop_confirm()
        self.assertTrue(self.worker.status()['stop_confirmed']);self.assertFalse(self.worker.status()['fatal'])
        self.offer(session=2);self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'active')
    def test_j6_negative_target_can_move_below_initial_angle(self):
        self.arm();self.clock.advance(.101);self.offer(deg=-90.);self.worker._cycle()
        self.assertLess(self.client.sent[0],0.)
        self.assertGreaterEqual(self.client.sent[0],-1.)
    def test_command_feedback_difference_is_diagnostic_without_host_cutoff(self):
        self.assertIsNone(self.worker.status()['following_error_limit_deg'])
        self.arm()
        for _ in range(10):
            self.clock.advance(.101);self.offer(deg=30.);self.worker._cycle()
        self.assertEqual(len(self.client.sent),10)
        self.assertGreater(self.client.sent[-1],3.)
        previous=0.
        for target in self.client.sent:
            self.assertLessEqual(abs(target-previous),1.+1e-9)
            previous=target
        self.assertFalse(self.worker.status()['fatal'])
        self.worker.revoke('grip_released');self.stop_confirm()
        self.assertTrue(self.worker.status()['stop_confirmed'])
    def test_controller_range_is_absolute_and_projection_supports_both_directions(self):
        anchor=(0.,)*5+(math.radians(47.),)
        limits=tuple(math.radians(v) for v in (-174.8998,174.8998))
        worker=J6FollowWorker(self.client,anchor,j6_range_rad=limits,clock=self.clock)
        self.assertEqual(worker.project_j6(-10.),limits[0])
        self.assertEqual(worker.project_j6(10.),limits[1])
        self.assertAlmostEqual(worker.status()['offset_range_deg'][0],-221.8998)
        self.assertAlmostEqual(worker.status()['offset_range_deg'][1],127.8998)
        self.assertEqual(worker.status()['range_source'],'controller_joint_ranges')
    def test_missing_invalid_or_anchor_excluding_controller_range_is_refused(self):
        for limits in ((),(0.,),(0.,0.),(1.,2.),(float('nan'),1.),(-1.,float('inf'))):
            with self.subTest(limits=limits), self.assertRaises(ValueError):
                J6FollowWorker(self.client,[0.]*6,j6_range_rad=limits,clock=self.clock)
        self.assertEqual(self.client.sent,[])
    def test_joint_feedback_is_not_limited_to_positive_ten_degrees(self):
        for angle in (-100.,100.):
            self.client.q[5]=math.radians(angle)
            q,_=self.worker._observe()
            self.assertAlmostEqual(math.degrees(q[5]),angle)
        self.client.q[5]=3.05+math.radians(.11)
        with self.assertRaisesRegex(RuntimeError,'outside_j6_envelope'):
            self.worker._observe()
    def test_rate_limited_commands_cannot_cross_either_controller_boundary(self):
        for sign in (-1.,1.):
            with self.subTest(sign=sign):
                self.setUp()
                self.client.q[5]=sign*3.0499
                self.arm()
                self.clock.advance(.101);self.offer(deg=sign*300.);self.worker._cycle()
                self.assertEqual(len(self.client.sent),1)
                self.assertAlmostEqual(math.radians(self.client.sent[0]),sign*3.05)
    def test_release_clears_pending_and_stops_before_fresh_engagement(self):
        self.arm();self.worker.revoke("grip_released")
        self.assertFalse(self.offer());self.stop_confirm()
        self.assertEqual(self.client.stops,1);self.assertTrue(self.worker.status()["stop_confirmed"])
        self.assertEqual(self.client.sent,[])
        self.offer(session=2);self.worker._cycle();self.assertEqual(self.worker.status()["state"],"active")
    def test_733ms_slow_stop_confirms_and_allows_fresh_engagement(self):
        self.arm();self.worker.revoke("grip_released")
        def slow_stop():
            started=self.clock()
            self.clock.advance(.3)
            self.worker.watchdog_tick()
            self.worker.revoke("grip_released")
            self.assertEqual(self.worker.status()["state"],"stopping")
            self.assertFalse(self.worker.status()["fatal"])
            self.clock.advance(.433)
            out=result(self.clock,op="slow_control_stop")
            out.update(duration_s=self.clock()-started,host_started_monotonic_s=started)
            return out
        self.client.stop=slow_stop
        self.stop_confirm()
        self.assertTrue(self.worker.status()["stop_confirmed"])
        self.assertFalse(self.worker.status()["fatal"])
        self.assertAlmostEqual(self.worker.status()["last_stop"]["duration_s"],.733)
        self.offer(session=2);self.worker._cycle()
        self.assertEqual(self.worker.status()["state"],"active")
    def test_late_stop_success_still_checks_feedback_but_cannot_rearm(self):
        self.arm();self.worker.revoke("grip_released")
        def late_stop():
            started=self.clock();self.clock.advance(2.1);self.worker.watchdog_tick()
            out=result(self.clock,op="slow_control_stop")
            out.update(duration_s=2.1,host_started_monotonic_s=started)
            return out
        self.client.stop=late_stop
        self.stop_confirm()
        self.assertTrue(self.worker.status()["fatal"])
        self.assertTrue(self.worker.status()["stop_confirmed"])
        self.assertFalse(self.worker.status()["stop_result_unknown"])
        self.assertTrue(self.worker.status()["operator_stop_required"])
        self.assertFalse(self.offer(session=2))
        self.assertEqual(self.worker.status()["first_fatal"]["sdk_inflight"]["operation"],"slow_stop")
    def test_300ms_fresh_stop_query_confirms_and_requires_new_engagement(self):
        self.arm();self.worker.revoke("grip_released")
        original=self.client.snapshot
        def delayed_snapshot():
            self.clock.advance(.3);self.worker.watchdog_tick()
            self.assertFalse(self.client.permitted)
            self.assertFalse(self.worker.status()['fatal'])
            return original()
        self.client.snapshot=delayed_snapshot
        with patch('teleop.aubo.j6_trial.time.monotonic',self.clock):
            self.stop_confirm()
        status=self.worker.status()
        self.assertTrue(status['stop_confirmed']);self.assertFalse(status['fatal'])
        self.assertTrue(status['last_snapshot']['observation_budget_exceeded'])
        self.assertIsNone(self.worker._wanted);self.assertEqual(self.client.sent,[])
        self.client.snapshot=original
        self.offer(session=2);self.worker._cycle()
        self.assertEqual(self.worker.status()['state'],'active')
    def test_300ms_query_with_fresh_input_and_feedback_no_longer_faults(self):
        self.arm();original=self.client.snapshot
        def delayed_snapshot():
            self.clock.advance(.3);self.offer();self.worker.watchdog_tick()
            return original()
        self.client.snapshot=delayed_snapshot
        with patch('teleop.aubo.j6_trial.time.monotonic',self.clock):
            self.worker._observe()
        self.assertFalse(self.worker.status()['fatal']);self.assertFalse(self.worker._stop_needed)
        self.assertFalse(self.worker.status()['last_snapshot']['sdk_observation_budget_exceeded'])
        self.assertEqual(self.client.sent,[])
    def test_query_over_500ms_still_faults(self):
        self.arm();original=self.client.snapshot
        def delayed_snapshot():
            self.clock.advance(.51);self.offer();self.worker.watchdog_tick()
            return original()
        self.client.snapshot=delayed_snapshot
        with patch('teleop.aubo.j6_trial.time.monotonic',self.clock), self.assertRaisesRegex(RuntimeError,'500 ms'):
            self.worker._observe()
        self.assertTrue(self.worker.status()['fatal'])
        self.assertTrue(self.worker._stop_needed)
        self.assertEqual(self.client.sent,[])
    def test_300ms_sdk_send_with_fresh_input_is_not_treated_as_late(self):
        self.arm();self.clock.advance(.101);self.offer()
        def during_call():
            self.clock.advance(.3);self.offer();self.worker.watchdog_tick()
        self.client.send_hook=during_call;self.worker._cycle()
        self.assertFalse(self.worker.status()['fatal']);self.assertFalse(self.worker._stop_needed)
        self.assertEqual(self.worker.status()['submitted'],1)
    def test_500ms_call_allowance_does_not_accept_300ms_old_joints(self):
        original=self.client.snapshot
        def stale_joints():
            snap=original();self.clock.advance(.3);snap['state']=result(self.clock,data=0)
            return snap
        self.client.snapshot=stale_joints
        with patch('teleop.aubo.j6_trial.time.monotonic',self.clock), self.assertRaisesRegex(RuntimeError,'feedback_reception_expired'):
            self.worker._observe()
        self.assertFalse(self.client.permitted);self.assertEqual(self.client.sent,[])
    def test_stop_query_uses_remaining_deadline_not_two_seconds_each_time(self):
        self.arm();self.worker.revoke('grip_released')
        original=self.client.snapshot;calls=[]
        def delayed_snapshot():
            calls.append(self.worker.status()['sdk_inflight']['budget_s'])
            self.clock.advance(.3 if len(calls)==1 else 1.8)
            self.worker.watchdog_tick()
            return original()
        self.client.snapshot=delayed_snapshot
        with patch('teleop.aubo.j6_trial.time.monotonic',self.clock):
            self.stop_confirm()
        self.assertEqual(len(calls),2);self.assertLess(calls[1],1.7)
        status=self.worker.status()
        self.assertTrue(status['stop_result_unknown']);self.assertFalse(status['stop_confirmed'])
        self.assertTrue(status['operator_stop_required']);self.assertFalse(self.offer(session=2))
        self.assertEqual(status['first_fatal']['sdk_inflight']['operation'],'stop_snapshot')
        self.assertIsNotNone(status['last_snapshot'])
    def test_delayed_state_cannot_make_old_joints_confirm_stop(self):
        self.arm();self.worker.revoke('grip_released');original=self.client.snapshot
        def delayed_state():
            snap=original();self.clock.advance(.3)
            snap['state']=result(self.clock,data=0)
            return snap
        self.client.snapshot=delayed_state
        with patch('teleop.aubo.j6_trial.time.monotonic',self.clock):
            self.stop_confirm()
        status=self.worker.status()
        self.assertEqual(status['fault'],'stop_feedback_reception_expired')
        self.assertFalse(status['stop_confirmed']);self.assertTrue(status['stop_result_unknown'])
    def test_slow_invalid_stop_feedback_remains_invalid(self):
        self.arm();self.worker.revoke('grip_released');original=self.client.snapshot
        def invalid_snapshot():
            self.clock.advance(.3);snap=original();snap['joints']['data'][5]=float('nan')
            return snap
        self.client.snapshot=invalid_snapshot
        with patch('teleop.aubo.j6_trial.time.monotonic',self.clock):
            self.stop_confirm()
        status=self.worker.status()
        self.assertFalse(status['stop_confirmed']);self.assertTrue(status['stop_result_unknown'])
        self.assertIn('validation_error',status['last_snapshot'])
    def test_stop_rc_zero_does_not_confirm_while_robot_reports_motion(self):
        self.arm();self.client.state=1;self.worker.revoke("grip_released");self.stop_confirm()
        self.assertTrue(self.worker.status()["stop_result_unknown"])
        self.assertFalse(self.worker.status()["stop_confirmed"])
        self.assertFalse(self.offer(session=2));self.worker._cycle();self.assertEqual(self.client.stops,1)
    def test_failed_stop_is_latched_and_never_retried(self):
        self.arm();self.client.stop_rc=10001;self.worker.revoke("grip_released");self.stop_confirm()
        self.assertTrue(self.worker.status()["operator_stop_required"])
        self.assertTrue(self.worker.status()["stop_result_unknown"])
        self.worker._cycle();self.assertEqual(self.client.stops,1)
        self.worker.revoke("grip_released");self.worker.revoke("hardware_feedback_unavailable")
        self.assertEqual(self.worker.status()["state"],"stop_unknown")
        self.assertFalse(self.worker._stop_needed)
        self.worker._cycle();self.assertEqual(self.client.stops,1)
    def test_non_j6_motion_revokes_and_requests_stop(self):
        self.arm();self.client.q[0]=math.radians(.11);self.clock.advance(.06)
        with patch.object(self.worker._quit,"is_set",side_effect=[False,False,True]):
            self.worker._run()
        self.assertTrue(self.worker.status()["fatal"])
        self.assertTrue(self.worker.status()["stop_result_unknown"])
        self.assertEqual(self.client.stops,1)
        self.assertEqual(self.client.sent,[])

    def test_close_before_threads_start_is_safe_and_does_not_stop_hardware(self):
        self.worker.close()
        self.assertEqual(self.client.stops,0)

    def test_stop_feedback_must_be_stable_as_well_as_state_zero(self):
        self.arm();self.worker.revoke("grip_released")
        original=self.client.snapshot
        def moving_snapshot():
            self.client.q[5]+=math.radians(.03)
            return original()
        self.client.snapshot=moving_snapshot
        self.stop_confirm()
        self.assertTrue(self.worker.status()["stop_result_unknown"])
        self.assertFalse(self.worker.status()["stop_confirmed"])
    def test_watchdog_rejects_old_input_without_waiting_for_sdk_owner(self):
        self.arm();self.clock.advance(1.01);self.worker.watchdog_tick()
        self.assertTrue(self.worker._stop_needed);self.assertIsNone(self.worker._wanted)
        self.assertEqual(self.worker.status()['first_stop_trigger']['reason'],'input_timeout')
        self.assertEqual(self.client.sent,[])
    def test_candidate_timeout_revokes_even_with_fresh_input(self):
        self.arm();self.clock.advance(.11);self.worker.note_input(self.clock());self.worker.watchdog_tick()
        self.assertTrue(self.worker._stop_needed)
    def test_first_fatal_survives_secondary_feedback_and_transport_errors(self):
        self.arm();self.clock.advance(.21)
        self.worker.revoke("send_period_over_budget",fatal=True)
        first=self.worker.status()["first_fatal"]
        self.worker.revoke("hardware_feedback_unavailable")
        self.worker.revoke("invalid_xr_frame",fatal=True)
        self.assertEqual(self.worker.status()["fault"],"send_period_over_budget")
        self.assertEqual(self.worker.status()["first_fatal"],first)
        self.assertEqual(first["prior_state"],"active")
        self.assertAlmostEqual(first["candidate_age_s"],.21)
        self.assertEqual(self.worker.status()["first_stop_trigger"],first)
        self.assertEqual(self.client.sent,[])
    def test_first_stop_trigger_is_preserved_until_new_engagement(self):
        self.arm();self.worker.revoke("grip_released")
        first=self.worker.status()["first_stop_trigger"]
        self.worker.revoke("waiting_for_stop_confirmation")
        self.stop_confirm()
        self.assertEqual(self.worker.status()["first_stop_trigger"],first)
        self.assertIsNone(self.worker.status()["first_fatal"])
        self.offer(session=2);self.worker._cycle()
        self.assertIsNone(self.worker.status()["first_stop_trigger"])
    def test_blocked_call_permission_revoked_and_late_result_not_rearmed(self):
        self.arm();self.clock.advance(.101);self.offer()
        def during_call():
            self.clock.advance(.51);self.worker.watchdog_tick()
            self.assertFalse(self.client.permitted)
            self.assertTrue(self.worker.status()["operator_stop_required"])
            self.assertEqual(self.client.stops,0) # No invented concurrent stop.
        self.client.send_hook=during_call;self.worker._cycle()
        self.assertTrue(self.worker.status()["fatal"])
        self.assertTrue(self.worker._stop_needed)
        self.assertEqual(self.worker.status()["last_attempt"]["outcome"],"sdk_accepted")
        self.assertEqual(len(self.client.sent),1)
        self.stop_confirm();self.assertEqual(self.client.stops,1)
        self.assertFalse(self.offer(session=2))
    def test_real_worker_watchdog_runs_during_blocking_python_device_call(self):
        client=Client(time.monotonic);worker=J6FollowWorker(client,[0.]*6,j6_range_rad=(-3.05,3.05))
        entered=threading.Event();release=threading.Event()
        client.send_hook=lambda:(entered.set(),release.wait(2.))
        worker.start()
        try:
            start=time.monotonic()
            while not entered.is_set() and time.monotonic()-start<1.:
                worker.note_input(time.monotonic())
                worker.offer(Candidate((0.,)*5+(.1,),time.monotonic(),1))
                time.sleep(.01)
            self.assertTrue(entered.is_set())
            deadline=time.monotonic()+1.
            while not worker.status().get("operator_stop_required") and time.monotonic()<deadline:time.sleep(.01)
            self.assertTrue(worker.status()["fatal"])
            self.assertFalse(client.permitted)
            self.assertEqual(client.stops,0)
        finally:
            release.set();worker.close()
        self.assertEqual(len(client.sent),1);self.assertEqual(client.stops,1)
    def test_nan_candidate_cannot_leave_old_target_pending(self):
        self.arm()
        self.assertFalse(self.worker.offer(Candidate((float("nan"),)*6,self.clock(),1)))
        self.assertIsNone(self.worker._wanted);self.assertTrue(self.worker._stop_needed)

    def test_inflight_status_snapshot_is_not_mutated_by_completed_call(self):
        self.arm();self.clock.advance(.101);self.offer()
        observed=[]
        self.client.send_hook=lambda:observed.append(self.worker.status()['last_attempt'])
        self.worker._cycle()
        self.assertEqual(observed[0]['outcome'],'in_flight_result_unknown')
        self.assertNotIn('sdk_result',observed[0])
        self.assertEqual(self.worker.status()['last_attempt']['outcome'],'sdk_accepted')

    def test_invalid_raw_feedback_remains_reportable_in_status(self):
        status=PreviewStatus();raw={'feedback':{'joints':[float('nan'),1.]}}
        status.update(raw)
        saved=status.read()
        self.assertEqual(saved['feedback']['joints'],[None,1.])
        self.assertIn('.feedback.joints[0]',saved['nonfinite_status_fields'])
        json.dumps(saved,allow_nan=False)
        self.assertTrue(math.isnan(raw['feedback']['joints'][0]))

    def test_rejected_follow_is_not_retried_and_raw_result_is_kept(self):
        self.arm();self.clock.advance(.101);self.offer()
        original=self.client.send_offset
        def fail(offset,token):
            out=original(offset,token);out['return_code']=10001;return out
        self.client.send_offset=fail
        self.worker._cycle()
        attempt=self.worker.status()['last_attempt']
        self.assertEqual(attempt['sdk_result']['return_code'],10001)
        self.assertEqual(attempt['outcome'],'sdk_rejected')
        self.stop_confirm();self.assertFalse(self.offer(session=2))
        self.assertEqual(len(self.client.sent),1)

    def test_exception_keeps_unknown_submission_record(self):
        self.arm();self.clock.advance(.101);self.offer()
        def fail():raise RuntimeError('lost response')
        self.client.send_hook=fail
        with self.assertRaisesRegex(RuntimeError,'lost response'):self.worker._cycle()
        attempt=self.worker.status()['last_attempt']
        self.assertEqual(attempt['outcome'],'exception_result_unknown')
        self.assertEqual(attempt['exception']['message'],'lost response')
        self.assertEqual(len(self.client.sent),1)


class QuestRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=Client(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.05,3.05),clock=self.clock)
        self.shadow=make_shadow([0.]*6,[[-3.05,3.05]]*6)
        self.runtime=J6QuestRuntime(self.shadow,self.worker,control_dt=.01,clock=self.clock)
    def tearDown(self):self.runtime.close()
    def frame(self,grip,seq=0,epoch="test"):
        return dict(type="xr_frame",epoch=epoch,seq=seq,t_client=10.,controllers={"right":dict(
            position=[0.,0.,0.],orientation=[0.,0.,0.,1.],buttons=[dict(p=False,v=0.),dict(p=grip,v=float(grip))])})
    def test_release_revokes_on_receiver_without_waiting_for_control_tick(self):
        self.worker._may_move=True;self.client.permitted=True
        self.runtime.submit_xr_frame(self.frame(False))
        self.assertFalse(self.client.permitted);self.assertTrue(self.worker._stop_needed)
    def test_missing_hand_or_reference_change_immediately_revokes(self):
        self.runtime.submit_xr_frame(self.frame(False))
        self.runtime.submit_xr_frame(self.frame(True,1))
        self.client.permitted=True;self.runtime.submit_xr_frame(self.frame(True,0,"new"))
        self.assertFalse(self.client.permitted)
        self.client.permitted=True;self.runtime.submit_xr_frame(dict(type="xr_frame",controllers={}))
        self.assertFalse(self.client.permitted)
    def test_pending_release_before_stop_confirmation_cannot_rearm(self):
        self.runtime.submit_xr_frame(self.frame(False))
        self.worker._status["state"]="stopping"
        self.runtime.step()
        self.assertIsNone(self.runtime._release_frame)
        self.assertIsNone(self.runtime._latest_frame)
        self.assertTrue(self.shadow.session.require_release)
    def test_feedback_expiry_after_stopped_fatal_keeps_original_reason(self):
        self.worker._cycle()
        self.worker.revoke("sdk_call_over_budget",fatal=True)
        self.clock.advance(.3)
        self.runtime.refresh_feedback()
        self.assertEqual(self.worker.status()["fault"],"sdk_call_over_budget")
        self.assertEqual(self.worker.status()["first_fatal"]["reason"],"sdk_call_over_budget")
        self.assertIsNone(self.shadow.session.cache.latest().q_rad)
        self.assertEqual(self.client.sent,[])
    def test_stale_feedback_during_serial_stop_is_not_a_new_device_fault(self):
        self.worker._cycle()
        self.worker._may_move=True
        self.worker.revoke("grip_released")
        self.worker._status["state"]="stopping"
        self.clock.advance(.3)
        revision=self.client.rev
        self.runtime.refresh_feedback()
        self.assertEqual(self.client.rev,revision)
        self.assertEqual(self.worker.status()["state"],"stopping")
        self.assertEqual(self.worker.status()["fault"],"grip_released")
        self.assertFalse(self.shadow.session.cache.latest().valid)
        self.assertIsNone(self.runtime.step())
        self.assertTrue(self.shadow.session.require_release)
        self.assertEqual(self.client.sent,[])
    def test_stop_unknown_does_not_lose_confirmation_error_to_expired_cache(self):
        self.worker._cycle()
        self.worker._status.update(state='stop_unknown',fault='Waypoint feedback missing or expired',
            stop_confirmation_error=dict(type='RuntimeError',message='Waypoint feedback missing or expired'))
        self.worker._fatal=self.worker._unknown=True
        self.clock.advance(.3);revision=self.client.rev
        self.runtime.refresh_feedback()
        status=self.worker.status()
        self.assertEqual(self.client.rev,revision)
        self.assertEqual(status['fault'],'Waypoint feedback missing or expired')
        self.assertEqual(status['stop_confirmation_error']['message'],'Waypoint feedback missing or expired')
        self.assertIsNone(self.runtime.step());self.assertEqual(self.client.sent,[])
    def test_real_websocket_release_routes_through_hardware_revoke(self):
        async def exercise():
            import websockets
            async def handler(ws,path=None):
                await ws.send(json.dumps(self.frame(False)))
                await ws.wait_closed()
            async with websockets.serve(handler,"127.0.0.1",0) as server:
                receiver=QuestReceiver(self.runtime,f"ws://127.0.0.1:{server.sockets[0].getsockname()[1]}")
                receiver.start();before=self.client.rev
                try:
                    deadline=time.monotonic()+1.
                    while self.client.rev==before and time.monotonic()<deadline:await asyncio.sleep(.01)
                    self.assertGreater(self.client.rev,before)
                    self.assertIsNone(self.shadow.input_time)
                    self.assertEqual(self.client.sent,[])
                finally:
                    receiver.stop.set();await asyncio.to_thread(receiver.close)
        asyncio.run(exercise())

    def test_author_ik_drives_j6_worker_without_fake_dispatch(self):
        q=(.18,.26,2.37,2.04,1.51,.8)
        self.client.q=list(q)
        self.worker=J6FollowWorker(self.client,q,j6_range_rad=(-3.05,3.05),clock=self.clock)
        self.shadow.close()
        self.shadow=make_shadow(q,[[-3.05,3.05]]*6,preview_speed_deg_s=5.)
        self.shadow.clock=self.clock
        self.shadow.session.clock=self.clock
        self.shadow.session.cache=FeedbackCache(self.clock)
        self.runtime=J6QuestRuntime(self.shadow,self.worker,control_dt=.01,clock=self.clock)
        generated=[]
        with patch("vr_teleop_kit.core.quest_teleop.time.monotonic",self.clock):
            for i in range(80):
                self.clock.advance(.01)
                self.runtime.submit_feedback(q,received_monotonic_s=self.clock())
                frame=self.frame(i>1,i)
                frame['t_client']=self.clock()*1000
                if i>2:
                    frame['controllers']['right']['orientation']=[0.,math.sin(.15),0.,math.cos(.15)]
                self.runtime.submit_xr_frame(frame)
                candidate=self.runtime.step()
                self.worker._cycle()
                if candidate:
                    self.assertEqual(candidate.q_rad[:5],q[:5])
                    generated.append(self.worker.status()['last_attempt'])
        self.assertTrue(self.shadow.session.engaged)
        self.assertGreater(len(self.client.sent),0)
        self.assertEqual(self.shadow.session.backend.submitted,[])
        latest=self.worker.status()['last_attempt']
        self.assertNotEqual(latest['generated_q_rad'],list(q))
        self.assertEqual(latest['session'],self.shadow.session.session)
        lo,hi=self.worker.offset_range_deg
        self.assertTrue(all(lo<=x<=hi for x in self.client.sent))


if __name__=="__main__":unittest.main(verbosity=2)
