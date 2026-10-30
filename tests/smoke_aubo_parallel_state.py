"""Production C++ binding with an offline SDK double. No vendor SDK or sockets."""
import importlib.util
import json
import os
from pathlib import Path
import sys
import threading
import time
import tempfile
import unittest
from unittest.mock import patch

root = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(root))
runtime = Path(os.environ.get("AUBO_RUNTIME_DIR", "/data/VR_teleoperation_runtime"))
binary = next((runtime/"build-follow/fake-python").glob("_aubo_follow*.so"))
def load(name):
    spec = importlib.util.spec_from_file_location(name, binary)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
native, controls = load("_aubo_follow"), load("_aubo_follow_controls")
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.follow import ObservationPending
from teleop.aubo.offline_session import Candidate
from teleop.aubo.quest_follow import J6QuestRuntime, main
from teleop.aubo.quest_preview import make_shadow


def wait_for(predicate, timeout=1.):
    deadline = time.monotonic()+timeout
    while not predicate():
        if time.monotonic() > deadline:
            raise AssertionError("Offline concurrency condition timed out")
        time.sleep(.002)


class NativeParallelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.a, cls.b = native.J6Follower(six_axis=True,follow_hz=25), native.RobotStateObserver()
        cls.a.login("offline", 1, "test", "test")
        cls.b.login("offline", 1, "test", "test")
        cls.a.start_waypoint_feedback()
        cls.b.query_state()
        cls.a.attach_state_observer(cls.b)
        prepared=cls.a.prepare()
        if prepared['send_period_s'] != .04:
            raise AssertionError('Native 25Hz prepare period mismatch')
    @classmethod
    def tearDownClass(cls):
        controls.block_query(False)
        cls.a.logout()
        cls.b.logout()
    def setUp(self):
        controls.state(0);controls.query_rc(0);controls.joint(0.);controls.push_enabled(True)
        controls.push_enabled_b(True)
        self.b.query_state();time.sleep(.004)
        self.a.revoke()
    def test_push_published_during_snapshot_read_remains_fresh(self):
        controls.push_enabled(False)
        try:
            controls.joint(.15)
            count=controls.interleaved_pushes()
            controls.push_on_observation_read()
            before=time.monotonic();snap=self.a.snapshot();after=time.monotonic()
            self.assertEqual(controls.interleaved_pushes(),count+1)
            self.assertGreaterEqual(snap['joints']['host_received_monotonic_s'],before)
            self.assertLessEqual(snap['joints']['host_received_monotonic_s'],after)
            self.assertEqual(snap['joints']['data'],[.15]*6)
            self.assertFalse(snap.get('observation_pending',False))
        finally:
            controls.push_enabled(True)

    def test_push_published_during_arm_read_does_not_refuse_engagement(self):
        controls.push_enabled(False)
        try:
            count=controls.interleaved_pushes()
            controls.push_on_observation_read()
            self.assertTrue(self.a.arm(self.a.revision()))
            self.assertEqual(controls.interleaved_pushes(),count+1)
        finally:
            controls.push_enabled(True)
    def test_push_published_during_send_read_does_not_refuse_target(self):
        token=self.a.revision();self.assertTrue(self.a.arm(token))
        controls.push_enabled(False)
        try:
            controls.joint(.15)
            count=controls.interleaved_pushes()
            controls.push_on_observation_read()
            result=self.a.send_joints([.2]*6,token)
            self.assertEqual(controls.interleaved_pushes(),count+1)
            self.assertTrue(result['sdk_invoked'],result)
            self.assertEqual(result['return_code'],0)
            self.assertEqual(result['pre_send']['measured_q_rad'],[.15]*6)
        finally:
            controls.push_enabled(True)
    def test_watchdog_accepts_state_published_between_clock_and_status_reads(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b)
        worker.state_query.query_once()
        worker._may_move=True;worker.note_input(time.monotonic())
        original=worker.state_query.status;published=False
        def status_after_publish():
            nonlocal published
            if not published:
                published=True;worker.state_query.query_once()
            return original()
        with patch.object(worker.state_query,'status',side_effect=status_after_publish):
            worker.watchdog_tick()
        self.assertTrue(published)
        self.assertFalse(worker.status()['fatal'],worker.status()['first_fatal'])
        self.assertIsNone(worker.state_query.status()['error'])
        self.assertFalse(worker._stop_needed)
    def test_native_stale_and_future_push_still_cannot_authorize_motion(self):
        controls.push_enabled(False)
        try:
            for offset in (-1.,1.):
                with self.subTest(offset=offset):
                    self.b.query_state()
                    controls.push_on_observation_read()
                    token=self.a.revision();self.assertTrue(self.a.arm(token))
                    before=controls.counts()[2]
                    controls.next_push_time_offset(offset)
                    controls.push_on_observation_read()
                    snap=self.a.snapshot()
                    self.assertTrue(snap.get('observation_pending'))
                    self.assertFalse(self.a.arm(token))
                    result=self.a.send_joints([.1]*6,token)
                    self.assertFalse(result['sdk_invoked'])
                    self.assertEqual(controls.counts()[2],before)
        finally:
            controls.next_push_time_offset(0.)
            controls.push_enabled(True)
    def test_watchdog_rejects_actually_future_and_stale_state(self):
        for offset in (-1.,1.):
            with self.subTest(offset=offset):
                worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
                    joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b)
                worker._may_move=True;worker.note_input(time.monotonic())
                raw=dict(return_code=0,data=0,
                    host_received_monotonic_s=time.monotonic()+offset)
                with patch.object(worker.state_query,'status',return_value=dict(
                        sdk_inflight=None,last_result=raw)):
                    worker.watchdog_tick()
                self.assertEqual(worker.status()['fatal'],offset > 0)
                self.assertTrue(worker._stop_needed)
                self.assertEqual(worker.state_query.status()['error'],
                    'state_feedback_invalid' if offset > 0 else None)
    def test_idle_push_gap_recovers_without_permanent_fault_or_buffered_intent(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b,
            diagnostics=True,follow_hz=50)
        worker.start()
        try:
            for session in (1,2,3):
                # Keep offering current intent; each new session has only one
                # engagement, separated by a normal stop and an idle push gap.
                deadline=time.monotonic()+1.
                while worker.status()['state']!='active':
                    self.assertLess(time.monotonic(),deadline,worker.status())
                    stamp=time.monotonic();worker.note_input(stamp)
                    self.assertTrue(worker.offer(Candidate((.1,)*6,stamp,session)))
                    time.sleep(.005)
                stamp=time.monotonic();worker.note_input(stamp)
                worker.offer(Candidate((.1,)*6,stamp,session))
                before=worker.status()['submitted']
                wait_for(lambda:worker.status()['submitted']>before)
                worker.revoke('grip_released')
                wait_for(lambda:worker.status()['stop_confirmed'])
                controls.push_enabled(False)
                wait_for(lambda:time.monotonic()-self.a.waypoint_feedback()['samples'][-1]['host_received_monotonic_s']>.22)
                self.assertFalse(worker.status()['fatal'],worker.status()['first_fatal'])
                # A fresh target while feedback is unavailable must not arm,
                # send, or stay buffered for an automatic resume.
                before=controls.counts()[2]
                stamp=time.monotonic();worker.note_input(stamp)
                worker.offer(Candidate((.2,)*6,stamp,session+10))
                wait_for(lambda:worker._wanted is None)
                self.assertFalse(worker.status()['feedback_ready'])
                self.assertEqual(controls.counts()[2],before)
                self.assertFalse(self.a.arm(self.a.revision()))
                controls.push_enabled(True)
                wait_for(lambda:worker.status()['feedback_ready'])
                time.sleep(.02)
                self.assertEqual(worker.status()['state'],'idle')
                self.assertEqual(controls.counts()[2],before)
                self.assertFalse(worker.status()['fatal'])
        finally:
            controls.push_enabled(True);worker.close()
    def test_idle_stale_state_waits_but_fresh_unexpected_motion_still_stops(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b)
        # An aged B observation is not an SDK error while no motion is allowed.
        time.sleep(.21)
        worker._cycle()
        self.assertFalse(worker.status()['fatal'])
        self.assertFalse(worker.status()['feedback_ready'])
        controls.push_enabled(False);time.sleep(.21)
        controls.state(1);self.b.query_state()
        worker._cycle()
        self.assertTrue(worker.status()['fatal'])
        self.assertTrue(worker._stop_needed)
        self.assertEqual(worker.status()['first_fatal']['reason'],'unexpected_motion_while_idle')
        controls.state(0);controls.push_enabled(True);self.b.query_state()
        worker.state_query.start()
        try:
            worker._cycle()
            self.assertTrue(worker.status()['stop_confirmed'])
        finally:
            worker.state_query.close()
    def test_active_push_gap_stops_without_killing_observation_or_sending_again(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b)
        worker.start()
        try:
            deadline=time.monotonic()+1.
            while worker.status()['state']!='active':
                self.assertLess(time.monotonic(),deadline)
                stamp=time.monotonic();worker.note_input(stamp)
                worker.offer(Candidate((.1,)*6,stamp,1));time.sleep(.005)
            controls.push_enabled(False)
            while worker.status()['state'] not in ('stop_requested','stopping'):
                self.assertLess(time.monotonic(),deadline)
                stamp=time.monotonic();worker.note_input(stamp)
                worker.offer(Candidate((.1,)*6,stamp,1));time.sleep(.005)
            before=controls.counts()[2]
            self.assertFalse(worker.offer(Candidate((.2,)*6,time.monotonic(),2)))
            controls.push_enabled(True)
            wait_for(lambda:worker.status()['stop_confirmed'])
            self.assertEqual(controls.counts()[2],before)
            self.assertFalse(worker.status()['fatal'])
            self.assertIsNone(worker.state_query.status()['error'])
        finally:
            controls.push_enabled(True);worker.close()
    def test_runtime_idle_gap_requires_fresh_release_then_one_press(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b)
        shadow=make_shadow([0.]*6,[[-3,3]]*6)
        runtime=J6QuestRuntime(shadow,worker,control_dt=.01)
        seq=0
        def tick(grip):
            nonlocal seq
            seq+=1
            runtime.refresh_feedback()
            runtime.submit_xr_frame(dict(type='xr_frame',epoch='offline',seq=seq,
                t_client=time.monotonic()*1000,controllers={'right':dict(
                    position=[0.,0.,0.],orientation=[0.,0.,0.,1.],
                    buttons=[dict(p=False,v=0.),dict(p=grip,v=float(grip))])}))
            runtime.step()
            time.sleep(.01)
        worker.start()
        try:
            wait_for(lambda:worker.status()['feedback_ready'])
            tick(False)
            for _ in range(4):tick(True)
            self.assertEqual(worker.status()['state'],'active')
            tick(False)
            wait_for(lambda:worker.status()['stop_confirmed'])
            tick(False)  # Consume the normal stop gate.
            controls.push_enabled(False)
            time.sleep(.23)
            tick(True)  # Pressing while observations are unavailable is discarded.
            self.assertFalse(worker.status()['fatal'])
            self.assertFalse(runtime.status()['engaged'])
            self.assertEqual(runtime.status()['clutch_state'],'waiting_for_feedback')
            controls.push_enabled(True)
            wait_for(lambda:worker.status()['feedback_ready'])
            before=controls.counts()[2]
            for _ in range(3):tick(True)
            self.assertEqual(worker.status()['state'],'idle')
            self.assertEqual(controls.counts()[2],before)
            tick(False)
            for _ in range(4):tick(True)  # One new press, held across frames.
            self.assertEqual(worker.status()['state'],'active')
            self.assertTrue(runtime.status()['engaged'])
        finally:
            controls.push_enabled(True);runtime.close();worker.close()
    def test_cached_reads_send_and_stop_do_not_wait_for_blocked_B(self):
        token = self.a.revision();self.assertTrue(self.a.arm(token))
        initial = controls.counts()[0]
        controls.block_query(True)
        query = threading.Thread(target=self.b.query_state)
        query.start()
        try:
            wait_for(controls.query_entered)
            before = time.monotonic()
            snap = self.a.snapshot()
            result = self.a.send_joints([.1]*6, token)
            stop = self.a.stop()
            self.assertLess(time.monotonic()-before, .1)
            self.assertEqual(result["return_code"], 0)
            self.assertEqual(stop["return_code"], 0)
            self.assertEqual(snap["state"]["source"], "independent_sdk_state_cache")
            self.assertEqual(controls.counts()[0], initial)
        finally:
            controls.block_query(False);query.join(1.)
    def test_failure_disallowed_and_invalidation_cannot_authorize_send(self):
        for mode in ("failed", "paused", "resumed", "unknown", "invalidated"):
            controls.state(0);controls.query_rc(0);self.b.query_state()
            token = self.a.revision();self.assertTrue(self.a.arm(token))
            if mode == "failed":controls.query_rc(1);self.b.query_state()
            elif mode == "paused":controls.state(2);self.b.query_state()
            elif mode == "resumed":controls.state(3);self.b.query_state()
            elif mode == "unknown":controls.state(99);self.b.query_state()
            else:self.b.invalidate()
            before = controls.counts()[2]
            result = self.a.send_joints([.1]*6, token)
            self.assertFalse(result["sdk_invoked"])
            self.assertEqual(controls.counts()[2], before)
    def test_fresh_push_updates_native_pre_send_without_query(self):
        token = self.a.revision();self.assertTrue(self.a.arm(token))
        initial = controls.counts()
        controls.joint(.2);time.sleep(.008)
        result = self.a.send_joints([.3]*6, token)
        self.assertEqual(result["pre_send"]["measured_q_rad"], [.2]*6)
        self.assertEqual(controls.counts()[:2], initial[:2])
    def test_invalidation_wins_over_late_query(self):
        controls.block_query(True)
        query = threading.Thread(target=self.b.query_state)
        query.start()
        try:
            wait_for(controls.query_entered)
            self.b.invalidate()
        finally:
            controls.block_query(False);query.join(1.)
        with self.assertRaisesRegex(RuntimeError, "expired"):
            self.a.snapshot()
        self.b.query_state()
        self.assertEqual(self.a.snapshot()["state"]["data"], 0)
    def test_stale_state_refuses_send_despite_fresh_push(self):
        token = self.a.revision();self.assertTrue(self.a.arm(token))
        time.sleep(.21)
        result = self.a.send_joints([.1]*6, token)
        self.assertFalse(result["sdk_invoked"])
    def test_stop_waits_for_query_crossing_200ms_instead_of_latching_unknown(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b,
            diagnostics=True)
        worker.state_query.start()
        pending=[];observe=worker._observe
        def tracked(**kwargs):
            try:return observe(**kwargs)
            except ObservationPending:
                pending.append(time.monotonic());raise
        worker._observe=tracked
        controls.block_query(True);worker.state_query.request_refresh()
        wait_for(controls.query_entered)
        worker._may_move=True;worker.revoke('grip_released')
        release=threading.Timer(.22,lambda:controls.block_query(False))
        release.start()
        worker._watch.start()
        try:
            worker._cycle()
            self.assertGreater(len(pending),0)
            status=worker.status()
            self.assertTrue(status['stop_confirmed'])
            self.assertFalse(status['stop_result_unknown'])
            self.assertFalse(status['fatal'])
            self.assertIsNone(status['state_observer']['error'])
            rows=worker.diagnostic_report()['events']
            stopped=next(r for r in rows if r['event']=='stop_returned')
            confirmed=next(r for r in rows if r['event']=='stop_confirmed')
            self.assertGreaterEqual(confirmed['feedback']['state']['host_started_monotonic_s'],stopped['host_monotonic_s'])
        finally:
            controls.block_query(False);release.join();worker.close()
    def test_stop_stale_observation_deadline_still_fails_without_killing_B_for_motion_age(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b)
        worker.stop_feedback_budget_s=.06
        worker.state_query.start()
        controls.block_query(True);worker.state_query.request_refresh()
        wait_for(controls.query_entered);time.sleep(.21)
        worker._may_move=True;worker.revoke('grip_released')
        try:
            worker._cycle()
            self.assertTrue(worker.status()['stop_result_unknown'])
            self.assertFalse(worker.status()['stop_confirmed'])
            self.assertEqual(worker.status()['stop_confirmation_error']['message'],'stop_confirmation_timeout')
            worker.watchdog_tick()
            self.assertIsNone(worker.state_query.status()['error'])
        finally:
            controls.block_query(False);worker.state_query.close()
    def test_worker_parallel_send_stop_confirmation_and_close(self):
        worker = J6FollowWorker(self.a, [0.]*6, j6_range_rad=(-3, 3),
            joint_ranges_rad=[[-3, 3]]*6, waypoint_feedback=True, state_observer=self.b,
            diagnostics=True, follow_hz=50)
        worker.start()
        try:
            def offer():
                stamp=time.monotonic();worker.note_input(stamp)
                worker.offer(Candidate((.1,)*6, stamp, 1))
            deadline=time.monotonic()+.3
            while worker.status()["submitted"] < 2:
                self.assertLess(time.monotonic(), deadline)
                offer();time.sleep(.005)
            worker.revoke("grip_released")
            wait_for(lambda:worker.status()["stop_confirmed"])
            rows=worker.diagnostic_report()["events"]
            returned=next(r for r in rows if r["event"]=="stop_returned")
            confirm=next(r for r in rows if r["event"]=="stop_confirmed")
            self.assertGreaterEqual(confirm["feedback"]["state"]["host_started_monotonic_s"], returned["host_monotonic_s"])
            self.assertGreaterEqual(confirm["feedback"]["joints"]["host_received_monotonic_s"]-confirm["quiet_since_monotonic_s"], .1)
            self.assertFalse(worker.status()["fatal"])
            # A normal release still permits first-press reengagement, with a
            # new B observation and current joint anchor rather than old targets.
            deadline=time.monotonic()+.3
            while worker.status()["state"] != "active":
                self.assertLess(time.monotonic(), deadline)
                stamp=time.monotonic();worker.note_input(stamp)
                worker.offer(Candidate((.2,)*6, stamp, 2));time.sleep(.005)
        finally:
            worker.close()
        self.assertFalse(worker.state_query._thread.is_alive())
        self.assertFalse(worker._thread.is_alive())
    def test_cli_wires_both_connections_and_closes_without_motion(self):
        seen=[]
        def serve(*args, **kwargs):
            status=kwargs['extra_status']()['hardware']
            seen.append(status)
            self.assertEqual(status['state_query_policy'],'independent_sdk_connection')
            self.assertEqual(status['state_query_mode'],'parallel')
            self.assertEqual(status['sdk_connections'],2)
            self.assertIsNone(status['state_observer']['error'])
            self.assertEqual(status['submitted'],0)
        with tempfile.TemporaryDirectory() as isolated_root, \
             patch('teleop.aubo.quest_follow.ROOT',Path(isolated_root)), \
             patch('teleop.aubo.quest_follow.load_follow_native',return_value=native), \
             patch('teleop.aubo.quest_follow.getpass.getpass',return_value='offline'), \
             patch('teleop.aubo.quest_follow.socket.socket'), \
             patch('teleop.aubo.quest_follow.serve_quest',side_effect=serve):
            self.assertEqual(main(['--execute-six-axis','--command-mode','follow','--follow-hz','25',
                '--waypoint-feedback','--parallel-state-query','--host','offline',
                '--port','1','--user','test','--usb','--headless','--duration','1']),0)
        self.assertEqual(len(seen),1)

    def test_cli_default_and_explicit_serial_never_construct_observer(self):
        for flags in ([], ['--serial-state-query']):
            with self.subTest(flags=flags):
                seen=[]
                def serve(*args, **kwargs):
                    status=kwargs['extra_status']()['hardware']
                    seen.append(status)
                    self.assertEqual(status['state_query_mode'],'serial')
                    self.assertEqual(status['sdk_connections'],1)
                    self.assertTrue(status['serial_sdk_calls'])
                    self.assertIsNone(status['state_observer'])
                    self.assertEqual(status['submitted'],0)
                with tempfile.TemporaryDirectory() as isolated_root, \
                     patch('teleop.aubo.quest_follow.ROOT',Path(isolated_root)), \
                     patch('teleop.aubo.quest_follow.load_follow_native',return_value=native), \
                     patch.object(native,'RobotStateObserver',side_effect=AssertionError('Serial mode created B')) as observer, \
                     patch('teleop.aubo.quest_follow.getpass.getpass',return_value='offline'), \
                     patch('teleop.aubo.quest_follow.socket.socket'), \
                     patch('teleop.aubo.quest_follow.serve_quest',side_effect=serve):
                    self.assertEqual(main(['--execute-six-axis','--command-mode','follow','--follow-hz','25',
                        '--waypoint-feedback','--host','offline','--port','1','--user','test',
                        '--usb','--headless','--duration','1']+flags),0)
                    observer.assert_not_called()
                self.assertEqual(len(seen),1)
    def test_worker_B_timeout_revokes_and_late_return_does_not_restore_cache(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b)
        worker.state_query.start()
        worker._may_move=True
        token=self.a.revision();self.assertTrue(self.a.arm(token))
        controls.block_query(True);worker.state_query.request_refresh()
        try:
            wait_for(controls.query_entered)
            # Expiry must be noticed even if A is blocked in its own SDK call.
            time.sleep(.51);worker.watchdog_tick()
            self.assertTrue(worker.status()['fatal'])
            self.assertTrue(worker._stop_needed)
            before=controls.counts()[2]
            with self.assertRaisesRegex(RuntimeError,'revoked'):
                self.a.send_joints([.1]*6,token)
            self.assertEqual(controls.counts()[2],before)
            controls.block_query(False)
            wait_for(lambda:not worker.state_query._thread.is_alive())
            with self.assertRaisesRegex(RuntimeError,'expired'):self.a.snapshot()
        finally:
            controls.block_query(False);worker.state_query.close();self.a.stop()
    def test_state_expiry_both_paths_keep_query_alive_confirm_stop_and_reengage(self):
        for path in ('watchdog','snapshot'):
            with self.subTest(path=path):
                controls.state(0);self.b.query_state()
                worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
                    joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,
                    state_observer=self.b,diagnostics=True,follow_hz=25)
                worker.state_query.start()
                token=self.a.revision();self.assertTrue(self.a.arm(token))
                worker._may_move=True;worker._status['state']='active'
                worker.note_input(time.monotonic())
                controls.block_query(True);worker.state_query.request_refresh()
                wait_for(controls.query_entered)
                try:
                    time.sleep(.21)
                    before=controls.counts()[2]
                    if path=='watchdog':worker.watchdog_tick()
                    else:worker._cycle()
                    self.assertTrue(worker._stop_needed)
                    self.assertFalse(worker.status()['fatal'])
                    self.assertIsNone(worker.state_query.status()['error'])
                    self.assertTrue(worker.state_query._thread.is_alive())
                    self.assertFalse(worker.offer(Candidate((.1,)*6,time.monotonic(),2)))
                    with self.assertRaisesRegex(RuntimeError,'revoked'):
                        self.a.send_joints([.1]*6,token)
                    release=threading.Timer(.025,lambda:controls.block_query(False))
                    release.start()
                    worker._cycle();release.join()
                    self.assertTrue(worker.status()['stop_confirmed'])
                    self.assertFalse(worker.status()['stop_result_unknown'])
                    self.assertFalse(worker.status()['fatal'])
                    self.assertTrue(worker.status()['stop_requires_post_confirm_release'])
                    self.assertEqual(controls.counts()[2],before)
                    rows=worker.diagnostic_report()['events']
                    stopped=next(r for r in rows if r['event']=='stop_returned')
                    confirmed=next(r for r in rows if r['event']=='stop_confirmed')
                    self.assertGreaterEqual(confirmed['feedback']['state']['host_started_monotonic_s'],stopped['host_monotonic_s'])
                    deadline=time.monotonic()+.5
                    while worker.status()['state']!='active':
                        self.assertLess(time.monotonic(),deadline)
                        stamp=time.monotonic();worker.note_input(stamp)
                        self.assertTrue(worker.offer(Candidate((.1,)*6,stamp,2)))
                        worker._cycle();time.sleep(.005)
                    worker.revoke('grip_released');worker._cycle()
                finally:
                    controls.block_query(False);worker.state_query.close()
    def test_native_send_expiry_after_python_observation_is_recoverable_stop(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,
            state_observer=self.b,follow_hz=25)
        worker.state_query.start()
        try:
            token=self.a.revision();self.assertTrue(self.a.arm(token))
            worker._may_move=True;worker._active_session=1;worker._token=token
            worker._last_send_s=time.monotonic()-.05;worker._last_sent_q=(0.,)*6
            worker._last_sent_offset=0.;worker._status['state']='active'
            stamp=time.monotonic();worker.note_input(stamp)
            worker.offer(Candidate((.1,)*6,stamp,1))
            observe=worker._observe
            def age_after_observation(**kwargs):
                value=observe(**kwargs)
                controls.push_enabled(False);controls.next_push_time_offset(-1.)
                controls.push_on_observation_read()
                return value
            before=controls.counts()[2]
            with patch.object(worker,'_observe',side_effect=age_after_observation):
                worker._cycle()
            self.assertEqual(controls.counts()[2],before)
            self.assertTrue(worker._stop_needed)
            self.assertFalse(worker.status()['fatal'])
            controls.push_enabled(True);worker._cycle()
            self.assertTrue(worker.status()['stop_confirmed'])
        finally:
            controls.push_enabled(True);worker.state_query.close()
    def test_stop_confirmation_keeps_polling_paused_resumed_until_fresh_stopped(self):
        worker=J6FollowWorker(self.a,[0.]*6,j6_range_rad=(-3,3),
            joint_ranges_rad=[[-3,3]]*6,waypoint_feedback=True,state_observer=self.b,
            diagnostics=True)
        worker.state_query.start()
        seen=[]
        observe=worker._observe
        def track(**kwargs):
            value=observe(**kwargs);seen.append(value[1]);return value
        worker._observe=track
        worker._may_move=True
        controls.state(2);self.b.query_state()
        worker.revoke('grip_released')
        thread=threading.Thread(target=worker._cycle)
        thread.start()
        try:
            wait_for(lambda:2 in seen)
            self.assertFalse(worker.status()['fatal'])
            self.assertFalse(worker.status()['stop_confirmed'])
            self.assertTrue(worker.state_query._thread.is_alive())
            controls.state(3);worker.state_query.request_refresh()
            wait_for(lambda:3 in seen)
            self.assertFalse(worker.status()['stop_confirmed'])
            controls.state(0);worker.state_query.request_refresh()
            thread.join(1.)
            self.assertFalse(thread.is_alive())
            self.assertTrue(worker.status()['stop_confirmed'])
            self.assertFalse(worker.status()['fatal'])
            self.assertFalse(worker.status()['stop_result_unknown'])
        finally:
            controls.state(0);thread.join(2.);worker.state_query.close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
