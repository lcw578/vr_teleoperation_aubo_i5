"""Read-only endpoint teaching: synthetic observations; no SDK or robot."""
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

from teleop.aubo.placement_teach import (
    collect_reference, main, read_local_status, save_new_reference, stationary_observation)


class Clock:
    def __init__(self):
        self.now = 100.

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def status(now, q=None):
    return dict(mode='aubo_six_axis_supervised_follow', gripper_installed=False,
        waypoint_feedback=True, closed=False, engaged=False,
        session_finished=False, session_finish_pending=False,
        home_return=dict(state='idle', target_q_rad=[0.]*6),
        # Candidate deliberately differs from the observed robot.
        candidate_q_rad=[2.]*6, measured_q_rad=[1.]*6,
        hardware=dict(state='idle', fatal=False, stop_result_unknown=False,
            execution_scope='six_axis', command_mode='follow', joint_ranges_deg=[[-170.,170.]]*6,
            last_snapshot=dict(
                joints=dict(return_code=0, source='sdk_waypoint_push', data=q or [.1]*6,
                            host_received_monotonic_s=now-.01),
                state=dict(return_code=0, data=0, host_received_monotonic_s=now-.02))))


class TeachingTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.home = self.root/'home.json'
        self.home.write_text(json.dumps(dict(schema='aubo_reference_home_v1',
            joint_source='sdk_waypoint_push', joint_names=[f'joint_{n}' for n in range(1,7)],
            q_rad=[0.]*6)))
        self.clock = Clock()

    def collect(self, reader):
        return collect_reference(self.home, read_status=reader, clock=self.clock, sleep=self.clock.sleep)

    def test_stable_capture_uses_actual_feedback_and_binds_home_without_enabling_motion(self):
        point = self.collect(lambda:status(self.clock()))
        self.assertEqual(point['q_rad'], [.1]*6)
        self.assertEqual(point['role'], 'basket_above')
        self.assertGreaterEqual(point['stability']['duration_s'], .6)
        self.assertGreaterEqual(point['stability']['state_duration_s'], .6)
        self.assertGreaterEqual(point['stability']['unique_joint_samples'], 5)
        self.assertEqual(point['home_reference']['q_rad'], [0.]*6)
        self.assertFalse(point['automatic_motion_enabled'])
        self.assertFalse(point['route_validated'])
        self.assertFalse(point['gripper_installed'])

    def test_motion_grip_unknown_stop_missing_flags_and_wrong_mode_are_rejected(self):
        cases = [('hardware', 'state', 'active'), ('hardware','fatal',True),
                 ('hardware','stop_result_unknown',True), ('hardware','command_mode','passthrough'),
                 ('hardware','execution_scope','j6_only'), ('home_return','state','moving'),
                 (None,'engaged',True), (None,'gripper_installed',True),
                 (None,'closed',True), (None,'session_finished',True),
                 (None,'mode','aubo_readonly_preview')]
        for parent, key, value in cases:
            with self.subTest(parent=parent, key=key):
                row = status(self.clock())
                (row[parent] if parent else row)[key] = value
                with self.assertRaises(ValueError):
                    stationary_observation(row, self.clock())
        row = status(self.clock()); del row['hardware']['fatal']
        with self.assertRaises(ValueError): stationary_observation(row, self.clock())

    def test_stale_future_failed_and_non_stopped_feedback_are_rejected(self):
        for channel, key, value in [('joints','host_received_monotonic_s',99.),
            ('state','host_received_monotonic_s',99.), ('joints','host_received_monotonic_s',101.),
            ('state','host_received_monotonic_s',float('nan')), ('state','data',2),
            ('joints','return_code',10007), ('joints','source','candidate')]:
            with self.subTest(channel=channel, key=key):
                row=status(self.clock()); row['hardware']['last_snapshot'][channel][key]=value
                with self.assertRaises(ValueError):stationary_observation(row,self.clock())

    def test_nonfinite_out_of_range_and_invalid_ranges_are_rejected(self):
        for q in ([float('nan')]*6, [4.]*6, [.1]*5):
            with self.subTest(q=q), self.assertRaises(ValueError):
                stationary_observation(status(self.clock(),q),self.clock())
        row=status(self.clock()); row['hardware']['joint_ranges_deg'][0]=[1.,1.]
        with self.assertRaises(ValueError):stationary_observation(row,self.clock())

    def test_repeated_cached_snapshot_is_not_a_stability_proof(self):
        cached=status(self.clock())
        with self.assertRaisesRegex(ValueError,'过期'):
            self.collect(lambda:copy.deepcopy(cached))

    def test_deceleration_or_drift_while_state_reports_stopped_is_rejected(self):
        def reader():
            return status(self.clock(),[.1+(self.clock()-100.)*.01]*6)
        with self.assertRaisesRegex(ValueError,'关节仍在变化'):self.collect(reader)

    def test_reordered_feedback_is_rejected(self):
        count=0
        def reader():
            nonlocal count
            count+=1
            row=status(self.clock())
            if count==2:row['hardware']['last_snapshot']['joints']['host_received_monotonic_s']=99.98
            return row
        with self.assertRaisesRegex(ValueError,'反馈顺序'):self.collect(reader)

    def test_home_mismatch_or_changes_during_capture_are_rejected(self):
        row=status(self.clock()); row['home_return']['target_q_rad']=[.1]*6
        with self.assertRaisesRegex(ValueError,'中转位不一致'):self.collect(lambda:row)
        def reader():
            if self.clock()>100.3:
                self.home.write_text(self.home.read_text()+' ')
            return status(self.clock())
        with self.assertRaisesRegex(ValueError,'配置发生变化'):self.collect(reader)

    def test_reference_save_cannot_overwrite_or_escape_allowed_storage(self):
        point=self.collect(lambda:status(self.clock()))
        path=self.root/'placement'/'basket.json'
        with patch.dict(os.environ, {'AUBO_ALLOWED_ROOTS':str(self.root)}):
            save_new_reference(path,point)
            original=path.read_bytes()
            with self.assertRaises(FileExistsError):save_new_reference(path,dict(point,q_rad=[.2]*6))
            self.assertEqual(path.read_bytes(),original)
            self.assertFalse(list(path.parent.glob('*.tmp')))
        with patch.dict(os.environ,{'AUBO_ALLOWED_ROOTS':''}):
            with self.assertRaises(ValueError):save_new_reference(self.root/'elsewhere.json',point)

    def test_default_command_does_not_request_status_and_capture_failure_writes_nothing(self):
        with patch('teleop.aubo.placement_teach.read_local_status',side_effect=ConnectionRefusedError) as reader:
            self.assertEqual(main([]),0)
            reader.assert_not_called()
            path=self.root/'should-not-exist.json'
            self.assertEqual(main(['--capture-basket','--home-reference',str(self.home),'--output',str(path)]),2)
            self.assertFalse(path.exists())

    def test_http_reads_only_get_status_and_refuses_redirects(self):
        requests=[]
        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                requests.append((self.command,self.path))
                self.send_response(200 if len(requests)==1 else 302)
                self.end_headers()
                self.wfile.write(json.dumps(status(100.)).encode())
            def log_message(self,*args): pass
        server=ThreadingHTTPServer(('127.0.0.1',0),Handler)
        thread=threading.Thread(target=server.serve_forever,daemon=True); thread.start()
        try:
            self.assertEqual(read_local_status(server.server_port)['hardware']['state'],'idle')
            with self.assertRaisesRegex(ValueError,'302'):read_local_status(server.server_port)
            self.assertEqual(requests,[('GET','/aubo/status')]*2)
        finally:
            server.shutdown();server.server_close();thread.join()


if __name__=='__main__':unittest.main()
