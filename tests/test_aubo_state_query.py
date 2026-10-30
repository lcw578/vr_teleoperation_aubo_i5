"""Independent observer scheduling, failure and late completion; no hardware."""
import io
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from teleop.aubo.state_query import StateQueryWorker, validate_state_result
from teleop.aubo.quest_follow import main


def result(start=None):
    now=time.monotonic()
    return dict(return_code=0, data=0, host_started_monotonic_s=now if start is None else start,
        host_received_monotonic_s=now, duration_s=0 if start is None else now-start)


class ObserverTests(unittest.TestCase):
    def make(self, client):
        failed=Mock();diagnose=Mock()
        query=StateQueryWorker(client, on_failure=failed, is_active=lambda:True, diagnose=diagnose)
        return query,failed,diagnose
    def test_failure_invalidates_and_reports_without_reconnect(self):
        client=Mock();client.query_state.return_value=dict(result(),return_code=1)
        query,failed,_=self.make(client)
        query.query_once();query.query_once()
        client.invalidate.assert_called_once();failed.assert_called_once()
        self.assertEqual(client.query_state.call_count,1)
        self.assertIsNone(query.status()["last_result"])
        self.assertEqual(query.status()["last_attempt"]["return_code"],1)
        self.assertIn('rc=1',query.status()['error'])
    def test_successful_result_delayed_in_host_is_observed_without_killing_channel(self):
        client=Mock()
        now=time.monotonic()
        client.query_state.return_value=dict(return_code=0,data=0,
            host_started_monotonic_s=now-.31,host_received_monotonic_s=now-.30,
            duration_s=.01)
        query,failed,_=self.make(client)
        query.query_once()
        self.assertIsNone(query.status()['error'])
        self.assertEqual(query.status()['last_result'],client.query_state.return_value)
        failed.assert_not_called();client.invalidate.assert_not_called()
        # A motion consumer must still reject the original stale timestamp.
        with self.assertRaises(RuntimeError):
            validate_state_result(query.status()['last_result'],time.monotonic())
    def test_period_means_query_start_interval_and_slow_calls_do_not_catch_up(self):
        now=[0.];starts=[];waits=[];durations=iter((.06,.15,.01))
        client=Mock()
        query=StateQueryWorker(client,on_failure=Mock(),is_active=lambda:True,
                               diagnose=Mock(),clock=lambda:now[0])
        def call():
            started=now[0];starts.append(started);now[0]+=next(durations)
            return dict(return_code=0,data=0,host_started_monotonic_s=started,
                        host_received_monotonic_s=now[0],duration_s=now[0]-started)
        def wait(delay):
            waits.append(delay);now[0]+=delay
            if len(waits)==3:query._quit.set()
        client.query_state.side_effect=call
        query._wake=Mock();query._wake.wait.side_effect=wait
        query._run()
        self.assertEqual(len(starts),3)
        for actual,expected in zip(starts,(0.,.1,.25)):self.assertAlmostEqual(actual,expected)
        for actual,expected in zip(waits,(.04,0.,.09)):self.assertAlmostEqual(actual,expected)
        self.assertIsNone(query.status()['error'])
    def test_bad_state_timestamp_and_duration_are_rejected(self):
        for patch_value in (dict(data=99),dict(return_code=1),dict(duration_s=.51),
                            dict(host_started_monotonic_s=float('nan')),
                            dict(host_received_monotonic_s=time.monotonic()-1)):
            with self.assertRaises(RuntimeError):validate_state_result(dict(result(),**patch_value),time.monotonic())
    def test_paused_resumed_are_observations_and_do_not_kill_query_worker(self):
        client=Mock()
        query,failed,diagnose=self.make(client)
        for state in (2,3,0):
            client.query_state.return_value=dict(result(),data=state)
            query.query_once()
            self.assertEqual(query.status()['last_result']['data'],state)
            self.assertIsNone(query.status()['error'])
        failed.assert_not_called();client.invalidate.assert_not_called()
        self.assertEqual(diagnose.call_count,3)
    def test_blocked_query_status_read_and_close_never_wait_for_network_lock(self):
        entered,release=threading.Event(),threading.Event()
        client=Mock()
        def blocked():
            start=time.monotonic();entered.set();release.wait(2.)
            return result(start)
        client.query_state.side_effect=blocked
        query,failed,_=self.make(client)
        thread=threading.Thread(target=query.query_once)
        thread.start();self.assertTrue(entered.wait(1.))
        try:
            before=time.monotonic();self.assertIsNotNone(query.status()["sdk_inflight"])
            query.fail("query_over_budget")
            self.assertLess(time.monotonic()-before,.1)
            release.set();thread.join(1.)
            self.assertIsNone(query.status()["last_result"])
            failed.assert_called_once()
        finally:release.set();thread.join(1.)
    def test_start_failure_is_exposed_before_motion_worker_starts(self):
        client=Mock();client.query_state.side_effect=RuntimeError('disconnected')
        query,failed,_=self.make(client)
        try:
            with self.assertRaisesRegex(RuntimeError,"failed"):query.start()
        finally:query.close()
        failed.assert_called_once()
    def test_stuck_query_close_refuses_logout_and_discards_late_result(self):
        entered,release=threading.Event(),threading.Event()
        client=Mock()
        def blocked():
            start=time.monotonic();entered.set();release.wait(3.)
            return result(start)
        client.query_state.side_effect=blocked
        query,_,_=self.make(client)
        query._thread.start();self.assertTrue(entered.wait(1.))
        try:
            with self.assertRaisesRegex(RuntimeError,'no concurrent logout'):query.close()
            client.logout.assert_not_called()
            self.assertIsNone(query.status()['last_result'])
        finally:
            release.set();query._thread.join(1.)
        self.assertIsNone(query.status()['last_result'])
    def test_cli_requires_explicit_six_axis_follow_push(self):
        with patch('teleop.aubo.quest_follow.load_follow_native') as load,patch('sys.stderr',io.StringIO()):
            for args in (["--parallel-state-query"],
                         ["--execute-six-axis","--waypoint-feedback","--parallel-state-query"],
                         ["--execute-j6","--parallel-state-query"]):
                with self.assertRaises(SystemExit):main(args)
            load.assert_not_called()

    def test_cli_rejects_conflicting_query_modes_before_sdk_loading(self):
        with patch('teleop.aubo.quest_follow.load_follow_native') as load,patch('sys.stderr',io.StringIO()):
            for args in (["--serial-state-query", "--parallel-state-query"],
                         ["--parallel-state-query", "--serial-state-query"]):
                with self.assertRaises(SystemExit):main(args)
            load.assert_not_called()


if __name__ == '__main__':unittest.main(verbosity=2)
