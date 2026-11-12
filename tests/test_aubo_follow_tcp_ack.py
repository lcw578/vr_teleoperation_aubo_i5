"""Follow ACK lifecycle using fake clients; no vendor SDK or robot connection."""
import contextlib
import io
import socket
import types
import unittest
from unittest import mock

from teleop.aubo.quest_follow import main


class FollowTcpAckTests(unittest.TestCase):
    def run_prepare_failure(self, mode=None, *, start_error=False, stop_error=False):
        calls = []

        class Client:
            def login(self, *args):
                calls.append('login')
                return dict(return_code=0)

            def start_waypoint_feedback(self):
                calls.append('subscribe')
                return dict(return_code=0, duration_s=0.)

            def waypoint_feedback(self):
                return dict(samples=[1])

            def prepare(self):
                calls.append('prepare')
                raise RuntimeError('injected prepare refusal')

            def logout(self):
                calls.append('logout')
                return dict(return_code=0)

            def abandon_cleanup(self):
                calls.append('abandon')

        class Experiment:
            def __init__(self, **kwargs):
                self.mode = kwargs['mode']
                self.host = kwargs['host']
                if self.host != '127.0.0.1':
                    raise AssertionError('Test host must remain loopback')

            def start(self):
                calls.append('ack_start:' + self.mode)
                if start_error:
                    raise RuntimeError('injected start failure')
                return self

            def stop(self):
                calls.append('ack_stop')
                if stop_error:
                    raise RuntimeError('injected stuck worker')
                return dict(stop_reason='requested', error=None)

        # Reserve a free local relay port; this test fails before serving it.
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', 0))
            port = probe.getsockname()[1]
        args = ['--execute-six-axis', '--command-mode', 'follow', '--waypoint-feedback',
                '--host', '127.0.0.1', '--port', '8899', '--user', 'test', '--usb',
                '--relay-port', str(port), '--duration', '30']
        if mode:
            args.extend(['--tcp-ack-mode', mode])
        with mock.patch('teleop.aubo.quest_follow.load_follow_native', return_value=types.SimpleNamespace(J6Follower=lambda **unused: Client())), \
                mock.patch('teleop.aubo.quest_follow.TcpAckExperiment', Experiment), \
                mock.patch('teleop.aubo.quest_follow.getpass.getpass', return_value='not-logged'), \
                contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(RuntimeError):
                main(args)
        return calls

    def test_default_does_not_create_option_worker(self):
        self.assertEqual(self.run_prepare_failure(), ['login', 'subscribe', 'prepare', 'logout'])

    def test_opt_in_starts_after_login_stops_before_logout_on_prepare_failure(self):
        self.assertEqual(self.run_prepare_failure('quickack'),
                         ['login', 'ack_start:quickack', 'subscribe', 'prepare', 'ack_stop', 'logout'])

    def test_baseline_uses_same_worker_without_quickack_mode(self):
        self.assertEqual(self.run_prepare_failure('baseline'),
                         ['login', 'ack_start:baseline', 'subscribe', 'prepare', 'ack_stop', 'logout'])

    def test_start_failure_cannot_reach_subscription_or_motion(self):
        self.assertEqual(self.run_prepare_failure('quickack', start_error=True),
                         ['login', 'ack_start:quickack', 'logout'])

    def test_stuck_option_worker_prevents_sdk_close(self):
        self.assertEqual(self.run_prepare_failure('quickack', stop_error=True),
                         ['login', 'ack_start:quickack', 'subscribe', 'prepare', 'ack_stop', 'abandon'])


if __name__ == '__main__':
    unittest.main()
