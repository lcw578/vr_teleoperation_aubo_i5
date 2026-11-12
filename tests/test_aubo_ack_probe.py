"""Stationary probe boundaries using fake SDK objects; never load vendor SDK."""
import contextlib
import io
import json
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import tempfile
import types
import unittest
from unittest import mock

from teleop.aubo import ack_probe


class FakeClient:
    def __init__(self, *, state=0, rc=0):
        self.calls = []
        self.state, self.rc = state, rc

    def login(self, *args):
        self.calls.append('login')
        return {'return_code': 0}

    def query(self, operation):
        self.calls.append(operation)
        return dict(operation=operation, return_code=self.rc, duration_s=.002,
                    data=self.state if operation == 'robot_state' else {'joint_positions_rad': [0.] * 6})

    def logout(self):
        self.calls.append('logout')
        return {'return_code': 0}


class AckProbeTests(unittest.TestCase):
    def test_default_plan_cannot_load_sdk_prompt_or_start_capture(self):
        with mock.patch.object(ack_probe, 'load_native') as load, \
                mock.patch.object(ack_probe.getpass, 'getpass') as prompt, \
                mock.patch.object(ack_probe.subprocess, 'Popen') as process, \
                contextlib.redirect_stdout(io.StringIO()) as output:
            self.assertEqual(ack_probe.main([]), 0)
        load.assert_not_called()
        prompt.assert_not_called()
        process.assert_not_called()
        self.assertFalse(json.loads(output.getvalue())['sdk_loaded'])

    def test_invalid_labels_and_arguments_fail_before_loading_sdk(self):
        with contextlib.redirect_stderr(io.StringIO()):
            for args in (['--label', 'B'], ['--mode', 'quickack'], ['--duration', '1800'],
                         ['--connect'], ['--port', '65536'], ['--duration', '-1']):
                with self.subTest(args=args), self.assertRaises(SystemExit):
                    ack_probe.parse_args(args)

    def test_sampling_only_uses_allowlisted_gets_and_no_catchup(self):
        now = [0.]
        waits = []
        client = FakeClient()
        original = client.query

        def query(name):
            now[0] += .060  # slower than 40ms: do not try to catch up.
            return original(name)

        def wait(value):
            waits.append(value)
            now[0] += value

        client.query = query
        experiment = types.SimpleNamespace(status=lambda: dict(alive=True, error=None, quickack_success=1))
        output = io.StringIO()
        result = ack_probe.sample_queries(client, experiment, .5, output,
                                          clock=lambda: now[0], wait=wait)
        self.assertEqual(set(client.calls), {'robot_state', 'joint_angles'})
        self.assertTrue(all(value == 0 for value in waits))
        self.assertEqual(result['query_count'], len(client.calls))
        self.assertEqual(result['operations']['robot_state'], 2)
        self.assertTrue(all(json.loads(line)['event'] == 'query' for line in output.getvalue().splitlines()))

    def test_nonstopped_state_or_failed_query_ends_without_retry(self):
        experiment = types.SimpleNamespace(status=lambda: dict(alive=True, error=None))
        for client in (FakeClient(state=1), FakeClient(rc=10007)):
            with self.subTest(client=client), self.assertRaises(RuntimeError):
                ack_probe.sample_queries(client, experiment, 1, io.StringIO())
            self.assertEqual(client.calls, ['robot_state'])

    def test_lost_ack_worker_prevents_further_sdk_queries(self):
        client = FakeClient()
        experiment = types.SimpleNamespace(status=lambda: dict(alive=False, error=None))
        with self.assertRaisesRegex(RuntimeError, 'stopped unexpectedly'):
            ack_probe.sample_queries(client, experiment, 1, io.StringIO())
        self.assertEqual(client.calls, [])

    def run_fake_session(self, client, *, sample_error=None):
        runtime = Path(os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'))
        directory = tempfile.TemporaryDirectory(dir=runtime)
        self.addCleanup(directory.cleanup)
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            listener.listen()
            with socket.create_connection(listener.getsockname()) as connection:
                peer, _ = listener.accept()
                with peer:
                    def sample(*args):
                        self.assertEqual(client.calls, ['login', 'robot_state'])
                        self.assertTrue(args[1].status()['alive'])
                        if sample_error:
                            raise sample_error
                        return {'query_count': 1}

                    def capture(*args):
                        self.assertEqual(client.calls, ['login', 'robot_state'])
                        return None, None

                    with mock.patch.object(ack_probe, 'load_native', return_value=types.SimpleNamespace(ReadOnlyClient=lambda: client)), \
                            mock.patch.object(ack_probe.getpass, 'getpass', return_value='test-secret'), \
                            mock.patch.object(ack_probe, 'sample_queries', side_effect=sample), \
                            mock.patch.object(ack_probe, '_start_capture', side_effect=capture) as start_capture, \
                            contextlib.redirect_stdout(io.StringIO()):
                        rc = ack_probe.main(['--connect', '--host', '127.0.0.1',
                                             '--port', str(listener.getsockname()[1]), '--user', 'test',
                                             '--runtime', directory.name, '--label', 'B', '--mode', 'quickack'])
        summary_file = next(Path(directory.name).glob('ack-stationary-*/summary.json'))
        summary = json.loads(summary_file.read_text())
        events = summary_file.with_name('events.jsonl').read_text()
        self.assertNotIn('test-secret', summary_file.read_text() + events)
        return rc, summary, start_capture.call_count

    def test_read_only_session_initializes_ack_and_cleans_up_before_logout(self):
        client = FakeClient()
        rc, summary, captured = self.run_fake_session(client)
        self.assertEqual(rc, 0)
        self.assertTrue(summary['completed'])
        self.assertFalse(summary['motion_api_called'])
        self.assertGreater(summary['ack_final']['quickack_success'], 0)
        self.assertFalse(summary['ack_final']['alive'])
        self.assertEqual(client.calls, ['login', 'robot_state', 'logout'])
        self.assertEqual(captured, 1)

    def test_sampling_failure_stops_worker_and_logs_out(self):
        client = FakeClient()
        rc, summary, _ = self.run_fake_session(client, sample_error=RuntimeError('injected'))
        self.assertEqual(rc, 1)
        self.assertFalse(summary['completed'])
        self.assertFalse(summary['ack_final']['alive'])
        self.assertEqual(client.calls[-1], 'logout')

    def test_initial_moving_state_refuses_capture_and_experiment(self):
        client = FakeClient(state=1)
        rc, summary, captured = self.run_fake_session(client)
        self.assertEqual(rc, 1)
        self.assertEqual(captured, 0)
        self.assertNotIn('ack_initial', summary)
        self.assertEqual(client.calls, ['login', 'robot_state', 'logout'])

    def test_capture_failure_does_not_start_sampling(self):
        args = types.SimpleNamespace(capture_interface='lo', host='127.0.0.1', port=8899, duration=30)
        runtime = Path(os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'))
        with tempfile.TemporaryDirectory(dir=runtime) as directory:
            process = mock.Mock()
            process.poll.return_value = 1
            with mock.patch.object(ack_probe.subprocess, 'Popen', return_value=process), \
                    mock.patch.object(ack_probe, '_stop_capture') as stop:
                with self.assertRaisesRegex(RuntimeError, 'Capture exited before sampling'):
                    ack_probe._start_capture(args, Path(directory), {'local': ['127.0.0.1', 12345]})
                stop.assert_called_once_with(process, Path(directory) / 'wire.pcap')

    def test_capture_completeness_refuses_missing_tail_despite_zero_kernel_drops(self):
        def frame(command):
            return bytes.fromhex('eeccffff') + struct.pack('<II', 12, command) + bytes(4) + bytes.fromhex('ffffffff')

        def packet(src, command, seq):
            return dict(src=src, seq=seq, payload=frame(command), wall_s=1.)

        host = '192.168.1.5'
        pc = '192.168.1.100'
        full = [packet(pc, 26, 100), packet(host, 26, 200),
                packet(pc, 10, 120), packet(host, 10, 220)]
        wanted = dict(robot_state=1, joint_angles=1)
        for captured in (full[:2], full[:-1], full):
            with self.subTest(packets=len(captured)), \
                    mock.patch.object(ack_probe, 'packets', return_value=iter(captured)):
                result = ack_probe._capture_queries('unused', host, wanted)
                self.assertEqual(result['complete'], captured == full)

    def test_capture_uses_immediate_mode_before_sampling(self):
        args = types.SimpleNamespace(capture_interface='lo', host='127.0.0.1', port=8899, duration=30)
        runtime = Path(os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'))
        with tempfile.TemporaryDirectory(dir=runtime) as directory:
            process = mock.Mock()
            process.poll.return_value = None
            def launch(command, **kwargs):
                self.assertIn('--immediate-mode', command)
                kwargs['stderr'].write('tcpdump: listening on lo\n')
                kwargs['stderr'].flush()
                return process
            with mock.patch.object(ack_probe.subprocess, 'Popen', side_effect=launch):
                result, log = ack_probe._start_capture(args, Path(directory), {'local': ['127.0.0.1', 12345]})
                self.assertIs(result, process)
                log.close()

    def test_cleanup_signals_exact_unprivileged_writer_through_wrapper(self):
        # Reproduce sudo/timeout's outer process: it waits on a writer without
        # forwarding signals addressed only to the wrapper. No sudo is used.
        runtime = Path(os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'))
        with tempfile.TemporaryDirectory(dir=runtime) as directory:
            target = Path(directory) / 'wire.pcap'
            code = ('import signal,sys,time\n'
                    'signal.signal(signal.SIGINT,lambda *unused: sys.exit(0))\n'
                    'print("writer-ready",flush=True)\n'
                    'time.sleep(30)\n')
            writer = ['tcpdump', '-c', code, '-w', str(target)]
            wrapper_code = ('import subprocess,sys\n'
                            f'p=subprocess.Popen({writer!r},executable={sys.executable!r})\n'
                            'sys.exit(p.wait())\n')
            wrapper = subprocess.Popen([sys.executable, '-c', wrapper_code], stdout=subprocess.PIPE, text=True)
            try:
                ready, _, _ = select.select([wrapper.stdout], [], [], 3)
                self.assertTrue(ready, 'fake capture writer did not initialize')
                self.assertEqual(wrapper.stdout.readline().strip(), 'writer-ready')
                self.assertEqual(ack_probe._stop_capture(wrapper, target), 0)
                self.assertEqual(wrapper.poll(), 0)
            finally:
                if wrapper.poll() is None:
                    ack_probe._stop_capture(wrapper, target)
                wrapper.stdout.close()

    def test_stationary_launcher_default_has_no_sdk_or_files(self):
        root = Path(__file__).resolve().parents[1]
        result = subprocess.run(['bash', str(root / 'scripts/aubo_ack_stationary.sh')],
                                text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertIn('"sdk_loaded": false', result.stdout)
        self.assertIn('Plan/check only', result.stdout)


if __name__ == '__main__':
    unittest.main()
