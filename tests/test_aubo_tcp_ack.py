"""Existing-socket ACK experiment: loopback only, never import/load the SDK."""
import ctypes
import errno
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time
import unittest
from unittest import mock

from teleop.aubo.tcp_ack import TcpAckExperiment


class TcpAckTests(unittest.TestCase):
    def pair(self, listener=None):
        if listener is None:
            listener = socket.socket()
            self.addCleanup(listener.close)
            listener.bind(('127.0.0.1', 0))
            listener.listen()
        client = socket.create_connection(listener.getsockname(), timeout=2)
        self.addCleanup(client.close)
        peer, _ = listener.accept()
        self.addCleanup(peer.close)
        return client, peer, listener

    def guard(self, client, mode='quickack', **kwargs):
        guard = TcpAckExperiment(host='127.0.0.1', port=client.getpeername()[1], mode=mode, **kwargs)
        self.addCleanup(guard.stop)
        return guard

    def await_ticks(self, guard, count=3):
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            report = guard.status()
            if report['ticks'] >= count or not report['alive']:
                break
            time.sleep(.002)
        self.assertGreaterEqual(guard.status()['ticks'], count, guard.status())

    def await_exit(self, guard):
        deadline = time.monotonic() + 2
        while guard.status()['alive'] and time.monotonic() < deadline:
            time.sleep(.002)
        self.assertFalse(guard.status()['alive'], guard.status())

    def test_periodic_hint_selects_actual_socket_and_preserves_payload_and_timeout(self):
        client, peer, _ = self.pair()
        before = client.gettimeout()
        guard = self.guard(client).start()
        self.await_ticks(guard)
        peer.sendall(b'from cabinet')
        self.assertEqual(client.recv(32), b'from cabinet')
        client.sendall(b'unchanged request')
        self.assertEqual(peer.recv(32), b'unchanged request')
        report = guard.stop()
        self.assertEqual(report['source_fd'], client.fileno())
        self.assertEqual(report['socket_inode'], os.fstat(client.fileno()).st_ino)
        self.assertGreaterEqual(report['quickack_success'], 3)
        self.assertEqual(report['option_failures'], 0)
        self.assertEqual(client.gettimeout(), before)
        self.assertEqual(report['stop_reason'], 'requested')
        self.assertTrue(report['fresh_connection_required'])
        self.assertIsNotNone(report['interval_mean_s'])

    def test_baseline_never_sets_socket_options(self):
        client, _, _ = self.pair()
        with mock.patch('teleop.aubo.tcp_ack._set_option') as setting:
            guard = self.guard(client, mode='baseline').start()
            self.await_ticks(guard)
            report = guard.stop()
            setting.assert_not_called()
        self.assertEqual(report['quickack_success'], 0)
        self.assertFalse(report['fresh_connection_required'])

    def test_nodelay_only_changes_selected_socket_and_restores_original(self):
        client, _, _ = self.pair()
        other, _, _ = self.pair()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0)
        original_other = other.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY)
        guard = self.guard(client, mode='nodelay').start()
        self.assertEqual(client.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY), 1)
        self.assertEqual(other.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY), original_other)
        report = guard.stop()
        self.assertEqual(client.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY), 0)
        self.assertTrue(report['nodelay_changed'])
        self.assertTrue(report['nodelay_restored'])
        self.assertEqual(report['quickack_success'], 0)

    def test_already_enabled_nodelay_is_observed_without_changing_it(self):
        client, _, _ = self.pair()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with mock.patch('teleop.aubo.tcp_ack._set_option') as setting:
            report = self.guard(client, mode='nodelay').start().stop()
            setting.assert_not_called()
        self.assertEqual(report['nodelay_original'], 1)
        self.assertFalse(report['nodelay_changed'])

    def test_combined_mode_reports_both_options(self):
        client, _, _ = self.pair()
        report = self.guard(client, mode='combined').start().stop()
        self.assertEqual(report['nodelay_active'], 1)
        self.assertGreater(report['quickack_success'], 0)

    def test_multiple_connections_to_same_peer_are_refused_without_leaks(self):
        client, _, listener = self.pair()
        self.pair(listener)
        count = len(list(Path('/proc/self/fd').iterdir()))
        with self.assertRaisesRegex(RuntimeError, 'found 2'):
            self.guard(client).start()
        self.assertEqual(len(list(Path('/proc/self/fd').iterdir())), count)

    def test_existing_alias_is_refused_instead_of_guessing_ownership(self):
        client, _, _ = self.pair()
        duplicate = os.dup(client.fileno())
        try:
            with self.assertRaisesRegex(RuntimeError, 'found 2'):
                self.guard(client).start()
        finally:
            os.close(duplicate)

    def test_quickack_alone_does_not_change_nodelay(self):
        client, _, _ = self.pair()
        for original in (0, 1):
            with self.subTest(original=original):
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, original)
                guard = self.guard(client).start()
                self.assertEqual(guard.status()['nodelay_active'], original)
                guard.stop()
                self.assertEqual(client.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY), original)

    def test_no_target_does_not_connect_or_set_options(self):
        client, _, listener = self.pair()
        port = listener.getsockname()[1]
        client.close()
        with mock.patch('teleop.aubo.tcp_ack._set_option') as setting:
            with self.assertRaisesRegex(RuntimeError, 'found 0'):
                TcpAckExperiment(host='127.0.0.1', port=port, mode='quickack').start()
            setting.assert_not_called()

    def test_explicit_source_still_requires_matching_peer(self):
        client, _, _ = self.pair()
        other, _, _ = self.pair()
        with self.assertRaisesRegex(ValueError, 'exact IPv4 TCP peer'):
            self.guard(client, source_fd=other.fileno()).start()

    def test_udp_with_matching_peer_is_refused(self):
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(udp.close)
        udp.connect(('127.0.0.1', 23456))
        with self.assertRaisesRegex(RuntimeError, 'found 0'):
            TcpAckExperiment(host='127.0.0.1', port=23456, mode='quickack').start()

    def test_original_close_and_fd_reuse_stop_worker_without_affecting_pipe(self):
        client, _, _ = self.pair()
        fd = client.fileno()
        guard = self.guard(client).start()
        client.close()
        reader, writer = os.pipe()
        try:
            if reader != fd:
                os.dup2(reader, fd)
                os.close(reader)
                reader = fd
            self.await_exit(guard)
            self.assertEqual(guard.status()['stop_reason'], 'source_descriptor_closed_or_replaced')
            os.write(writer, b'pipe unchanged')
            self.assertEqual(os.read(reader, 32), b'pipe unchanged')
        finally:
            os.close(reader)
            os.close(writer)

    def test_remote_close_stops_worker_and_releases_duplicate(self):
        client, peer, _ = self.pair()
        before = len(list(Path('/proc/self/fd').iterdir()))
        guard = self.guard(client).start()
        peer.close()
        self.await_exit(guard)
        self.assertEqual(guard.status()['stop_reason'], 'tcp_no_longer_established')
        self.assertEqual(len(list(Path('/proc/self/fd').iterdir())), before - 1)
        self.assertEqual(client.recv(1), b'')

    def test_option_failure_is_reported_and_not_hidden(self):
        client, _, _ = self.pair()
        guard = self.guard(client)
        with mock.patch('teleop.aubo.tcp_ack._set_option', side_effect=PermissionError(errno.EPERM, 'injected')):
            with self.assertRaisesRegex(RuntimeError, 'initialization failed'):
                guard.start()
        report = guard.stop()
        self.assertEqual(report['quickack_success'], 0)
        self.assertEqual(report['option_failures'], 1)
        self.assertEqual(report['error']['errno'], errno.EPERM)
        self.assertFalse(report['alive'])

    def test_failure_after_start_stops_and_restores_nodelay(self):
        from teleop.aubo import tcp_ack
        client, _, _ = self.pair()
        client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 0)
        original_set = tcp_ack._set_option
        calls = []

        def setting(sock, option, value):
            if option == socket.TCP_QUICKACK:
                calls.append(option)
                if len(calls) > 1:
                    raise OSError(errno.EINVAL, 'injected after first tick')
            original_set(sock, option, value)

        with mock.patch('teleop.aubo.tcp_ack._set_option', side_effect=setting):
            guard = self.guard(client, mode='combined').start()
            self.await_exit(guard)
        report = guard.stop()
        self.assertEqual(report['quickack_success'], 1)
        self.assertEqual(report['option_failures'], 1)
        self.assertTrue(report['nodelay_restored'])
        self.assertEqual(client.getsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY), 0)

    def test_stop_releases_pin_and_worker_and_instance_cannot_restart(self):
        client, _, _ = self.pair()
        before = len(list(Path('/proc/self/fd').iterdir()))
        guard = self.guard(client).start()
        guard.stop()
        self.assertFalse(guard._thread.is_alive())
        self.assertEqual(len(list(Path('/proc/self/fd').iterdir())), before)
        with self.assertRaisesRegex(RuntimeError, 'single-use'):
            guard.start()

    @unittest.skipUnless(hasattr(os, 'RTLD_DEEPBIND'), 'Linux DEEPBIND required')
    def test_deepbind_receiver_keeps_working_while_actual_socket_option_is_set(self):
        runtime = Path(os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'))
        with tempfile.TemporaryDirectory(dir=runtime) as directory:
            source = Path(directory) / 'receiver.c'
            library = Path(directory) / 'receiver.so'
            source.write_text('#include <sys/socket.h>\n#include <sys/types.h>\n'
                              'ssize_t receive_exact(int fd, void *p, size_t n) '
                              '{ return recv(fd, p, n, MSG_WAITALL); }\n')
            subprocess.run(['gcc', '-shared', '-fPIC', '-Wall', '-Wextra', '-Werror',
                            str(source), '-o', str(library)], check=True)
            isolated = ctypes.CDLL(str(library), mode=os.RTLD_NOW | os.RTLD_LOCAL | os.RTLD_DEEPBIND)
            isolated.receive_exact.argtypes = (ctypes.c_int, ctypes.c_void_p, ctypes.c_size_t)
            isolated.receive_exact.restype = ctypes.c_ssize_t
            client, peer, _ = self.pair()
            payload = b'unchanged deepbind data'
            peer.sendall(payload)
            guard = self.guard(client).start()
            self.await_ticks(guard)
            buffer = ctypes.create_string_buffer(len(payload))
            self.assertEqual(isolated.receive_exact(client.fileno(), buffer, len(payload)), len(payload))
            self.assertEqual(buffer.raw, payload)
            self.assertGreater(guard.stop()['quickack_success'], 0)

    def test_invalid_configuration_is_rejected_before_socket_selection(self):
        for kwargs in ({'host': 'localhost'}, {'port': 0}, {'mode': 'implicit'},
                       {'interval_s': float('nan')}, {'interval_s': .1}, {'source_fd': -1}):
            with self.subTest(kwargs=kwargs):
                values = dict(host='127.0.0.1', port=8899, mode='quickack')
                values.update(kwargs)
                with self.assertRaises(ValueError):
                    TcpAckExperiment(**values)


if __name__ == '__main__':
    unittest.main()
