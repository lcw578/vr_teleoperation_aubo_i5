"""Regression against the actual SDK and a loopback-only fake cabinet."""
import os
from pathlib import Path
import socket
import struct
import subprocess
import threading
import time
import unittest

RUNTIME = Path(os.environ.get('AUBO_RUNTIME_DIR', '/data/VR_teleoperation_runtime'))
BINARY = RUNTIME/'build-follow/test_stop_order_sdk'


def frame(command, payload=b''):
    return (bytes.fromhex('eeccffff')+struct.pack('<II', 12+len(payload), command)
            +payload+bytes(4)+bytes.fromhex('ffffffff'))


ACK = frame(89, bytes.fromhex('0801120c08001208537563636573732e'))
DONE = frame(2, bytes.fromhex('082c10001a2d')
             +b'{"code":0,"text":"RobotMoveControlStopDone."}')


@unittest.skipUnless(BINARY.exists(), 'Build test_stop_order_sdk for vendor SDK regression')
class StopOrderSDKTests(unittest.TestCase):
    def run_peer(self, mode, order, reply=ACK):
        errors, requests = [], []
        with socket.socket() as server:
            server.bind(('127.0.0.1', 0))
            server.listen()
            server.settimeout(9)

            def serve():
                try:
                    peer, _ = server.accept()
                    with peer:
                        peer.settimeout(8)
                        data = b''
                        while len(data) < 20:
                            block = peer.recv(4096)
                            if not block:
                                raise RuntimeError('SDK disconnected before request')
                            data += block
                        requests.append(data[:20])
                        time.sleep(.01)
                        if order == 'event_first':
                            peer.sendall(DONE)
                            time.sleep(.01)
                            peer.sendall(reply+reply)
                        elif order == 'reply_first':
                            peer.sendall(reply)
                            time.sleep(.01)
                            peer.sendall(DONE)
                        elif order == 'coalesced':
                            peer.sendall(DONE+reply+reply)
                        elif order == 'missing_reply':
                            peer.sendall(DONE)
                        elif order == 'missing_event':
                            peer.sendall(reply)
                        else:
                            raise AssertionError(order)
                        try:
                            while peer.recv(4096):
                                pass
                        except ConnectionResetError:
                            pass
                except Exception as exc:
                    errors.append(repr(exc))

            thread = threading.Thread(target=serve, daemon=True)
            thread.start()
            result = subprocess.run([str(BINARY), str(server.getsockname()[1]), mode],
                                    capture_output=True, text=True, timeout=9)
            thread.join(timeout=9)
        self.assertFalse(thread.is_alive(), 'Loopback peer did not finish')
        self.assertEqual(errors, [])
        self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
        self.assertEqual(requests, [frame(89)])  # Unchanged 20-byte SlowStop request.
        lines = [line.split() for line in result.stdout.splitlines() if line.startswith('RESULT ')]
        self.assertEqual(len(lines), 1, result.stdout)
        return int(lines[0][1]), float(lines[0][2])

    def test_original_event_before_ack_reproduces_10007(self):
        rc, ms = self.run_peer('original', 'event_first')
        self.assertEqual(rc, 10007)
        self.assertGreater(ms, 4900)

    def test_original_ack_before_event_succeeds(self):
        rc, ms = self.run_peer('original', 'reply_first')
        self.assertEqual(rc, 0)
        self.assertLess(ms, 1000)

    def test_fixed_accepts_both_orders_and_combined_frames_repeatedly(self):
        for order in ('event_first', 'reply_first', 'coalesced'):
            for repeat in range(3):
                with self.subTest(order=order, repeat=repeat):
                    rc, ms = self.run_peer('fixed', order)
                    self.assertEqual(rc, 0)
                    self.assertLess(ms, 1000)

    def test_missing_ack_still_times_out(self):
        rc, ms = self.run_peer('fixed', 'missing_reply')
        self.assertEqual(rc, 10007)
        self.assertGreater(ms, 4900)

    def test_request_ack_does_not_require_completion_event(self):
        # ACK alone acknowledges a request; Python still must verify standstill.
        rc, ms = self.run_peer('fixed', 'missing_event')
        self.assertEqual(rc, 0)
        self.assertLess(ms, 1000)

    def test_failed_or_malformed_reply_is_not_success(self):
        for payload, expected in ((bytes.fromhex('0800120c08001208537563636573732e'), 11025),
                                  (bytes.fromhex('0801120c0801120852656a6563746564'), 11025),
                                  (b'\x08', 10010), (b'', 10009)):
            with self.subTest(payload=payload.hex()):
                rc, _ = self.run_peer('fixed', 'event_first', frame(89, payload))
                self.assertEqual(rc, expected)


if __name__ == '__main__':
    unittest.main()
