"""Real loopback sockets only; no SDK, relay service, or robot connection."""
import errno
import socket
import unittest

from teleop.aubo.quest_preview import check_relay_port


class RelayPortTests(unittest.TestCase):
    def test_live_listener_is_rejected(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', 0))
            server.listen(1)
            with self.assertRaises(OSError) as result:
                check_relay_port(server.getsockname()[1])
            self.assertEqual(result.exception.errno, errno.EADDRINUSE)

    def test_recent_server_close_is_reusable_without_waiting(self):
        with socket.socket() as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(('127.0.0.1', 0))
            port = server.getsockname()[1]
            server.listen(1)
            with socket.socket() as client:
                client.settimeout(2.)
                client.connect(('127.0.0.1', port))
                conn, _ = server.accept()
                with conn:
                    conn.settimeout(2.)
                    # Server initiates close, so its local port enters TIME_WAIT.
                    conn.shutdown(socket.SHUT_WR)
                    self.assertEqual(client.recv(1), b'')
                client.shutdown(socket.SHUT_WR)
        with socket.socket() as plain_probe:
            with self.assertRaises(OSError) as result:
                plain_probe.bind(('127.0.0.1', port))
            self.assertEqual(result.exception.errno, errno.EADDRINUSE)
        check_relay_port(port)
        # The check releases its socket; the real server can start afterward.
        with socket.socket() as next_server:
            next_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            next_server.bind(('127.0.0.1', port))
            next_server.listen(1)


if __name__ == '__main__':
    unittest.main()
