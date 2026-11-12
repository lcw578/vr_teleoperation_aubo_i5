"""IPC failures must invalidate observations; receipt never refreshes time."""
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from teleop.aubo.process_state import ProcessStateObserver


class Cache:
    def __init__(self):
        self.generation = 0
        self.value = None

    def invalidate(self):
        self.generation += 1
        self.value = None
        return self.generation

    def publish(self, state, rc, started, received, generation):
        if generation != self.generation:
            return False
        self.value = (state, rc, started, received)
        return True


class ProcessStateTests(unittest.TestCase):
    def setUp(self):
        self.observer = ProcessStateObserver(SimpleNamespace(RobotStateCache=Cache), "/tmp")

    def tearDown(self):
        self.observer.abandon_cleanup()

    def test_ipc_receipt_preserves_child_reception_time(self):
        stamp = time.monotonic() - .15
        raw = dict(data=0, return_code=0, host_started_monotonic_s=stamp-.005,
                   host_received_monotonic_s=stamp, duration_s=.005)
        self.observer._request = Mock(return_value=raw)
        self.observer._process = SimpleNamespace(pid=123, poll=lambda:0)
        result = self.observer.query_state()
        self.assertEqual(self.observer.cache.value[3], stamp)
        self.assertEqual(result["host_received_monotonic_s"], stamp)
        self.assertGreater(result["ipc_received_monotonic_s"] - stamp, .14)

    def test_result_inflight_at_invalidation_cannot_revive_cache(self):
        entered, release = threading.Event(), threading.Event()
        now = time.monotonic()
        def delayed(*args):
            entered.set()
            self.assertTrue(release.wait(1))
            return dict(data=0, return_code=0, host_started_monotonic_s=now,
                        host_received_monotonic_s=now+.001, duration_s=.001)
        self.observer._request = delayed
        self.observer._process = SimpleNamespace(pid=123, poll=lambda:0)
        results = []
        thread = threading.Thread(target=lambda: results.append(self.observer.query_state()))
        thread.start()
        self.assertTrue(entered.wait(1))
        self.observer.invalidate()
        release.set()
        thread.join(1)
        self.assertFalse(thread.is_alive())
        self.assertFalse(results[0]["cache_published"])
        self.assertIsNone(self.observer.cache.value)

    def test_child_exit_invalidates_cache_without_reconnect(self):
        self.observer._process = SimpleNamespace(pid=123, poll=lambda:1)
        generation = self.observer._generation
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            self.observer._request("query")
        self.assertGreater(self.observer._generation, generation)

    def test_query_timeout_invalidates_and_refuses_later_request(self):
        self.observer._process = SimpleNamespace(pid=123, poll=lambda:None)
        self.observer._pipe.close()
        self.observer._pipe = Mock()
        self.observer._pipe.poll.return_value = False
        with self.assertRaises(TimeoutError):
            self.observer._request("query", timeout=.001)
        self.observer._pipe.send.assert_called_once()
        self.assertTrue(self.observer._broken)
        with self.assertRaisesRegex(RuntimeError, "unavailable"):
            self.observer._request("query")
        self.observer._pipe.send.assert_called_once()
        self.observer._process = None


if __name__ == "__main__":
    unittest.main()
