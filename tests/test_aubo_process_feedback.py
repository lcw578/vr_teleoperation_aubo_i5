"""Joint telemetry must progress while the independent state request blocks."""
import socket
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from teleop.aubo.process_state import ProcessStateObserver, SplitProcessStateObserver, _JOINT_PACKET, _serve
from tests.test_aubo_process_state import Cache


class JointCache:
    def __init__(self):
        self.generation = 0
        self.samples = []

    def invalidate(self):
        self.generation += 1
        self.samples.clear()
        return self.generation

    def publish(self, q, received, sequence, generation):
        if generation != self.generation:
            return False
        if self.samples and sequence <= self.samples[-1][2]:
            return False
        self.samples.append((q, received, sequence))
        return True


class FeedbackTests(unittest.TestCase):
    def setUp(self):
        self.observer = ProcessStateObserver(SimpleNamespace(
            RobotStateCache=Cache, WaypointFeedbackCache=JointCache), '/tmp', waypoint_feedback=True)
        self.observer._process = SimpleNamespace(pid=123, poll=lambda:None)

    def tearDown(self):
        self.observer._process = None
        self.observer.abandon_cleanup()

    def start_receiver(self):
        self.observer._telemetry_thread = threading.Thread(target=self.observer._receive_waypoints)
        self.observer._telemetry_thread.start()

    def wait_for(self, predicate):
        deadline = time.monotonic()+1
        while not predicate():
            if time.monotonic() > deadline:
                self.fail('telemetry condition timed out')
            time.sleep(.002)

    def test_joint_receiver_preserves_original_timestamp(self):
        self.start_receiver()
        stamp = time.monotonic()-.15
        self.observer._child_telemetry.send(_JOINT_PACKET.pack(1,stamp,*([.1]*6)))
        self.wait_for(lambda:self.observer._telemetry_sequence==1)
        self.assertEqual(self.observer.waypoint_cache.samples[-1][1], stamp)
        self.assertGreater(self.observer.status()['telemetry_ipc_received_monotonic_s']-stamp,.14)

    def test_late_packet_cannot_repopulate_invalidated_session(self):
        self.start_receiver()
        self.observer._invalidate_waypoints('test_session_end')
        self.observer._child_telemetry.send(_JOINT_PACKET.pack(1,time.monotonic(),*([0.]*6)))
        self.wait_for(lambda:self.observer._telemetry_error is not None and
                      'out-of-session' in self.observer._telemetry_error)
        self.assertEqual(self.observer.waypoint_cache.samples, [])

    def test_malformed_packet_invalidates_both_caches(self):
        self.start_receiver()
        generation = self.observer._generation
        self.observer._child_telemetry.send(b'bad')
        self.wait_for(lambda:self.observer._telemetry_error is not None)
        self.assertGreater(self.observer._generation,generation)
        self.assertEqual(self.observer.waypoint_cache.samples, [])

    def test_serve_publishes_joints_while_query_is_blocked(self):
        entered, release = threading.Event(), threading.Event()
        sequence = 0
        class Client:
            def login(self,*args):return dict(return_code=0)
            def start_waypoint_feedback(self):return dict(return_code=0,duration_s=0.)
            def waypoint_feedback_since(self,after):
                nonlocal sequence
                sequence+=1
                return dict(samples=[dict(sequence=sequence,q_rad=[.2]*6,
                    host_received_monotonic_s=time.monotonic())])
            def query_state(self):
                entered.set()
                if not release.wait(1):raise RuntimeError('test release timeout')
                return dict(return_code=0,data=0)
            def logout(self):return dict(return_code=0)
        parent,child=self.observer._context.Pipe()
        tx,rx=socket.socketpair(socket.AF_UNIX,socket.SOCK_DGRAM)
        tx.setblocking(False);rx.settimeout(1.)
        with patch('teleop.aubo.follow.load_follow_native',return_value=SimpleNamespace(RobotStateObserver=Client)):
            thread=threading.Thread(target=_serve,args=(child,'/tmp',tx));thread.start()
            try:
                parent.send(('login',('unused',1,'unused','unused')));self.assertTrue(parent.recv()[0])
                parent.send(('start_feedback',()));self.assertTrue(parent.recv()[0])
                first=_JOINT_PACKET.unpack(rx.recv(_JOINT_PACKET.size))[0]
                parent.send(('query',()));self.assertTrue(entered.wait(1))
                next_sample=_JOINT_PACKET.unpack(rx.recv(_JOINT_PACKET.size))[0]
                self.assertGreater(next_sample,first)
                self.assertFalse(parent.poll(.02),'state query unexpectedly completed')
                release.set();self.assertTrue(parent.recv()[0])
                parent.send(('logout',()));self.assertTrue(parent.recv()[0])
            finally:
                release.set();parent.close();rx.close();thread.join(1)
                self.assertFalse(thread.is_alive())


class SplitObserverTests(unittest.TestCase):
    def setUp(self):
        self.state, self.joints = Mock(), Mock()
        self.state.login.return_value = dict(return_code=0)
        self.joints.login.return_value = dict(return_code=0)
        self.state.logout.return_value = dict(return_code=0)
        self.joints.logout.return_value = dict(return_code=0)
        with patch('teleop.aubo.process_state.ProcessStateObserver',
                   side_effect=[self.state, self.joints]):
            self.observer = SplitProcessStateObserver(None, '/tmp')

    def test_state_and_joint_operations_are_routed_to_different_owners(self):
        self.observer.login('unused', 1, 'unused', 'unused')
        self.observer.start_waypoint_feedback()
        self.observer.query_state()
        self.assertIs(self.observer.cache, self.state.cache)
        self.assertIs(self.observer.waypoint_cache, self.joints.waypoint_cache)
        self.state.start_waypoint_feedback.assert_not_called()
        self.joints.query_state.assert_not_called()
        self.observer.invalidate()
        self.state.invalidate.assert_called_once()
        self.joints.invalidate.assert_not_called()

    def test_C_is_cleaned_up_when_B_logout_raises(self):
        self.state.logout.side_effect = RuntimeError('B failed')
        with self.assertRaisesRegex(RuntimeError, 'B failed'):
            self.observer.logout()
        self.joints.logout.assert_called_once()

    def test_C_logout_failure_is_not_reported_as_success(self):
        self.joints.logout.return_value = dict(return_code=17)
        self.assertEqual(self.observer.logout()['return_code'], 17)


if __name__=='__main__':unittest.main()
