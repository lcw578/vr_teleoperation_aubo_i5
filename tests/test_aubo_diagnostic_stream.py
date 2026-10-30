"""Fault evidence must survive recovery and disk lag without controlling a robot."""
import io
import gc
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_aubo_follow import Client, Clock
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.diagnostic_stream import DiagnosticStream


class DiagnosticTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.client = Client(self.clock)
        self.worker = J6FollowWorker(self.client, [0.]*6, j6_range_rad=(-3., 3.),
                                     clock=self.clock, diagnostics=True, waypoint_feedback=True)
        self.output = io.StringIO()

    def rows(self):
        return [json.loads(s) for s in self.output.getvalue().splitlines()]

    def test_overflow_is_explicit_and_does_not_block_producer(self):
        stream = DiagnosticStream(self.output, self.worker, snapshot=lambda:{})
        self.worker._stream_events = __import__('collections').deque(maxlen=2)
        for n in range(5):
            self.worker._diagnose('test', n=n)
        self.assertEqual(self.output.getvalue(), '')
        stream.poll()
        rows = self.rows()
        self.assertEqual(rows[0]['event'], 'diagnostic_stream_gap')
        self.assertEqual(rows[0]['missing_events'], 3)
        self.assertEqual([r['n'] for r in rows if r['event']=='test'], [3,4])
        self.assertEqual(len(self.worker.diagnostic_report()['events']), 5)

    def test_stream_keeps_small_history_without_truncating_written_log(self):
        stream = DiagnosticStream(self.output, self.worker, snapshot=lambda:{})
        count = self.worker.streaming_history_capacity+20
        for n in range(count):self.worker._diagnose('test', n=n)
        while stream.poll() == 256:pass
        self.assertEqual(len(self.worker._trace),self.worker.streaming_history_capacity)
        self.assertEqual(stream.written_events,count)
        self.assertEqual(stream.missing_events,0)
        self.assertEqual([r['n'] for r in self.rows() if r['event']=='test'],list(range(count)))

    def test_slow_gc_callback_is_buffered_without_synchronous_serialization(self):
        stream = DiagnosticStream(self.output,self.worker,snapshot=lambda:{})
        with patch('teleop.aubo.diagnostic_stream.time.monotonic',side_effect=(10.,10.12)):
            stream._gc_callback('start',dict(generation=2))
            stream._gc_callback('stop',dict(generation=2))
        self.assertEqual(self.output.getvalue(),'')
        stream.poll()
        event=next(r for r in self.rows() if r['event']=='python_gc_pause')
        self.assertAlmostEqual(event['duration_s'],.12)

    def test_incident_and_post_stop_feedback_are_saved_before_session_exit(self):
        trigger = dict(reason='hardware_feedback_unavailable', host_monotonic_s=self.clock())
        frozen = dict(trigger=trigger, recent_received=[dict(grip=True)])
        stream = DiagnosticStream(self.output, self.worker,
            snapshot=lambda:dict(input_stop=frozen), followup_s=0.)
        self.worker._diagnose('stop_requested', trigger=trigger)
        stream.poll()
        self.worker._diagnose('push_observed', sequence=7)
        stream.poll()
        rows = self.rows()
        self.assertEqual(sum(r['event']=='incident_input_frozen' for r in rows), 1)
        self.assertTrue(any(r['event']=='incident_followup' for r in rows))
        self.assertTrue(any(r['event']=='push_observed' for r in rows))
        self.assertEqual(self.client.sent, [])
        self.assertEqual(self.client.stops, 0)

    def test_exact_expiry_and_cache_read_time_are_in_stop_trigger(self):
        frame = dict(sequence=1, host_received_monotonic_s=self.clock(), q_rad=[0.]*6)
        self.client.waypoint_feedback = lambda:dict(samples=[frame])
        self.worker.measured_feedback()
        self.clock.advance(.21)
        try:
            self.worker.measured_feedback()
        except RuntimeError as exc:
            self.worker.note_feedback_failure(exc)
        self.worker._may_move = True
        self.worker.revoke('hardware_feedback_unavailable')
        event = self.worker.status()['first_stop_trigger']
        self.assertEqual(event['feedback_error']['error'], 'feedback_reception_expired')
        self.assertEqual(event['feedback_read']['sequence'], 1)
        self.assertEqual(event['feedback_read']['duration_s'], 0.)
        self.assertAlmostEqual(self.clock()-event['feedback_read']['callback_received_monotonic_s'], .21)

    def test_writer_failure_is_visible_without_revoking_motion(self):
        class Broken(io.StringIO):
            def write(self, value): raise OSError('disk unavailable')
        stream = DiagnosticStream(Broken(), self.worker, snapshot=lambda:{}, period_s=.001)
        stream.start()
        stream._thread.join(timeout=.5)
        self.assertIn('disk unavailable', stream.error)
        self.assertFalse(self.worker.status()['fatal'])
        self.assertEqual(self.client.rev, 0)
        with self.assertRaisesRegex(RuntimeError, 'Diagnostic writer failed'):
            stream.close()
        self.assertNotIn(stream._gc_callback,gc.callbacks)


if __name__ == '__main__':
    unittest.main()
