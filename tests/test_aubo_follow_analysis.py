"""Offline diagnostic math; no device dependency."""
import io
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teleop.aubo.analyze_follow import summarize, timing, main


class AnalysisTests(unittest.TestCase):
    def send(self, stamp, session=1, invoked=True):
        return dict(event='send_returned', attempt=dict(session=session, outcome='sdk_accepted',
            generated_monotonic_s=stamp-.002, sdk_result=dict(sdk_invoked=invoked,
                host_started_monotonic_s=stamp, duration_s=.007)))
    def test_frequency_excludes_released_gaps_and_local_refusals(self):
        report=summarize([self.send(1),self.send(1.05),self.send(2,invoked=False),
                          self.send(50,session=2),self.send(50.05,session=2)])
        self.assertEqual(report['invoked_follow_count'],4)
        self.assertEqual(report['within_engagement_send_intervals']['count'],2)
        self.assertAlmostEqual(report['aggregate_within_engagement_hz'],20.)
    def test_empty_and_unfinished_diagnostics_do_not_claim_success(self):
        report=summarize([])
        self.assertFalse(report['session_end_present'])
        self.assertIsNone(report['aggregate_within_engagement_hz'])
        self.assertIsNone(report['final_status']['stop_confirmed'])
    def test_nearest_rank_p95_and_nonfinite_filter(self):
        stats=timing([i/1000 for i in range(1,21)]+[float('nan'),-1])
        self.assertEqual(stats['count'],20);self.assertEqual(stats['p95_ms'],19.)
    def test_parallel_query_count_does_not_count_repeated_cache_reads(self):
        cached=dict(event='feedback',phase='active',snapshot=dict(state=dict(
            duration_s=.06,source='independent_sdk_state_cache')))
        report=summarize([cached]*20+[dict(event='state_query_returned',phase='active',
            result=dict(duration_s=.06))])
        self.assertEqual(report['state_query_duration']['count'],1)
        self.assertEqual(report['active_state_query_duration']['count'],1)
        self.assertEqual(summarize([cached])['state_query_duration']['count'],0)
    def test_stop_phases_and_incomplete_stop_remain_distinct(self):
        rows=[dict(event='stop_requested',host_monotonic_s=1.,trigger=dict(reason='grip_released')),
              dict(event='slow_stop_call_started',host_monotonic_s=1.02),
              dict(event='stop_returned',host_monotonic_s=2.,result=dict(duration_s=.98)),
              dict(event='stop_confirmed',host_monotonic_s=2.2),
              dict(event='stop_requested',host_monotonic_s=4.,trigger=dict(reason='input_timeout'))]
        report=summarize(rows)
        self.assertAlmostEqual(report['stop_request_to_call_start']['mean_ms'],20.)
        self.assertAlmostEqual(report['stop_request_to_confirm']['mean_ms'],1200.)
        self.assertEqual(len(report['stops']),2)
        self.assertNotIn('confirmed_s',report['stops'][1])
    def test_cli_reads_only_input_by_default(self):
        with patch.object(Path,'read_text',return_value='{"event":"session_start"}\n'), \
             patch.object(Path,'open') as opened,patch('sys.stdout',io.StringIO()) as stdout:
            self.assertEqual(main(['input.jsonl']),0)
            opened.assert_not_called()
        self.assertFalse(json.loads(stdout.getvalue())['session_end_present'])


if __name__ == '__main__':
    unittest.main(verbosity=2)
