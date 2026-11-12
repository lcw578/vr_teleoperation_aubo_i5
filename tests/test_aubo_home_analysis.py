"""Synthetic file replay; no SDK or network access."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from teleop.aubo.analyze_home import audit, summarize, transitions


def home(state, stamp, requested=10., **extra):
    return dict(kind='home_return',state=state,host_monotonic_s=stamp,
        requested_monotonic_s=requested,error=None,**extra)


class HomeAnalysisTests(unittest.TestCase):
    def test_requests_not_events_and_already_home_is_not_moving(self):
        rows=[home('waiting_for_stop',10.),home('ready',10.1),home('moving',10.2),
              home('complete',12.),home('complete',20.,requested=20.)]
        report=summarize(rows,[])
        self.assertEqual(report['request_count'],2)
        self.assertEqual(report['moving_request_count'],1)

    def test_repeated_snapshots_do_not_inflate_calls_or_fatal_events(self):
        attempt=dict(session=-2,control_source='home_return',outcome='sdk_accepted',
            sdk_result=dict(operation='follow',host_started_monotonic_s=10.2,
                host_received_monotonic_s=10.25,duration_s=.05,return_code=0))
        snapshot=dict(last_attempt=attempt,first_fatal=dict(host_monotonic_s=12.,
            reason='stop_completion_over_budget_operator_stop_required'),last_stop=dict(
                operation='stop',host_started_monotonic_s=11.,host_received_monotonic_s=16.,
                duration_s=5.,return_code=10007))
        rows=[home('moving',10.1),home('stopping',11.),dict(home('failed',12.),
            error='hardware_fault_or_session_finished'),dict(kind='sdk_command',
                phase='returned',attempt=attempt)]
        result=summarize(rows,[snapshot]*9)['rounds'][0]
        self.assertEqual(result['observed_home_sdk_calls'],1)
        self.assertEqual(len(result['first_fatals']),1)
        self.assertEqual(len(result['stop_calls']),1)
        self.assertEqual(result['stop_calls'][0]['return_code'],10007)
        self.assertEqual(result['final_state'],'failed')
        self.assertIsNone(result['completed_monotonic_s'])

    def test_missing_tracking_sample_is_explicit_not_reconstructed(self):
        row=dict(home('failed',11.),error='home_tracking_error',max_error_deg=39.)
        result=summarize([home('moving',10.1),row],[])['rounds'][0]
        self.assertIsNone(result['failure_evidence'])
        self.assertIn('trigger_feedback_not_saved',result['limitations'][0])
        self.assertEqual(result['final_distance_to_home_max_deg'],39.)
        unfinished=summarize([home('stopping',11.)],[])['rounds'][0]
        self.assertIsNone(unfinished['completed_monotonic_s'])
        self.assertIn('at_target_is_not_stop_confirmed',unfinished['limitations'][0])

    def test_file_replay_retains_hashes_and_new_failure_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            evidence=dict(decision_feedback=dict(q_rad=[.2]*6))
            rows=[home('moving',10.1),dict(home('failed',11.),error='home_tracking_error',
                failure_evidence=evidence)]
            (root/'streams.jsonl').write_text('\n'.join(json.dumps(r) for r in rows))
            (root/'session_status.json').write_text(json.dumps(dict(closed=True,dropped_records=0)))
            log=root/'test.log';log.write_text('prefix Hardware transition: {"state":"idle"}\n')
            result=audit(root,log)
            self.assertEqual(result['rounds'][0]['failure_evidence'],evidence)
            self.assertEqual(len(result['sources']),3)
            self.assertTrue(all(len(s['sha256'])==64 for s in result['sources']))
            self.assertEqual(list(transitions(log.read_text().splitlines())),[dict(state='idle')])


if __name__=='__main__':unittest.main(verbosity=2)
