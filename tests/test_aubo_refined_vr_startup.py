"""Qualified VR default selects exactly the accepted stream and A socket order."""
import json,tempfile,time,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from teleop.aubo.quest_follow import resolve_refined_stream,main

class RefinedVRTests(unittest.TestCase):
    def test_manifest_promotes_defaults_only_for_passthrough(self):
        with tempfile.TemporaryDirectory() as d:
            Path(d,'follow-build.json').write_text(json.dumps(dict(refined_stream_candidate=dict(default_enabled=True,execution_quality_qualified=True))))
            a=SimpleNamespace(refined_stream=None,execute_six_axis=True,runtime=d,tcp_ack_mode=None)
            self.assertTrue(resolve_refined_stream(a,'passthrough'))
            self.assertTrue(a.fixed_tick_batch);self.assertTrue(a.parallel_state_query);self.assertTrue(a.separate_waypoint_connection)
            self.assertEqual(a.tcp_ack_mode,'nodelay')
            self.assertFalse(resolve_refined_stream(a,'follow'))
    def test_unqualified_default_refuses_before_sdk(self):
        with tempfile.TemporaryDirectory() as d:
            Path(d,'follow-build.json').write_text('{"refined_stream_candidate":{"default_enabled":true,"execution_quality_qualified":false}}')
            a=SimpleNamespace(refined_stream=None,execute_six_axis=True,runtime=d,tcp_ack_mode=None)
            with self.assertRaisesRegex(ValueError,'qualification'):resolve_refined_stream(a,'passthrough')
    def test_prepared_c1_trial_does_not_claim_physical_qualification(self):
        with tempfile.TemporaryDirectory() as d:
            file=Path(d,'follow-build.json')
            for ready,accepted in ((True,True),('true',False),(False,False)):
                file.write_text(json.dumps(dict(refined_stream_candidate=dict(
                    default_enabled=True,execution_quality_qualified=False,supervised_c1_trial_ready=ready))))
                args=SimpleNamespace(refined_stream=None,execute_six_axis=True,runtime=d,tcp_ack_mode=None)
                if accepted:self.assertTrue(resolve_refined_stream(args,'passthrough'))
                else:
                    with self.assertRaises(ValueError):resolve_refined_stream(args,'passthrough')

    def test_a_nodelay_starts_before_b_login_and_worker_receives_refined_flags(self):
        calls=[];now=time.monotonic();client=Mock();observer=Mock();ack=Mock();native=Mock()
        native.hot_state_max_age_s=native.hot_joint_max_age_s=.5;native.priming_fast_s=.04
        native.J6Follower.return_value=client
        client.login.side_effect=lambda *a:(calls.append('A_login') or dict(return_code=0))
        client.logout.return_value=dict(return_code=0)
        client.prepare.return_value=dict(return_code=0,prepared=True,anchor_rad=[0.]*6,joint_ranges_rad=[[-3,3]]*6)
        observer.login.side_effect=lambda *a:(calls.append('BC_login') or dict(return_code=0,duration_s=0.))
        observer.logout.return_value=dict(return_code=0)
        observer.start_waypoint_feedback.return_value=dict(return_code=0,duration_s=0.)
        observer.waypoint_cache.feedback.return_value=dict(samples=[1])
        observer.query_state.return_value=dict(return_code=0,data=0,host_started_monotonic_s=now,host_received_monotonic_s=now,duration_s=0.)
        ack.start.side_effect=lambda:(calls.append('NODELAY') or ack)
        ack.status.return_value=dict(nodelay_active=1)
        ack.stop.return_value=dict(stop_reason='requested',error=None)
        worker=Mock(side_effect=RuntimeError('worker intercepted'))
        worker.sdk_budget_s=.5;worker.hot_state_max_age_s=.5;worker.hot_joint_max_age_s=.5;worker.priming_fast_s=.04
        with tempfile.TemporaryDirectory() as d:
            Path(d,'follow-build.json').write_text('{"refined_stream_candidate":{"default_enabled":true,"execution_quality_qualified":true}}')
            with patch('teleop.aubo.quest_follow.ROOT',Path(d)),patch('teleop.aubo.quest_follow.check_relay_port'), \
                 patch('teleop.aubo.quest_follow.load_follow_native',return_value=native), \
                 patch('teleop.aubo.quest_follow.SplitProcessStateObserver',return_value=observer), \
                 patch('teleop.aubo.quest_follow.TcpAckExperiment',return_value=ack), \
                 patch('teleop.aubo.quest_follow.J6FollowWorker',worker), \
                 patch('teleop.aubo.quest_follow.getpass.getpass',return_value='offline'), \
                 patch('teleop.aubo.quest_follow.validate_prepare_profile',return_value=dict(max_velocity_rad_s=None)):
                with self.assertRaisesRegex(RuntimeError,'worker intercepted'):
                    main(['--execute-six-axis','--waypoint-feedback','--runtime',d,'--usb',
                          '--host','127.0.0.1','--port','8899','--user','offline'])
        self.assertEqual(calls,['A_login','NODELAY','BC_login'])
        kw=worker.call_args.kwargs
        self.assertTrue(kw['refined_stream']);self.assertTrue(kw['wall_clock_batch']);self.assertTrue(kw['fixed_tick_batch'])
        self.assertEqual(kw['batch_point_limit'],8)
        ack.stop.assert_called_once();client.logout.assert_called_once();observer.logout.assert_called_once()
if __name__=='__main__':unittest.main()
