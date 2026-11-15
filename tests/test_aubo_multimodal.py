"""Raw continuity, planner-boundary labels, and causal multi-camera alignment."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_aubo_fixed_tick_batch import FixedTickWorkerTests
from test_aubo_follow import Clock
from teleop.aubo.export_lerobot import CameraReader, multimodal_ticks, infer_task_stages
from teleop.aubo.check_dataset import validate_retry_audit
from teleop.aubo.recording import RawRecorder, tool_values
from teleop.aubo.cameras import CameraCapture, load_camera_config


class PlannerRecordingTests(unittest.TestCase):
    def test_batch_labels_are_planner_input_not_smoothed_endpoint(self):
        fixture = FixedTickWorkerTests()
        fixture.setUp()
        worker, clock = fixture.worker, fixture.clock
        with tempfile.TemporaryDirectory() as root:
            recorder = RawRecorder(Path(root)/'raw', task='Synthetic planner boundary', clock=clock)
            worker.recorder = recorder
            try:
                recorder.begin()
                worker._fixed_tick_send((.1,)*6, 'active')
                recorder.end()
            finally:
                recorder.close()
            rows = [json.loads(line) for line in (recorder.path/'streams.jsonl').read_text().splitlines()]
            admitted = [row for row in rows if row['kind']=='planner_target']
            self.assertEqual(admitted[0]['q_rad'], [.1]*6)
            self.assertNotEqual(admitted[0]['q_rad'], list(fixture.client.batches[-1][-1]))
            self.assertFalse(any(row['kind']=='sdk_command' for row in rows))

    def test_absolute_tool_is_measured_separately_and_never_toggle_pulse(self):
        data = dict(settings=dict(open_position=100, close_position=900), position=300,
                    feedback_received_s=10., commanded_closed=True, desired_closed=False)
        self.assertEqual(tool_values(data), dict(position=.25, target=1., received_monotonic_s=10.))
        self.assertIsNone(tool_values(dict(data, fault='serial error')))
        self.assertIsNone(tool_values(dict(data, settings={})))

    def test_camera_config_accepts_nonempty_subset_and_disabled_entries(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'cameras.json'
            specs = {
                'scene_left':dict(serial='left', width=64, height=48),
                'scene_right':dict(enabled=False),
                'wrist':dict(enabled=False),
            }
            path.write_text(json.dumps(specs))
            active = load_camera_config(path)
            self.assertEqual(list(active), ['scene_left'])
            specs['scene_right'] = dict(serial='duplicate', width=64, height=48)
            specs['scene_left']['serial'] = 'duplicate'
            path.write_text(json.dumps(specs))
            with self.assertRaises(ValueError):
                load_camera_config(path)

    def test_camera_config_accepts_wrapped_serial_number_format(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'cameras.json'
            path.write_text(json.dumps({'cameras': {
                'scene_left':dict(serial_number='L515-serial', width=1280, height=720, fps=30,
                                  enabled=True),
                'scene_right':dict(enabled=False),
            }}))
            active = load_camera_config(path)
            self.assertEqual(list(active), ['scene_left'])
            self.assertEqual(active['scene_left']['serial'], 'L515-serial')
            specs = {'scene_left':dict(enabled=False)}
            path.write_text(json.dumps(specs))
            with self.assertRaises(ValueError):
                load_camera_config(path)

    def test_camera_overflow_is_counted_and_not_silent_success(self):
        failure = Mock()
        capture = CameraCapture('/tmp/not-written', 'wrist', dict(serial='synthetic'),
                                session_id='test', on_error=failure, capacity=1)
        kwargs = dict(frame_number=1, device_timestamp_ms=0., timestamp_domain='synthetic', stamp=10.)
        self.assertTrue(capture.submit(None, **kwargs))
        self.assertFalse(capture.submit(None, **kwargs))
        self.assertEqual(capture.dropped, 1)
        failure.assert_called_once()

    def test_default_camera_queue_holds_two_seconds_at_30fps(self):
        capture = CameraCapture('/tmp/not-written', 'wrist', dict(serial='synthetic'),
                                session_id='test', on_error=Mock())
        self.assertEqual(capture.queue.maxsize, 60)


class StageInferenceTests(unittest.TestCase):
    def rows(self):
        rows = []
        for n in range(301):
            at = 10+n/30
            rows.append(dict(kind='sample', sequence=n+1, host_monotonic_s=at,
                task_stage='unknown', pause_type='none', hardware_state='active',
                measured_q_rad=[max(0.,at-17)*.1]*6, feedback_valid=True,
                feedback_received_monotonic_s=at-.01,
                planner_target=dict(q_rad=[.2]*6, admitted_monotonic_s=10., revision=1,
                                    control_source='teleop', mode='active'),
                tool=dict(position=0., target=0., received_monotonic_s=at-.01)))
        return rows

    def event(self, kind, at, **fields):
        self.seq = getattr(self, 'seq', 1000)+1
        return dict(kind=kind, host_monotonic_s=at, sequence=self.seq, **fields)

    def command(self, at, closed):
        return self.event('gripper_event', at, event='command_sent', closed=closed)

    def annotation(self, rows, at):
        events = sorted((r for r in rows if r['kind']=='task_annotation'),
                        key=lambda r:(r['host_monotonic_s'],r['sequence']))
        return [e for e in events if e['host_monotonic_s']<=at][-1]

    def test_retries_use_next_attempt_alignment_and_only_terminal_release(self):
        rows = self.rows()+[self.command(12,True),self.command(13,False),
                            self.command(16,True),self.command(19,False),
                            self.event('gripper_event',16.8,event='target_reached',closed=True)]
        original = json.dumps(rows)
        annotated,audit = infer_task_stages(rows)
        self.assertEqual(json.dumps(rows),original)
        self.assertEqual(audit['retry_count'],1)
        self.assertEqual(audit['completed_command_cycles'],2)
        for at,stage,aid in [(10,'approach',1),(11.1,'align',1),(12.1,'retry',1),
                              (13.1,'align',2),(16.1,'clamp_cut',2),(18,'transport',2),(19.1,'release',2)]:
            v = self.annotation(annotated,at)
            self.assertEqual((v['stage'],v['attempt_id']),(stage,aid))
        self.assertEqual(audit['attempts'][1]['transport_evidence'],'motion_after_closed_feedback')
        self.assertFalse(audit['physical_success_verified'])
        validate_retry_audit(audit)
        audit['retry_count']=0
        with self.assertRaisesRegex(ValueError,'Retry count mismatch'):
            validate_retry_audit(audit)

    def test_no_marks_or_arrival_uses_explicit_fallback_and_deduplicates(self):
        rows=self.rows()+[self.command(14,True),self.command(14.1,True),self.command(19,False)]
        rows += [self.event('gripper_event',12,event='trigger_toggle',closed=True)]
        annotated,audit=infer_task_stages(rows)
        self.assertEqual(len(audit['attempts']),1)
        self.assertEqual(len(audit['duplicate_command_sequences']),1)
        self.assertEqual(self.annotation(annotated,13.5)['stage'],'align')
        self.assertEqual(self.annotation(annotated,15)['stage'],'transport')
        self.assertTrue(any(b['assumed'] for b in audit['boundaries']))

    def test_a_and_sparse_marks_follow_tool_context_not_mark_count(self):
        buttons=[dict(p=False) for _ in range(6)];buttons[4]['p']=True
        rows=self.rows()+[self.event('xr',11,frame=dict(controllers=dict(right=dict(buttons=buttons)))),
            self.event('operator_event',11.5,command='mark'),self.command(14,True),
            self.event('gripper_event',14.2,event='target_reached',closed=True),
            self.event('operator_event',15,command='mark'),self.command(19,False)]
        annotated,audit=infer_task_stages(rows)
        self.assertEqual(audit['mark_count'],2)
        self.assertEqual(self.annotation(annotated,11.1)['inference_source'],'precision_button')
        self.assertEqual(self.annotation(annotated,14.5)['stage'],'clamp_cut')
        self.assertEqual(self.annotation(annotated,15)['inference_source'],'closed_mark')
        self.assertEqual(self.annotation(annotated,19.1)['stage'],'release')

    def test_incomplete_cycle_and_leading_open_do_not_claim_release(self):
        rows=self.rows()+[self.command(11,False),self.command(14,True)]
        annotated,audit=infer_task_stages(rows)
        self.assertFalse(audit['final_release_candidate'])
        self.assertEqual(audit['attempts'][0]['outcome'],'incomplete')
        self.assertEqual(len(audit['unmatched_open_sequences']),1)
        self.assertNotIn('release',[b['stage'] for b in audit['boundaries']])
        self.assertEqual(self.annotation(annotated,19)['stage'],'clamp_cut')

    def test_explicit_labels_win_but_attempts_still_export(self):
        rows=self.rows()+[self.command(14,True),self.command(19,False)]
        rows += [self.event('task_stage',17,stage='verify')]
        annotated,audit=infer_task_stages(rows)
        ticks=[t for t,f in multimodal_ticks(annotated,30,{}) if t]
        self.assertEqual(ticks[-1]['task_stage'],'verify')
        self.assertEqual(ticks[-1]['stage_evidence'],'online_label')
        self.assertEqual(ticks[-1]['attempt_id'],1)
        self.assertEqual(ticks[0]['task_stage'],'approach')

    def test_reset_overlay_and_episode_local_ids(self):
        rows=self.rows()+[self.event('pause',10.5,reason='home_return',pause_type='automatic_motion'),
            self.event('home_return',11,state='moving'),self.event('home_return',12,state='complete'),
            self.command(14,True),self.command(19,False)]
        annotated,audit=infer_task_stages(rows)
        self.assertEqual((self.annotation(annotated,11.5)['stage'],self.annotation(annotated,11.5)['attempt_id']),('reset',0))
        self.assertEqual(self.annotation(annotated,12.5)['attempt_id'],1)
        self.assertEqual(infer_task_stages(rows)[1]['attempts'][0]['attempt_id'],1)

    def test_three_cycles_count_attempts_and_do_not_count_arrival_as_command(self):
        rows=self.rows()+[self.command(12,True),self.command(13,False),self.command(14,True),
            self.command(15,False),self.command(17,True),self.command(19,False),
            self.event('gripper_event',17.2,event='target_reached',closed=True)]
        annotated,audit=infer_task_stages(rows)
        self.assertEqual([a['attempt_id'] for a in audit['attempts']],[1,2,3])
        self.assertEqual(audit['retry_count'],2)
        self.assertEqual([a['outcome'] for a in audit['attempts']],['retry_candidate','retry_candidate','final_candidate'])
        validate_retry_audit(audit)


class TickTests(unittest.TestCase):
    def test_default_image_age_accepts_120ms_but_rejects_160ms_and_future_frames(self):
        camera = CameraReader.__new__(CameraReader)
        for age, accepted in ((.12, True), (.16, False), (-.01, False)):
            camera.times = [10.-age]
            camera.rows = [dict(host_received_monotonic_s=10.-age, video_frame_index=0, frame_number=1)]
            tick, failure = next(multimodal_ticks(self.rows()[:1], 30, {'wrist':camera}))
            self.assertEqual(tick is not None, accepted)
            if not accepted:
                self.assertEqual(failure['reason'], 'missing_or_stale_image')
        camera.times = [9.88]
        self.assertEqual(next(multimodal_ticks(self.rows()[:1], 30, {'wrist':camera},
            image_max_age_s=.1))[1]['reason'], 'missing_or_stale_image')

    def rows(self):
        return [dict(kind='sample', sequence=n+1, host_monotonic_s=10.+n/30,
            measured_q_rad=[0.]*6, feedback_valid=True, feedback_received_monotonic_s=10.+n/30-.01,
            hardware_state='holding', operator_paused=True, pause_type='clutch_reanchor', task_stage='release',
            planner_target=dict(q_rad=[.2]*6, admitted_monotonic_s=9.9, revision=1,
                                control_source='placement', mode='holding'),
            tool=dict(position=.4, target=0., received_monotonic_s=10.+n/30-.02)) for n in range(5)]

    def test_hold_transport_and_release_are_kept_without_sdk_stream(self):
        camera = Mock()
        camera.select.side_effect=lambda stamp, age:dict(host_received_monotonic_s=stamp-.005,
                                                       video_frame_index=0, frame_number=1)
        output=list(multimodal_ticks(self.rows(), 30, {'wrist':camera}))
        self.assertEqual(len(output), 5)
        self.assertTrue(all(failure is None for _, failure in output))
        self.assertEqual(output[0][0]['action'], [.2]*6+[0.])
        self.assertEqual(output[0][0]['state'], [0.]*6+[.4])

    def test_exact_pause_and_planner_invalidation_between_samples(self):
        rows=self.rows()
        rows.insert(1,dict(kind='planner_invalid', sequence=10, host_monotonic_s=10.02))
        output=list(multimodal_ticks(rows,30,{}))
        self.assertIsNotNone(output[0][0])
        self.assertEqual(output[1][1]['reason'],'missing_admitted_planner_target')
        rows=self.rows()+[dict(kind='pause', sequence=20, host_monotonic_s=10.02, pause_type='task_wait')]
        output=list(multimodal_ticks(rows,30,{},exclude_pause_types=['task_wait']))
        self.assertIsNotNone(output[0][0])
        self.assertEqual(output[1][1]['reason'],'filtered_metadata')

    def test_future_images_and_stale_feedback_are_rejected(self):
        camera=Mock();camera.select.return_value=None
        output=list(multimodal_ticks(self.rows(),30,{'wrist':camera}))
        self.assertTrue(all(f['reason']=='missing_or_stale_image' for _,f in output))
        rows=self.rows();rows[0]['feedback_received_monotonic_s']=8.
        self.assertEqual(next(multimodal_ticks(rows,30,{}))[1]['reason'],'stale_joint_feedback')


if __name__=='__main__':
    unittest.main()
