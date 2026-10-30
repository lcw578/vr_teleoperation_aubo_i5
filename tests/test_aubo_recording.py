"""Isolated synthetic records and controls; never connects to hardware."""
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_aubo_follow import Clock
from test_aubo_six_axis import SixClient
from teleop.aubo.feedback import Feedback
from teleop.aubo.follow import J6FollowWorker
from teleop.aubo.quest_follow import J6QuestRuntime, main
from teleop.aubo.quest_preview import make_shadow
from teleop.aubo.operator_controls import OperatorButtons, xr_record
from teleop.aubo.recording import RawRecorder
from teleop.aubo.export_lerobot import load_recording, segments


def frame(seq, stamp, *, right=(), left=(), left_axes=(0., 0.), right_axes=(0., 0.),
          epoch='test', left_present=True):
    def controller(pressed, axes):
        return dict(position=[0.,0.,0.], orientation=[0.,0.,0.,1.],
                    buttons=[dict(p=i in pressed, v=float(i in pressed)) for i in range(6)],
                    axes=list(axes))
    controllers = dict(right=controller(right, right_axes))
    if left_present:
        controllers['left'] = controller(left, left_axes)
    return dict(type='xr_frame', epoch=epoch, seq=seq, t_client=stamp*1000., controllers=controllers)


def accepted(stamp, target=.1, episode=0):
    return dict(kind='sdk_command', sequence=1, episode_id=episode, host_monotonic_s=stamp,
        phase='returned', attempt=dict(outcome='sdk_accepted', recording_episode_id=episode,
            sdk_result=dict(return_code=0,sdk_invoked=True,target_rad=[target]*6,
                host_started_monotonic_s=stamp-.01,host_received_monotonic_s=stamp)))


def sample(stamp, **values):
    return dict(kind='sample', sequence=int(stamp*1000), episode_id=0, host_monotonic_s=stamp,
        measured_q_rad=[.05]*6,feedback_received_monotonic_s=stamp-.01,feedback_valid=True,
        hardware_state='active', operator_paused=False, **values)


class ButtonTests(unittest.TestCase):
    def test_short_press_survives_release_before_motion_tick_and_hold_is_not_repeat(self):
        buttons=OperatorButtons()
        buttons.feed(frame(0,10.),10.)
        buttons.feed(frame(1,10.01,left_axes=(-.8,0.)),10.01)
        buttons.feed(frame(2,10.02,left_axes=(-.9,0.)),10.02)
        buttons.feed(frame(3,10.03),10.03)
        self.assertEqual([r['command'] for r in buttons.drain()],['restart_loop'])
    def test_right_b_has_no_operator_binding(self):
        buttons=OperatorButtons()
        buttons.feed(frame(0,10.),10.)
        buttons.feed(frame(1,10.01,right=(5,)),10.01)
        buttons.feed(frame(2,10.02),10.02)
        self.assertEqual(buttons.drain(),[])
    def test_reappearing_held_left_button_needs_release_and_never_gates_right(self):
        buttons=OperatorButtons()
        buttons.feed(frame(0,10.,left_axes=(-.8,0.),left_present=False),10.)
        buttons.feed(frame(1,10.01,left_axes=(-.8,0.)),10.01)
        self.assertEqual(buttons.drain(),[])
        buttons.feed(frame(2,10.02),10.02)
        buttons.feed(frame(3,10.03,left_axes=(-.8,0.)),10.03)
        self.assertEqual(buttons.drain()[0]['command'],'restart_loop')
    def test_left_x_has_no_legacy_long_press_actions(self):
        buttons=OperatorButtons()
        buttons.feed(frame(0,10.),10.)
        for seq in range(1,15):
            stamp=10.+seq*.1
            buttons.feed(frame(seq,stamp,left=(4,)),stamp)
        self.assertEqual(buttons.drain(),[])
        buttons.feed(frame(15,11.5),11.5)
        buttons.feed(frame(16,11.6,left=(4,)),11.6)
        buttons.feed(frame(17,13.,left=(4,)),13.)
        self.assertEqual(buttons.drain(),[])
    def test_left_y_is_a_continuous_home_permit_with_release_event(self):
        buttons=OperatorButtons()
        buttons.feed(frame(0,10.),10.)
        self.assertEqual(buttons.feed(frame(1,10.01,left=(5,)),10.01),['home_hold_start'])
        self.assertTrue(buttons.home_hold_active(10.02))
        self.assertFalse(buttons.home_hold_active(10.22))
        self.assertEqual(buttons.feed(frame(2,10.03,left_present=False),10.03),['home_hold_end'])
        self.assertFalse(buttons.home_hold_active(10.03))
        self.assertEqual(buttons.feed(frame(3,10.04,left=(5,)),10.04),[])
    def test_duplicates_epoch_and_stale_frames_never_repeat_commands(self):
        buttons=OperatorButtons()
        buttons.feed(frame(0,10.),10.)
        buttons.feed(frame(1,10.01,left_axes=(-.8,0.)),10.01)
        buttons.feed(frame(1,10.01,left_axes=(-.8,0.)),10.02)
        buttons.feed(frame(2,10.02),10.03)
        buttons.feed(frame(3,10.03,left=(4,)),10.5)
        buttons.feed(frame(0,10.51,left=(4,),epoch='new'),10.51)
        self.assertEqual([v['command'] for v in buttons.drain()],['restart_loop'])
    def test_left_stick_maps_loop_controls_and_ignores_down(self):
        buttons=OperatorButtons()
        buttons.feed(frame(0,10.),10.)
        buttons.feed(frame(1,10.01,left_axes=(0.,0.,-.8,0.)),10.01)
        buttons.feed(frame(2,10.02),10.02)
        buttons.feed(frame(3,10.03,left_axes=(.8,0.)),10.03)
        buttons.feed(frame(4,10.04),10.04)
        buttons.feed(frame(5,10.05,left_axes=(0.,-.8)),10.05)
        buttons.feed(frame(6,10.06),10.06)
        buttons.feed(frame(7,10.07,left_axes=(0.,.8)),10.07)
        self.assertEqual([v['command'] for v in buttons.drain()],
                         ['restart_loop','exit_loop','exit_record'])
    def test_grip_has_no_operator_pause_binding_and_right_stick_marks(self):
        buttons=OperatorButtons()
        buttons.feed(frame(0,10.),10.)
        buttons.feed(frame(1,10.01,left=(1,)),10.01)
        buttons.feed(frame(2,10.02,right=(3,)),10.02)
        buttons.feed(frame(3,10.03,right=(3,)),10.03)
        self.assertEqual([v['command'] for v in buttons.drain()],['mark'])
    def test_overflow_is_visible(self):
        buttons=OperatorButtons(capacity=1)
        self.assertTrue(buttons.request('mark',10.))
        self.assertFalse(buttons.request('mark',10.01))
        self.assertEqual(buttons.error,'operator_event_queue_overflow')
    def test_optional_malformed_left_pose_is_not_serialized_as_nan(self):
        value=frame(0,10.)
        value['controllers']['left']['position']=[float('nan'),0,0]
        record=xr_record(value)
        self.assertNotIn('position',record['controllers']['left'])
        json.dumps(record,allow_nan=False)


class RecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'synthetic'
    def test_save_ack_is_after_flush_and_invalid_raw_is_retained(self):
        recorder=RawRecorder(self.path,task='Isolated synthetic test',metadata=dict(synthetic=True))
        recorder.begin()
        recorder.record('operator_event',command='mark')
        recorder.end(disposition='invalid')
        recorder.close()
        self.assertTrue(recorder.status()['last_result']['flushed'])
        summary=json.loads((self.path/'episodes/episode_000000.json').read_text())
        self.assertEqual(summary['disposition'],'invalid')
        self.assertEqual(summary['records'],3)
        _,_,episodes=load_recording(self.path)
        self.assertEqual(episodes,{})
        self.assertIn('mark',(self.path/'streams.jsonl').read_text())
    def test_status_exposes_active_episode_elapsed_time_and_keeps_previous_save(self):
        clock=Clock();recorder=RawRecorder(self.path,task='Isolated synthetic HUD test',clock=clock)
        try:
            self.assertTrue(recorder.begin())
            clock.advance(12.4)
            status=recorder.status()
            self.assertEqual(status['active_episode'],0)
            self.assertAlmostEqual(status['active_episode_elapsed_s'],12.4)
            self.assertEqual(status['saving_episodes'],[])
            self.assertTrue(recorder.end())
            recorder._queue.join()
            first=recorder.status()['last_result']
            self.assertEqual(first,dict(episode_id=0,disposition='saved',flushed=True))
            self.assertTrue(recorder.begin())
            status=recorder.status()
            self.assertEqual(status['active_episode'],1)
            self.assertEqual(status['last_result'],first)
        finally:
            recorder.close()
    def test_active_episode_on_exit_is_interrupted_not_silently_saved(self):
        recorder=RawRecorder(self.path,task='Isolated synthetic test')
        recorder.begin();recorder.close()
        summary=json.loads((self.path/'episodes/episode_000000.json').read_text())
        self.assertEqual(summary['disposition'],'interrupted')
        self.assertEqual(load_recording(self.path)[2],{})
    def test_no_catchup_and_true_feedback_time_is_preserved(self):
        clock=Clock();recorder=RawRecorder(self.path,task='Isolated synthetic test',clock=clock)
        recorder.begin()
        feedback=Feedback((.1,)*6,clock()-.02,'synthetic',True,0)
        self.assertTrue(recorder.sample(feedback=feedback,hardware=dict(state='active')))
        clock.advance(.01)
        self.assertFalse(recorder.sample(feedback=feedback,hardware=dict(state='active')))
        clock.advance(.2)
        self.assertTrue(recorder.sample(feedback=feedback,hardware=dict(state='active')))
        recorder.end();recorder.close()
        rows=[json.loads(l) for l in (self.path/'streams.jsonl').read_text().splitlines()]
        samples=[v for v in rows if v['kind']=='sample']
        self.assertEqual(len(samples),2)
        self.assertEqual(samples[0]['feedback_received_monotonic_s'],feedback.received_monotonic_s)
        self.assertEqual(samples[1]['feedback_received_monotonic_s'],feedback.received_monotonic_s)
    def test_pause_tag_holds_same_episode_and_suspends_regular_records(self):
        clock=Clock();recorder=RawRecorder(self.path,task='Synthetic pause lifecycle',clock=clock)
        try:
            recorder.begin()
            feedback=Feedback((.1,)*6,clock(),'synthetic',True,0)
            self.assertTrue(recorder.sample(feedback=feedback,hardware=dict(state='active')))
            clock.advance(.01)
            self.assertTrue(recorder.pause_episode('grip_released'))
            status=recorder.status()
            self.assertEqual(status['state'],'paused')
            self.assertEqual(status['active_episode'],0)
            self.assertEqual(status['pause_reason'],'grip_released')
            elapsed=status['active_episode_elapsed_s']
            self.assertFalse(recorder.sample(feedback=feedback,hardware=dict(state='idle')))
            self.assertFalse(recorder.record('xr',frame={}))
            self.assertFalse(recorder.record('sdk_command',phase='returned'))
            clock.advance(2.)
            self.assertAlmostEqual(recorder.status()['active_episode_elapsed_s'],elapsed)
            self.assertTrue(recorder.resume_episode(stamp=clock()))
            self.assertEqual(recorder.status()['active_episode'],0)
            self.assertTrue(recorder.sample(feedback=feedback,hardware=dict(state='active')))
            self.assertTrue(recorder.end())
        finally:
            recorder.close()
        rows=[json.loads(line) for line in (self.path/'streams.jsonl').read_text().splitlines()]
        self.assertEqual([row['kind'] for row in rows].count('pause'),1)
        self.assertEqual([row['kind'] for row in rows].count('resume'),1)
        self.assertEqual({row['episode_id'] for row in rows},{0})
        summary=json.loads((self.path/'episodes/episode_000000.json').read_text())
        self.assertEqual(summary['sample_count'],2)
    def test_busy_cycle_sampling_keeps_nominal_cadence_without_catchup(self):
        clock=Clock();recorder=RawRecorder(self.path,task='Isolated synthetic test',clock=clock)
        recorder.begin()
        for _ in range(201):
            recorder.sample(feedback=Feedback((0.,)*6,clock(),'synthetic',True,0),hardware=dict(state='active'))
            clock.advance(.005)
        recorder.end();recorder.close()
        summary=json.loads((self.path/'episodes/episode_000000.json').read_text())
        self.assertIn(summary['sample_count'],(25,26))
    def test_slow_writer_never_blocks_producer_and_overflow_prevents_export(self):
        gate=threading.Event();original=RawRecorder._run
        def stalled(recorder):
            gate.wait(1.);original(recorder)
        with patch.object(RawRecorder,'_run',stalled):
            recorder=RawRecorder(self.path,task='Isolated synthetic test',capacity=1)
        try:
            recorder.begin()
            start=time.monotonic()
            self.assertFalse(recorder.record('operator_event',command='mark'))
            self.assertLess(time.monotonic()-start,.05)
            self.assertIn('overflow',recorder.error)
        finally:
            gate.set()
            with self.assertRaisesRegex(RuntimeError,'incomplete'):recorder.close()
        with self.assertRaisesRegex(ValueError,'loss-free'):load_recording(self.path)
    def test_bad_cli_never_loads_sdk_or_creates_data(self):
        with patch('teleop.aubo.quest_follow.load_follow_native') as native,patch('sys.stderr',io.StringIO()):
            for args in (['--record'],['--task','test'],
                         ['--execute-six-axis','--command-mode','follow','--waypoint-feedback','--record','/tmp/no','--task','test']):
                with self.assertRaises(SystemExit):main(args)
            native.assert_not_called()


class ExportTests(unittest.TestCase):
    def test_future_command_is_never_paired_and_pauses_split_segments(self):
        rows=[accepted(10.08),*[sample(10.+i*.04) for i in range(10)]]
        rows[6]=dict(rows[6],operator_paused=True)
        result=segments(rows,25)
        self.assertEqual(len(result),2)
        self.assertGreaterEqual(result[0][0]['host_monotonic_s'],10.08)
        for part in result:
            for row in part:
                self.assertLessEqual(row['command_returned_monotonic_s'],row['host_monotonic_s'])
    def test_stale_feedback_and_rejected_or_previous_episode_command_are_excluded(self):
        old=dict(sample(10.1),feedback_received_monotonic_s=9.)
        self.assertEqual(segments([accepted(10.),old],25),[])
        command=accepted(10.)
        command['attempt']['outcome']='sdk_rejected'
        self.assertEqual(segments([command,sample(10.1)],25),[])
        command=accepted(10.);command['attempt']['recording_episode_id']=99
        self.assertEqual(segments([command,sample(10.1)],25),[])
    def test_stop_clears_action_until_new_accepted_command(self):
        rows=[accepted(10.),sample(10.),dict(kind='hardware_event',sequence=3,
                host_monotonic_s=10.01,reason='grip_released'),sample(10.04),sample(10.08)]
        result=segments(rows,25)
        self.assertEqual(len(result),1)
        self.assertEqual(len(result[0]),1)
    def test_pause_and_resume_events_cut_exported_actions(self):
        rows=[accepted(10.), sample(10.), sample(10.04),
            dict(kind='pause',sequence=3,episode_id=0,host_monotonic_s=10.05,tag='pause'),
            sample(10.08), sample(10.12),
            dict(kind='resume',sequence=6,episode_id=0,host_monotonic_s=10.13,tag='resume'),
            accepted(10.14), sample(10.16)]
        result=segments(rows,25)
        self.assertEqual([len(part) for part in result],[2,1])


class RuntimeControlTests(unittest.TestCase):
    def setUp(self):
        self.clock=Clock();self.client=SixClient(self.clock)
        self.worker=J6FollowWorker(self.client,[0.]*6,j6_range_rad=(-3.,3.),
            joint_ranges_rad=[[-3.,3.]]*6,clock=self.clock,follow_hz=25)
        self.shadow=make_shadow([0.]*6,[[-3.,3.]]*6)
        self.shadow.clock=self.clock;self.shadow.session.clock=self.clock
        self.shadow.session.cache.clock=self.clock
        self.runtime=J6QuestRuntime(self.shadow,self.worker,control_dt=.005,clock=self.clock)
        self.seq=0
        self.patch=patch('vr_teleop_kit.core.quest_teleop.time.monotonic',self.clock)
        self.patch.start();self.addCleanup(self.patch.stop)
    def tick(self, grip=False, *, right=(), left=(), left_axes=(0.,0.)):
        self.clock.advance(.005)
        self.shadow.feedback(self.client.q,received_monotonic_s=self.clock())
        self.runtime.submit_xr_frame(frame(self.seq,self.clock(),right=((1,) if grip else ())+tuple(right),
            left=left,left_axes=left_axes))
        self.seq+=1
        candidate=self.runtime.step()
        self.worker._cycle()
        return candidate
    def test_pause_blocks_held_grip_and_resume_requires_new_release_press(self):
        self.tick(False);self.tick(True)
        self.assertEqual(self.worker.status()['state'],'active')
        self.runtime.request_operator_command('pause')
        with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
            self.tick(True)
        self.assertTrue(self.runtime.operator_paused)
        self.assertTrue(self.worker.status()['stop_confirmed'])
        before=len(self.client.sent)
        self.tick(True);self.tick(True)
        self.assertEqual(len(self.client.sent),before)
        self.runtime.request_operator_command('pause')
        self.tick(True);self.tick(True)
        self.assertFalse(self.runtime.operator_paused)
        self.assertFalse(self.shadow.session.engaged)
        self.tick(False);self.tick(True)
        self.assertEqual(self.worker.status()['state'],'active')
    def test_left_missing_does_not_interrupt_and_no_auto_home(self):
        self.tick(False)
        self.clock.advance(.005)
        self.runtime.submit_xr_frame(frame(self.seq,self.clock(),right=(1,3),left_present=False))
        self.shadow.feedback(self.client.q,received_monotonic_s=self.clock())
        self.assertIsNotNone(self.runtime.step())
        self.assertFalse(self.shadow.core._arms['right']['ramp_active'])
        self.assertIn('未开启录制',self.runtime.last_operator_message)

    def test_right_b_does_not_pause_active_following(self):
        self.tick(False);self.tick(True)
        self.assertEqual(self.worker.status()['state'],'active')
        self.tick(True,right=(5,))
        self.assertFalse(self.runtime.operator_paused)
        self.assertFalse(self.runtime.episode_waiting_start)
        self.assertEqual(self.worker.status()['state'],'active')

    def test_worker_input_timeout_tags_pause_without_ending_episode(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'synthetic',task='Synthetic timeout pause',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                recorder.begin()
                self.worker._may_move=True
                self.worker._status['state']='active'
                self.worker.revoke('input_timeout')
                status=recorder.status()
                self.assertEqual(status['state'],'paused')
                self.assertEqual(status['active_episode'],0)
                self.assertEqual(status['pause_reason'],'input_timeout')
            finally:
                self.runtime.recorder=self.worker.recorder=None
                recorder.close()

    def test_grip_release_and_reengagement_resume_same_recording_segment(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic'
            recorder=RawRecorder(path,task='Synthetic clutch pause/resume',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                self.runtime.request_operator_command('toggle_episode')
                self.tick(False)
                self.tick(False)
                self.tick(True)
                self.clock.advance(.05);self.tick(True)
                self.assertEqual(self.worker.status()['state'],'active')
                self.assertEqual(recorder.status()['active_episode'],0)
                with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
                    self.tick(False)
                self.assertEqual(recorder.status()['state'],'paused')
                self.assertEqual(recorder.status()['active_episode'],0)
                self.tick(False)
                self.tick(True)
                self.tick(True)
                self.tick(True)
                self.assertEqual(self.worker.status()['state'],'active')
                self.assertEqual(recorder.status()['state'],'recording')
                self.assertEqual(recorder.status()['active_episode'],0)
            finally:
                self.runtime.recorder=self.worker.recorder=None
                recorder.close()
            kinds=[json.loads(line)['kind'] for line in (path/'streams.jsonl').read_text().splitlines()]
            self.assertIn('pause',kinds)
            self.assertIn('resume',kinds)

    def test_exit_loop_with_no_active_episode_never_starts_one(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'synthetic',task='Synthetic exit loop',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                self.runtime.request_operator_command('exit_loop')
                self.runtime._operator_tick()
                status=recorder.status()
                self.assertIsNone(status['active_episode'])
                self.assertEqual(status['saved_episodes'],0)
                self.assertFalse(self.runtime.session_finished)
            finally:
                self.runtime.recorder=self.worker.recorder=None
                recorder.close()

    def test_exit_loop_saves_current_episode_without_finishing_session(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'synthetic',task='Synthetic exit loop save',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                recorder.begin()
                feedback=Feedback((0.,)*6,self.clock(),'synthetic',True,0)
                with patch.object(self.runtime,'_stationary_feedback',return_value=feedback):
                    self.runtime.request_operator_command('exit_loop')
                    self.runtime._operator_tick()
                    recorder._queue.join()
                    self.runtime._operator_tick()
                self.assertEqual(recorder.status()['saved_episodes'],1)
                self.assertIsNone(recorder.status()['active_episode'])
                self.assertFalse(self.runtime.session_finished)
                self.assertTrue(self.runtime.episode_waiting_start)
            finally:
                self.runtime.recorder=self.worker.recorder=None
                recorder.close()

    def test_exit_record_saves_active_episode_after_stop_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'synthetic',task='Synthetic exit record',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                recorder.begin()
                self.tick(False);self.tick(True)
                self.assertEqual(self.worker.status()['state'],'active')
                self.runtime.request_operator_command('exit_record')
                with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
                    self.tick(True)
                    self.tick(True)
                recorder._queue.join()
                self.runtime._operator_tick()
                self.assertTrue(self.runtime.session_finished)
                self.assertIsNone(recorder.status()['active_episode'])
                self.assertEqual(recorder.status()['saved_episodes'],1)
                self.assertEqual(recorder.status()['last_result']['disposition'],'saved')
            finally:
                self.runtime.recorder=self.worker.recorder=None
                recorder.close()

    def test_x_saves_and_automatically_restarts_without_held_grip_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic'
            recorder=RawRecorder(path,task='Isolated synthetic X control test',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                self.tick(False)
                self.tick(False,left_axes=(-.8,0.))
                self.assertEqual(recorder.status()['active_episode'],0)
                self.tick(False);self.tick(True)
                self.clock.advance(.05);self.tick(True)
                self.assertEqual(self.worker.status()['state'],'active')
                self.assertGreater(len(self.client.sent),0)
                self.clock.advance(.005)
                self.runtime.submit_xr_frame(frame(self.seq,self.clock(),right=(1,),left_axes=(-.8,0.)))
                self.seq+=1
                # X revokes before the control owner consumes its event.
                self.assertFalse(self.client.permitted)
                self.assertTrue(self.runtime._operator_barrier)
                with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
                    self.runtime.step();self.worker._cycle()
                recorder._queue.join()
                self.assertTrue(self.runtime.episode_waiting_start)
                self.assertFalse(self.runtime.operator_paused)
                before=len(self.client.sent)
                self.tick(True)
                recorder._queue.join()
                self.tick(True)
                self.assertEqual(recorder.status()['active_episode'],1)
                self.assertFalse(self.runtime.episode_waiting_start)
                self.tick(True);self.tick(True)
                self.assertEqual(len(self.client.sent),before)
                self.assertFalse(self.shadow.session.engaged)
                self.tick(False);self.tick(True)
                self.assertEqual(self.worker.status()['state'],'active')
                self.clock.advance(.05);self.tick(True)
                self.assertGreater(len(self.client.sent),before)
                self.runtime.request_operator_command('toggle_episode')
                with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
                    self.tick(True)
                self.tick(True)
            finally:
                recorder.close()
                self.runtime.recorder=self.worker.recorder=None
            _,summaries,episodes=load_recording(path)
            self.assertEqual(set(episodes),{1})
            self.assertEqual(summaries[0]['disposition'],'invalid')
            self.assertTrue(all(not v['abnormal_stop'] for v in summaries.values()))

    def test_starting_episode_cannot_clear_web_pause_or_hardware_fault(self):
        with tempfile.TemporaryDirectory() as directory:
            recorder=RawRecorder(Path(directory)/'synthetic',task='Isolated synthetic latch test',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                self.tick(False)
                self.runtime.request_operator_command('pause');self.tick(False)
                self.worker.revoke('synthetic_hardware_fault',fatal=True)
                self.runtime.request_operator_command('toggle_episode');self.tick(False)
                self.assertTrue(self.runtime.operator_paused)
                self.assertTrue(self.worker.status()['fatal'])
                self.assertFalse(self.client.permitted)
                self.assertEqual(self.client.sent,[])
            finally:
                recorder.close()
                self.runtime.recorder=self.worker.recorder=None

    def test_actual_worker_commands_and_button_episode_end_are_captured(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic'
            recorder=RawRecorder(path,task='Isolated synthetic worker test',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                self.runtime.request_operator_command('toggle_episode')
                self.tick(False);self.tick(False);self.tick(True)
                self.clock.advance(.05);self.tick(True)
                self.tick(True)
                self.runtime.request_operator_command('toggle_episode')
                with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
                    self.tick(True)
                self.assertTrue(self.runtime.episode_waiting_start)
                self.tick(True)
            finally:
                recorder.close()
                self.runtime.recorder=self.worker.recorder=None
            rows=[json.loads(v) for v in (path/'streams.jsonl').read_text().splitlines()]
            returned=[v for v in rows if v['kind']=='sdk_command' and v['phase']=='returned']
            self.assertEqual(len(returned),len(self.client.sent))
            self.assertGreater(len(returned),0)
            self.assertEqual(returned[0]['attempt']['sdk_result']['target_rad'],list(self.client.sent[0]))
            self.assertEqual(returned[0]['attempt']['recording_episode_id'],0)
            self.assertTrue(load_recording(path)[1][0]['complete'])

    def test_software_reset_waits_for_stop_keeps_episode_and_requires_fresh_grip(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic'
            recorder=RawRecorder(path,task='Isolated synthetic reset test',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                recorder.begin()
                self.tick(False);self.tick(True)
                self.clock.advance(.05);self.tick(True)
                self.runtime.request_operator_command('software_reset')
                self.assertFalse(self.client.permitted)
                with patch.object(self.shadow,'reject_input',wraps=self.shadow.reject_input) as reset:
                    self.runtime._operator_tick()
                    self.assertTrue(self.runtime.software_reset_pending)
                    reset.assert_not_called()
                    with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
                        self.worker._cycle()
                    self.runtime._operator_tick()
                    self.assertFalse(self.runtime.software_reset_pending)
                    reset.assert_called_once_with('operator_software_reset')
                self.assertEqual(recorder.status()['active_episode'],0)
                self.assertEqual(recorder.status()['saved_episodes'],0)
                self.assertIsNone(self.worker._wanted)
                self.assertIsNone(self.shadow.last_candidate)
                self.assertFalse(self.shadow.core._arms['right']['ramp_active'])
                self.assertTrue(self.shadow.session.require_release)
                before=len(self.client.sent)
                self.tick(True);self.tick(True)
                self.assertEqual(len(self.client.sent),before)
                self.tick(False);self.tick(True)
                self.assertEqual(self.worker.status()['state'],'active')
                self.runtime.request_operator_command('toggle_episode')
                with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
                    self.tick(True)
                self.tick(True)
            finally:
                recorder.close()
                self.runtime.recorder=self.worker.recorder=None
            _,summaries,episodes=load_recording(path)
            self.assertEqual(set(episodes),{0})
            self.assertFalse(summaries[0]['abnormal_stop'])
            self.assertTrue(any(r['kind']=='software_reset' for r in episodes[0]))

    def test_reset_does_not_complete_with_stale_feedback_or_clear_fatal_fault(self):
        self.tick(False)
        self.runtime.request_operator_command('software_reset')
        self.clock.advance(.21)
        self.runtime._operator_tick()
        self.assertTrue(self.runtime.software_reset_pending)
        self.worker.revoke('synthetic_hardware_fault',fatal=True)
        with patch.object(self.shadow,'reject_input') as reset:
            self.runtime._operator_tick()
            reset.assert_not_called()
        self.assertFalse(self.runtime.software_reset_pending)
        self.assertTrue(self.worker.status()['fatal'])
        self.assertFalse(self.client.permitted)
        self.assertEqual(self.client.sent,[])

    def test_restart_waits_for_disk_ack_and_y_cancels_queued_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic'
            recorder=RawRecorder(path,task='Isolated synthetic save barrier test',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            from teleop.aubo.recording import atomic_json
            entered=threading.Event();release=threading.Event()
            def delayed_summary(target,value):
                if target.parent.name=='episodes':
                    entered.set()
                    if not release.wait(2.):
                        raise RuntimeError('Synthetic disk barrier timed out')
                atomic_json(target,value)
            try:
                self.tick(False)
                recorder.begin()
                with patch('teleop.aubo.recording.atomic_json',side_effect=delayed_summary):
                    self.runtime.request_operator_command('restart_episode')
                    self.tick(False)
                    self.assertTrue(entered.wait(1.))
                    self.tick(False)
                    self.assertEqual(recorder.status()['pending_saves'],1)
                    self.assertEqual(recorder.status()['saving_episodes'],[0])
                    self.assertIsNone(recorder.status()['active_episode'])
                    self.assertIsNotNone(self.runtime._episode_transition)
                    self.runtime.request_operator_command('invalidate');self.tick(False)
                    release.set();recorder._queue.join()
                    self.tick(False)
                    self.assertIsNone(self.runtime._episode_transition)
                    self.assertIsNone(recorder.status()['active_episode'])
                    self.assertTrue(self.runtime.episode_waiting_start)
            finally:
                release.set();recorder.close()
                self.runtime.recorder=self.worker.recorder=None

    def test_invalidate_ends_without_automatic_restart(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'synthetic'
            recorder=RawRecorder(path,task='Isolated synthetic invalidation test',clock=self.clock)
            self.runtime.recorder=self.worker.recorder=recorder
            try:
                recorder.begin()
                self.tick(False);self.tick(True)
                self.runtime.request_operator_command('invalidate')
                with patch('teleop.aubo.follow.time.sleep',side_effect=lambda _:self.clock.advance(.051)):
                    self.tick(True)
                self.tick(True);recorder._queue.join();self.tick(True)
                self.assertIsNone(recorder.status()['active_episode'])
                self.assertEqual(recorder.status()['invalid_episodes'],1)
                self.assertTrue(self.runtime.episode_waiting_start)
                self.assertEqual(recorder.status()['saved_episodes'],0)
                self.assertEqual(self.client.sent,[])
            finally:
                recorder.close()
                self.runtime.recorder=self.worker.recorder=None
            self.assertEqual(load_recording(path)[2],{})


if __name__ == '__main__':unittest.main(verbosity=2)
